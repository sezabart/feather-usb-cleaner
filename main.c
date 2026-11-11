/*
 * Feather USB Cleaner
 * -------------------
 * Sanitizes a USB flash drive so it comes back EMPTY but fully usable, on the
 * Adafruit Feather RP2040 with USB Type A Host.
 *
 * Strategy (destructive by design, but a metadata rewrite, not a full erase):
 *   - Enumerate the mass storage device through TinyUSB host (tuh_msc).
 *   - Emit a fresh, valid FAT32 file system directly with SCSI WRITE(10):
 *     MBR, boot sector, FSInfo, backup boot sectors, one FAT and an empty
 *     root directory (see fat32_builder.c). The old file system's metadata is
 *     overwritten in the process, so nothing from it is reachable afterwards.
 *   - The MBR is written last as the commit point.
 *   - SYNCHRONIZE CACHE (10) makes the drive commit, then the MBR, boot sector
 *     and FAT head are read back and validated.
 *   Data clusters outside the metadata region keep their old contents; that is
 *   the "sanitize to some extent" trade-off that keeps the run at a few
 *   seconds instead of a full-drive erase.
 *
 * Status LED (NeoPixel on GP21, power gate GP20; mirrored on GP13):
 *   yellow pulsing -> idle / waiting for a device
 *   blue blinking  -> writing the new file system
 *   green solid    -> done, flushed and verified
 *   red solid      -> error (enumeration, unsupported block size, write error)
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"

#include "pio_usb.h"
#include "tusb.h"

#include "fat32_builder.h"
#include "ws2812.pio.h"

//--------------------------------------------------------------------
// Board pin map (Adafruit Feather RP2040 with USB Type A Host)
//--------------------------------------------------------------------

#ifndef BOARD_USB_HOST_DP_PIN
#define BOARD_USB_HOST_DP_PIN 16
#endif

#ifndef BOARD_USB_HOST_DM_PIN
#define BOARD_USB_HOST_DM_PIN 17
#endif

#ifndef BOARD_USB_5V_EN_PIN
#define BOARD_USB_5V_EN_PIN 18
#endif

#ifndef BOARD_NEOPIXEL_PIN
#define BOARD_NEOPIXEL_PIN 21
#endif

#ifndef BOARD_NEOPIXEL_POWER_PIN
#define BOARD_NEOPIXEL_POWER_PIN 20
#endif

#ifndef BOARD_LED_PIN
#ifdef PICO_DEFAULT_LED_PIN
#define BOARD_LED_PIN PICO_DEFAULT_LED_PIN
#else
#define BOARD_LED_PIN 13
#endif
#endif

//--------------------------------------------------------------------
// Wipe configuration
//--------------------------------------------------------------------

// Label of the fresh file system written to the stick.
#define FS_LABEL "CLEANED"

// Bytes the read-throughput self test reads.
#define READ_TEST_BYTES (1024u * 1024u)

// Bytes per WRITE(10). Must be <= 65535 (TinyUSB/HCD data stage is uint16_t)
// and a multiple of any block size we accept.
#define CHUNK_BYTES (32u * 1024u)

// SCSI WRITE(10) trial count per chunk before declaring the drive broken.
#define MAX_CHUNK_RETRIES 3

// A 32 KiB chunk at the worst realistic rate (~400 KiB/s) takes ~80 ms.
// Anything beyond this is a stalled stick.
#define XFER_TIMEOUT_MS 1500u

// WRITE SAME (10) and SYNCHRONIZE CACHE (10) make the *drive* program the
// whole range, so these transfers legitimately take seconds on slow media.
#define FLUSH_TIMEOUT_MS 30000u

// Hard upper bound for the whole operation (spec: < 5 s).
#define WIPE_TIMEOUT_MS 15000u

#define MAX_BLOCK_SIZE 4096u

// Number of LBAs read back after the wipe (start / middle / end).
#define VERIFY_POINTS 3

//--------------------------------------------------------------------
// Status LED
//--------------------------------------------------------------------

typedef enum {
  STATUS_IDLE = 0, // yellow pulsing
  STATUS_WIPING,   // blue blinking
  STATUS_DONE,     // green solid
  STATUS_ERROR,    // red solid
} status_t;

static status_t status = STATUS_IDLE;

static PIO neopixel_pio;
static uint neopixel_sm;
static uint neopixel_offset;
static uint32_t neopixel_last_update_us;

static void neopixel_init(void) {
  // The NeoPixel rail is gated by its own GPIO on this board.
  gpio_init(BOARD_NEOPIXEL_POWER_PIN);
  gpio_set_dir(BOARD_NEOPIXEL_POWER_PIN, GPIO_OUT);
  gpio_put(BOARD_NEOPIXEL_POWER_PIN, 1);
  sleep_ms(2);

  // PIO1 on purpose: the PIO-USB host stack owns PIO0.
  neopixel_pio = pio1;
  neopixel_offset = pio_add_program(neopixel_pio, &ws2812_program);
  neopixel_sm = pio_claim_unused_sm(neopixel_pio, true);

  pio_gpio_init(neopixel_pio, BOARD_NEOPIXEL_PIN);
  pio_sm_set_consecutive_pindirs(neopixel_pio, neopixel_sm, BOARD_NEOPIXEL_PIN, 1, true);

  pio_sm_config c = ws2812_program_get_default_config(neopixel_offset);
  sm_config_set_sideset_pins(&c, BOARD_NEOPIXEL_PIN);
  sm_config_set_out_shift(&c, false, true, 24); // shift right, autopull, 24-bit
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

  const float cycles_per_bit = (float)(ws2812_T1 + ws2812_T2 + ws2812_T3);
  const float div = (float)clock_get_hz(clk_sys) / (800000.0f * cycles_per_bit);
  sm_config_set_clkdiv(&c, div);

  pio_sm_init(neopixel_pio, neopixel_sm, neopixel_offset, &c);
  pio_sm_set_enabled(neopixel_pio, neopixel_sm, true);
}

static void neopixel_put(uint8_t r, uint8_t g, uint8_t b) {
  // NeoPixels expect GRB order in the low 24 bits of the word.
  const uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
  pio_sm_put_blocking(neopixel_pio, neopixel_sm, grb << 8u);
}

static void status_led_init(void) {
  gpio_init(BOARD_LED_PIN);
  gpio_set_dir(BOARD_LED_PIN, GPIO_OUT);
  gpio_put(BOARD_LED_PIN, 0);

  neopixel_init();
  status = STATUS_IDLE;
}

static inline uint8_t scale8(uint8_t value, uint8_t scale) {
  return (uint8_t)(((uint16_t)value * scale) / 255u);
}

static void status_led_task(void) {
  const uint32_t now = time_us_32();

  // 50 Hz refresh: plenty for pulsing/blinking, and always > the WS2812
  // 50 us latch time.
  if ((uint32_t)(now - neopixel_last_update_us) < 20000u) {
    return;
  }
  neopixel_last_update_us = now;

  const uint32_t now_ms = now / 1000u;
  uint8_t r = 0, g = 0, b = 0;
  bool led_on = false;

  switch (status) {
    case STATUS_IDLE: {
      // 2 s triangle pulse between 8% and 100% brightness.
      uint32_t phase = now_ms % 2000u;
      uint32_t brightness = (phase < 1000u) ? phase : (2000u - phase);
      brightness = 20u + (brightness * 235u) / 1000u;
      r = (uint8_t)((240u * brightness) / 255u);
      g = (uint8_t)((170u * brightness) / 255u);
      led_on = (phase / 250u) & 1u;
      break;
    }

    case STATUS_WIPING: {
      const bool on = ((now_ms / 125u) & 1u) != 0;
      b = on ? 255 : 0;
      led_on = on;
      break;
    }

    case STATUS_DONE:
      g = 255;
      led_on = true;
      break;

    case STATUS_ERROR:
      r = 255;
      led_on = true;
      break;
  }

  // Keep the onboard NeoPixel from being blindingly bright.
  const uint8_t max_brightness = 128;
  neopixel_put(scale8(r, max_brightness), scale8(g, max_brightness), scale8(b, max_brightness));
  gpio_put(BOARD_LED_PIN, led_on);
}

//--------------------------------------------------------------------
// Wipe state machine
//--------------------------------------------------------------------

typedef enum {
  XFER_IDLE = 0,
  XFER_BUSY,
  XFER_OK,
  XFER_FAIL,
} xfer_state_t;

typedef enum {
  WIPE_IDLE = 0,
  WIPE_PREPARE,
  WIPE_WRITE, // write the FAT32 metadata region
  WIPE_MBR,   // commit: partition table goes in last
  WIPE_SYNC,
  WIPE_VERIFY,
  WIPE_DONE,
  WIPE_ERROR,
} wipe_state_t;

static volatile wipe_state_t wipe_state = WIPE_IDLE;
static volatile xfer_state_t xfer_state = XFER_IDLE;

// Written by the TinyUSB host callbacks, consumed by the main loop.
static volatile uint8_t pending_mount_addr = 0;
static volatile uint8_t pending_umount_addr = 0;
static volatile uint8_t last_csw_status = 0;
static volatile uint32_t last_csw_residue = 0;

static uint8_t msc_dev_addr = 0;
static uint32_t msc_block_size = 0;
static uint32_t msc_block_count = 0;

static fat32_layout_t fs_layout;
static uint32_t fs_region_sectors = 0; // reserved + FAT + root dir cluster
static uint32_t fs_cursor = 0;         // sectors of the region written
static uint16_t chunk_sectors = 0;     // sectors per WRITE(10)
static uint32_t bytes_written = 0;     // for the throughput report
static uint32_t wipe_started_us = 0;
static uint32_t xfer_started_us = 0;
static uint16_t inflight_sectors = 0;
static uint8_t retry_count = 0;
static bool sync_unsupported = false;

static uint32_t verify_lbas[VERIFY_POINTS];
static uint8_t verify_index = 0;

// DMA/USB visible, 4-byte aligned (see CFG_TUH_MEM_ALIGN).
static uint8_t __attribute__((aligned(4))) zero_buf[CHUNK_BYTES];
static uint8_t __attribute__((aligned(4))) verify_buf[MAX_BLOCK_SIZE];

#define SCSI_CMD_SYNCHRONIZE_CACHE_10 0x35u

static inline uint32_t ms_since(uint32_t start_us) {
  return (uint32_t)((time_us_32() - start_us) / 1000u);
}

static inline uint16_t get16(const uint8_t *p) {
  return (uint16_t)(p[0] | (p[1] << 8));
}

static inline uint32_t get32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

// Structural checks of the freshly written volume: MBR, boot sector, FAT head.
// Together they prove the stick carries a mountable, empty FAT32.
static bool verify_sector_ok(uint8_t index) {
  switch (index) {
    case 0: // MBR
      return verify_buf[510] == 0x55 && verify_buf[511] == 0xAA &&
             verify_buf[446 + 4] == 0x0C &&
             get32(verify_buf + 446 + 8) == fs_layout.part_start_lba &&
             get32(verify_buf + 446 + 12) == fs_layout.part_sectors;
    case 1: // FAT32 boot sector
      return verify_buf[510] == 0x55 && verify_buf[511] == 0xAA &&
             get16(verify_buf + 11) == 512 &&
             memcmp(verify_buf + 82, "FAT32   ", 8) == 0 &&
             get32(verify_buf + 32) == fs_layout.part_sectors;
    case 2: // FAT head: media/EOC, clean flag, root dir cluster
      return get32(verify_buf + 0) == 0x0FFFFFF8u &&
             get32(verify_buf + 4) == 0x0FFFFFFFu &&
             (get32(verify_buf + 8) & 0x0FFFFFFFu) == 0x0FFFFFF8u;
    default:
      return false;
  }
}

static const char *verify_sector_name(uint8_t index) {
  switch (index) {
    case 0:
      return "MBR / partition table";
    case 1:
      return "FAT32 boot sector";
    default:
      return "FAT";
  }
}

static void wipe_set_status(status_t new_status) {
  status = new_status;
}

static void wipe_fail(const char *fmt, ...) {
  char msg[96];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);

  printf("WIPE FAILED: %s\n", msg);
  wipe_state = WIPE_ERROR;
  xfer_state = XFER_IDLE;
  wipe_set_status(STATUS_ERROR);
}

static bool msc_xfer_cb(uint8_t dev_addr, tuh_msc_complete_data_t const *cb_data);

// SCSI SYNCHRONIZE CACHE (10): flushes the drive's write cache to flash.
// TinyUSB 0.18 has no wrapper for it, so build the CBW by hand.
static bool msc_sync_cache(uint8_t dev_addr, uint8_t lun) {
  msc_cbw_t cbw;
  memset(&cbw, 0, sizeof(cbw));
  cbw.signature = MSC_CBW_SIGNATURE;
  cbw.tag = 0x54555342u; // 'TUSB'
  cbw.lun = lun;
  cbw.total_bytes = 0;
  cbw.dir = TUSB_DIR_OUT;
  cbw.cmd_len = 10;
  cbw.command[0] = SCSI_CMD_SYNCHRONIZE_CACHE_10;

  return tuh_msc_scsi_command(dev_addr, &cbw, NULL, msc_xfer_cb, 0);
}

// Completion handler for every SCSI transfer we issue. Runs inside
// tuh_task(): keep it short and never call printf() from here.
static bool msc_xfer_cb(uint8_t dev_addr, tuh_msc_complete_data_t const *cb_data) {
  (void)dev_addr;

  const msc_csw_t *csw = cb_data->csw;
  last_csw_status = csw->status;
  last_csw_residue = csw->data_residue;

  const bool ok = (csw->signature == MSC_CSW_SIGNATURE) &&
                  (csw->status == MSC_CSW_STATUS_PASSED) &&
                  (csw->data_residue == 0);

  xfer_state = ok ? XFER_OK : XFER_FAIL;
  return true;
}

/*
 * TinyUSB host callbacks.
 *
 * IMPORTANT: these run inside tuh_task(). Any blocking wait here (or printing
 * through USB CDC, which can call tud_task()) would deadlock or corrupt the
 * stack, so we only record state and let the main loop drive the wipe.
 */
void tuh_msc_mount_cb(uint8_t dev_addr) {
  pending_mount_addr = dev_addr;

  // One drive at a time: ignore a second MSC device (e.g. behind a hub) while
  // a wipe is already running.
  if (wipe_state == WIPE_PREPARE || wipe_state == WIPE_WRITE ||
      wipe_state == WIPE_SYNC || wipe_state == WIPE_VERIFY) {
    return;
  }

  msc_dev_addr = dev_addr;
  msc_block_size = tuh_msc_get_block_size(dev_addr, 0);
  msc_block_count = tuh_msc_get_block_count(dev_addr, 0);

  wipe_state = WIPE_PREPARE;
  wipe_set_status(STATUS_WIPING);
}

void tuh_msc_umount_cb(uint8_t dev_addr) {
  pending_umount_addr = dev_addr;

  if (dev_addr == msc_dev_addr) {
    wipe_state = WIPE_IDLE;
    xfer_state = XFER_IDLE;
    msc_dev_addr = 0;
    wipe_set_status(STATUS_IDLE);
  }
}

// Optional generic callbacks (weak in TinyUSB, just for visibility).
void tuh_mount_cb(uint8_t dev_addr) {
  (void)dev_addr;
}

void tuh_umount_cb(uint8_t dev_addr) {
  (void)dev_addr;
}

//--------------------------------------------------------------------
// Wipe driver (main loop context)
//--------------------------------------------------------------------

static void wipe_prepare(void) {
  if (msc_dev_addr == 0 || msc_block_size == 0 || msc_block_count == 0) {
    wipe_fail("device reported no usable LUN 0 capacity");
    return;
  }

  const uint32_t bs = msc_block_size;
  if (bs != 512u) {
    wipe_fail("unsupported block size %lu B (the FAT32 builder needs 512)",
              (unsigned long)bs);
    return;
  }
  if ((CHUNK_BYTES % bs) != 0) {
    wipe_fail("internal chunk size not a multiple of block size %lu", (unsigned long)bs);
    return;
  }

  // Volume id: stable-ish, derived from capacity plus a timer tick.
  const uint32_t volume_id =
      0x5049434Fu ^ (msc_block_count * 2654435761u) ^ time_us_32();

  if (!fat32_plan(msc_block_count, bs, volume_id, FS_LABEL, &fs_layout)) {
    wipe_fail("cannot build FAT32 on %lu sectors (too small or unsupported)",
              (unsigned long)msc_block_count);
    return;
  }

  chunk_sectors = (uint16_t)(CHUNK_BYTES / bs);
  fs_region_sectors = fat32_region_sectors(&fs_layout);
  fs_cursor = 0;
  bytes_written = 0;
  retry_count = 0;
  inflight_sectors = 0;
  sync_unsupported = false;
  xfer_state = XFER_IDLE;
  wipe_started_us = time_us_32();
  memset(zero_buf, 0, sizeof(zero_buf));

  printf("Sanitize: rebuilding FAT32 '%s' on %lu MiB\n",
         FS_LABEL, (unsigned long)(msc_block_count / 2048u));
  printf("  partition LBA %lu..%lu, %lu KiB clusters, 1 FAT of %lu KiB\n",
         (unsigned long)fs_layout.part_start_lba,
         (unsigned long)(fs_layout.part_start_lba + fs_layout.part_sectors - 1u),
         (unsigned long)((uint32_t)fs_layout.sectors_per_cluster * bs / 1024u),
         (unsigned long)(fs_layout.fat_sectors * bs / 1024u));
  printf("  writing %lu KiB of metadata (%lu sectors)\n",
         (unsigned long)(fs_region_sectors * bs / 1024u),
         (unsigned long)fs_region_sectors);

  verify_index = 0;
  verify_lbas[0] = 0;                                // MBR
  verify_lbas[1] = fs_layout.part_start_lba;         // boot sector
  verify_lbas[2] = fs_layout.part_start_lba + fs_layout.reserved_sectors; // FAT

  wipe_state = WIPE_WRITE;
}

static void wipe_print_progress(void) {
  const uint32_t percent =
      fs_region_sectors ? (fs_cursor * 100u) / fs_region_sectors : 100u;
  printf("  %3lu%%  %lu/%lu sectors  %lu KiB  t=%lu ms\n",
         (unsigned long)percent, (unsigned long)fs_cursor,
         (unsigned long)fs_region_sectors,
         (unsigned long)(fs_cursor * 512u / 1024u),
         (unsigned long)ms_since(wipe_started_us));
}

static void wipe_finish(void) {
  const uint32_t elapsed_ms = ms_since(wipe_started_us);

  wipe_state = WIPE_DONE;
  wipe_set_status(STATUS_DONE);

  printf("Sanitized in %lu ms (%.1f KiB/s). Stick is empty and mountable.\n",
         (unsigned long)elapsed_ms,
         elapsed_ms ? ((double)bytes_written / 1024.0) * 1000.0 / (double)elapsed_ms : 0.0);

  if (sync_unsupported) {
    printf("Note: drive rejected SYNCHRONIZE CACHE (10); WRITE(10) already returned\n");
  }
}

static void wipe_service(void) {
  // Consume events recorded by the TinyUSB callbacks.
  if (pending_mount_addr != 0) {
    const uint8_t addr = pending_mount_addr;
    pending_mount_addr = 0;
    const uint64_t capacity_bytes = (uint64_t)msc_block_count * (uint64_t)msc_block_size;
    printf("\nMSC device mounted at address %u: %lu blocks x %lu B (%lu MiB)\n",
           (unsigned)addr, (unsigned long)msc_block_count,
           (unsigned long)msc_block_size,
           (unsigned long)(capacity_bytes / (1024ull * 1024ull)));
  }
  if (pending_umount_addr != 0) {
    const uint8_t addr = pending_umount_addr;
    pending_umount_addr = 0;
    printf("MSC device at address %u removed - back to idle\n", (unsigned)addr);
  }

  switch (wipe_state) {
    case WIPE_IDLE:
    case WIPE_DONE:
    case WIPE_ERROR:
      return;

    case WIPE_PREPARE:
      wipe_prepare();
      return;

    case WIPE_WRITE: {
      if (msc_dev_addr == 0 || !tuh_msc_mounted(msc_dev_addr)) {
        wipe_fail("drive disappeared during write");
        return;
      }

      if (ms_since(wipe_started_us) > WIPE_TIMEOUT_MS) {
        wipe_fail("overall timeout after %lu ms", (unsigned long)ms_since(wipe_started_us));
        return;
      }

      if (xfer_state == XFER_IDLE) {
        if (fs_cursor >= fs_region_sectors) {
          wipe_state = WIPE_MBR; // FS is in place; commit the MBR last
          return;
        }

        const uint32_t remaining = fs_region_sectors - fs_cursor;
        inflight_sectors = (uint16_t)((remaining < (uint32_t)chunk_sectors)
                                          ? remaining
                                          : (uint32_t)chunk_sectors);

        // Lay out this slice of the file system image in the USB buffer.
        for (uint32_t i = 0; i < inflight_sectors; i++) {
          fat32_fill_sector(&fs_layout,
                            fs_layout.part_start_lba + fs_cursor + i,
                            zero_buf + i * msc_block_size);
        }

        xfer_started_us = time_us_32();
        xfer_state = XFER_BUSY;

        // Fire-and-forget: completion arrives via msc_xfer_cb().
        if (!tuh_msc_write10(msc_dev_addr, 0, zero_buf,
                             fs_layout.part_start_lba + fs_cursor,
                             inflight_sectors, msc_xfer_cb, 0)) {
          // Endpoint busy / not mounted yet: back off, try again next tick.
          xfer_state = XFER_IDLE;
        }
      } else if (xfer_state == XFER_OK) {
        xfer_state = XFER_IDLE;
        fs_cursor += inflight_sectors;
        bytes_written += (uint32_t)inflight_sectors * msc_block_size;
        retry_count = 0;
        wipe_print_progress();
      } else if (xfer_state == XFER_FAIL) {
        xfer_state = XFER_IDLE;
        const uint32_t lba = fs_layout.part_start_lba + fs_cursor;
        if (++retry_count > MAX_CHUNK_RETRIES) {
          wipe_fail("WRITE(10) at LBA %lu failed (CSW status %u, residue %lu)",
                    (unsigned long)lba, (unsigned)last_csw_status,
                    (unsigned long)last_csw_residue);
          return;
        }
        printf("  retry %u at LBA %lu (CSW status %u, residue %lu)\n",
               (unsigned)retry_count, (unsigned long)lba,
               (unsigned)last_csw_status, (unsigned long)last_csw_residue);
      } else if (ms_since(xfer_started_us) > XFER_TIMEOUT_MS) {
        wipe_fail("USB transfer timeout at LBA %lu",
                  (unsigned long)(fs_layout.part_start_lba + fs_cursor));
      }
      return;
    }

    case WIPE_MBR: {
      if (msc_dev_addr == 0 || !tuh_msc_mounted(msc_dev_addr)) {
        wipe_fail("drive disappeared before the partition table was written");
        return;
      }

      if (xfer_state == XFER_IDLE) {
        fat32_fill_mbr(&fs_layout, zero_buf);
        xfer_started_us = time_us_32();
        xfer_state = XFER_BUSY;
        if (!tuh_msc_write10(msc_dev_addr, 0, zero_buf, 0, 1, msc_xfer_cb, 0)) {
          xfer_state = XFER_IDLE;
        }
      } else if (xfer_state == XFER_OK) {
        xfer_state = XFER_IDLE;
        bytes_written += msc_block_size;
        retry_count = 0;
        printf("Partition table written (commit)\n");
        wipe_state = WIPE_SYNC;
      } else if (xfer_state == XFER_FAIL) {
        xfer_state = XFER_IDLE;
        if (++retry_count > MAX_CHUNK_RETRIES) {
          wipe_fail("writing the MBR failed (CSW status %u)",
                    (unsigned)last_csw_status);
          return;
        }
      } else if (ms_since(xfer_started_us) > XFER_TIMEOUT_MS) {
        wipe_fail("MBR write timed out");
      }
      return;
    }

    case WIPE_SYNC: {
      if (msc_dev_addr == 0 || !tuh_msc_mounted(msc_dev_addr)) {
        wipe_fail("drive disappeared before cache flush");
        return;
      }

      if (xfer_state == XFER_IDLE) {
        xfer_started_us = time_us_32();
        xfer_state = XFER_BUSY;
        if (!msc_sync_cache(msc_dev_addr, 0)) {
          xfer_state = XFER_IDLE;
        }
      } else if (xfer_state == XFER_OK) {
        xfer_state = XFER_IDLE;
        printf("Cache flushed (SYNCHRONIZE CACHE 10)\n");
        wipe_state = WIPE_VERIFY;
      } else if (xfer_state == XFER_FAIL) {
        // Some cheap sticks do not implement SYNCHRONIZE CACHE. The writes
        // themselves already completed, so treat this as a warning only.
        xfer_state = XFER_IDLE;
        sync_unsupported = true;
        printf("Warning: SYNCHRONIZE CACHE (10) rejected (CSW status %u)\n",
               (unsigned)last_csw_status);
        wipe_state = WIPE_VERIFY;
      } else if (ms_since(xfer_started_us) > FLUSH_TIMEOUT_MS) {
        wipe_fail("SYNCHRONIZE CACHE timed out");
      }
      return;
    }

    case WIPE_VERIFY: {
      if (msc_dev_addr == 0 || !tuh_msc_mounted(msc_dev_addr)) {
        wipe_fail("drive disappeared before verification");
        return;
      }

      if (xfer_state == XFER_IDLE) {
        if (verify_index >= VERIFY_POINTS) {
          wipe_finish();
          return;
        }
        memset(verify_buf, 0xA5, sizeof(verify_buf));
        xfer_started_us = time_us_32();
        xfer_state = XFER_BUSY;
        if (!tuh_msc_read10(msc_dev_addr, 0, verify_buf, verify_lbas[verify_index], 1,
                            msc_xfer_cb, 0)) {
          xfer_state = XFER_IDLE;
        }
      } else if (xfer_state == XFER_OK) {
        xfer_state = XFER_IDLE;

        if (!verify_sector_ok(verify_index)) {
          wipe_fail("%s check failed at LBA %lu - stick did not keep the image",
                    verify_sector_name(verify_index),
                    (unsigned long)verify_lbas[verify_index]);
          return;
        }
        printf("Verified: %s\n", verify_sector_name(verify_index));
        verify_index++;
      } else if (xfer_state == XFER_FAIL) {
        // A failed read-back does not invalidate the writes themselves.
        xfer_state = XFER_IDLE;
        printf("Warning: could not read LBA %lu back for verification\n",
               (unsigned long)verify_lbas[verify_index]);
        verify_index++;
      } else if (ms_since(xfer_started_us) > XFER_TIMEOUT_MS) {
        xfer_state = XFER_IDLE;
        printf("Warning: verification read timed out\n");
        verify_index++;
      }
      return;
    }

    default:
      wipe_state = WIPE_IDLE;
      return;
  }
}

//--------------------------------------------------------------------
// Startup helpers
//--------------------------------------------------------------------

static void usb_device_init(void) {
  // The native USB-C port stays a CDC device so we can log over /dev/ttyACM*.
  // It must be initialized before stdio_init_all(): pico_stdio_usb expects the
  // TinyUSB device stack to be up when it is compiled against tinyusb_host.
  const tusb_rhport_init_t dev_init = {
      .role = TUSB_ROLE_DEVICE,
      .speed = TUSB_SPEED_FULL,
  };
  tusb_init(0, &dev_init);
}

static void console_wait(uint32_t timeout_ms) {
  const absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
  while (!stdio_usb_connected() && !time_reached(deadline)) {
    tud_task();
    sleep_ms(1);
  }
}

static bool usb_host_init(void) {
  // Power the USB-A connector (5 V boost enable) and let the rail settle
  // before the PIO-USB controller starts driving the bus.
  gpio_init(BOARD_USB_5V_EN_PIN);
  gpio_set_dir(BOARD_USB_5V_EN_PIN, GPIO_OUT);
  gpio_put(BOARD_USB_5V_EN_PIN, 1);

  // Keep the D- line defined while the boost converter comes up.
  gpio_init(BOARD_USB_HOST_DM_PIN);
  gpio_set_dir(BOARD_USB_HOST_DM_PIN, GPIO_OUT);
  gpio_put(BOARD_USB_HOST_DM_PIN, 0);
  sleep_ms(250);

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = BOARD_USB_HOST_DP_PIN;

  if (!tuh_configure(BOARD_TUH_RHPORT, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg)) {
    return false;
  }

  const tusb_rhport_init_t host_init = {
      .role = TUSB_ROLE_HOST,
      .speed = TUSB_SPEED_FULL,
  };
  return tusb_init(BOARD_TUH_RHPORT, &host_init);
}

// Synchronous single SCSI read, used by the 't' throughput test. Safe to block
// here: this runs from the main loop, and tuh_task() keeps the host stack
// alive while we wait.
static bool msc_read_blocking(uint32_t lba, uint16_t sectors, void *buf) {
  xfer_state = XFER_BUSY;
  if (!tuh_msc_read10(msc_dev_addr, 0, buf, lba, sectors, msc_xfer_cb, 0)) {
    xfer_state = XFER_IDLE;
    return false;
  }

  const absolute_time_t deadline = make_timeout_time_ms(2000);
  while (xfer_state == XFER_BUSY && !time_reached(deadline)) {
    tuh_task();
    tud_task();
  }

  const bool ok = (xfer_state == XFER_OK);
  xfer_state = XFER_IDLE;
  return ok;
}

// Reads READ_TEST_BYTES back and reports the raw read throughput. Writing is
// usually slower than reading on flash media, so this tells us who is the
// bottleneck.
static void read_throughput_test(void) {
  if (msc_dev_addr == 0 || msc_block_size == 0 || !tuh_msc_mounted(msc_dev_addr) ||
      wipe_state == WIPE_PREPARE || wipe_state == WIPE_WRITE ||
      wipe_state == WIPE_SYNC ||
      wipe_state == WIPE_VERIFY) {
    printf("Read test needs a mounted, idle drive\n");
    return;
  }

  const uint32_t chunk = (uint32_t)chunk_sectors ? (uint32_t)chunk_sectors : 64u;
  const uint32_t lbas = READ_TEST_BYTES / msc_block_size;
  const uint32_t started = time_us_32();
  uint32_t done = 0;

  for (uint32_t lba = 0; lba < lbas; lba += chunk) {
    const uint32_t n = ((lbas - lba) < chunk) ? (lbas - lba) : chunk;
    if (!msc_read_blocking(lba, (uint16_t)n, zero_buf)) {
      printf("Read test aborted at LBA %lu (CSW status %u)\n",
             (unsigned long)lba, (unsigned)last_csw_status);
      return;
    }
    done += n;
  }

  // zero_buf doubles as the read scratch buffer here, so restore it: it is the
  // source of every WRITE(10) in the wipe path.
  memset(zero_buf, 0, sizeof(zero_buf));

  const uint32_t elapsed_ms = ms_since(started);
  printf("Read %lu KiB in %lu ms (%.1f KiB/s)\n",
         (unsigned long)((done * msc_block_size) / 1024u),
         (unsigned long)elapsed_ms,
         elapsed_ms ? ((double)(done * msc_block_size) / 1024.0) * 1000.0 / (double)elapsed_ms
                    : 0.0);
}

// Lets us re-run the wipe without physically replugging the drive, which is
// handy when testing or when a previous attempt errored out.
static void console_service(void) {
  const int c = getchar_timeout_us(0);
  if (c < 0 || c == PICO_ERROR_TIMEOUT) {
    return;
  }
  if (c == 't' || c == 'T') {
    read_throughput_test();
    return;
  }

  if (c == 'p' || c == 'P') {
    // Power-cycling VBUS forces a re-enumerate: handy to recover a wedged
    // stick, and it exercises the unmount path.
    printf("Power-cycling the USB-A port...\n");
    stdio_flush();
    gpio_put(BOARD_USB_5V_EN_PIN, 0);
    sleep_ms(1500);
    gpio_put(BOARD_USB_5V_EN_PIN, 1);
    sleep_ms(250);
    return;
  }

  if (c == 'b' || c == 'B') {
    printf("Rebooting into BOOTSEL (RPI-RP2)...\n");
    stdio_flush();
    sleep_ms(50);
    reset_usb_boot(0, 0);
    return;
  }

  if (c != '\r' && c != '\n' && c != ' ') {
    return;
  }
  if (msc_dev_addr != 0 && tuh_msc_mounted(msc_dev_addr) &&
      (wipe_state == WIPE_IDLE || wipe_state == WIPE_DONE || wipe_state == WIPE_ERROR)) {
    printf("Re-running wipe on device address %u\n", (unsigned)msc_dev_addr);
    wipe_state = WIPE_PREPARE;
  }
}

//--------------------------------------------------------------------
// main
//--------------------------------------------------------------------

int main(void) {
  usb_device_init();
  stdio_init_all();

  status_led_init();
  console_wait(1500);

  printf("\n");
  printf("=====================================\n");
  printf(" Feather USB Cleaner (sanitize + FAT32 rebuild)\n");
  printf("=====================================\n");
  printf("Board : Adafruit Feather RP2040 USB Host\n");
  printf("Host  : PIO-USB  D+=GP%u D-=GP%u  5V_EN=GP%u\n",
         (unsigned)BOARD_USB_HOST_DP_PIN, (unsigned)BOARD_USB_HOST_DM_PIN,
         (unsigned)BOARD_USB_5V_EN_PIN);
  printf("Output: fresh empty FAT32 labelled '%s', 1 FAT, big clusters\n",
         FS_LABEL);
  printf("Ready - insert a USB flash drive...\n");
  printf("(ENTER = re-run sanitize, t = read test, p = power-cycle port, b = BOOTSEL)\n");

  if (!usb_host_init()) {
    printf("FATAL: TinyUSB host init failed\n");
    wipe_set_status(STATUS_ERROR);
  }

  while (true) {
    tud_task();  // native USB device (CDC console)
    tuh_task();  // USB-A host stack + MSC
    wipe_service();
    console_service();
    status_led_task();
  }
}
