/*
 * Feather USB Cleaner
 * -------------------
 * Sanitizes a USB flash drive so it comes back EMPTY but fully usable, on the
 * Adafruit Feather RP2040 with USB Type A Host.
 *
 * Battery operation (see "Power off" below): when there is nothing to do the
 * board turns the USB-A rail and the NeoPixel rail off and drops into RP2040
 * DORMANT mode. It stays dark until the reset button is pressed; RST boots it,
 * it sanitizes whatever is in the port, and it goes dark again.
 *
 * Strategy (destructive by design, but a metadata rewrite, not a full erase):
 *   - Any drive size is accepted. Above 32 GiB, where a drive no longer ships
 *     FAT32, the LED shows magenta for a moment before the wipe starts.
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
 *   yellow pulsing   -> idle / waiting for a device
 *   blue blinking    -> writing the new file system
 *   green solid      -> done, flushed and verified
 *   magenta blinking -> drive refused (larger than 32 GiB)
 *   red solid        -> error (enumeration, unsupported block size, write error)
 *   off              -> powered down (until RESET)
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
#include "hardware/pll.h"
#include "hardware/rosc.h"
#include "hardware/sync.h"
#include "hardware/xosc.h"
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

// Backstop for the whole operation, scaled with the amount of metadata: the
// biggest FAT this can build is ~8 MiB per 128 GiB of drive, and cheap sticks
// write at ~100 KiB/s. A fixed bound would either be too tight for a big drive
// or useless for a small one. A wedge is actually caught by XFER_TIMEOUT_MS per
// chunk; this only catches a wipe that runs away without failing a chunk.
#define WIPE_TIMEOUT_BASE_MS 15000u
#define WIPE_MIN_BYTES_PER_MS 100u // ~100 KiB/s

#define MAX_BLOCK_SIZE 4096u

// Number of LBAs read back after the wipe (start / middle / end).
#define VERIFY_POINTS 3

// Above this size a drive no longer ships FAT32 (Windows will not create one
// that big), so the wipe is worth announcing: such a drive gets a magenta
// heads-up before the blue writes start. It is cleaned exactly like any other -
// 32 GiB is where being an ordinary FAT32 stick ends, not a limit.
#define LARGE_DRIVE_BYTES (32ull * 1024ull * 1024ull * 1024ull)
#define LARGE_NOTICE_MS 2000u

// How long the result colour stays on before the board powers down.
#define RESULT_HOLD_MS 2500u

// How long the board stays awake with nothing mounted before it powers down.
// Covers the enumeration of a drive that is already plugged in at boot.
#define IDLE_SLEEP_MS 5000u

// How long a drive that is already attached at boot gets to enumerate before we
// call it unrecognised.
#define NO_ENUM_MS 4000u

// Optional hardware: this board wires USB-C VBUS only to the charger and the
// load-share FET, so there is nothing to *see* USB-C power from the RP2040. The
// most it can tell is "a USB host enumerated me", which misses dumb chargers.
// Fit a 2x100k divider from the Feather's USB (VBUS) pin to A0 and define the
// pin below to sense real USB-C power (2.5 V on USB, 0 V on battery). Leave it
// undefined on an unmodified board: a floating A0 reads noise, and that would
// keep the board awake forever.
// #define USB_POWER_SENSE_PIN 26 // A0

#ifdef USB_POWER_SENSE_PIN
#include "hardware/adc.h"
#define USB_POWER_SENSE_ADC_INPUT 0
#define USB_POWER_SENSE_THRESHOLD 1500u // ~1.2 V of 3.3 V over 4095
#endif

// How long the boot-time bus sense waits for a drive to show its pull-up after
// it sees VBUS. Older drives are slow to start.
#define BUS_SENSE_MS 400u

// How long to wait for the USB-C console to attach at boot, so the banner is
// not printed into the void on a workbench. Costs the same on every battery
// wake, so keep it small.
#define CONSOLE_WAIT_MS 1500u

//--------------------------------------------------------------------
// Status LED
//--------------------------------------------------------------------

typedef enum {
  STATUS_IDLE = 0, // yellow pulsing
  STATUS_WIPING,   // blue blinking
  STATUS_DONE,     // green solid
  STATUS_LARGE,    // magenta blinking: drive over LARGE_DRIVE_BYTES
  STATUS_UNRECOGNISED, // red blinking: drive attached, never enumerated
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

    case STATUS_LARGE: {
      // Slow magenta blink: "heads-up, this one is big".
      const bool on = ((now_ms / 250u) & 1u) != 0;
      r = on ? 200 : 0;
      b = on ? 255 : 0;
      led_on = on;
      break;
    }

    case STATUS_UNRECOGNISED: {
      // Blinking red: something is plugged in, but it never answered the host.
      const bool on = ((now_ms / 250u) & 1u) != 0;
      r = on ? 255 : 0;
      led_on = on;
      break;
    }

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

// Goes dark and cuts the NeoPixel rail; power_off() also drops the USB-A rail.
static void status_led_shutdown(void) {
  neopixel_put(0, 0, 0);
  sleep_ms(1); // > the 50 us WS2812 latch time, so the frame is accepted
  gpio_put(BOARD_NEOPIXEL_POWER_PIN, 0);
  gpio_put(BOARD_LED_PIN, 0);
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
  WIPE_LARGE, // heads-up colour before writing a drive over 32 GiB
  WIPE_ERROR,
} wipe_state_t;

static volatile wipe_state_t wipe_state = WIPE_IDLE;
static volatile xfer_state_t xfer_state = XFER_IDLE;

// When the current state was entered. Drives the "show the result, then go back
// to sleep" timing.
static volatile uint32_t state_since_us = 0;

// Written by the TinyUSB host callbacks, consumed by the main loop.
static volatile uint8_t pending_attach_addr = 0;
static volatile uint8_t pending_mount_addr = 0;
static volatile uint8_t pending_umount_addr = 0;
static volatile uint8_t last_csw_status = 0;
static volatile uint32_t last_csw_residue = 0;

static uint8_t msc_dev_addr = 0;
static uint32_t msc_block_size = 0;
static uint32_t msc_block_count = 0;

// A drive was already showing its pull-up when the port came up at boot, and
// which line it used (the host decides full-speed vs low-speed from that).
static bool device_at_boot = false;
static bool sense_dp_high = false;
static bool sense_dm_high = false;

static fat32_layout_t fs_layout;
static uint32_t fs_region_sectors = 0; // reserved + FAT + root dir cluster
static uint32_t fs_cursor = 0;         // sectors of the region written
static uint16_t chunk_sectors = 0;     // sectors per WRITE(10)
static uint32_t bytes_written = 0;     // for the throughput report
static uint32_t wipe_started_us = 0;
static uint32_t wipe_timeout_ms = 0; // scaled with the metadata size
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

static void wipe_set_state(wipe_state_t new_state) {
  wipe_state = new_state;
  state_since_us = time_us_32();
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
  wipe_set_state(WIPE_ERROR);
  xfer_state = XFER_IDLE;
  wipe_set_status(STATUS_ERROR);
}

// Back to "nothing to do": used when the drive goes away and by the state
// machine's default branch. power_service() takes it from here.
static void wipe_reset_to_idle(void) {
  wipe_set_state(WIPE_IDLE);
  xfer_state = XFER_IDLE;
  wipe_set_status(STATUS_IDLE);
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
  if (wipe_state == WIPE_PREPARE || wipe_state == WIPE_LARGE ||
      wipe_state == WIPE_WRITE || wipe_state == WIPE_SYNC ||
      wipe_state == WIPE_VERIFY) {
    return;
  }

  msc_dev_addr = dev_addr;
  msc_block_size = tuh_msc_get_block_size(dev_addr, 0);
  msc_block_count = tuh_msc_get_block_count(dev_addr, 0);

  wipe_set_state(WIPE_PREPARE);
  wipe_set_status(STATUS_WIPING);
}

void tuh_msc_umount_cb(uint8_t dev_addr) {
  pending_umount_addr = dev_addr;

  if (dev_addr == msc_dev_addr) {
    msc_dev_addr = 0;
    wipe_reset_to_idle();
  }
}

// Generic callbacks (weak in TinyUSB): they fire for *any* device that finishes
// enumeration, not just mass storage. Recorded and printed from the main loop,
// because printing here would pump tud_task() from inside tuh_task().
void tuh_mount_cb(uint8_t dev_addr) {
  pending_attach_addr = dev_addr;
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
  memset(zero_buf, 0, sizeof(zero_buf));

  // The backstop has to grow with the FAT: 8 MiB of metadata is legitimate on a
  // 128 GiB drive and takes ~40 s on a cheap stick.
  wipe_timeout_ms = WIPE_TIMEOUT_BASE_MS +
                    (fs_region_sectors * 512u) / WIPE_MIN_BYTES_PER_MS;

  const uint64_t capacity_bytes = (uint64_t)msc_block_count * (uint64_t)bs;
  const bool large = capacity_bytes > LARGE_DRIVE_BYTES;

  printf("Sanitize: rebuilding FAT32 '%s' on %lu MiB%s\n",
         FS_LABEL, (unsigned long)(msc_block_count / 2048u),
         large ? " (large drive)" : "");
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

  if (large) {
    // Magenta for a moment: this is one of the big ones. The write itself
    // starts from WIPE_LARGE below, so the timing still covers only the writes.
    wipe_set_status(STATUS_LARGE);
    wipe_set_state(WIPE_LARGE);
    return;
  }

  wipe_set_status(STATUS_WIPING);
  wipe_started_us = time_us_32();
  wipe_set_state(WIPE_WRITE);
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

  wipe_set_state(WIPE_DONE);
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
  if (pending_attach_addr != 0) {
    const uint8_t addr = pending_attach_addr;
    pending_attach_addr = 0;
    uint16_t vid = 0, pid = 0;
    tuh_vid_pid_get(addr, &vid, &pid);
    printf("USB device attached at address %u (VID %04X PID %04X)\n",
           (unsigned)addr, (unsigned)vid, (unsigned)pid);
  }
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

      if (ms_since(wipe_started_us) > wipe_timeout_ms) {
        wipe_fail("overall timeout after %lu ms", (unsigned long)ms_since(wipe_started_us));
        return;
      }

      if (xfer_state == XFER_IDLE) {
        if (fs_cursor >= fs_region_sectors) {
          wipe_set_state(WIPE_MBR); // FS is in place; commit the MBR last
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
        wipe_set_state(WIPE_SYNC);
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
        wipe_set_state(WIPE_VERIFY);
      } else if (xfer_state == XFER_FAIL) {
        // Some cheap sticks do not implement SYNCHRONIZE CACHE. The writes
        // themselves already completed, so treat this as a warning only.
        xfer_state = XFER_IDLE;
        sync_unsupported = true;
        printf("Warning: SYNCHRONIZE CACHE (10) rejected (CSW status %u)\n",
               (unsigned)last_csw_status);
        wipe_set_state(WIPE_VERIFY);
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

    case WIPE_LARGE:
      // Heads-up colour shown; start writing once it has had its moment.
      if (msc_dev_addr == 0 || !tuh_msc_mounted(msc_dev_addr)) {
        wipe_fail("drive disappeared before the wipe started");
        return;
      }
      if (ms_since(state_since_us) >= LARGE_NOTICE_MS) {
        wipe_set_status(STATUS_WIPING);
        wipe_started_us = time_us_32();
        wipe_set_state(WIPE_WRITE);
      }
      return;

    default:
      wipe_reset_to_idle();
      return;
  }
}

//--------------------------------------------------------------------
// Power off: dark and quiet until the reset button
//--------------------------------------------------------------------
//
// There is nothing to detect while asleep, so nothing is left running: the
// USB-A rail and the NeoPixel rail are switched off and the RP2040 drops into
// DORMANT mode (crystal and PLLs off, all clocks stopped, SRAM retained).
//
// Waking up is the reset button. RST resets the chip, the firmware boots from
// the bootrom exactly like it does at power-on, sees whatever is in the USB-A
// port, sanitizes it, and switches off again. No wake-up source to configure,
// no host stack to resume, and nothing that can go stale while the board sits
// in a drawer: every run starts from a cold boot.

// Adapted from pico-extras' pico_sleep (BSD-3-Clause, (c) 2020 Raspberry Pi
// (Trading) Ltd.): a dormant RP2040 must be clocked from the crystal with no
// PLLs running, so everything else is stopped first. Kept local instead of
// pulling in the whole pico-extras dependency for one function.
static void clocks_prepare_for_dormant(void) {
  const uint32_t xosc_hz = XOSC_HZ;

  // CLK_REF = XOSC, CLK_SYS = CLK_REF
  clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, xosc_hz,
                  xosc_hz);
  clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF, 0, xosc_hz,
                  xosc_hz);

  // USB and ADC have to go: they cannot be stopped once the chip is dormant.
  clock_stop(clk_usb);
  clock_stop(clk_adc);

  // RTC = XOSC / 256.
  clock_configure(clk_rtc, 0, CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, xosc_hz,
                  46875);
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, xosc_hz,
                  xosc_hz);

  pll_deinit(pll_sys);
  pll_deinit(pll_usb);
  rosc_disable();
}

// Switches off everything that can be switched off and stays there until the
// reset button is pressed. Nothing is printed on the way out: an open console
// is the one reason not to come here at all.
static void power_off(void) {
  // LEDs first, while there is still a fast clock for the NeoPixel protocol.
  status_led_shutdown();

  // The USB-A rail goes too: the drive is left unpowered, the board's 5 V LED
  // goes out, and the boost converter stops drawing its quiescent current.
  gpio_put(BOARD_USB_5V_EN_PIN, 0);

  // Sign off the USB-C port as well. The PHY keeps its pull-up while the clocks
  // are stopped, so without this the host would keep a dead CDC port in its
  // device list instead of seeing the board go away.
  tud_disconnect();

  // Interrupts off before the clocks go: the PIO-USB frame alarm must not try
  // to drive the bus while clk_sys is being taken down.
  const uint32_t ints = save_and_disable_interrupts();
  (void)ints;
  clocks_prepare_for_dormant();

  // DORMANT is the RP2040's lowest-power state: crystal and PLLs off, all
  // clocks stopped, SRAM retained. Nothing but a reset gets the CPU going
  // again, which is exactly the wake-up we want.
  xosc_dormant();

  // Only reached if the crystal never stopped; the board is then still dark and
  // idle at 12 MHz, and RST still wakes it.
  for (;;) {
    tight_loop_contents();
  }
}

// A drive that is plugged in but never enumerates is not the same as an empty
// port: say so instead of pulsing the idle colour and going dark.
static void presence_service(void) {
  if (!device_at_boot || wipe_state != WIPE_IDLE || status != STATUS_IDLE) {
    return;
  }
  if (ms_since(state_since_us) < NO_ENUM_MS) {
    return;
  }

  printf("Note: a drive is attached but never enumerated - unsupported drive?\n");
  wipe_set_status(STATUS_UNRECOGNISED);
  // Restart the dwell so the blink is visible before the board powers down.
  wipe_set_state(WIPE_IDLE);
}

static void usb_power_sense_init(void) {
#ifdef USB_POWER_SENSE_PIN
  adc_init();
  adc_gpio_init(USB_POWER_SENSE_PIN);
#endif
}

// True when the board is running from USB-C. Without the optional ADC divider
// the best we can do is "a USB host enumerated us" (PC, laptop, powered hub) or
// an open console - a dumb charger is invisible, and the board powers down on
// it just like on battery.
static bool usb_power_present(void) {
#ifdef USB_POWER_SENSE_PIN
  adc_select_input(USB_POWER_SENSE_ADC_INPUT);
  uint32_t sum = 0;
  for (int i = 0; i < 4; i++) {
    sum += adc_read();
  }
  if ((sum / 4u) > USB_POWER_SENSE_THRESHOLD) {
    return true;
  }
#endif
  return tud_mounted() || stdio_usb_connected();
}

// Called from the main loop. Returns when the board runs on USB-C power (it then
// stays awake); otherwise it powers down for good.
static void power_service(void) {
  const bool idle = (wipe_state == WIPE_IDLE) && (msc_dev_addr == 0) &&
                    (pending_mount_addr == 0);
  const bool result = (wipe_state == WIPE_DONE) || (wipe_state == WIPE_ERROR);
  if (!idle && !result) {
    return;
  }

  const uint32_t hold_ms = idle ? IDLE_SLEEP_MS : RESULT_HOLD_MS;
  if (ms_since(state_since_us) < hold_ms) {
    return;
  }

  // On USB-C power the board never powers down: a desk/bench session should not
  // be cut short, and going dark mid-test only means a trip to the reset button.
  if (usb_power_present()) {
    return;
  }

  power_off();
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

// Powers the USB-A port and starts the host. *device_attached reports whether a
// drive was already showing its pull-up when the port came up, which is the only
// moment we can tell "nothing plugged in" from "plugged in but never enumerated"
// (once PIO-USB owns the pins it drives them, so the pad reads are its own).
static bool usb_host_init(bool *device_attached) {
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

  // Bus sense, before PIO-USB takes the pins: plain inputs with the internal
  // pull-downs, so a high line can only come from the drive.
  gpio_init(BOARD_USB_HOST_DP_PIN);
  gpio_set_dir(BOARD_USB_HOST_DP_PIN, GPIO_IN);
  gpio_set_input_enabled(BOARD_USB_HOST_DP_PIN, true);
  gpio_pull_down(BOARD_USB_HOST_DP_PIN);
  gpio_init(BOARD_USB_HOST_DM_PIN);
  gpio_set_dir(BOARD_USB_HOST_DM_PIN, GPIO_IN);
  gpio_set_input_enabled(BOARD_USB_HOST_DM_PIN, true);
  gpio_pull_down(BOARD_USB_HOST_DM_PIN);

  *device_attached = false;
  for (uint32_t waited = 0; waited < BUS_SENSE_MS; waited += 10) {
    sense_dp_high = gpio_get(BOARD_USB_HOST_DP_PIN) != 0;
    sense_dm_high = gpio_get(BOARD_USB_HOST_DM_PIN) != 0;
    if (sense_dp_high || sense_dm_high) {
      *device_attached = true;
      break;
    }
    sleep_ms(10);
  }

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
      wipe_state == WIPE_PREPARE || wipe_state == WIPE_LARGE ||
      wipe_state == WIPE_WRITE || wipe_state == WIPE_SYNC ||
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

  if (c == 's' || c == 'S') {
    // Raw bus state: if D+ is high the drive is powered and pulling up, so a
    // missing mount means enumeration failed; a low D+ means nothing is there
    // (no power, bad contact, or no device).
    printf("Host %s | D+=%u D-=%u | MSC %s addr %u | state %d | boot sense D+=%u D-=%u | USB pwr %u\n",
           tuh_inited() ? "up" : "down",
           (unsigned)gpio_get(BOARD_USB_HOST_DP_PIN),
           (unsigned)gpio_get(BOARD_USB_HOST_DM_PIN),
           (msc_dev_addr != 0 && tuh_msc_mounted(msc_dev_addr)) ? "mounted" : "none",
           (unsigned)msc_dev_addr, (int)wipe_state,
           (unsigned)sense_dp_high, (unsigned)sense_dm_high,
           (unsigned)usb_power_present());
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
      (wipe_state == WIPE_IDLE || wipe_state == WIPE_DONE ||
       wipe_state == WIPE_ERROR)) {
    printf("Re-running wipe on device address %u\n", (unsigned)msc_dev_addr);
    wipe_set_state(WIPE_PREPARE);
  }
}

//--------------------------------------------------------------------
// main
//--------------------------------------------------------------------

int main(void) {
  usb_device_init();
  stdio_init_all();

  status_led_init();
  usb_power_sense_init();
  console_wait(CONSOLE_WAIT_MS);

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
  printf("Size: no limit; drives over %lu MiB show magenta first, then blue\n",
         (unsigned long)(LARGE_DRIVE_BYTES >> 20));
  printf("Ready - insert a USB flash drive...\n");
  printf("Powers down after %lu ms without a drive, or %lu ms after a result.\n",
         (unsigned long)IDLE_SLEEP_MS, (unsigned long)RESULT_HOLD_MS);
  printf("Press RESET (with a drive in the port) to clean one.\n");
  printf("(ENTER = re-run, t = read test, s = bus state, p = power-cycle port, b = BOOTSEL)\n");

  if (!usb_host_init(&device_at_boot)) {
    printf("FATAL: TinyUSB host init failed\n");
    wipe_set_state(WIPE_ERROR);
    wipe_set_status(STATUS_ERROR);
  } else {
    // Start the "nothing is happening" clock now that the host is up, so a
    // drive that is already plugged in is enumerated before we sleep.
    wipe_set_state(WIPE_IDLE);
  }
  if (device_at_boot) {
    printf("Drive on the port at boot: D+=%u D-=%u (pull-up on %s)\n",
           (unsigned)sense_dp_high, (unsigned)sense_dm_high,
           sense_dp_high && sense_dm_high ? "both lines!"
           : sense_dp_high               ? "D+, full-speed"
                                         : "D-, low-speed");
  }

  while (true) {
    tud_task();  // native USB device (CDC console)
    tuh_task();  // USB-A host stack + MSC
    wipe_service();
    console_service();
    status_led_task();
    presence_service(); // "attached but never enumerated" is worth a colour
    power_service();    // switches the board off unless something is going on
  }
}
