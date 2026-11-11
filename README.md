# Feather USB Cleaner

Sanitize a USB flash drive on an **Adafruit Feather RP2040 with USB Type A Host**,
in plain C with the **Pico SDK** and the **TinyUSB host stack** (`tuh_msc`).

Plug a stick in and the firmware **rebuilds an empty FAT32 file system on it**:
the old partition table, boot sector, FAT and root directory are replaced by a
fresh, valid layout, so the stick comes back **empty and immediately usable** on
Windows/Linux/macOS — no PC-side format step.

> **Destructive by design.** Everything on the inserted stick becomes
> unreachable. This is *sanitization to some extent*, not a certified erase:
> data clusters outside the metadata region are not overwritten, but nothing
> references them any more.

---

## What the stick looks like afterwards

| | Before | After |
|---|---|---|
| Partition table | whatever was there | MBR, one partition type `0x0C`, starting at LBA 2048, covering the whole device |
| File system | FAT16/FAT32/exFAT/... | fresh **FAT32**, label `CLEANED`, 1 FAT |
| Contents | user files | empty (one volume-label entry) |
| Mount | — | mounts read/write on Windows, Linux, macOS |

`fsck.fat` reports the generated layout as clean (verified offline, see
*Validating the image* below).

### Cluster size trade-off

The cluster size is chosen so that the **FAT stays small enough to rewrite
quickly**:

| Device size | Cluster | FAT size (1 copy) | Metadata to write |
|---|---|---|---|
| < 2 GiB | 4 KiB | varies | ~0.3–2 MiB |
| 2–8 GiB | 32 KiB | ~0.3–0.6 MiB | ~0.4–0.7 MiB |
| ≥ 8 GiB | **64 KiB** | ~1 MiB | ~1.04 MiB |

FAT32 stores 4 bytes *per cluster*, and every free cluster must be zero, so the
FAT is the dominant write cost. 64 KiB clusters keep a 16–32 GiB stick at about
1 MiB of metadata. Windows, Linux and macOS all mount 64 KiB-cluster FAT32;
old car stereos may not.

---

## Measured results

Hardware: Feather RP2040 USB Host, 125 MHz, USB-A port in Full-Speed host mode,
15 000 MiB USB 2.0 stick, 512-byte logical blocks.

| Operation | Result |
|---|---|
| Sanitize + FAT32 rebuild (≈1.04 MiB written) | **~4.3 s** (~244 KiB/s) — same write volume as the old 1 MiB wipe |
| Raw read throughput of the same build | 653 KiB/s |
| Upstream Pico-PIO-USB, before the patch below | 64 KiB/s ceiling → ~16 s for 1 MiB |

The write rate is the stick's, not the firmware's: the same build reads at
653 KiB/s but the stick accepts only ~4 bulk packets per frame of writes.

### Why not sub-second

* Full-Speed USB allows at most 19 × 64-byte bulk packets per 1 ms frame →
  ~1.216 MB/s ceiling. A valid FAT32 for 16 GiB needs ~1 MiB of FAT bytes, so
  **≥0.9 s even at the theoretical maximum**, ~4 s at this stick's real write
  rate.
* A SCSI `WRITE SAME (10)` fast path was implemented and tested: this stick
  accepts it but never completes the following `SYNCHRONIZE CACHE` (30 s hang),
  so it is not shipped.
* exFAT would cut the metadata to ~0.1 MiB (its allocation bitmap is 32× smaller
  than a FAT), i.e. well under a second, at the price of exFAT-only
  compatibility. Not implemented.

---

## Sequence on mount

1. `tuh_msc_mount_cb()` records capacity and block size and raises a flag. It
   must not block — it runs inside `tuh_task()`, and anything that waits there
   (or prints through USB CDC, which pumps `tud_task()`) deadlocks the host
   stack.
2. The main-loop state machine writes the FAT32 metadata region in 32 KiB
   `WRITE(10)` chunks, starting at LBA 2048.
3. The **MBR is written last**, as the commit point: a power loss before that
   leaves a stick with no partition table rather than a half-built volume.
4. `SYNCHRONIZE CACHE (10)` flushes the drive's cache.
5. Three structures are read back and checked: MBR (signature, type `0x0C`,
   start/size), boot sector (`0x55AA`, `FAT32`, total sectors), FAT head
   (`0x0FFFFFF8/0x0FFFFFFF` + root cluster).

No file system is mounted and no FAT library is linked; the layout is emitted
directly as bytes.

---

## Why this needs a patched Pico-PIO-USB

The RP2040 has **no USB host controller** — TinyUSB's host role runs on the
PIO-based [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB).
Upstream services each endpoint **once per 1 ms frame**, capping bulk throughput
at 64 KiB/s (upstream issue
[#185](https://github.com/sekigon-gonnoc/Pico-PIO-USB/issues/185)).

`third_party/Pico-PIO-USB` carries a small patch in `src/pio_usb_host.c`: bulk
endpoints are serviced repeatedly within a frame while a 700 µs budget lasts
(USB 2.0 allows up to 19 full-speed bulk packets per frame). Control and
interrupt endpoints still get one transaction per frame. See
`third_party/Pico-PIO-USB/PATCHES.md`; `-DPIO_USB_BULK_FRAME_BUDGET_US=0`
restores upstream behaviour.

---

## Status LED

| Colour | Meaning |
|---|---|
| Yellow pulsing | Idle, waiting for a drive |
| Blue blinking | Writing the new file system |
| Green solid | Done: cache flushed, structures verified |
| Red solid | Error: block size, write failure, verification failure, timeout |

NeoPixel data **GP21**, power gate **GP20**; the onboard red LED on **GP13**
follows along.

---

## Pin map (Feather RP2040 USB Host)

| Function | GPIO |
|---|---|
| USB host D+ | 16 |
| USB host D− | 17 |
| USB host 5 V enable | 18 |
| NeoPixel data | 21 |
| NeoPixel power gate | 20 |
| Onboard LED | 13 |

GP18 is driven high at boot to power the USB-A connector.

---

## Build

Requirements: Pico SDK 2.x (tested 2.3.1), `arm-none-eabi-gcc` (tested 14.2),
CMake ≥ 3.13, Pico-PIO-USB (bundled/patched).

```bash
export PICO_SDK_PATH=$HOME/pico-sdk
export PATH=$HOME/toolchain/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi/bin:$PATH

cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Output: `build/feather_usb_cleaner.uf2` (plus `.elf`, `.bin`, `.hex`, `.map`).

---

## Flash

1. Hold **BOOT**, tap **RST**, release → an `RPI-RP2` drive appears.
2. `cp build/feather_usb_cleaner.uf2 /media/$USER/RPI-RP2/ && sync`

Afterwards the button is not needed: send **`b`** on the serial console to
reboot into the bootloader.

---

## Using it

Console on the native USB-C CDC port (115200 8N1, baud is ignored):

```
=====================================
 Feather USB Cleaner (sanitize + FAT32 rebuild)
=====================================
Output: fresh empty FAT32 labelled 'CLEANED', 1 FAT, big clusters
Ready - insert a USB flash drive...
(ENTER = re-run sanitize, t = read test, p = power-cycle port, b = BOOTSEL)

MSC device mounted at address 1: 30720000 blocks x 512 B (15000 MiB)
Sanitize: rebuilding FAT32 'CLEANED' on 15000 MiB
  partition LBA 2048..30719999, 64 KiB clusters, 1 FAT of 937 KiB
  writing 1065 KiB of metadata (2131 sectors)
    3%  64/2131 sectors  32 KiB  t=131 ms
 ...
  100%  2131/2131 sectors  1065 KiB  t=...
Partition table written (commit)
Cache flushed (SYNCHRONIZE CACHE 10)
Verified: MBR / partition table
Verified: FAT32 boot sector
Verified: FAT
Sanitized in ... ms (... KiB/s). Stick is empty and mountable.
```

| Key | Action |
|---|---|
| ENTER | Re-run sanitize on the inserted stick |
| `t` | Read 1 MiB and report raw read throughput |
| `p` | Power-cycle the USB-A port (recover a wedged stick) |
| `b` | Reboot into BOOTSEL |

---

## Validating the image without hardware

`tools/fat32_image.c` compiles the same builder on a PC and emits a sparse disk
image plus the partition slice:

```bash
gcc -O2 -Wall -Wextra -std=c11 -I. -o fat32_image tools/fat32_image.c fat32_builder.c
./fat32_image 30720000 /tmp/stick        # 15 GB stick
fsck.fat -n -v /tmp/stick_part.img
```

Known-good results: 4 GiB, 8 GiB, 30 GB and 64 GB geometries are `fsck.fat`
clean (1 file = the volume label, 1 cluster in use = root directory).

---

## Tuning

At the top of `main.c`:

| Macro | Default | Meaning |
|---|---|---|
| `FS_LABEL` | `"CLEANED"` | Volume label of the new file system |
| `CHUNK_BYTES` | 32 KiB | Bytes per `WRITE(10)`; must stay ≤ 65535 |
| `MAX_CHUNK_RETRIES` | 3 | Retries per chunk before giving up |
| `XFER_TIMEOUT_MS` | 1500 | Stalled bulk transfer watchdog |
| `FLUSH_TIMEOUT_MS` | 30000 | `SYNCHRONIZE CACHE` watchdog |
| `WIPE_TIMEOUT_MS` | 15000 | Whole-operation watchdog |

Cluster size is picked in `fat32_builder.c` (`fat32_plan`) from the device size.

---

## Limitations

* 512-byte logical sectors only. A 4K-native stick is refused with a red LED
  rather than producing a bogus volume.
* Cluster count must be ≥ 65525 (true for anything above ~4 GiB at the chosen
  cluster sizes); smaller devices are refused.
* Old file *data* is still on the flash. The new FAT32 makes it unreachable,
  but forensic recovery is possible — this is not a DoD/NIST purge.
* One drive at a time; a second MSC device behind a hub is ignored while a
  sanitize run is in progress.

---

## Layout

```
CMakeLists.txt                 pico_stdlib + tinyusb_host + PIO-USB
main.c                         USB host, sanitize state machine, NeoPixel
fat32_builder.c / .h           dependency-free FAT32 layout generator
tusb_config.h                  dual role: CDC device (console) + MSC host
ws2812.pio                     NeoPixel PIO program
tools/fat32_image.c            host-side image generator (fsck.fat validation)
third_party/Pico-PIO-USB/      vendored, patched (bulk batching)
```

## License

MIT for the project code. `third_party/Pico-PIO-USB` keeps its own MIT license
and copyright; local modifications are documented in its `PATCHES.md`.
