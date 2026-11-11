# Local patches to Pico-PIO-USB

Base: https://github.com/sekigon-gonnoc/Pico-PIO-USB @ `b186e36`
(0.7.3, 2026-10-23).

## 1. Bulk endpoints can be serviced more than once per frame

`src/pio_usb_host.c`, `pio_usb_host_frame()`.

Upstream walks the endpoint pool exactly once per 1 ms SOF tick and performs one
transaction per endpoint, i.e. at most one 64-byte bulk packet per frame. That
caps USB MSC throughput at 64 KB/s regardless of the device (upstream issue
#185), which makes a 1 MiB metadata wipe take ~16 s.

The patch wraps the endpoint walk in a loop that keeps servicing **bulk**
endpoints while

* the previous transaction actually advanced the transfer
  (`ep->actual_len` changed – so NAKs or errors stop the loop), and
* less than `PIO_USB_BULK_FRAME_BUDGET_US` (default 700 µs) of the 1 ms frame
  has been used, and
* fewer than 19 passes were done (USB 2.0 full-speed bulk limit per frame).

Control (EP0) and interrupt endpoints are skipped on every pass after the
first, so their per-frame scheduling and interval counters are unchanged.
Set `PIO_USB_BULK_FRAME_BUDGET_US=0` at build time to restore upstream
behaviour.

### Measured effect

Target board, 15 000 MiB USB 2.0 stick, 512-byte blocks, 1 MiB wipe range:

| Host stack                        | 1 MiB WRITE(10) | 1 MiB read      |
|-----------------------------------|-----------------|-----------------|
| upstream (1 packet per frame)     | ~16.3 s (63 KiB/s)  | ~1.6 s      |
| this patch, 700 µs frame budget   | **4.20 s (244 KiB/s)** | **1.57 s (653 KiB/s)** |
| this patch, 1500 µs frame budget  | 3.59 s (285 KiB/s) | –           |

Reads reach ~10 bulk packets per frame, so the host loop is no longer the
bottleneck; writes are limited by the stick (~4 packets per frame before it
NAKs). Raising the budget beyond 700 µs buys little because the device, not the
host, becomes the constraint.

Other findings while testing this patch: moving `configure_root_port()` out of
the per-pass loop (it clears the TX FIFOs) made no measurable difference, and a
SCSI `WRITE SAME (10)` fast path was rejected by the test drive (it accepted the
command but never completed `SYNCHRONIZE CACHE (10)`), so it is not shipped.
