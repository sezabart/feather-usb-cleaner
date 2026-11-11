/*
 * TinyUSB configuration for the Feather RP2040 USB Host "fast metadata wiper".
 *
 * Two controllers are used at the same time:
 *   - rhport 0: native USB-C connector, USB device role, CDC-ACM for stdio/printf
 *   - rhport 1: USB-A connector, USB host role, driven by Pico-PIO-USB
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// Common
//--------------------------------------------------------------------

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined (the Pico SDK sets it for you)
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

#ifndef CFG_TUH_MEM_SECTION
#define CFG_TUH_MEM_SECTION
#endif

#ifndef CFG_TUH_MEM_ALIGN
#define CFG_TUH_MEM_ALIGN __attribute__((aligned(4)))
#endif

//--------------------------------------------------------------------
// Device (native USB-C port) - CDC-ACM for stdio
//--------------------------------------------------------------------

#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#define CFG_TUD_ENABLED 1
#define CFG_TUD_CDC 1
#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256
#define CFG_TUD_CDC_EP_BUFSIZE 64
#define CFG_TUD_VENDOR 0

//--------------------------------------------------------------------
// Host (USB-A port) - PIO-USB + MSC
//--------------------------------------------------------------------

#define CFG_TUH_ENABLED 1

// RP2040 has no USB host controller: use the PIO based one.
#define CFG_TUH_RPI_PIO_USB 1

// Host controller lives on root hub port 1 (port 0 is the native device port).
#define BOARD_TUH_RHPORT 1
#define BOARD_TUH_MAX_SPEED OPT_MODE_FULL_SPEED
#define CFG_TUH_MAX_SPEED BOARD_TUH_MAX_SPEED

#define CFG_TUH_ENUMERATION_BUFSIZE 512

// A hub is allowed so a stick behind an external hub still enumerates.
#define CFG_TUH_HUB 1
#define CFG_TUH_MSC 1
#define CFG_TUH_CDC 0
#define CFG_TUH_HID 0
#define CFG_TUH_VENDOR 0

#define CFG_TUH_DEVICE_MAX (3 * CFG_TUH_HUB + 1)

// LUN 0 is used; card readers with several LUNs still work (only LUN 0 wiped).
#define CFG_TUH_MSC_MAXLUN 2

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
