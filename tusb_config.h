#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#include "pico.h"

#if defined(PICO_RP2350)
#define CFG_TUSB_MCU OPT_MCU_RP2350
#else
#define CFG_TUSB_MCU OPT_MCU_RP2040
#endif

#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE
#define CFG_TUSB_OS OPT_OS_PICO
/*
 * Disabled on purpose for throughput-focused runs as requested.
 * Tradeoff: some host/controller combos may enumerate less reliably.
 */
#define TUD_OPT_RP2040_USB_DEVICE_ENUMERATION_FIX 0

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC 2
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0

/*
 * Large software buffers reduce host/device turnarounds for serprog bulk
 * traffic and improve sustained throughput for flashrom reads/writes.
 */
#define CFG_TUD_CDC_RX_BUFSIZE 1024
#define CFG_TUD_CDC_TX_BUFSIZE 1024
#define CFG_TUD_CDC_EP_BUFSIZE 64

#endif
