#include "bsp/board_api.h"
#include "pico/unique_id.h"
#include "tusb.h"
#include <string.h>

#define USB_VID 0x2E8A
#define USB_PID 0x000A
#define USB_BCD 0x0200

// CDC functions exposed:
// 1) serprog binary transport
// 2) raw UART bridge console
// 3) (optional) human-readable diagnostic console
#define ITF_NUM_CDC_SERPROG 0
#define ITF_NUM_CDC_SERPROG_DATA 1
#define ITF_NUM_CDC_UART 2
#define ITF_NUM_CDC_UART_DATA 3
#if SP_ENABLE_DIAG_CONSOLE
#define ITF_NUM_CDC_CONSOLE 4
#define ITF_NUM_CDC_CONSOLE_DATA 5
#define ITF_NUM_TOTAL 6
#else
#define ITF_NUM_TOTAL 4
#endif

#define EPNUM_CDC_SERPROG_NOTIF 0x81
#define EPNUM_CDC_SERPROG_OUT 0x02
#define EPNUM_CDC_SERPROG_IN 0x82
#define EPNUM_CDC_UART_NOTIF 0x83
#define EPNUM_CDC_UART_OUT 0x04
#define EPNUM_CDC_UART_IN 0x84
#if SP_ENABLE_DIAG_CONSOLE
#define EPNUM_CDC_CONSOLE_NOTIF 0x85
#define EPNUM_CDC_CONSOLE_OUT 0x06
#define EPNUM_CDC_CONSOLE_IN 0x86
#endif

#if SP_ENABLE_DIAG_CONSOLE
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + (3 * TUD_CDC_DESC_LEN))
#else
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + (2 * TUD_CDC_DESC_LEN))
#endif

static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

static const uint8_t desc_configuration[] = {
    // bmAttributes=0x00 keeps descriptor minimal; bus-powered behavior is
    // implied for this device class in TinyUSB examples.
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_SERPROG, 4, EPNUM_CDC_SERPROG_NOTIF, 8,
                       EPNUM_CDC_SERPROG_OUT, EPNUM_CDC_SERPROG_IN,
                       CFG_TUD_CDC_EP_BUFSIZE),
#if SP_ENABLE_DIAG_CONSOLE
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_UART, 6, EPNUM_CDC_UART_NOTIF, 8,
                       EPNUM_CDC_UART_OUT, EPNUM_CDC_UART_IN,
                       CFG_TUD_CDC_EP_BUFSIZE),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_CONSOLE, 5, EPNUM_CDC_CONSOLE_NOTIF, 8,
                       EPNUM_CDC_CONSOLE_OUT, EPNUM_CDC_CONSOLE_IN,
                       CFG_TUD_CDC_EP_BUFSIZE),
#else
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_UART, 5, EPNUM_CDC_UART_NOTIF, 8,
                       EPNUM_CDC_UART_OUT, EPNUM_CDC_UART_IN,
                       CFG_TUD_CDC_EP_BUFSIZE),
#endif
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

static const char *string_desc_arr[] = {
    (const char[]){0x09, 0x04},
    "rpi-spi",
    "Anatol Pomozov SPI programmer",
    NULL, // iSerialNumber is generated dynamically from unique board ID.
    "serprog",
#if SP_ENABLE_DIAG_CONSOLE
    "diag-console",
#endif
    "uart-console",
};

static uint16_t _desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;

    if (index == 0) {
        _desc_str[1] = 0x0409;
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
            return NULL;
        }
        if (index == 3) {
            // Convert binary board ID into uppercase hex USB serial string.
            static char serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
            pico_unique_board_id_t id;
            pico_get_unique_board_id(&id);
            for (size_t i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
                static const char hex[] = "0123456789ABCDEF";
                serial[2 * i] = hex[id.id[i] >> 4];
                serial[2 * i + 1] = hex[id.id[i] & 0x0F];
            }
            serial[sizeof(serial) - 1] = '\0';
            string_desc_arr[3] = serial;
        }

        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) {
            chr_count = 31;
        }

        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
