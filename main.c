#include <stdbool.h>
#include <stdint.h>

#include "app.h"
#include "bsp/board_api.h"
#include "hardware/gpio.h"
#if SP_ENABLE_UART_CONSOLE
#include "hardware/regs/uart.h"
#include "hardware/uart.h"
#endif
#include "pico/stdlib.h"
#include "status_led.h"
#include "tusb.h"

bool pin_drivers_enabled = true;
cs_mode_t cs_mode = CS_MODE_AUTO;
spi_mode_t spi_mode = SPI_MODE_HALF_DUPLEX;
uint32_t spi_hz_current = SP_DEFAULT_SPI_HZ;
// True while a serprog command is actively executing.
// Diagnostic console checks this to avoid bus ownership races.
bool serprog_active = false;
#if SP_ENABLE_UART_CONSOLE
static uint32_t uart_baud_current = SP_DEFAULT_UART_BAUD;
static uint64_t uart_last_flush_us;
static bool uart_flush_pending;

#define UART_DR_ERROR_BITS \
    (UART_UARTDR_OE_BITS | UART_UARTDR_BE_BITS | UART_UARTDR_PE_BITS | UART_UARTDR_FE_BITS)
#define UART_USB_FLUSH_INTERVAL_US 4000u
#endif

void tinyusb_poll(void) {
    tud_task();
    status_led_task();
}

void cs_assert(void) { gpio_put(SP_PIN_CS, 0); }
void cs_deassert(void) { gpio_put(SP_PIN_CS, 1); }

bool pin_is_valid(int pin) { return pin >= 0; }

static void set_optional_pin_level(int pin, bool active, bool active_high) {
    if (!pin_is_valid(pin)) {
        return;
    }
    gpio_put((uint)pin, (active == active_high) ? 1 : 0);
}

int optional_pin_level(int pin) {
    if (!pin_is_valid(pin)) {
        return -1;
    }
    return gpio_get((uint)pin) ? 1 : 0;
}

void set_flash_active_pin(bool active) {
    set_optional_pin_level(SP_PIN_FLASH_ACTIVE_EN, active,
                           SP_PIN_FLASH_ACTIVE_EN_ACTIVE_HIGH != 0);
}

void set_pin_drivers(bool enabled) {
    pin_drivers_enabled = enabled;

    if (!enabled) {
        // Put bus pins in a high-impedance/safe state so an external host
        // can drive the target without fighting this MCU.
        cs_deassert();
        gpio_set_function(SP_PIN_MISO, GPIO_FUNC_NULL);
        gpio_set_function(SP_PIN_MOSI, GPIO_FUNC_NULL);
        gpio_set_function(SP_PIN_SCK, GPIO_FUNC_NULL);
        gpio_set_dir(SP_PIN_CS, GPIO_IN);
    } else {
        // Restore SPI pinmux and actively drive CS again.
        gpio_set_function(SP_PIN_MISO, GPIO_FUNC_SPI);
        gpio_set_function(SP_PIN_MOSI, GPIO_FUNC_SPI);
        gpio_set_function(SP_PIN_SCK, GPIO_FUNC_SPI);
        gpio_set_dir(SP_PIN_CS, GPIO_OUT);
        cs_deassert();
    }
}

uint32_t spi_set_speed(uint32_t req_hz) {
    if (req_hz == 0) {
        return 0;
    }
    uint32_t actual = spi_set_baudrate(SP_SPI_PORT, req_hz);
    spi_hz_current = actual;
    return actual;
}

#if SP_ENABLE_UART_CONSOLE
void uart_bridge_set_baudrate(uint32_t baud) {
    if (baud == 0) {
        return;
    }
    uart_baud_current = uart_init(SP_UART_PORT, baud);
}

uint32_t uart_bridge_get_baudrate(void) { return uart_baud_current; }

void uart_bridge_init(void) {
    if (!pin_is_valid(SP_PIN_UART_TX) || !pin_is_valid(SP_PIN_UART_RX) ||
        SP_PIN_UART_TX == SP_PIN_UART_RX) {
        return;
    }
    gpio_set_function(SP_PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(SP_PIN_UART_RX, GPIO_FUNC_UART);
    // UART idles high. Keep RX deterministic while the target is reset,
    // disconnected, or otherwise not actively driving its TX pin.
    gpio_pull_up(SP_PIN_UART_RX);
    uart_bridge_set_baudrate(SP_DEFAULT_UART_BAUD);
    uart_set_hw_flow(SP_UART_PORT, false, false);
    uart_set_format(SP_UART_PORT, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(SP_UART_PORT, true);
}

void uart_bridge_poll(void) {
    bool uart_packet_full = false;

    if (!pin_is_valid(SP_PIN_UART_TX) || !pin_is_valid(SP_PIN_UART_RX) ||
        SP_PIN_UART_TX == SP_PIN_UART_RX) {
        return;
    }

    // Keep flashrom's latency-sensitive command stream ahead of console traffic.
    if (tud_cdc_n_available(CDC_SERPROG_ITF) > 0) {
        return;
    }

    if (tud_cdc_n_available(CDC_UART_ITF) > 0) {
        uint8_t buf[64];
        uint32_t n = tud_cdc_n_read(CDC_UART_ITF, buf, sizeof(buf));
        if (n > 0) {
            uart_write_blocking(SP_UART_PORT, buf, n);
            status_led_notify(STATUS_LED_EVENT_UART_TRAFFIC);
        }
    }

    if (tud_mounted()) {
        uint8_t out[64];
        uint32_t n = 0;
        uint32_t max_n = (uint32_t)tud_cdc_n_write_available(CDC_UART_ITF);
        if (max_n > sizeof(out)) {
            max_n = sizeof(out);
        }
        while (n < max_n && uart_is_readable(SP_UART_PORT)) {
            uint32_t dr = uart_get_hw(SP_UART_PORT)->dr;
            if ((dr & UART_DR_ERROR_BITS) == 0) {
                out[n++] = (uint8_t)dr;
            }
        }
        if (n > 0) {
            tud_cdc_n_write(CDC_UART_ITF, out, n);
            uart_flush_pending = true;
            uart_packet_full = (n == sizeof(out));
            status_led_notify(STATUS_LED_EVENT_UART_TRAFFIC);
        }
    }

    uint64_t now = time_us_64();
    if (uart_flush_pending &&
        (uart_packet_full || now - uart_last_flush_us >= UART_USB_FLUSH_INTERVAL_US)) {
        tud_cdc_n_write_flush(CDC_UART_ITF);
        uart_last_flush_us = now;
        uart_flush_pending = false;
    }
}
#endif

void apply_cs_mode(cs_mode_t mode) {
    cs_mode = mode;
    if (!pin_drivers_enabled) {
        // Defer physical CS manipulation until drivers are re-enabled.
        return;
    }
    if (mode == CS_MODE_SELECTED) {
        cs_assert();
    } else {
        cs_deassert();
    }
}

static void usb_wait_for_host(void) {
    while (!tud_mounted()) {
        tinyusb_poll();
    }
}

static void init_gpio_and_spi(void) {
    // Default SPI framing expected by most SPI NOR flash parts.
    spi_init(SP_SPI_PORT, SP_DEFAULT_SPI_HZ);
    spi_set_format(SP_SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_init(SP_PIN_CS);
    gpio_set_dir(SP_PIN_CS, GPIO_OUT);
    cs_deassert();

    if (pin_is_valid(SP_PIN_FLASH_ACTIVE_EN)) {
        gpio_init((uint)SP_PIN_FLASH_ACTIVE_EN);
        gpio_set_dir((uint)SP_PIN_FLASH_ACTIVE_EN, GPIO_OUT);
    }

    set_flash_active_pin(false);
    set_pin_drivers(true);
    spi_set_speed(SP_DEFAULT_SPI_HZ);
    uart_bridge_init();
}

int main(void) {
    board_init();
    status_led_init();
    tusb_init();

    init_cmdmap();
    init_gpio_and_spi();

    usb_wait_for_host();
    console_print_ready();
    console_print_prompt();

    while (true) {
        tinyusb_poll();

        // Serprog endpoint takes raw binary commands (flashrom protocol).
        if (tud_cdc_n_available(CDC_SERPROG_ITF)) {
            uint8_t cmd;
            if (tud_cdc_n_read(CDC_SERPROG_ITF, &cmd, 1) == 1) {
                handle_serprog_command(cmd);
                continue;
            }
        }

        // Console endpoint accepts line-oriented human commands.
        console_poll();
        uart_bridge_poll();
    }
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *p_line_coding) {
#if SP_ENABLE_UART_CONSOLE
    if (itf == CDC_UART_ITF) {
        uart_bridge_set_baudrate(p_line_coding->bit_rate);
    }
#else
    (void)itf;
    (void)p_line_coding;
#endif
}
