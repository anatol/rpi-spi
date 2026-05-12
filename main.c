#include <stdbool.h>
#include <stdint.h>

#include "app.h"
#include "bsp/board_api.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "tusb.h"

bool pin_drivers_enabled = true;
cs_mode_t cs_mode = CS_MODE_AUTO;
spi_mode_t spi_mode = SPI_MODE_HALF_DUPLEX;
uint32_t spi_hz_current = SP_DEFAULT_SPI_HZ;
bool serprog_active = false;

void tinyusb_poll(void) { tud_task(); }

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
        cs_deassert();
        gpio_set_function(SP_PIN_MISO, GPIO_FUNC_NULL);
        gpio_set_function(SP_PIN_MOSI, GPIO_FUNC_NULL);
        gpio_set_function(SP_PIN_SCK, GPIO_FUNC_NULL);
        gpio_set_dir(SP_PIN_CS, GPIO_IN);
    } else {
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

void apply_cs_mode(cs_mode_t mode) {
    cs_mode = mode;
    if (!pin_drivers_enabled) {
        return;
    }
    if (mode == CS_MODE_SELECTED) {
        cs_assert();
    } else {
        cs_deassert();
    }
}

static void usb_wait_for_host(void) {
    while (true) {
        tinyusb_poll();
        if (tud_cdc_n_connected(CDC_SERPROG_ITF) || tud_cdc_n_connected(CDC_CONSOLE_ITF)) {
            return;
        }
    }
}

static void init_gpio_and_spi(void) {
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
}

int main(void) {
    board_init();
    tusb_init();

    init_cmdmap();
    init_gpio_and_spi();

    usb_wait_for_host();
    console_print_ready();
    console_print_prompt();

    while (true) {
        tinyusb_poll();

        if (tud_cdc_n_available(CDC_SERPROG_ITF)) {
            uint8_t cmd;
            if (tud_cdc_n_read(CDC_SERPROG_ITF, &cmd, 1) == 1) {
                handle_serprog_command(cmd);
            }
        }

        console_poll();
    }
}
