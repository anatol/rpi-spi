#include <stdbool.h>
#include <stdint.h>

#include "app.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "status_led.h"
#include "status_led.pio.h"
#include "tusb.h"

#if SP_STATUS_LED_ENABLED

typedef struct {
    uint32_t color;
    uint8_t blinks;
    uint8_t priority;
    uint16_t on_ms;
    uint16_t off_ms;
} led_pattern_t;

enum {
    LED_PRIORITY_ACTIVITY = 1,
    LED_PRIORITY_STATE = 2,
    LED_PRIORITY_CONNECTION = 3,
    LED_PRIORITY_ERROR = 4,
    LED_PRIORITY_BOOT = 5,
};

#define RGB(r, g, b) (((uint32_t)(r) << 16u) | ((uint32_t)(g) << 8u) | (uint32_t)(b))

static PIO led_pio;
static uint led_sm;
static uint led_offset;
static bool led_ready;
static bool pattern_on;
static bool last_serprog_connected;
static uint8_t pattern_priority;
static uint8_t transitions_left;
static uint64_t next_transition_us;
static uint64_t spi_activity_cooldown_us;
static uint64_t uart_activity_cooldown_us;
static uint32_t pattern_color;
static uint16_t pattern_on_ms;
static uint16_t pattern_off_ms;

static void led_write(uint32_t rgb) {
    if (led_ready) {
        pio_sm_put_blocking(led_pio, led_sm, rgb << 8u);
    }
}

static uint32_t idle_color(void) {
    if (tud_cdc_n_connected(CDC_SERPROG_ITF)) {
        return pin_drivers_enabled ? RGB(0, 10, 0) : RGB(10, 3, 8);
    }
    if (tud_cdc_n_connected(CDC_UART_ITF)) {
        return RGB(0, 5, 8);
    }
    if (tud_mounted()) {
        return RGB(0, 0, 8);
    }
    return RGB(3, 0, 5);
}

static void start_pattern(led_pattern_t pattern) {
    if (!led_ready || (transitions_left != 0 && pattern.priority < pattern_priority)) {
        return;
    }

    pattern_color = pattern.color;
    pattern_priority = pattern.priority;
    pattern_on_ms = pattern.on_ms;
    pattern_off_ms = pattern.off_ms;
    transitions_left = (uint8_t)(pattern.blinks * 2u);
    pattern_on = true;
    led_write(pattern_color);
    next_transition_us = time_us_64() + (uint64_t)pattern_on_ms * 1000u;
}

void status_led_init(void) {
    led_ready = pio_claim_free_sm_and_add_program_for_gpio_range(
        &status_led_ws2812_program, &led_pio, &led_sm, &led_offset, SP_STATUS_LED_PIN, 1,
        true);
    if (!led_ready) {
        return;
    }

    status_led_ws2812_program_init(led_pio, led_sm, led_offset, SP_STATUS_LED_PIN);
    sleep_us(80);
    start_pattern((led_pattern_t){
        .color = RGB(18, 18, 18),
        .blinks = 3,
        .priority = LED_PRIORITY_BOOT,
        .on_ms = 100,
        .off_ms = 100,
    });
}

void status_led_notify(status_led_event_t event) {
    uint64_t now = time_us_64();
    led_pattern_t pattern = {
        .on_ms = 70,
        .off_ms = 70,
        .priority = LED_PRIORITY_ACTIVITY,
    };

    switch (event) {
    case STATUS_LED_EVENT_SPI_TRAFFIC:
        if (now < spi_activity_cooldown_us) {
            return;
        }
        spi_activity_cooldown_us = now + 50000u;
        pattern.color = RGB(0, 28, 0);
        pattern.blinks = 1;
        break;
    case STATUS_LED_EVENT_UART_TRAFFIC:
        if (now < uart_activity_cooldown_us) {
            return;
        }
        uart_activity_cooldown_us = now + 150000u;
        pattern.color = RGB(0, 18, 24);
        pattern.blinks = 2;
        break;
    case STATUS_LED_EVENT_SPI_ENABLED:
        pattern.color = RGB(24, 14, 0);
        pattern.blinks = 2;
        pattern.priority = LED_PRIORITY_STATE;
        pattern.on_ms = 100;
        pattern.off_ms = 100;
        break;
    case STATUS_LED_EVENT_SPI_ISOLATED:
        pattern.color = RGB(20, 0, 18);
        pattern.blinks = 2;
        pattern.priority = LED_PRIORITY_STATE;
        pattern.on_ms = 100;
        pattern.off_ms = 100;
        break;
    case STATUS_LED_EVENT_ERROR:
        pattern.color = RGB(30, 0, 0);
        pattern.blinks = 3;
        pattern.priority = LED_PRIORITY_ERROR;
        pattern.on_ms = 120;
        pattern.off_ms = 100;
        break;
    }

    start_pattern(pattern);
}

void status_led_task(void) {
    if (!led_ready) {
        return;
    }

    bool serprog_connected = tud_cdc_n_connected(CDC_SERPROG_ITF);
    if (serprog_connected && !last_serprog_connected) {
        start_pattern((led_pattern_t){
            .color = RGB(0, 8, 28),
            .blinks = 2,
            .priority = LED_PRIORITY_CONNECTION,
            .on_ms = 120,
            .off_ms = 100,
        });
    }
    last_serprog_connected = serprog_connected;

    if (transitions_left == 0 || time_us_64() < next_transition_us) {
        return;
    }

    transitions_left--;
    if (transitions_left == 0) {
        pattern_priority = 0;
        pattern_on = false;
        led_write(idle_color());
        return;
    }

    pattern_on = !pattern_on;
    led_write(pattern_on ? pattern_color : 0);
    next_transition_us =
        time_us_64() + (uint64_t)(pattern_on ? pattern_on_ms : pattern_off_ms) * 1000u;
}

#else

void status_led_init(void) {}
void status_led_task(void) {}
void status_led_notify(status_led_event_t event) { (void)event; }

#endif
