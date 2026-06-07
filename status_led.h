#pragma once

#include <stdbool.h>

typedef enum {
    STATUS_LED_EVENT_SPI_TRAFFIC,
    STATUS_LED_EVENT_UART_TRAFFIC,
    STATUS_LED_EVENT_SPI_ENABLED,
    STATUS_LED_EVENT_SPI_ISOLATED,
    STATUS_LED_EVENT_ERROR,
} status_led_event_t;

void status_led_init(void);
void status_led_task(void);
void status_led_notify(status_led_event_t event);
