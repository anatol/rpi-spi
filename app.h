#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "hardware/spi.h"

#ifndef SP_PIN_MISO
#define SP_PIN_MISO 0
#endif
#ifndef SP_PIN_CS
#define SP_PIN_CS 1
#endif
#ifndef SP_PIN_SCK
#define SP_PIN_SCK 2
#endif
#ifndef SP_PIN_MOSI
#define SP_PIN_MOSI 3
#endif
#ifndef SP_SPI_PORT
#define SP_SPI_PORT spi0
#endif

#ifndef SP_PIN_FLASH_ACTIVE_EN
#define SP_PIN_FLASH_ACTIVE_EN 20
#endif
#ifndef SP_PIN_FLASH_ACTIVE_EN_ACTIVE_HIGH
#define SP_PIN_FLASH_ACTIVE_EN_ACTIVE_HIGH 1
#endif

#ifndef SP_DEFAULT_SPI_HZ
#define SP_DEFAULT_SPI_HZ 12000000u
#endif
#ifndef SP_MAX_READ_CHUNK
#define SP_MAX_READ_CHUNK 16384u
#endif
#ifndef SP_MAX_WRITE_CHUNK
#define SP_MAX_WRITE_CHUNK 16384u
#endif
#ifndef SP_OPBUF_SIZE
#define SP_OPBUF_SIZE 8192u
#endif

#ifndef SP_DIAG_PROBE_RETRIES
#define SP_DIAG_PROBE_RETRIES 6u
#endif
#ifndef SP_DIAG_SAMPLE_MS
#define SP_DIAG_SAMPLE_MS 120u
#endif
#ifndef SP_DIAG_SPEED_LOW_HZ
#define SP_DIAG_SPEED_LOW_HZ 1000000u
#endif
#ifndef SP_DIAG_SPEED_MID_HZ
#define SP_DIAG_SPEED_MID_HZ 4000000u
#endif
#ifndef SP_DIAG_SPEED_HIGH_HZ
#define SP_DIAG_SPEED_HIGH_HZ 12000000u
#endif

#ifndef SP_FW_VERSION
#define SP_FW_VERSION "dev"
#endif

#define SERPROG_IFACE_VERSION 0x0001u
// USB CDC interface index assignments (must match usb_descriptors.c ordering).
#define CDC_SERPROG_ITF 0u
#define CDC_CONSOLE_ITF 1u

typedef enum {
    // Write then read phases are separated.
    SPI_MODE_HALF_DUPLEX = 0,
    // Simultaneous write/read allowed where commands use it.
    SPI_MODE_FULL_DUPLEX = 1,
} spi_mode_t;

typedef enum {
    // Automatically assert/deassert CS around each command transaction.
    CS_MODE_AUTO = 0,
    // Keep CS asserted across commands until mode changes.
    CS_MODE_SELECTED = 1,
    // Keep CS deasserted regardless of command activity.
    CS_MODE_DESELECTED = 2,
} cs_mode_t;

extern bool pin_drivers_enabled;
extern cs_mode_t cs_mode;
extern spi_mode_t spi_mode;
extern uint32_t spi_hz_current;
extern bool serprog_active;

void tinyusb_poll(void);
void cs_assert(void);
void cs_deassert(void);
bool pin_is_valid(int pin);
void set_flash_active_pin(bool active);
int optional_pin_level(int pin);
void set_pin_drivers(bool enabled);
uint32_t spi_set_speed(uint32_t req_hz);
void apply_cs_mode(cs_mode_t mode);

void init_cmdmap(void);
void handle_serprog_command(uint8_t cmd);
void console_print_ready(void);
void console_print_prompt(void);
void console_poll(void);
