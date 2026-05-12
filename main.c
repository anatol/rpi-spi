#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "serprog.h"
#include "tusb.h"

/*
 * Default wiring aims to match common Pico hardware SPI0 breakout pins.
 * You can change these at compile time with -D flags.
 */
#ifndef SP_PIN_MISO
#define SP_PIN_MISO 16
#endif
#ifndef SP_PIN_CS
#define SP_PIN_CS 17
#endif
#ifndef SP_PIN_SCK
#define SP_PIN_SCK 18
#endif
#ifndef SP_PIN_MOSI
#define SP_PIN_MOSI 19
#endif
#ifndef SP_SPI_PORT
#define SP_SPI_PORT spi0
#endif

/*
 * Optional GPIO that controls an external analog switch/transceiver enable.
 * Set to -1 to disable this feature.
 */
#ifndef SP_PIN_ISOLATE_EN
#define SP_PIN_ISOLATE_EN 20
#endif
#ifndef SP_PIN_ISOLATE_EN_ACTIVE_HIGH
#define SP_PIN_ISOLATE_EN_ACTIVE_HIGH 1
#endif

/* Optional target power control pin (e.g. controlling load switch EN). */
#ifndef SP_PIN_TARGET_PWR
#define SP_PIN_TARGET_PWR 21
#endif
#ifndef SP_PIN_TARGET_PWR_ACTIVE_HIGH
#define SP_PIN_TARGET_PWR_ACTIVE_HIGH 1
#endif

#ifndef SP_DEFAULT_SPI_HZ
#define SP_DEFAULT_SPI_HZ 12000000u
#endif
#ifndef SP_MAX_READ_CHUNK
#define SP_MAX_READ_CHUNK 4096u
#endif
#ifndef SP_MAX_WRITE_CHUNK
#define SP_MAX_WRITE_CHUNK 4096u
#endif
#ifndef SP_OPBUF_SIZE
#define SP_OPBUF_SIZE 8192u
#endif

#define SERPROG_IFACE_VERSION 0x0001u
#define SUPPORTED_BUSTYPE S_BUS_SPI

typedef enum {
    SPI_MODE_HALF_DUPLEX = 0,
    SPI_MODE_FULL_DUPLEX = 1,
} spi_mode_t;

typedef enum {
    CS_MODE_AUTO = 0,
    CS_MODE_SELECTED = 1,
    CS_MODE_DESELECTED = 2,
} cs_mode_t;

static uint8_t cmdmap[32];
static uint8_t opbuf[SP_OPBUF_SIZE];
static uint32_t opbuf_len;

static uint8_t io_buf[SP_MAX_WRITE_CHUNK > SP_MAX_READ_CHUNK ? SP_MAX_WRITE_CHUNK
                                                           : SP_MAX_READ_CHUNK];

static bool pin_drivers_enabled = true;
static cs_mode_t cs_mode = CS_MODE_AUTO;
static spi_mode_t spi_mode = SPI_MODE_HALF_DUPLEX;
static uint32_t spi_hz_current = SP_DEFAULT_SPI_HZ;

static inline void tinyusb_poll(void) { tud_task(); }

static void cdc_write_all(const void *data, uint32_t len) {
    const uint8_t *p = (const uint8_t *)data;
    while (len > 0) {
        tinyusb_poll();
        uint32_t space = tud_cdc_write_available();
        if (space == 0) {
            tud_cdc_write_flush();
            continue;
        }
        uint32_t chunk = len < space ? len : space;
        uint32_t wr = tud_cdc_write(p, chunk);
        tud_cdc_write_flush();
        p += wr;
        len -= wr;
    }
}

static void cdc_write_u8(uint8_t v) { cdc_write_all(&v, 1); }

static bool cdc_read_exact(void *dst, uint32_t len) {
    uint8_t *p = (uint8_t *)dst;
    while (len > 0) {
        tinyusb_poll();
        uint32_t avail = tud_cdc_available();
        if (avail == 0) {
            continue;
        }
        uint32_t chunk = len < avail ? len : avail;
        uint32_t rd = tud_cdc_read(p, chunk);
        p += rd;
        len -= rd;
    }
    return true;
}

static uint32_t read_le24(void) {
    uint8_t b[3];
    cdc_read_exact(b, sizeof(b));
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8u) | ((uint32_t)b[2] << 16u);
}

static uint32_t read_le32(void) {
    uint8_t b[4];
    cdc_read_exact(b, sizeof(b));
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8u) | ((uint32_t)b[2] << 16u) |
           ((uint32_t)b[3] << 24u);
}

static void write_le16(uint16_t v) {
    uint8_t b[2] = {(uint8_t)(v & 0xFFu), (uint8_t)(v >> 8u)};
    cdc_write_all(b, sizeof(b));
}

static void write_le24(uint32_t v) {
    uint8_t b[3] = {(uint8_t)(v & 0xFFu), (uint8_t)((v >> 8u) & 0xFFu),
                    (uint8_t)((v >> 16u) & 0xFFu)};
    cdc_write_all(b, sizeof(b));
}

static void write_le32(uint32_t v) {
    uint8_t b[4] = {(uint8_t)(v & 0xFFu), (uint8_t)((v >> 8u) & 0xFFu),
                    (uint8_t)((v >> 16u) & 0xFFu), (uint8_t)((v >> 24u) & 0xFFu)};
    cdc_write_all(b, sizeof(b));
}

static void cs_assert(void) { gpio_put(SP_PIN_CS, 0); }
static void cs_deassert(void) { gpio_put(SP_PIN_CS, 1); }

static inline bool pin_is_valid(int pin) { return pin >= 0; }

static void set_optional_pin_level(int pin, bool active, bool active_high) {
    if (!pin_is_valid(pin)) {
        return;
    }
    gpio_put((uint)pin, (active == active_high) ? 1 : 0);
}

/*
 * S_PIN_STATE hooks both firmware-level tri-state handling and optional external
 * electrical isolation/power gating pins so the programmer can be disconnected
 * from the target without unplugging headers.
 */
static void set_pin_drivers(bool enabled) {
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

    set_optional_pin_level(SP_PIN_ISOLATE_EN, enabled,
                           SP_PIN_ISOLATE_EN_ACTIVE_HIGH != 0);
    set_optional_pin_level(SP_PIN_TARGET_PWR, enabled,
                           SP_PIN_TARGET_PWR_ACTIVE_HIGH != 0);
}

static uint32_t spi_set_speed(uint32_t req_hz) {
    if (req_hz == 0) {
        return 0;
    }
    uint32_t actual = spi_set_baudrate(SP_SPI_PORT, req_hz);
    spi_hz_current = actual;
    return actual;
}

static void apply_cs_mode(cs_mode_t mode) {
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

static void cmdmap_set(uint8_t cmd) { cmdmap[cmd / 8u] |= (uint8_t)(1u << (cmd % 8u)); }

static void init_cmdmap(void) {
    memset(cmdmap, 0, sizeof(cmdmap));
    cmdmap_set(S_CMD_NOP);
    cmdmap_set(S_CMD_Q_IFACE);
    cmdmap_set(S_CMD_Q_CMDMAP);
    cmdmap_set(S_CMD_Q_PGMNAME);
    cmdmap_set(S_CMD_Q_SERBUF);
    cmdmap_set(S_CMD_Q_BUSTYPE);
    cmdmap_set(S_CMD_Q_OPBUF);
    cmdmap_set(S_CMD_Q_WRNMAXLEN);
    cmdmap_set(S_CMD_R_BYTE);
    cmdmap_set(S_CMD_R_NBYTES);
    cmdmap_set(S_CMD_O_INIT);
    cmdmap_set(S_CMD_O_WRITEB);
    cmdmap_set(S_CMD_O_WRITEN);
    cmdmap_set(S_CMD_O_DELAY);
    cmdmap_set(S_CMD_O_EXEC);
    cmdmap_set(S_CMD_SYNCNOP);
    cmdmap_set(S_CMD_Q_RDNMAXLEN);
    cmdmap_set(S_CMD_S_BUSTYPE);
    cmdmap_set(S_CMD_O_SPIOP);
    cmdmap_set(S_CMD_S_SPI_FREQ);
    cmdmap_set(S_CMD_S_PIN_STATE);
    cmdmap_set(S_CMD_S_SPI_CS);
    cmdmap_set(S_CMD_S_SPI_MODE);
    cmdmap_set(S_CMD_S_CS_MODE);
}

/*
 * For legacy op-buffer based commands we keep SPI payload bytes only.
 * Address arguments are ignored because SPI flashes are command-stream devices.
 */
static bool opbuf_append(const uint8_t *data, uint32_t len) {
    if (len > (SP_OPBUF_SIZE - opbuf_len)) {
        return false;
    }
    memcpy(opbuf + opbuf_len, data, len);
    opbuf_len += len;
    return true;
}

static void spi_transfer_write_then_read(uint32_t wlen, uint32_t rlen) {
    if (!pin_drivers_enabled) {
        cdc_write_u8(S_NAK);
        return;
    }

    if (wlen > SP_MAX_WRITE_CHUNK || rlen > SP_MAX_READ_CHUNK) {
        cdc_write_u8(S_NAK);
        for (uint32_t i = 0; i < wlen; i++) {
            uint8_t sink;
            cdc_read_exact(&sink, 1);
        }
        return;
    }

    cdc_read_exact(io_buf, wlen);

    if (cs_mode == CS_MODE_AUTO) {
        cs_assert();
    }

    if (wlen > 0) {
        if (spi_mode == SPI_MODE_FULL_DUPLEX && rlen > 0) {
            uint32_t first = (wlen < rlen) ? wlen : rlen;
            spi_write_read_blocking(SP_SPI_PORT, io_buf, io_buf, first);
            if (wlen > first) {
                spi_write_blocking(SP_SPI_PORT, io_buf + first, wlen - first);
            }
            cdc_write_u8(S_ACK);
            cdc_write_all(io_buf, first);
            rlen -= first;
        } else {
            spi_write_blocking(SP_SPI_PORT, io_buf, wlen);
            cdc_write_u8(S_ACK);
        }
    } else {
        cdc_write_u8(S_ACK);
    }

    while (rlen > 0) {
        uint32_t chunk = (rlen < SP_MAX_READ_CHUNK) ? rlen : SP_MAX_READ_CHUNK;
        memset(io_buf, 0, chunk);
        spi_read_blocking(SP_SPI_PORT, 0x00, io_buf, chunk);
        cdc_write_all(io_buf, chunk);
        rlen -= chunk;
    }

    if (cs_mode == CS_MODE_AUTO) {
        cs_deassert();
    }
}

static void handle_serprog_command(uint8_t cmd) {
    switch (cmd) {
    /* Keep-alive/probe command used by host to verify link is alive. */
    case S_CMD_NOP:
        cdc_write_u8(S_ACK);
        break;

    /* Report serprog interface version for protocol compatibility checks. */
    case S_CMD_Q_IFACE:
        cdc_write_u8(S_ACK);
        write_le16(SERPROG_IFACE_VERSION);
        break;

    /* Return bitmap of all commands this firmware supports. */
    case S_CMD_Q_CMDMAP:
        cdc_write_u8(S_ACK);
        cdc_write_all(cmdmap, sizeof(cmdmap));
        break;

    /* Return fixed programmer name shown by host tooling. */
    case S_CMD_Q_PGMNAME: {
        const char name[16] = "rpi-spi\0\0\0\0\0\0\0\0\0";
        cdc_write_u8(S_ACK);
        cdc_write_all(name, sizeof(name));
        break;
    }

    /* Report serial RX/TX buffer capacity hint for host chunking. */
    case S_CMD_Q_SERBUF:
        cdc_write_u8(S_ACK);
        write_le16(0xFFFF);
        break;

    /* Advertise supported bus types (SPI only in this firmware). */
    case S_CMD_Q_BUSTYPE:
        cdc_write_u8(S_ACK);
        cdc_write_u8(SUPPORTED_BUSTYPE);
        break;

    /* Report operation-buffer size used by staged O_* commands. */
    case S_CMD_Q_OPBUF:
        cdc_write_u8(S_ACK);
        write_le16((uint16_t)SP_OPBUF_SIZE);
        break;

    /* Report maximum single write length accepted by streaming ops. */
    case S_CMD_Q_WRNMAXLEN:
        cdc_write_u8(S_ACK);
        write_le24(SP_MAX_WRITE_CHUNK);
        break;

    /* Report maximum single read length accepted by streaming ops. */
    case S_CMD_Q_RDNMAXLEN:
        cdc_write_u8(S_ACK);
        write_le24(SP_MAX_READ_CHUNK);
        break;

    /* Resynchronization marker used by host after line/protocol desync. */
    case S_CMD_SYNCNOP:
        cdc_write_u8(S_NAK);
        cdc_write_u8(S_ACK);
        break;

    /* Select active bus type mask requested by the host. */
    case S_CMD_S_BUSTYPE: {
        uint8_t bustype;
        cdc_read_exact(&bustype, 1);
        bool valid = (bustype != 0) && ((bustype & ~SUPPORTED_BUSTYPE) == 0);
        cdc_write_u8(valid ? S_ACK : S_NAK);
        break;
    }

    /* Set SPI clock and return the nearest achievable hardware baudrate. */
    case S_CMD_S_SPI_FREQ: {
        uint32_t requested = read_le32();
        if (requested == 0) {
            cdc_write_u8(S_NAK);
            break;
        }
        uint32_t actual = spi_set_speed(requested);
        cdc_write_u8(actual ? S_ACK : S_NAK);
        if (actual) {
            write_le32(actual);
        }
        break;
    }

    /* Enable/disable SPI pin drivers and optional isolate/power controls. */
    case S_CMD_S_PIN_STATE: {
        uint8_t enabled;
        cdc_read_exact(&enabled, 1);
        set_pin_drivers(enabled != 0);
        cdc_write_u8(S_ACK);
        break;
    }

    /* Select chip-select index (only index 0 is implemented). */
    case S_CMD_S_SPI_CS: {
        uint8_t cs_index;
        cdc_read_exact(&cs_index, 1);
        cdc_write_u8((cs_index == 0) ? S_ACK : S_NAK);
        break;
    }

    /* Set half-duplex vs full-duplex transfer behavior. */
    case S_CMD_S_SPI_MODE: {
        uint8_t mode;
        cdc_read_exact(&mode, 1);
        if (mode == SPI_MODE_HALF_DUPLEX || mode == SPI_MODE_FULL_DUPLEX) {
            spi_mode = (spi_mode_t)mode;
            cdc_write_u8(S_ACK);
        } else {
            cdc_write_u8(S_NAK);
        }
        break;
    }

    /* Set CS handling policy (auto/asserted/deasserted). */
    case S_CMD_S_CS_MODE: {
        uint8_t mode;
        cdc_read_exact(&mode, 1);
        if (mode <= CS_MODE_DESELECTED) {
            apply_cs_mode((cs_mode_t)mode);
            cdc_write_u8(S_ACK);
        } else {
            cdc_write_u8(S_NAK);
        }
        break;
    }

    /* Execute immediate SPI write-then-read transaction with explicit lengths. */
    case S_CMD_O_SPIOP: {
        uint32_t wlen = read_le24();
        uint32_t rlen = read_le24();
        spi_transfer_write_then_read(wlen, rlen);
        break;
    }

    /* Reset staged operation buffer before appending command bytes. */
    case S_CMD_O_INIT:
        opbuf_len = 0;
        cdc_write_u8(S_ACK);
        break;

    /* Append one byte to staged operation buffer (legacy op API). */
    case S_CMD_O_WRITEB: {
        uint32_t addr = read_le24();
        uint8_t value;
        (void)addr;
        cdc_read_exact(&value, 1);
        cdc_write_u8(opbuf_append(&value, 1) ? S_ACK : S_NAK);
        break;
    }

    /* Append multiple bytes to staged operation buffer (legacy op API). */
    case S_CMD_O_WRITEN: {
        uint32_t len = read_le24();
        uint32_t addr = read_le24();
        (void)addr;
        if (len > SP_MAX_WRITE_CHUNK || len > (SP_OPBUF_SIZE - opbuf_len)) {
            for (uint32_t i = 0; i < len; i++) {
                uint8_t sink;
                cdc_read_exact(&sink, 1);
            }
            cdc_write_u8(S_NAK);
            break;
        }
        cdc_read_exact(io_buf, len);
        cdc_write_u8(opbuf_append(io_buf, len) ? S_ACK : S_NAK);
        break;
    }

    /* Insert host-requested microsecond delay between staged operations. */
    case S_CMD_O_DELAY: {
        uint32_t usec = read_le32();
        sleep_us(usec);
        cdc_write_u8(S_ACK);
        break;
    }

    /* Execute staged operation buffer as one SPI write burst. */
    case S_CMD_O_EXEC:
        if (!pin_drivers_enabled) {
            opbuf_len = 0;
            cdc_write_u8(S_NAK);
            break;
        }
        if (cs_mode == CS_MODE_AUTO) {
            cs_assert();
        }
        if (opbuf_len > 0) {
            spi_write_blocking(SP_SPI_PORT, opbuf, opbuf_len);
        }
        if (cs_mode == CS_MODE_AUTO) {
            cs_deassert();
        }
        opbuf_len = 0;
        cdc_write_u8(S_ACK);
        break;

    /* Read one byte from SPI (clocking dummy data on MOSI). */
    case S_CMD_R_BYTE:
        if (!pin_drivers_enabled) {
            cdc_write_u8(S_NAK);
            break;
        }
        if (cs_mode == CS_MODE_AUTO) {
            cs_assert();
        }
        spi_read_blocking(SP_SPI_PORT, 0x00, io_buf, 1);
        if (cs_mode == CS_MODE_AUTO) {
            cs_deassert();
        }
        cdc_write_u8(S_ACK);
        cdc_write_all(io_buf, 1);
        break;

    /* Read N bytes from SPI (clocking dummy data on MOSI). */
    case S_CMD_R_NBYTES: {
        uint32_t len = read_le24();
        if (!pin_drivers_enabled || len > SP_MAX_READ_CHUNK) {
            cdc_write_u8(S_NAK);
            break;
        }
        if (cs_mode == CS_MODE_AUTO) {
            cs_assert();
        }
        memset(io_buf, 0, len);
        spi_read_blocking(SP_SPI_PORT, 0x00, io_buf, len);
        if (cs_mode == CS_MODE_AUTO) {
            cs_deassert();
        }
        cdc_write_u8(S_ACK);
        cdc_write_all(io_buf, len);
        break;
    }

    /* Unknown command opcode; host receives NAK. */
    default:
        cdc_write_u8(S_NAK);
        break;
    }
}

static void usb_wait_for_host(void) {
    while (true) {
        tinyusb_poll();
        if (tud_cdc_connected()) {
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

    if (pin_is_valid(SP_PIN_ISOLATE_EN)) {
        gpio_init((uint)SP_PIN_ISOLATE_EN);
        gpio_set_dir((uint)SP_PIN_ISOLATE_EN, GPIO_OUT);
    }
    if (pin_is_valid(SP_PIN_TARGET_PWR)) {
        gpio_init((uint)SP_PIN_TARGET_PWR);
        gpio_set_dir((uint)SP_PIN_TARGET_PWR, GPIO_OUT);
    }

    set_pin_drivers(true);
    spi_set_speed(SP_DEFAULT_SPI_HZ);
}

int main(void) {
    board_init();
    tusb_init();

    init_cmdmap();
    init_gpio_and_spi();

    usb_wait_for_host();

    while (true) {
        tinyusb_poll();
        if (!tud_cdc_available()) {
            continue;
        }
        uint8_t cmd;
        if (tud_cdc_read(&cmd, 1) == 1) {
            handle_serprog_command(cmd);
        }
    }
}
