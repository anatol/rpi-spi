#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app.h"
#include "pico/stdlib.h"
#include "serprog.h"
#include "tusb.h"

#define SUPPORTED_BUSTYPE S_BUS_SPI

static uint8_t cmdmap[32];
static uint8_t opbuf[SP_OPBUF_SIZE];
static uint32_t opbuf_len;
static uint8_t io_buf[SP_MAX_WRITE_CHUNK > SP_MAX_READ_CHUNK ? SP_MAX_WRITE_CHUNK
                                                              : SP_MAX_READ_CHUNK];

static void cdc_write_all_itf(uint8_t itf, const void *data, uint32_t len) {
    const uint8_t *p = (const uint8_t *)data;
    while (len > 0) {
        tinyusb_poll();
        uint32_t space = tud_cdc_n_write_available(itf);
        if (space == 0) {
            tud_cdc_n_write_flush(itf);
            continue;
        }
        uint32_t chunk = len < space ? len : space;
        uint32_t wr = tud_cdc_n_write(itf, p, chunk);
        tud_cdc_n_write_flush(itf);
        p += wr;
        len -= wr;
    }
}

static void cdc_write_u8_itf(uint8_t itf, uint8_t v) { cdc_write_all_itf(itf, &v, 1); }

static bool cdc_read_exact_itf(uint8_t itf, void *dst, uint32_t len) {
    uint8_t *p = (uint8_t *)dst;
    while (len > 0) {
        tinyusb_poll();
        uint32_t avail = tud_cdc_n_available(itf);
        if (avail == 0) {
            continue;
        }
        uint32_t chunk = len < avail ? len : avail;
        uint32_t rd = tud_cdc_n_read(itf, p, chunk);
        p += rd;
        len -= rd;
    }
    return true;
}

static uint32_t read_le24(void) {
    uint8_t b[3];
    cdc_read_exact_itf(CDC_SERPROG_ITF, b, sizeof(b));
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8u) | ((uint32_t)b[2] << 16u);
}

static uint32_t read_le32(void) {
    uint8_t b[4];
    cdc_read_exact_itf(CDC_SERPROG_ITF, b, sizeof(b));
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8u) | ((uint32_t)b[2] << 16u) |
           ((uint32_t)b[3] << 24u);
}

static void write_le16(uint16_t v) {
    uint8_t b[2] = {(uint8_t)(v & 0xFFu), (uint8_t)(v >> 8u)};
    cdc_write_all_itf(CDC_SERPROG_ITF, b, sizeof(b));
}

static void write_le24(uint32_t v) {
    uint8_t b[3] = {(uint8_t)(v & 0xFFu), (uint8_t)((v >> 8u) & 0xFFu),
                    (uint8_t)((v >> 16u) & 0xFFu)};
    cdc_write_all_itf(CDC_SERPROG_ITF, b, sizeof(b));
}

static void write_le32(uint32_t v) {
    uint8_t b[4] = {(uint8_t)(v & 0xFFu), (uint8_t)((v >> 8u) & 0xFFu),
                    (uint8_t)((v >> 16u) & 0xFFu), (uint8_t)((v >> 24u) & 0xFFu)};
    cdc_write_all_itf(CDC_SERPROG_ITF, b, sizeof(b));
}

static void cmdmap_set(uint8_t cmd) { cmdmap[cmd / 8u] |= (uint8_t)(1u << (cmd % 8u)); }

void init_cmdmap(void) {
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
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
        return;
    }

    if (wlen > SP_MAX_WRITE_CHUNK || rlen > SP_MAX_READ_CHUNK) {
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
        for (uint32_t i = 0; i < wlen; i++) {
            uint8_t sink;
            cdc_read_exact_itf(CDC_SERPROG_ITF, &sink, 1);
        }
        return;
    }

    cdc_read_exact_itf(CDC_SERPROG_ITF, io_buf, wlen);

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
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
            cdc_write_all_itf(CDC_SERPROG_ITF, io_buf, first);
            rlen -= first;
        } else {
            spi_write_blocking(SP_SPI_PORT, io_buf, wlen);
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        }
    } else {
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
    }

    while (rlen > 0) {
        uint32_t chunk = (rlen < SP_MAX_READ_CHUNK) ? rlen : SP_MAX_READ_CHUNK;
        memset(io_buf, 0, chunk);
        spi_read_blocking(SP_SPI_PORT, 0x00, io_buf, chunk);
        cdc_write_all_itf(CDC_SERPROG_ITF, io_buf, chunk);
        rlen -= chunk;
    }

    if (cs_mode == CS_MODE_AUTO) {
        cs_deassert();
    }
}

void handle_serprog_command(uint8_t cmd) {
    serprog_active = true;
    switch (cmd) {
    case S_CMD_NOP:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    case S_CMD_Q_IFACE:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le16(SERPROG_IFACE_VERSION);
        break;
    case S_CMD_Q_CMDMAP:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        cdc_write_all_itf(CDC_SERPROG_ITF, cmdmap, sizeof(cmdmap));
        break;
    case S_CMD_Q_PGMNAME: {
        const char name[16] = "rpi-spi\0\0\0\0\0\0\0\0\0";
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        cdc_write_all_itf(CDC_SERPROG_ITF, name, sizeof(name));
        break;
    }
    case S_CMD_Q_SERBUF:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le16(0xFFFF);
        break;
    case S_CMD_Q_BUSTYPE:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        cdc_write_u8_itf(CDC_SERPROG_ITF, SUPPORTED_BUSTYPE);
        break;
    case S_CMD_Q_OPBUF:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le16((uint16_t)SP_OPBUF_SIZE);
        break;
    case S_CMD_Q_WRNMAXLEN:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le24(SP_MAX_WRITE_CHUNK);
        break;
    case S_CMD_Q_RDNMAXLEN:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le24(SP_MAX_READ_CHUNK);
        break;
    case S_CMD_SYNCNOP:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    case S_CMD_S_BUSTYPE: {
        uint8_t bustype;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &bustype, 1);
        bool valid = (bustype != 0) && ((bustype & ~SUPPORTED_BUSTYPE) == 0);
        cdc_write_u8_itf(CDC_SERPROG_ITF, valid ? S_ACK : S_NAK);
        break;
    }
    case S_CMD_S_SPI_FREQ: {
        uint32_t requested = read_le32();
        if (requested == 0) {
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
            break;
        }
        uint32_t actual = spi_set_speed(requested);
        cdc_write_u8_itf(CDC_SERPROG_ITF, actual ? S_ACK : S_NAK);
        if (actual) {
            write_le32(actual);
        }
        break;
    }
    case S_CMD_S_PIN_STATE: {
        uint8_t enabled;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &enabled, 1);
        bool state = (enabled != 0);
        set_pin_drivers(state);
        set_flash_active_pin(state);
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    }
    case S_CMD_S_SPI_CS: {
        uint8_t cs_index;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &cs_index, 1);
        cdc_write_u8_itf(CDC_SERPROG_ITF, (cs_index == 0) ? S_ACK : S_NAK);
        break;
    }
    case S_CMD_S_SPI_MODE: {
        uint8_t mode;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &mode, 1);
        if (mode == SPI_MODE_HALF_DUPLEX || mode == SPI_MODE_FULL_DUPLEX) {
            spi_mode = (spi_mode_t)mode;
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        } else {
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
        }
        break;
    }
    case S_CMD_S_CS_MODE: {
        uint8_t mode;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &mode, 1);
        if (mode <= CS_MODE_DESELECTED) {
            apply_cs_mode((cs_mode_t)mode);
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        } else {
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
        }
        break;
    }
    case S_CMD_O_SPIOP: {
        uint32_t wlen = read_le24();
        uint32_t rlen = read_le24();
        spi_transfer_write_then_read(wlen, rlen);
        break;
    }
    case S_CMD_O_INIT:
        opbuf_len = 0;
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    case S_CMD_O_WRITEB: {
        uint32_t addr = read_le24();
        uint8_t value;
        (void)addr;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &value, 1);
        cdc_write_u8_itf(CDC_SERPROG_ITF, opbuf_append(&value, 1) ? S_ACK : S_NAK);
        break;
    }
    case S_CMD_O_WRITEN: {
        uint32_t len = read_le24();
        uint32_t addr = read_le24();
        (void)addr;
        if (len > SP_MAX_WRITE_CHUNK || len > (SP_OPBUF_SIZE - opbuf_len)) {
            for (uint32_t i = 0; i < len; i++) {
                uint8_t sink;
                cdc_read_exact_itf(CDC_SERPROG_ITF, &sink, 1);
            }
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
            break;
        }
        cdc_read_exact_itf(CDC_SERPROG_ITF, io_buf, len);
        cdc_write_u8_itf(CDC_SERPROG_ITF, opbuf_append(io_buf, len) ? S_ACK : S_NAK);
        break;
    }
    case S_CMD_O_DELAY: {
        uint32_t usec = read_le32();
        sleep_us(usec);
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    }
    case S_CMD_O_EXEC:
        if (!pin_drivers_enabled) {
            opbuf_len = 0;
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
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
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    case S_CMD_R_BYTE:
        if (!pin_drivers_enabled) {
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
            break;
        }
        if (cs_mode == CS_MODE_AUTO) {
            cs_assert();
        }
        spi_read_blocking(SP_SPI_PORT, 0x00, io_buf, 1);
        if (cs_mode == CS_MODE_AUTO) {
            cs_deassert();
        }
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        cdc_write_all_itf(CDC_SERPROG_ITF, io_buf, 1);
        break;
    case S_CMD_R_NBYTES: {
        uint32_t len = read_le24();
        if (!pin_drivers_enabled || len > SP_MAX_READ_CHUNK) {
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
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
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        cdc_write_all_itf(CDC_SERPROG_ITF, io_buf, len);
        break;
    }
    default:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
        break;
    }
    serprog_active = false;
}
