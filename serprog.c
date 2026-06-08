#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "serprog.h"
#include "status_led.h"
#include "tusb.h"

#define SUPPORTED_BUSTYPE S_BUS_SPI

// Serprog command bitmap returned by S_CMD_Q_CMDMAP.
// One bit per command ID across 256 possible command values.
static uint8_t cmdmap[32];
// Staging buffer for operation-buffer mode (O_INIT/O_WRITE*/O_EXEC).
static uint8_t opbuf[SP_OPBUF_SIZE];
// Current valid length in opbuf.
static uint32_t opbuf_len;
// Shared temporary I/O buffer for both writes and reads.
// Sized to the larger of max write chunk and max read chunk.
static uint8_t io_buf[SP_MAX_WRITE_CHUNK > SP_MAX_READ_CHUNK ? SP_MAX_WRITE_CHUNK
                                                              : SP_MAX_READ_CHUNK];

static bool cdc_read_exact_itf(uint8_t itf, void *dst, uint32_t len);

static void cdc_drain_bytes_itf(uint8_t itf, uint32_t len) {
    uint8_t sink[64];
    while (len > 0) {
        uint32_t chunk = len < sizeof(sink) ? len : (uint32_t)sizeof(sink);
        cdc_read_exact_itf(itf, sink, chunk);
        len -= chunk;
    }
}

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
        p += wr;
        len -= wr;
    }
    // Force submission once per logical payload to avoid per-packet flush overhead.
    tud_cdc_n_write_flush(itf);
}

static void cdc_write_u8_itf(uint8_t itf, uint8_t v) { cdc_write_all_itf(itf, &v, 1); }

static void reject_command(void) {
    status_led_notify(STATUS_LED_EVENT_ERROR);
    cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
}

static void write_command_result(bool accepted) {
    if (accepted) {
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
    } else {
        reject_command();
    }
}

static bool cdc_read_exact_itf(uint8_t itf, void *dst, uint32_t len) {
    uint8_t *p = (uint8_t *)dst;
    while (len > 0) {
        tinyusb_poll();
        uint32_t avail = tud_cdc_n_available(itf);
        if (avail == 0) {
            // Busy-wait until host provides enough bytes for this command.
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
    // Serprog commonly uses 24-bit little-endian length/address fields.
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
    // Advertise exactly the commands implemented in handle_serprog_command().
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
        // Prevent overflow of operation buffer.
        return false;
    }
    memcpy(opbuf + opbuf_len, data, len);
    opbuf_len += len;
    return true;
}

static void spi_transfer_write_then_read(uint32_t wlen, uint32_t rlen) {
    if (!pin_drivers_enabled) {
        // Refuse bus activity while external pin drivers are disabled.
        reject_command();
        return;
    }

    if (wlen > SP_MAX_WRITE_CHUNK || rlen > SP_MAX_READ_CHUNK) {
        reject_command();
        // Consume incoming write payload to keep command stream aligned.
        cdc_drain_bytes_itf(CDC_SERPROG_ITF, wlen);
        return;
    }

    cdc_read_exact_itf(CDC_SERPROG_ITF, io_buf, wlen);
    status_led_notify(STATUS_LED_EVENT_SPI_TRAFFIC);

    if (cs_mode == CS_MODE_AUTO) {
        // Auto mode wraps each transfer in one CS assertion window.
        cs_assert();
    }

    if (wlen > 0) {
        if (spi_mode == SPI_MODE_FULL_DUPLEX && rlen > 0) {
            // Full-duplex: first min(wlen, rlen) bytes are simultaneous write/read.
            uint32_t first = (wlen < rlen) ? wlen : rlen;
            spi_write_read_blocking(SP_SPI_PORT, io_buf, io_buf, first);
            if (wlen > first) {
                // Remaining TX-only tail when wlen > rlen.
                spi_write_blocking(SP_SPI_PORT, io_buf + first, wlen - first);
            }
            // ACK precedes read payload in this protocol.
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
            cdc_write_all_itf(CDC_SERPROG_ITF, io_buf, first);
            rlen -= first;
        } else {
            // Half-duplex or pure write phase.
            spi_write_blocking(SP_SPI_PORT, io_buf, wlen);
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        }
    } else {
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
    }

    while (rlen > 0) {
        // Read remaining bytes in bounded chunks.
        uint32_t chunk = (rlen < SP_MAX_READ_CHUNK) ? rlen : SP_MAX_READ_CHUNK;
        spi_read_blocking(SP_SPI_PORT, 0x00, io_buf, chunk);
        cdc_write_all_itf(CDC_SERPROG_ITF, io_buf, chunk);
        rlen -= chunk;
    }

    if (cs_mode == CS_MODE_AUTO) {
        cs_deassert();
    }
}

static bool flash_read_jedec_id(uint8_t id[3]) {
    if (!pin_drivers_enabled) {
        return false;
    }

    uint8_t cmd = 0x9Fu;
    cs_assert();
    spi_write_blocking(SP_SPI_PORT, &cmd, 1);
    spi_read_blocking(SP_SPI_PORT, 0x00, id, 3);
    cs_deassert();
    return true;
}

static bool flash_jedec_id_valid(const uint8_t id[3]) {
    bool all_zero = id[0] == 0x00u && id[1] == 0x00u && id[2] == 0x00u;
    bool all_ff = id[0] == 0xFFu && id[1] == 0xFFu && id[2] == 0xFFu;
    return !all_zero && !all_ff;
}

static void wait_flash_ready_after_pin_enable(void) {
    absolute_time_t deadline = make_timeout_time_ms(500);
    uint8_t id[3];

    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        if (flash_read_jedec_id(id) && flash_jedec_id_valid(id)) {
            return;
        }
        tinyusb_poll();
    }
}

void handle_serprog_command(uint8_t cmd) {
    // Signals to other subsystems that serprog currently owns the interface.
    serprog_active = true;
    switch (cmd) {
    case S_CMD_NOP:
        // Liveness ping: no payload, ACK only.
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    case S_CMD_Q_IFACE:
        // Report protocol interface version.
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le16(SERPROG_IFACE_VERSION);
        break;
    case S_CMD_Q_CMDMAP:
        // Return 256-bit command support map.
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        cdc_write_all_itf(CDC_SERPROG_ITF, cmdmap, sizeof(cmdmap));
        break;
    case S_CMD_Q_PGMNAME: {
        // Fixed 16-byte programmer name, NUL-padded as required by serprog.
        const char name[16] = "rpi-spi\0\0\0\0\0\0\0\0\0";
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        cdc_write_all_itf(CDC_SERPROG_ITF, name, sizeof(name));
        break;
    }
    case S_CMD_Q_SERBUF:
        // Streaming receive capability; 0xFFFF conventionally means "large enough".
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le16(0xFFFF);
        break;
    case S_CMD_Q_BUSTYPE:
        // Only SPI is implemented.
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        cdc_write_u8_itf(CDC_SERPROG_ITF, SUPPORTED_BUSTYPE);
        break;
    case S_CMD_Q_OPBUF:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le16((uint16_t)SP_OPBUF_SIZE);
        break;
    case S_CMD_Q_WRNMAXLEN:
        // Maximum write length accepted by O_SPIOP write phase.
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le24(SP_MAX_WRITE_CHUNK);
        break;
    case S_CMD_Q_RDNMAXLEN:
        // Maximum read length accepted in one read command.
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le24(SP_MAX_READ_CHUNK);
        break;
    case S_CMD_SYNCNOP:
        // Resynchronization primitive: emit NAK then ACK exactly.
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    case S_CMD_S_BUSTYPE: {
        uint8_t bustype;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &bustype, 1);
        // Valid if non-zero and no unsupported bus bits are requested.
        bool valid = (bustype != 0) && ((bustype & ~SUPPORTED_BUSTYPE) == 0);
        write_command_result(valid);
        break;
    }
    case S_CMD_S_SPI_FREQ: {
        // Host requests target SPI clock in Hz.
        uint32_t requested = read_le32();
        if (requested == 0) {
            reject_command();
            break;
        }
        uint32_t actual = spi_set_speed(requested);
        // ACK only if clock setup succeeded; then return chosen real frequency.
        write_command_result(actual != 0);
        if (actual) {
            write_le32(actual);
        }
        break;
    }
    case S_CMD_S_PIN_STATE: {
        uint8_t enabled;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &enabled, 1);
        // Any non-zero input is treated as "enable outputs".
        bool state = (enabled != 0);
        // Keep bus drivers and optional flash-active GPIO in sync.
        set_pin_drivers(state);
        set_flash_active_pin(state);
        if (state) {
            // Delay the ACK until a newly powered or connected flash responds.
            wait_flash_ready_after_pin_enable();
        }
        status_led_notify(state ? STATUS_LED_EVENT_SPI_ENABLED
                                : STATUS_LED_EVENT_SPI_ISOLATED);
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    }
    case S_CMD_S_SPI_CS: {
        uint8_t cs_index;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &cs_index, 1);
        // Single-CS design: only CS#0 is supported.
        write_command_result(cs_index == 0);
        break;
    }
    case S_CMD_S_SPI_MODE: {
        uint8_t mode;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &mode, 1);
        // Protocol-specific duplex mode, not CPOL/CPHA.
        if (mode == SPI_MODE_HALF_DUPLEX || mode == SPI_MODE_FULL_DUPLEX) {
            spi_mode = (spi_mode_t)mode;
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        } else {
            reject_command();
        }
        break;
    }
    case S_CMD_S_CS_MODE: {
        uint8_t mode;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &mode, 1);
        // Accept only defined CS policy enum values.
        if (mode <= CS_MODE_DESELECTED) {
            apply_cs_mode((cs_mode_t)mode);
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        } else {
            reject_command();
        }
        break;
    }
    case S_CMD_O_SPIOP: {
        // One-shot SPI operation with separate write/read lengths.
        uint32_t wlen = read_le24();
        uint32_t rlen = read_le24();
        spi_transfer_write_then_read(wlen, rlen);
        break;
    }
    case S_CMD_O_INIT:
        // Begin buffered-operation mode by clearing any stale staged bytes.
        opbuf_len = 0;
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    case S_CMD_O_WRITEB: {
        // Address is ignored in SPI mode; only payload byte is queued.
        uint32_t addr = read_le24();
        uint8_t value;
        (void)addr;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &value, 1);
        write_command_result(opbuf_append(&value, 1));
        break;
    }
    case S_CMD_O_WRITEN: {
        // Queue N bytes into opbuf. Address field is ignored for SPI.
        uint32_t len = read_le24();
        uint32_t addr = read_le24();
        (void)addr;
        if (len > SP_MAX_WRITE_CHUNK || len > (SP_OPBUF_SIZE - opbuf_len)) {
            // Drain payload even on reject so the next command starts on byte boundary.
            cdc_drain_bytes_itf(CDC_SERPROG_ITF, len);
            reject_command();
            break;
        }
        cdc_read_exact_itf(CDC_SERPROG_ITF, io_buf, len);
        write_command_result(opbuf_append(io_buf, len));
        break;
    }
    case S_CMD_O_DELAY: {
        // Busy delay in microseconds between staged ops.
        uint32_t usec = read_le32();
        sleep_us(usec);
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    }
    case S_CMD_O_EXEC:
        // Execute staged writes as one CS window in auto mode.
        if (!pin_drivers_enabled) {
            // Drop queued data when bus is unavailable to avoid stale replay.
            opbuf_len = 0;
            reject_command();
            break;
        }
        if (cs_mode == CS_MODE_AUTO) {
            cs_assert();
        }
        if (opbuf_len > 0) {
            status_led_notify(STATUS_LED_EVENT_SPI_TRAFFIC);
            spi_write_blocking(SP_SPI_PORT, opbuf, opbuf_len);
        }
        if (cs_mode == CS_MODE_AUTO) {
            cs_deassert();
        }
        opbuf_len = 0;
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    case S_CMD_R_BYTE:
        // Read exactly one byte by clocking a dummy 0x00 on MOSI.
        // This command is common in byte-at-a-time flash probing flows.
        if (!pin_drivers_enabled) {
            // Do not touch bus if output drivers are intentionally disabled.
            reject_command();
            break;
        }
        if (cs_mode == CS_MODE_AUTO) {
            // In auto mode, own CS only for this single byte transaction.
            cs_assert();
        }
        // Read one byte from MISO; io_buf[0] receives sampled value.
        status_led_notify(STATUS_LED_EVENT_SPI_TRAFFIC);
        spi_read_blocking(SP_SPI_PORT, 0x00, io_buf, 1);
        if (cs_mode == CS_MODE_AUTO) {
            cs_deassert();
        }
        // Protocol order is ACK first, then the returned byte payload.
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        cdc_write_all_itf(CDC_SERPROG_ITF, io_buf, 1);
        break;
    case S_CMD_R_NBYTES: {
        // Bulk read command with 24-bit length argument.
        uint32_t len = read_le24();
        if (!pin_drivers_enabled || len > SP_MAX_READ_CHUNK) {
            reject_command();
            break;
        }
        if (cs_mode == CS_MODE_AUTO) {
            cs_assert();
        }
        status_led_notify(STATUS_LED_EVENT_SPI_TRAFFIC);
        spi_read_blocking(SP_SPI_PORT, 0x00, io_buf, len);
        if (cs_mode == CS_MODE_AUTO) {
            cs_deassert();
        }
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        cdc_write_all_itf(CDC_SERPROG_ITF, io_buf, len);
        break;
    }
    default:
        // Unknown or unsupported command.
        reject_command();
        break;
    }
    // Clear ownership marker after each command completion.
    serprog_active = false;
}
