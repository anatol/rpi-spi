#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/unique_id.h"
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
 * Optional flash-active control pin for external gating (isolation and/or power).
 * This pin is kept inactive by default and follows S_PIN_STATE.
 * Set to -1 to disable this feature.
 */
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
#define SP_MAX_READ_CHUNK 4096u
#endif
#ifndef SP_MAX_WRITE_CHUNK
#define SP_MAX_WRITE_CHUNK 4096u
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
#define SUPPORTED_BUSTYPE S_BUS_SPI

#define CDC_SERPROG_ITF 0u
#define CDC_CONSOLE_ITF 1u

typedef enum {
    SPI_MODE_HALF_DUPLEX = 0,
    SPI_MODE_FULL_DUPLEX = 1,
} spi_mode_t;

typedef enum {
    CS_MODE_AUTO = 0,
    CS_MODE_SELECTED = 1,
    CS_MODE_DESELECTED = 2,
} cs_mode_t;

typedef enum {
    CHECK_PASS = 0,
    CHECK_WARN = 1,
    CHECK_FAIL = 2,
} check_result_t;

typedef struct {
    const char *name;
    check_result_t result;
    char evidence[96];
    char cause[96];
    char action[128];
} diag_check_t;

typedef struct {
    diag_check_t checks[10];
    uint8_t count;
    bool force;
} diag_report_t;

static uint8_t cmdmap[32];
static uint8_t opbuf[SP_OPBUF_SIZE];
static uint32_t opbuf_len;

static uint8_t io_buf[SP_MAX_WRITE_CHUNK > SP_MAX_READ_CHUNK ? SP_MAX_WRITE_CHUNK
                                                              : SP_MAX_READ_CHUNK];

static bool pin_drivers_enabled = true;
static cs_mode_t cs_mode = CS_MODE_AUTO;
static spi_mode_t spi_mode = SPI_MODE_HALF_DUPLEX;
static uint32_t spi_hz_current = SP_DEFAULT_SPI_HZ;
static bool serprog_active = false;

static char console_line[96];
static uint8_t console_line_len = 0;

#define SP_STR_INNER(x) #x
#define SP_STR(x) SP_STR_INNER(x)

#ifndef PICO_BOARD
#define PICO_BOARD unknown
#endif

static inline void tinyusb_poll(void) { tud_task(); }

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

static void console_printf(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }
    uint32_t out = (uint32_t)n;
    if (out >= sizeof(buf)) {
        out = sizeof(buf) - 1;
    }
    cdc_write_all_itf(CDC_CONSOLE_ITF, buf, out);
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

static int optional_pin_level(int pin) {
    if (!pin_is_valid(pin)) {
        return -1;
    }
    return gpio_get((uint)pin) ? 1 : 0;
}

static void set_flash_active_pin(bool active) {
    set_optional_pin_level(SP_PIN_FLASH_ACTIVE_EN, active,
                           SP_PIN_FLASH_ACTIVE_EN_ACTIVE_HIGH != 0);
}

/* S_PIN_STATE controls SPI pin tri-state handling in firmware. */
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

static bool spi_jedec_id(uint32_t speed_hz, uint8_t out_id[3]) {
    uint8_t cmd = 0x9Fu;
    uint8_t in[3] = {0, 0, 0};
    if (!pin_drivers_enabled) {
        return false;
    }

    uint32_t restore_hz = spi_hz_current;
    if (!spi_set_speed(speed_hz)) {
        return false;
    }

    cs_assert();
    spi_write_blocking(SP_SPI_PORT, &cmd, 1);
    spi_read_blocking(SP_SPI_PORT, 0x00, in, 3);
    cs_deassert();

    (void)spi_set_speed(restore_hz);
    memcpy(out_id, in, 3);
    return true;
}

static bool is_same_id(const uint8_t a[3], const uint8_t b[3]) {
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

static bool id_all_value(const uint8_t id[3], uint8_t v) {
    return id[0] == v && id[1] == v && id[2] == v;
}

static bool sample_pin_toggling(uint pin, uint32_t ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    bool first = gpio_get(pin);
    while ((to_ms_since_boot(get_absolute_time()) - start) < ms) {
        bool cur = gpio_get(pin);
        if (cur != first) {
            return true;
        }
        tinyusb_poll();
    }
    return false;
}

static void report_add(diag_report_t *r, const char *name, check_result_t result,
                       const char *evidence, const char *cause, const char *action) {
    if (r->count >= (sizeof(r->checks) / sizeof(r->checks[0]))) {
        return;
    }
    diag_check_t *c = &r->checks[r->count++];
    c->name = name;
    c->result = result;
    snprintf(c->evidence, sizeof(c->evidence), "%s", evidence);
    snprintf(c->cause, sizeof(c->cause), "%s", cause);
    snprintf(c->action, sizeof(c->action), "%s", action);
}

static void run_diagnostics(diag_report_t *r) {
    bool saved_pin_drivers = pin_drivers_enabled;
    cs_mode_t saved_cs_mode = cs_mode;
    uint32_t saved_hz = spi_hz_current;

    bool sck_toggle = false;
    bool mosi_toggle = false;
    bool cs_toggle = false;

    set_pin_drivers(false);
    gpio_pull_up(SP_PIN_SCK);
    gpio_pull_up(SP_PIN_MOSI);
    gpio_pull_up(SP_PIN_CS);
    sck_toggle = sample_pin_toggling(SP_PIN_SCK, SP_DIAG_SAMPLE_MS);
    mosi_toggle = sample_pin_toggling(SP_PIN_MOSI, SP_DIAG_SAMPLE_MS);
    cs_toggle = sample_pin_toggling(SP_PIN_CS, SP_DIAG_SAMPLE_MS);

    if (sck_toggle || mosi_toggle || cs_toggle) {
        report_add(r, "bus_contention", CHECK_WARN,
                   "activity detected on idle lines with our drivers disabled",
                   "another domain may be driving the SPI bus",
                   "hold target SoC in reset or power it down; add/verify SPI isolation hardware");
    } else {
        report_add(r, "bus_contention", CHECK_PASS,
                   "no idle line toggling observed over sample window",
                   "no obvious external driver activity",
                   "none");
    }

    if (pin_is_valid(SP_PIN_FLASH_ACTIVE_EN)) {
        int before = optional_pin_level(SP_PIN_FLASH_ACTIVE_EN);
        set_flash_active_pin(false);
        sleep_ms(20);
        set_flash_active_pin(true);
        sleep_ms(20);
        int after = optional_pin_level(SP_PIN_FLASH_ACTIVE_EN);

        if (before == -1 || after == before) {
            report_add(r, "control_pins", CHECK_WARN,
                       "FLASH_ACTIVE_EN level unchanged after toggle sequence",
                       "flash-active control may be unconnected or not wired to expected path",
                       "verify GPIO20 wiring or build with SP_PIN_FLASH_ACTIVE_EN=-1");
        } else {
            report_add(r, "control_pins", CHECK_PASS,
                       "FLASH_ACTIVE_EN changed level during cycle",
                       "control path appears electrically present",
                       "none");
        }
        if (before != -1) {
            bool before_active = (before == ((SP_PIN_FLASH_ACTIVE_EN_ACTIVE_HIGH != 0) ? 1 : 0));
            set_flash_active_pin(before_active);
        }
    } else {
        report_add(r, "control_pins", CHECK_PASS,
                   "FLASH_ACTIVE_EN not configured",
                   "optional control path intentionally unavailable",
                   "none");
    }

    set_pin_drivers(true);
    apply_cs_mode(CS_MODE_AUTO);

    uint8_t id_ref[3] = {0};
    uint8_t id_cur[3] = {0};
    uint32_t speeds[3] = {SP_DIAG_SPEED_LOW_HZ, SP_DIAG_SPEED_MID_HZ, SP_DIAG_SPEED_HIGH_HZ};
    bool got_any = false;
    bool unstable = false;
    bool high_speed_fail = false;
    bool cs_same_in_force = false;

    for (uint32_t s = 0; s < 3; s++) {
        bool speed_ok = true;
        bool have_ref_this_speed = false;
        uint8_t ref[3] = {0};
        for (uint32_t i = 0; i < SP_DIAG_PROBE_RETRIES; i++) {
            if (!spi_jedec_id(speeds[s], id_cur)) {
                speed_ok = false;
                continue;
            }
            if (!have_ref_this_speed) {
                memcpy(ref, id_cur, 3);
                have_ref_this_speed = true;
                if (!got_any) {
                    memcpy(id_ref, id_cur, 3);
                    got_any = true;
                }
            } else if (!is_same_id(ref, id_cur)) {
                unstable = true;
            }
        }

        if (!have_ref_this_speed || id_all_value(ref, 0x00) || id_all_value(ref, 0xFF)) {
            speed_ok = false;
        }

        if (s == 2 && !speed_ok) {
            high_speed_fail = true;
        }
    }

    char id_text[32];
    snprintf(id_text, sizeof(id_text), "JEDEC ID %02X %02X %02X", id_ref[0], id_ref[1], id_ref[2]);

    if (!got_any || id_all_value(id_ref, 0x00) || id_all_value(id_ref, 0xFF)) {
        report_add(r, "flash_detect", CHECK_FAIL, id_text,
                   "target flash did not return a valid JEDEC ID",
                   "ensure target flash is powered at correct voltage, clip is seated, CS/SCK/MOSI/MISO mapping is correct");
    } else {
        report_add(r, "flash_detect", CHECK_PASS, id_text,
                   "flash responded to JEDEC probe",
                   "none");
    }

    if (unstable) {
        report_add(r, "id_stability", CHECK_FAIL,
                   "JEDEC ID changed across repeated reads",
                   "signal integrity issue or competing bus master",
                   "shorten wires, improve ground, hold host SoC reset, then rerun check and retry flashrom at 1M/4M");
    } else {
        report_add(r, "id_stability", CHECK_PASS,
                   "JEDEC ID consistent across retries",
                   "read path appears stable",
                   "none");
    }

    if (high_speed_fail && got_any) {
        report_add(r, "speed_margin", CHECK_WARN,
                   "probe stable at low speed but failed at high speed",
                   "SPI clock likely too high for wiring/setup",
                   "run flashrom with spispeed=1M first; if stable try 4M then 12M");
    } else {
        report_add(r, "speed_margin", CHECK_PASS,
                   "high-speed probe did not show additional failures",
                   "current speed margin appears acceptable",
                   "none");
    }

    if (r->force) {
        uint8_t with_cs[3] = {0};
        uint8_t without_cs[3] = {0};
        bool ok_with = spi_jedec_id(SP_DIAG_SPEED_LOW_HZ, with_cs);

        set_pin_drivers(true);
        apply_cs_mode(CS_MODE_DESELECTED);
        sleep_ms(2);
        bool ok_without = spi_jedec_id(SP_DIAG_SPEED_LOW_HZ, without_cs);
        apply_cs_mode(CS_MODE_AUTO);

        if (ok_with && ok_without && is_same_id(with_cs, without_cs)) {
            cs_same_in_force = true;
            report_add(r, "cs_effect", CHECK_WARN,
                       "same JEDEC response with CS deasserted and asserted",
                       "CS line may be miswired/stuck or another device is driving MISO",
                       "verify CS wire reaches target chip select pin and no second device shares bus without isolation");
        } else {
            report_add(r, "cs_effect", CHECK_PASS,
                       "response changed when CS behavior changed",
                       "CS line appears to affect target selection",
                       "none");
        }
    }

    if (cs_same_in_force) {
        report_add(r, "likely_cs_disconnected", CHECK_WARN,
                   "CS-effect test produced same response in both CS states",
                   "[high confidence] CS path likely open, miswired, or not reaching the flash package",
                   "re-seat clip and continuity-check programmer CS pin to flash CS#; rerun `check force`");
    }

    if (!got_any || id_all_value(id_ref, 0x00) || id_all_value(id_ref, 0xFF)) {
        if (id_all_value(id_ref, 0xFF) || id_all_value(id_ref, 0x00)) {
            report_add(r, "likely_miso_disconnected", CHECK_WARN,
                       "JEDEC data was constant 0xFF/0x00 across probes",
                       "[medium confidence] MISO may be open/stuck, or flash may be unpowered",
                       "verify flash VCC/GND first, then continuity-check MISO from clip to flash DO pin");
        }

        if (!sck_toggle && !mosi_toggle && !cs_toggle) {
            report_add(r, "likely_clk_mosi_path_issue", CHECK_WARN,
                       "no valid JEDEC response and no external line activity seen",
                       "[low confidence] MOSI/SCK path may be open, or target is absent/unpowered",
                       "continuity-check SCK and MOSI, reseat hook, then rerun `check` at 1MHz");
        }
    }

    set_pin_drivers(saved_pin_drivers);
    apply_cs_mode(saved_cs_mode);
    (void)spi_set_speed(saved_hz);
}

static void print_diag_report(const diag_report_t *r) {
    const char *overall = "PASS";
    for (uint8_t i = 0; i < r->count; i++) {
        if (r->checks[i].result == CHECK_FAIL) {
            overall = "FAIL";
            break;
        }
        if (r->checks[i].result == CHECK_WARN && strcmp(overall, "FAIL") != 0) {
            overall = "WARN";
        }
    }

    console_printf("\r\n=== SPI Diagnostic Report ===\r\n");
    console_printf("Overall: %s\r\n", overall);
    for (uint8_t i = 0; i < r->count; i++) {
        const char *res = r->checks[i].result == CHECK_PASS
                              ? "PASS"
                              : (r->checks[i].result == CHECK_WARN ? "WARN" : "FAIL");
        console_printf("- %s: %s\r\n", r->checks[i].name, res);
        console_printf("  evidence: %s\r\n", r->checks[i].evidence);
        console_printf("  cause: %s\r\n", r->checks[i].cause);
    }

    console_printf("Recommended Actions (Priority Order):\r\n");
    uint8_t action_idx = 1;
    for (uint8_t i = 0; i < r->count; i++) {
        if (r->checks[i].result == CHECK_PASS) {
            continue;
        }
        console_printf("%u. %s\r\n", action_idx++, r->checks[i].action);
    }
    if (action_idx == 1) {
        console_printf("1. No immediate action required. Proceed with flashrom at your current speed.\r\n");
    }

    console_printf("Next Validation:\r\n");
    console_printf("1. Rerun `check` after applying fixes.\r\n");
    console_printf("2. Run flashrom with conservative speed first: spispeed=1M.\r\n");
    console_printf("diag> ");
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

static void handle_serprog_command(uint8_t cmd) {
    serprog_active = true;
    switch (cmd) {
    /* Keep-alive/probe command used by host to verify link is alive. */
    case S_CMD_NOP:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;

    /* Report serprog interface version for protocol compatibility checks. */
    case S_CMD_Q_IFACE:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le16(SERPROG_IFACE_VERSION);
        break;

    /* Return bitmap of all commands this firmware supports. */
    case S_CMD_Q_CMDMAP:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        cdc_write_all_itf(CDC_SERPROG_ITF, cmdmap, sizeof(cmdmap));
        break;

    /* Return fixed programmer name shown by host tooling. */
    case S_CMD_Q_PGMNAME: {
        const char name[16] = "rpi-spi\0\0\0\0\0\0\0\0\0";
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        cdc_write_all_itf(CDC_SERPROG_ITF, name, sizeof(name));
        break;
    }

    /* Report serial RX/TX buffer capacity hint for host chunking. */
    case S_CMD_Q_SERBUF:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le16(0xFFFF);
        break;

    /* Advertise supported bus types (SPI only in this firmware). */
    case S_CMD_Q_BUSTYPE:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        cdc_write_u8_itf(CDC_SERPROG_ITF, SUPPORTED_BUSTYPE);
        break;

    /* Report operation-buffer size used by staged O_* commands. */
    case S_CMD_Q_OPBUF:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le16((uint16_t)SP_OPBUF_SIZE);
        break;

    /* Report maximum single write length accepted by streaming ops. */
    case S_CMD_Q_WRNMAXLEN:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le24(SP_MAX_WRITE_CHUNK);
        break;

    /* Report maximum single read length accepted by streaming ops. */
    case S_CMD_Q_RDNMAXLEN:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        write_le24(SP_MAX_READ_CHUNK);
        break;

    /* Resynchronization marker used by host after line/protocol desync. */
    case S_CMD_SYNCNOP:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;

    /* Select active bus type mask requested by the host. */
    case S_CMD_S_BUSTYPE: {
        uint8_t bustype;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &bustype, 1);
        bool valid = (bustype != 0) && ((bustype & ~SUPPORTED_BUSTYPE) == 0);
        cdc_write_u8_itf(CDC_SERPROG_ITF, valid ? S_ACK : S_NAK);
        break;
    }

    /* Set SPI clock and return the nearest achievable hardware baudrate. */
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

    /* Enable/disable SPI pin drivers and flash-active gating control. */
    case S_CMD_S_PIN_STATE: {
        uint8_t enabled;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &enabled, 1);
        bool state = (enabled != 0);
        set_pin_drivers(state);
        set_flash_active_pin(state);
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    }

    /* Select chip-select index (only index 0 is implemented). */
    case S_CMD_S_SPI_CS: {
        uint8_t cs_index;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &cs_index, 1);
        cdc_write_u8_itf(CDC_SERPROG_ITF, (cs_index == 0) ? S_ACK : S_NAK);
        break;
    }

    /* Set half-duplex vs full-duplex transfer behavior. */
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

    /* Set CS handling policy (auto/asserted/deasserted). */
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
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;

    /* Append one byte to staged operation buffer (legacy op API). */
    case S_CMD_O_WRITEB: {
        uint32_t addr = read_le24();
        uint8_t value;
        (void)addr;
        cdc_read_exact_itf(CDC_SERPROG_ITF, &value, 1);
        cdc_write_u8_itf(CDC_SERPROG_ITF, opbuf_append(&value, 1) ? S_ACK : S_NAK);
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
                cdc_read_exact_itf(CDC_SERPROG_ITF, &sink, 1);
            }
            cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
            break;
        }
        cdc_read_exact_itf(CDC_SERPROG_ITF, io_buf, len);
        cdc_write_u8_itf(CDC_SERPROG_ITF, opbuf_append(io_buf, len) ? S_ACK : S_NAK);
        break;
    }

    /* Insert host-requested microsecond delay between staged operations. */
    case S_CMD_O_DELAY: {
        uint32_t usec = read_le32();
        sleep_us(usec);
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_ACK);
        break;
    }

    /* Execute staged operation buffer as one SPI write burst. */
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

    /* Read one byte from SPI (clocking dummy data on MOSI). */
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

    /* Read N bytes from SPI (clocking dummy data on MOSI). */
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

    /* Unknown command opcode; host receives NAK. */
    default:
        cdc_write_u8_itf(CDC_SERPROG_ITF, S_NAK);
        break;
    }
    serprog_active = false;
}

static void console_print_prompt(void) { console_printf("diag> "); }

static void console_print_help(void) {
    console_printf("Commands:\r\n");
    console_printf("  help        - show this help\r\n");
    console_printf("  info        - show firmware/build/board info\r\n");
    console_printf("  status      - show SPI/lock/status\r\n");
    console_printf("  check       - run diagnostics\r\n");
    console_printf("  check force - run diagnostics with extra CS-effect check\r\n");
}

static void console_print_info(void) {
    pico_unique_board_id_t id;
    char id_hex[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    pico_get_unique_board_id(&id);
    for (size_t i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        static const char hex[] = "0123456789abcdef";
        id_hex[2 * i] = hex[(id.id[i] >> 4) & 0x0Fu];
        id_hex[2 * i + 1] = hex[id.id[i] & 0x0Fu];
    }
    id_hex[sizeof(id_hex) - 1] = '\0';

    console_printf("firmware_version=%s\r\n", SP_FW_VERSION);
    console_printf("build_date=%s\r\n", __DATE__);
    console_printf("build_time=%s\r\n", __TIME__);
    console_printf("board=%s\r\n", SP_STR(PICO_BOARD));
#if defined(PICO_RP2350)
    console_printf("mcu=RP2350\r\n");
#elif defined(PICO_RP2040)
    console_printf("mcu=RP2040\r\n");
#else
    console_printf("mcu=unknown\r\n");
#endif
    console_printf("board_id=%s\r\n", id_hex);
    console_printf("serprog_iface_version=0x%04x\r\n", SERPROG_IFACE_VERSION);
    console_printf("default_spi_hz=%u\r\n", SP_DEFAULT_SPI_HZ);
    console_printf("spi_pins cs=%d sck=%d mosi=%d miso=%d\r\n", SP_PIN_CS, SP_PIN_SCK, SP_PIN_MOSI,
                   SP_PIN_MISO);
    if (pin_is_valid(SP_PIN_FLASH_ACTIVE_EN)) {
        console_printf("flash_active_en_pin=%d\r\n", SP_PIN_FLASH_ACTIVE_EN);
        console_printf("flash_active_en_active=%s\r\n",
                       SP_PIN_FLASH_ACTIVE_EN_ACTIVE_HIGH ? "high" : "low");
    } else {
        console_printf("flash_active_en_pin=disabled\r\n");
    }
}

static void console_print_status(void) {
    console_printf("serprog_active=%s\r\n", serprog_active ? "yes" : "no");
    console_printf("pin_drivers_enabled=%s\r\n", pin_drivers_enabled ? "yes" : "no");
    console_printf("spi_hz=%lu\r\n", (unsigned long)spi_hz_current);
    console_printf("spi_mode=%s\r\n", spi_mode == SPI_MODE_FULL_DUPLEX ? "full" : "half");
    console_printf("cs_mode=%s\r\n",
                   cs_mode == CS_MODE_AUTO
                       ? "auto"
                       : (cs_mode == CS_MODE_SELECTED ? "selected" : "deselected"));
    if (pin_is_valid(SP_PIN_FLASH_ACTIVE_EN)) {
        console_printf("flash_active_en_level=%d\r\n", optional_pin_level(SP_PIN_FLASH_ACTIVE_EN));
    } else {
        console_printf("flash_active_en=not-configured\r\n");
    }
}

static void handle_console_line(char *line) {
    while (*line == ' ') {
        line++;
    }

    if (*line == '\0') {
        console_print_prompt();
        return;
    }

    if (strcmp(line, "help") == 0) {
        console_print_help();
        console_print_prompt();
        return;
    }

    if (strcmp(line, "status") == 0) {
        console_print_status();
        console_print_prompt();
        return;
    }

    if (strcmp(line, "info") == 0) {
        console_print_info();
        console_print_prompt();
        return;
    }

    if (strcmp(line, "check") == 0 || strcmp(line, "check force") == 0) {
        if (serprog_active) {
            console_printf("SPI is currently used by serprog/flashrom; retry after current transaction finishes.\r\n");
            console_print_prompt();
            return;
        }

        diag_report_t report;
        memset(&report, 0, sizeof(report));
        report.force = (strcmp(line, "check force") == 0);

        console_printf("Running diagnostics...\r\n");
        run_diagnostics(&report);
        print_diag_report(&report);
        return;
    }

    console_printf("unknown command: %s\r\n", line);
    console_print_prompt();
}

static void console_poll(void) {
    while (tud_cdc_n_available(CDC_CONSOLE_ITF) > 0) {
        char ch;
        if (tud_cdc_n_read(CDC_CONSOLE_ITF, &ch, 1) != 1) {
            return;
        }
        if (ch == '\r' || ch == '\n') {
            if (console_line_len > 0) {
                console_line[console_line_len] = '\0';
                handle_console_line(console_line);
                console_line_len = 0;
            } else {
                console_print_prompt();
            }
            continue;
        }
        if ((uint8_t)ch < 32u || (uint8_t)ch > 126u) {
            continue;
        }
        if (console_line_len + 1 < sizeof(console_line)) {
            console_line[console_line_len++] = ch;
        }
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
    console_printf("rpi-spi diagnostic console ready\r\n");
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
