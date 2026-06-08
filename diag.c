#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "pico/unique_id.h"
#include "tusb.h"

#define SP_STR_INNER(x) #x
#define SP_STR(x) SP_STR_INNER(x)

#ifndef PICO_BOARD
#define PICO_BOARD unknown
#endif

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
    // Number of populated entries in checks[].
    uint8_t count;
    // Enables additional intrusive checks (currently CS-effect test).
    bool force;
} diag_report_t;

static char console_line[96];
static uint8_t console_line_len = 0;

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

static bool spi_jedec_id(uint32_t speed_hz, uint8_t out_id[3]) {
    // JEDEC RDID opcode for SPI NOR flash.
    uint8_t cmd = 0x9Fu;
    uint8_t in[3] = {0, 0, 0};
    if (!pin_drivers_enabled) {
        return false;
    }

    uint32_t restore_hz = spi_hz_current;
    if (!spi_set_speed(speed_hz)) {
        return false;
    }

    // Direct transfer here intentionally bypasses serprog wrappers because
    // diagnostics own the bus synchronously while serprog is idle.
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
    // Detect any edge transition during the sample window.
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
    // Save and restore runtime state so diagnostics are non-destructive.
    bool saved_pin_drivers = pin_drivers_enabled;
    cs_mode_t saved_cs_mode = cs_mode;
    uint32_t saved_hz = spi_hz_current;

    bool sck_toggle = false;
    bool mosi_toggle = false;
    bool cs_toggle = false;

    set_pin_drivers(false);
    // Pull-ups provide a deterministic idle bias while watching for
    // unexpected external activity (possible bus contention).
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
        // Probe across low/mid/high speed to classify margin issues.
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
            // All-0x00/0xFF is considered invalid ID capture.
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
        // Optional A/B test: compare responses with CS auto vs forced deselect.
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

void console_print_prompt(void) { console_printf("diag> "); }
void console_print_ready(void) { console_printf("rpi-spi diagnostic console ready\r\n"); }

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
    console_printf("default_uart_baud=%u\r\n", SP_DEFAULT_UART_BAUD);
    console_printf("spi_pins cs=%d sck=%d mosi=%d miso=%d\r\n", SP_PIN_CS, SP_PIN_SCK, SP_PIN_MOSI,
                   SP_PIN_MISO);
#if SP_ENABLE_UART_CONSOLE
    if (pin_is_valid(SP_PIN_UART_TX) && pin_is_valid(SP_PIN_UART_RX) &&
        SP_PIN_UART_TX != SP_PIN_UART_RX) {
        console_printf("uart_pins tx=%d rx=%d\r\n", SP_PIN_UART_TX, SP_PIN_UART_RX);
    } else {
        console_printf("uart_pins=disabled\r\n");
    }
#else
    console_printf("uart_console=disabled\r\n");
#endif
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
#if SP_ENABLE_UART_CONSOLE
    console_printf("uart_baud=%lu\r\n", (unsigned long)uart_bridge_get_baudrate());
#else
    console_printf("uart_console=disabled\r\n");
#endif
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
    // Trim only leading spaces; commands are intentionally simple.
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
            // Avoid concurrent access to SPI from serprog and diagnostics.
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

void console_poll(void) {
    while (tud_cdc_n_available(CDC_CONSOLE_ITF) > 0) {
        char ch;
        if (tud_cdc_n_read(CDC_CONSOLE_ITF, &ch, 1) != 1) {
            return;
        }
        if (ch == '\r' || ch == '\n') {
            // CR/LF commits the current line.
            if (console_line_len > 0) {
                console_line[console_line_len] = '\0';
                handle_console_line(console_line);
                console_line_len = 0;
            } else {
                console_print_prompt();
            }
            continue;
        }
        // Keep input ASCII-printable only.
        if ((uint8_t)ch < 32u || (uint8_t)ch > 126u) {
            continue;
        }
        if (console_line_len + 1 < sizeof(console_line)) {
            console_line[console_line_len++] = ch;
        }
    }
}
