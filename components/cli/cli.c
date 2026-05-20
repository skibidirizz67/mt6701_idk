#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include "driver/i2c_types.h"
#include <stdbool.h>
#include "fcntl.h"
#include "freertos/idf_additions.h"
#include "i2c_mt6701.h"

#include "cli.h"

StringView sv(const char *str) {
    return (StringView) {
        .data = str,
        .count = strlen(str),
    };
}

bool sv_cmp(StringView *a, StringView *b) {
    return (a->count == b->count) && memcmp(a->data, b->data, a->count) == 0;
}

bool sv_scmp(StringView *a, const char *b) {
    return (a->count == strlen(b)) && memcmp(a->data, b, a->count) == 0;
}

bool sv_starts_with(StringView *a, const char *prefix) {
    size_t plen = strlen(prefix);
    return a->count >= plen && memcmp(a->data, prefix, plen) == 0;
}

bool sv_isuint(StringView *sv) {
    if (sv->count == 0) return false;
    for (size_t i = 0; i < sv->count; i++) {
        if (!isdigit((unsigned char)sv->data[i])) {
            return false;
        }
    }
    return true;
}

bool sv_isfloat(StringView *sv) {
    if (sv->count == 0) return 0;
    bool seen_digit = 0, seen_dot = 0;
    for (size_t i = 0; i < sv->count; i++) {
        unsigned char c = (unsigned char)sv->data[i];
        if (isdigit(c)) {
            seen_digit = true;
            continue;
        }
        if (c == '.' && !seen_dot) {
            seen_dot = true;
            continue;
        }
        return false;
    }
    return seen_digit;
}

bool sv_tol(StringView *sv, long *out) {
    if (!out || sv->count == 0 || sv->count >= 64) return false;
    char buff[64];
    memcpy(buff, sv->data, sv->count);
    buff[sv->count] = '\0';
    errno = 0;
    char *end = NULL;
    long v = strtol(buff, &end, 10);
    if (errno != 0 || end == buff || *end != '\0') return false;
    *out = v;
    return true;
}

bool sv_tolh(StringView *sv, long *out) {
    if (!out || sv->count == 0 || sv->count >= 64) return false;
    char buff[64];
    memcpy(buff, sv->data, sv->count);
    buff[sv->count] = '\0';
    errno = 0;
    char *end = NULL;
    unsigned long v = strtoul(buff, &end, 16);
    if (errno != 0 || end == buff || *end != '\0') return false;
    *out = v;
    return true;
}

bool sv_tod(StringView *sv, double *out) {
    if (!out || sv->count == 0 || sv->count >= 64) return false;
    char buff[64];
    memcpy(buff, sv->data, sv->count);
    buff[sv->count] = '\0';
    errno = 0;
    char *end = NULL;
    double v = strtod(buff, &end);
    if (errno != 0 || end == buff || *end != '\0') return false;
    *out = v;
    return true;
}

// for some reason if you print using printf (e.g. printf("%-10s ...", "string")), stack overflows while printing big strings
// that's the reason for this solution
void printpad(const char *str, size_t col) {
    size_t len = strlen(str);
    printf ("   %s", str);
    if (len < col) {
        for (size_t i = 0; i < col - len; i++) {
            putchar(' ');
        }
    }
    else {
        putchar(' ');
    }
}

MTOpts lookup_option(StringView *sv) {
    if (sv_scmp(sv, "sensor")) return OPT_SENSOR;
    if (sv_scmp(sv, "EEPROM")) return OPT_EEPROM;
    if (sv_scmp(sv, "UVW_MUX")) return OPT_UVW_MUX;
    if (sv_scmp(sv, "ABZ_MUX")) return OPT_ABZ_MUX;
    if (sv_scmp(sv, "DIR")) return OPT_DIR;
    if (sv_scmp(sv, "UVW_RES")) return OPT_UVW_RES;
    if (sv_scmp(sv, "ABZ_RES")) return OPT_ABZ_RES;
    if (sv_scmp(sv, "HYST")) return OPT_HYST;
    if (sv_scmp(sv, "Z_PULSE_WIDTH")) return OPT_Z_PULSE_WIDTH;
    if (sv_scmp(sv, "ZERO")) return OPT_ZERO;
    if (sv_scmp(sv, "PWM_FREQ")) return OPT_PWM_FREQ;
    if (sv_scmp(sv, "PWM_POL")) return OPT_PWM_POL;
    if (sv_scmp(sv, "OUT_MODE")) return OPT_OUT_MODE;
    if (sv_scmp(sv, "A_START")) return OPT_A_START;
    if (sv_scmp(sv, "A_STOP")) return OPT_A_STOP;
    return OPT_COUNT;
}

const char *option_name(MTOpts opt) {
    switch (opt) {
        case OPT_UVW_MUX: return "UVW_MUX";
        case OPT_ABZ_MUX: return "ABZ_MUX";
        case OPT_DIR: return "DIR";
        case OPT_UVW_RES: return "UVW_RES";
        case OPT_ABZ_RES: return "ABZ_RES";
        case OPT_HYST: return "HYST";
        case OPT_Z_PULSE_WIDTH: return "Z_PULSE_WIDTH";
        case OPT_ZERO: return "ZERO";
        case OPT_PWM_FREQ: return "PWM_FREQ";
        case OPT_PWM_POL: return "PWM_POL";
        case OPT_OUT_MODE: return "OUT_MODE";
        case OPT_A_START: return "A_START";
        case OPT_A_STOP: return "A_STOP";
        case OPT_SENSOR: return "SENSOR";
        case OPT_EEPROM: return "EEPROM";
        default: return "<invalid>";
    }
}

void print_generic_help() {
    size_t col = 36;
    printpad("help", col); printf("Display this message\n");
    printpad("read <option> [--raw]", col); printf("Read encoder value (either sensor or EEPROM configuration)\n");
    printpad("monitor <sensor> <delay_ms> [--raw]", col); printf("Monitor encoder sensor\n");
    printpad("set <option> <value> [--raw]", col); printf("Set EEPROM configuration value\n");
    printpad("pending <option>", col); printf("Manipulate pending EEPROM modifications\n");
    printpad("exit", col); printf("Quit program\n");
    printpad("quit", col); printf("Quit program\n");
    printf("\n   Use \"help [command]\" to display command-specific help\n");
}

void print_read_help() {
    size_t col = 10;
    printf(" Read the sensor value or the EEPROM configuration value.\n");
    printf(" Usage: read <option> [--raw]\n");
    printf(" Example: read ABZ_RES\n");
    printpad("option", col); printf("Sensor or EEPROM configuration option\n");
    printpad("--raw", col); printf("Display raw hex value\n");
    col = 15;
    printf("\n Available options:\n");
    printpad("sensor", col); printf("Angle position encoder sensor\n");
    printpad("EEPROM", col); printf("Display whole EEPROM\n");
    printpad("UVW_MUX", col); printf("UVW output type\n");
    printpad("ABZ_MUX", col); printf("ABZ output type\n");
    printpad("DIR", col); printf("Output rotation direction\n");
    printpad("UVW_RES", col); printf("UVW output resolution\n");
    printpad("ABZ_RES", col); printf("ABZ output resolution (PPR)\n");
    printpad("HYST", col); printf("Hysteresis filter parameter\n");
    printpad("Z_PULSE_WIDTH", col); printf("Z pulse width\n");
    printpad("ZERO", col); printf("Zero-degree position\n");
    printpad("PWM_FREQ", col); printf("PWM frame frequency\n");
    printpad("PWM_POL", col); printf("PWM polarity\n");
    printpad("OUT_MODE", col); printf("Out pin mode\n");
    printpad("A_START", col); printf("Start-point of analog output\n");
    printpad("A_STOP", col); printf("Stop-point of analog output\n");
}

void print_monitor_help() {
    size_t col = 10;
    printf(" Monitor sensor value continiously. Press Enter to stop.\n");
    printf(" Usage: monitor <sensor> <delay_ms> [--raw]\n");
    printf(" Example: monitor sensor 500\n");
    printpad("sensor", col); printf("Encoder sensor\n");
    printpad("--raw", col); printf("Display raw hex value\n");
    col = 15;
    printf("\n Available sensors:\n");
    printpad("sensor", col); printf("Angle position encoder sensor\n");
}

void print_set_help() {
    size_t col = 10;
    printf(" Set EEPROM configuration value.\n");
    printf(" Usage: set <option> <value> [--raw]\n");
    printf(" Example: set A_START 180.5\n");
    printf(" Example: set HYST 0x04 --raw\n");
    printf(" Example: set OUT_MODE 0\n");
    printpad("option", col); printf("EEPROM configuration option\n");
    printpad("value", col); printf("Value to set. If value is out of range, it will be truncated or rounded to the nearest valid value. For some options, like OUT_MODE, only raw set is available (i.e. number)\n");
    printpad("--raw", col); printf("Set value in raw hex withour processing.\n");
    col = 15;
    printf("\n Available options:\n");
    printpad("UVW_MUX", col); printf("UVW output type (UVW, -A-B-Z)\n");
    printpad("ABZ_MUX", col); printf("ABZ output type (ABZ, UVW)\n");
    printpad("DIR", col); printf("Output rotation direction (CCW, CW)\n");
    printpad("UVW_RES", col); printf("UVW output resolution (1,2..,15,16)\n");
    printpad("ABZ_RES", col); printf("ABZ output resolution (PPR) (1,2..,1023,1024)\n");
    printpad("HYST", col); printf("Hysteresis filter parameter (1,2,4,8,0,0.25,0.5,1)\n");
    printpad("Z_PULSE_WIDTH", col); printf("Z pulse width (1,2,4,8,12,16,180,1)\n");
    printpad("ZERO", col); printf("Zero-degree position (0.0,0.088,..,359.824,359.912)\n");
    printpad("PWM_FREQ", col); printf("PWM frame frequency (994.4,497.2)\n");
    printpad("PWM_POL", col); printf("PWM polarity (high,low)\n");
    printpad("OUT_MODE", col); printf("Out pin mode (analog,PWM)\n");
    printpad("A_START", col); printf("Start-point of analog output (0.0,0.088,..,359.824,359.912)\n");
    printpad("A_STOP", col); printf("Stop-point of analog output (0.0,0.088,..,359.824,359.912)\n");
}

void print_pending_help() {
    size_t col = 10;
    printf(" Manipulate pending EEPROM modifications.\n");
    printf(" Usage: pending <option>\n");
    printf(" Example: pending show\n");
    printpad("<option>", col); printf("Action to do with pendings\n");
    col = 15;
    printf("\n Available options:\n");
    printpad("show", col); printf("Show pending EEPROM modifications\n");
    printpad("apply", col); printf("Apply all pending EEPROM modifications\n");
    printpad("clear", col); printf("Clear all pending EEPROM modifications\n");
}

CmdId lookup_cmd(CliContext *ctx, StringView *name) {
    for (size_t i = 0; i < CMD_COUNT; i++) {
        if (sv_cmp(&ctx->cmdv[i].name, name)) return ctx->cmdv[i].id;
    }
    return CMD_COUNT;
}

Command *lookup_cmd_meta(struct CliContext *ctx, StringView *name) {
    for (size_t i = 0; i < CMD_COUNT; i++) {
        if (sv_cmp(&ctx->cmdv[i].name, name)) return &ctx->cmdv[i];
    }
    return NULL;
}

void show_pending(CliContext *ctx) {
    bool any = false;
    for (size_t i = 0; i < OPT_COUNT; i++) {
        if (ctx->pending[i].modified) {
            any = true;
            printf("   %-12s = %.3f\n", option_name(i), ctx->pending[i].val); // #TODO: possibly raw needs other print format
        }
    }
    if (!any) printf("   No pending changes\n");
}

void clear_pending(CliContext *ctx) {
    for (size_t i = 0; i < OPT_COUNT; i++) {
        ctx->pending[i] = (SetBuff){0};
    }
    printf("   Pending changes cleared\n");
}

void apply_pending(CliContext *ctx) {
    for (size_t i = 0; i < OPT_COUNT; i++) {
        if (!ctx->pending[i].modified) continue;
        if (!ctx->table[i].writable) continue;
        if (ctx->pending[i].raw) {
            if (!ctx->table[i].write_raw) {
                printf("%s: %s\n", ERR_CLI_INVALID_OPT, option_name(i));
                return;
            }
            ctx->table[i].write_raw(ctx->dev_handle, (uint16_t)ctx->pending[i].val);
        }
        else {
            if (!ctx->table[i].write) {
                printf("%s: %s\n", ERR_CLI_INVALID_OPT, option_name(i));
                return;
            }
            ctx->table[i].write(ctx->dev_handle, ctx->pending[i].val);
        }
    }
    clear_pending(ctx);
    printf("   Pending changes applied\n");
}

int tokenize(char *str, StringView *argv, size_t *argc) {
    static const char *ws = " \t";
    *argc = 0;

    while (*str) {
        str += strspn(str, ws);
        if (!*str) break;
        if (*argc >= CLI_MAX_TOKENS) {
            printf("   Exceeded argument limit: %d\n", CLI_MAX_TOKENS);
            return -1;
        }

        argv[*argc].data = str;
        argv[*argc].count = strcspn(str, ws);
        str += argv[*argc].count;
        (*argc)++;
    }

    return 0;
}

bool parse_value(StringView *val, bool raw, double *out) {
    *out = 0.0;
    if (raw) {
        if (!sv_starts_with(val, "0x") && !sv_starts_with(val, "0X")) return false;
        StringView slice = { .data = val->data + 2, .count = val->count - 2 };
        long hexv = 0;
        if (!sv_tolh(&slice, &hexv)) return false;
        *out = (double)hexv;
        return true;
    }
    return sv_tod(val, out);
}

void exec_help(struct CliContext *ctx, StringView *argv, size_t argc, uint8_t flags) {
    if (argc > 1) {
        printf("%s\n", ERR_CLI_TOO_MANY_ARG);
        return;
    }
    if (argc == 0) {
        print_generic_help();
        return;
    }

    Command *meta = lookup_cmd_meta(ctx, &argv[0]);
    if (!meta || !meta->help_fn) {
        print_generic_help();
        return;
    }
    meta->help_fn();
}

void exec_read(struct CliContext *ctx, StringView *argv, size_t argc, uint8_t flags) {
    if (argc < 1) {
        printf("%s: <option>\n", ERR_CLI_MISSING_ARG);
        return;
    }
    if (argc > 1) {
        printf("%s\n", ERR_CLI_TOO_MANY_ARG);
        return;
    }

    MTOpts opt = lookup_option(&argv[0]);
    if (opt == OPT_COUNT || !ctx->table[opt].readable) {
        printf("%s: %.*s\n", ERR_CLI_INVALID_ARG, (int)argv[0].count, argv[0].data);
        return;
    }

    if (flags & CLI_RAW_FLAG) {
        if (!ctx->table[opt].read_raw) {
            printf("%s: %s\n", ERR_CLI_INVALID_OPT, option_name(opt));
            return;
        }
        printf("   0x%04X\n", (uint16_t)ctx->table[opt].read_raw(ctx->dev_handle));
    }
    else {
        if (!ctx->table[opt].read) {
            printf("%s: %s\n", ERR_CLI_INVALID_OPT, option_name(opt));
            return;
        }
        printf("   %.3f\n", ctx->table[opt].read(ctx->dev_handle));
    }
}

void exec_monitor(struct CliContext *ctx, StringView *argv, size_t argc, uint8_t flags) {
    if (argc < 1) {
        printf("%s: <sensor>\n", ERR_CLI_MISSING_ARG);
        return;
    }
    if (argc > 2) {
        printf("%s\n", ERR_CLI_TOO_MANY_ARG);
        return;
    }

    MTOpts opt = lookup_option(&argv[0]);
    if (opt == OPT_COUNT || !ctx->table[opt].readable) {
        printf("%s: %.*s\n", ERR_CLI_INVALID_ARG, (int)argv[0].count, argv[0].data);
        return;
    }
    
    long delay_ms = 500;
    if (argc == 2) {
        if (!sv_isuint(&argv[1]) || !sv_tol(&argv[1], &delay_ms) || delay_ms <= 0) {
            printf("%s: %.*s\n", ERR_CLI_INVALID_ARG, (int)argv[1].count, argv[1].data);
            return;
        }
    }

    if (flags & CLI_RAW_FLAG) {
        if (!ctx->table[opt].read_raw) {
            printf("%s: sensor\n", ERR_CLI_INVALID_OPT);
            return;
        }
        printf("   Press Enter to stop\n");
        while (1) {
            int c = getchar();
            if (c == '\n' || c == '\r') break;
            printf("   0x%04X\n", (uint16_t)ctx->table[opt].read_raw(ctx->dev_handle));
            vTaskDelay(pdMS_TO_TICKS((uint32_t)delay_ms)); // #TODO: replace with usleep when separated
        }
    }
    else {
        if (!ctx->table[opt].read) {
            printf("%s: sensor\n", ERR_CLI_INVALID_OPT);
            return;
        }
        printf("   Press Enter to stop\n");
        while (1) {
            int c = getchar();
            if (c == '\n' || c == '\r') break;
            printf("   %.3f\n", ctx->table[opt].read(ctx->dev_handle));
            vTaskDelay(pdMS_TO_TICKS((uint32_t)delay_ms)); // #TODO: replace with usleep when separated
        }
    }
}

void exec_set(struct CliContext *ctx, StringView *argv, size_t argc, uint8_t flags) {
    if (argc < 2) {
        printf("%s: <option> <value>\n", ERR_CLI_MISSING_ARG);
        return;
    }
    if (argc > 2) {
        printf("%s\n", ERR_CLI_TOO_MANY_ARG);
        return;
    }

    MTOpts opt = lookup_option(&argv[0]);
    if (opt == OPT_COUNT || !ctx->table[opt].writable) {
        printf("%s: %.*s\n", ERR_CLI_INVALID_ARG, (int)argv[0].count, argv[0].data);
        return;
    }
    
    bool raw = (flags & CLI_RAW_FLAG);
    double val = 0.0;
    if (!parse_value(&argv[1], raw != 0, &val)) {
        printf("%s: %.*s\n", ERR_CLI_INVALID_ARG, (int)argv[1].count, argv[1].data);
        return;
    }

    ctx->pending[opt] = (SetBuff){
        .val = val,
        .raw = raw,
        .modified = true,
    };

    printf("   queued %s = %.3f%s\n", option_name(opt), val, raw? " (raw)" : "");
}

void exec_pending(struct CliContext *ctx, StringView *argv, size_t argc, uint8_t flags) {
    if (argc < 1) {
        printf("%s: <option>\n", ERR_CLI_MISSING_ARG);
        return;
    }
    if (argc > 1) {
        printf("%s\n", ERR_CLI_TOO_MANY_ARG);
        return;
    }

    if (sv_scmp(&argv[0], "show")) show_pending(ctx);
    else if (sv_scmp(&argv[0], "apply")) apply_pending(ctx);
    else if (sv_scmp(&argv[0], "clear")) clear_pending(ctx);
    else {
        printf("%s: %.*s\n", ERR_CLI_INVALID_OPT, (int)argv[0].count, argv[0].data);
        return;
    }
}

void exec_quit(struct CliContext *ctx, StringView *argv, size_t argc, uint8_t flags) {
    exit(0);
}

ParsedCommand parse(CliContext *ctx, StringView *argv, size_t argc) {
    ParsedCommand pc = {0};
    if (argc == 0) {
        pc.id = CMD_COUNT;
        return pc;
    }
    pc.id = lookup_cmd(ctx, &argv[0]);
    if (pc.id == CMD_COUNT) return pc;

    for (size_t i = 1; i < argc; i++) {
        if (sv_scmp(&argv[i], "--raw")) {
            pc.flags |= CLI_RAW_FLAG;
            continue;
        }
        if (pc.argc >= CLI_MAX_TOKENS) {
            pc.id = CMD_COUNT;
            return pc;
        }
        pc.argv[pc.argc++] = argv[i];
    }

    return pc;
}

void init_cli(CliContext *ctx) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    ctx->cmdv[CMD_HELP] = (Command){
        .name = sv("help"), .id = CMD_HELP, .exec_fn = exec_help, .help_fn = print_generic_help,
    };
    ctx->cmdv[CMD_READ] = (Command){
        .name = sv("read"), .id = CMD_READ, .exec_fn = exec_read, .help_fn = print_read_help,
    };
    ctx->cmdv[CMD_MONITOR] = (Command){
        .name = sv("monitor"), .id = CMD_MONITOR, .exec_fn = exec_monitor, .help_fn = print_monitor_help,
    };
    ctx->cmdv[CMD_SET] = (Command){
        .name = sv("set"), .id = CMD_SET, .exec_fn = exec_set, .help_fn = print_set_help,
    };
    ctx->cmdv[CMD_PENDING] = (Command){
        .name = sv("pending"), .id = CMD_PENDING, .exec_fn = exec_pending, .help_fn = print_pending_help,
    };
    ctx->cmdv[CMD_EXIT] = (Command){
        .name = sv("exit"), .id = CMD_EXIT, .exec_fn = exec_quit, .help_fn = NULL,
    };
    ctx->cmdv[CMD_QUIT] = (Command){
        .name = sv("quit"), .id = CMD_QUIT, .exec_fn = exec_quit, .help_fn = NULL,
    };

    ctx->table[OPT_UVW_MUX] = (MTEntry){
        .name = sv("UVW_MUX"), .read_raw = &mt6701_uvw_mux_read_raw, .read = &mt6701_uvw_mux_read, .write_raw = &mt6701_uvw_mux_write_raw, .write = &mt6701_uvw_mux_write, .readable = true, .writable = true,
    };
    ctx->table[OPT_ABZ_MUX] = (MTEntry){
        .name = sv("ABZ_MUX"), .read_raw = &mt6701_abz_mux_read_raw, .read = &mt6701_abz_mux_read, .write_raw = &mt6701_abz_mux_write_raw, .write = &mt6701_abz_mux_write, .readable = true, .writable = true,
    };
    ctx->table[OPT_DIR] = (MTEntry){
        .name = sv("DIR"), .read_raw = &mt6701_dir_read_raw, .read = &mt6701_dir_read, .write_raw = &mt6701_dir_write_raw, .write = &mt6701_dir_write, .readable = true, .writable = true,
    };
    ctx->table[OPT_UVW_RES] = (MTEntry){
        .name = sv("UVW_RES"), .read_raw = &mt6701_uvw_res_read_raw, .read = &mt6701_uvw_res_read, .write_raw = &mt6701_uvw_res_write_raw, .write = &mt6701_uvw_res_write, .readable = true, .writable = true,
    };
    ctx->table[OPT_ABZ_RES] = (MTEntry){
        .name = sv("ABZ_RES"), .read_raw = &mt6701_abz_res_read_raw, .read = &mt6701_abz_res_read, .write_raw = &mt6701_abz_res_write_raw, .write = &mt6701_abz_res_write, .readable = true, .writable = true,
    };
    ctx->table[OPT_HYST] = (MTEntry){
        .name = sv("HYST"), .read_raw = &mt6701_hyst_read_raw, .read = &mt6701_hyst_read, .write_raw = &mt6701_hyst_write_raw, .write = &mt6701_hyst_write, .readable = true, .writable = true,
    };
    ctx->table[OPT_Z_PULSE_WIDTH] = (MTEntry){
        .name = sv("Z_PULSE_WIDTH"), .read_raw = &mt6701_z_pulse_width_read_raw, .read = &mt6701_z_pulse_width_read, .write_raw = &mt6701_z_pulse_width_write_raw, .write = &mt6701_z_pulse_width_write, .readable = true, .writable = true,
    };
    ctx->table[OPT_ZERO] = (MTEntry){
        .name = sv("ZERO"), .read_raw = &mt6701_zero_read_raw, .read = &mt6701_zero_read, .write_raw = &mt6701_zero_write_raw, .write = &mt6701_zero_write, .readable = true, .writable = true,
    };
    ctx->table[OPT_PWM_FREQ] = (MTEntry){
        .name = sv("PWM_FREQ"), .read_raw = &mt6701_pwm_freq_read_raw, .read = &mt6701_pwm_freq_read, .write_raw = &mt6701_pwm_freq_write_raw, .write = &mt6701_pwm_freq_write, .readable = true, .writable = true,
    };
    ctx->table[OPT_PWM_POL] = (MTEntry){
        .name = sv("PWM_POL"), .read_raw = &mt6701_pwm_pol_read_raw, .read = &mt6701_pwm_pol_read, .write_raw = &mt6701_pwm_pol_write_raw, .write = &mt6701_pwm_pol_write, .readable = true, .writable = true,
    };
    ctx->table[OPT_OUT_MODE] = (MTEntry){
        .name = sv("OUT_MODE"), .read_raw = &mt6701_out_mode_read_raw, .read = &mt6701_out_mode_read, .write_raw = &mt6701_out_mode_write_raw, .write = &mt6701_out_mode_write, .readable = true, .writable = true,
    };
    ctx->table[OPT_A_START] = (MTEntry){
        .name = sv("A_START"), .read_raw = &mt6701_a_start_read_raw, .read = &mt6701_a_start_read, .write_raw = &mt6701_a_start_write_raw, .write = &mt6701_a_start_write, .readable = true, .writable = true,
    };
    ctx->table[OPT_A_STOP] = (MTEntry){
        .name = sv("A_STOP"), .read_raw = &mt6701_a_stop_read_raw, .read = &mt6701_a_stop_read, .write_raw = &mt6701_a_stop_write_raw, .write = &mt6701_a_stop_write, .readable = true, .writable = true,
    };
    ctx->table[OPT_SENSOR] = (MTEntry){
        .name = sv("SENSOR"), .read_raw = &mt6701_sensor_read_raw, .read = &mt6701_sensor_read, .write_raw = NULL, .write = NULL, .readable = true, .writable = false,
    };
    ctx->table[OPT_EEPROM] = (MTEntry){ // #TODO: change eeprom read bicycle
        .name = sv("EEPROM"), .read_raw = NULL, .read = &mt6701_log_eeprom, .write_raw = NULL, .write = NULL, .readable = true, .writable = false,
    };
}

void start_cli(i2c_master_dev_handle_t dev_handle) {
    CliContext ctx = {0};
    ctx.dev_handle = dev_handle;
    init_cli(&ctx);

    char buff[CLI_MAX_STR_LEN];
    size_t len = 0;

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    printf("[mt6701]> ");
    while (1) {
        int c = getchar();
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else if (c == '\r' || c == '\n') {
            putchar('\n');
            if (len > 0) {
                buff[len] = '\0';

                StringView argv[CLI_MAX_TOKENS];
                size_t argc = 0;
                if (tokenize(buff, argv, &argc) == 0) {
                    ParsedCommand pc = parse(&ctx, argv, argc);
                    if (pc.id == CMD_COUNT) {
                        printf("   %s: %.*s\n", ERR_CLI_INVALID_CMD, (int)argv[0].count, argv[0].data);
                        printf("   Use \"help\" for a list of available commands\n");
                    }
                    else {
                        ctx.cmdv[pc.id].exec_fn(&ctx, pc.argv, pc.argc, pc.flags);
                    }
                }
            }
            len = 0;
            printf("[mt6701]> ");
        }
        else if (c == '\b' || c == 127) {
            if (len > 0) {
                len--;
                printf("\b \b");
            }
        }
        else if (len < sizeof(buff) - 1) {
            buff[len++] = (char)c;
            putchar(c);
        }
    }
}
