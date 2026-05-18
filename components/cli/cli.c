#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include "driver/i2c_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "fcntl.h"
#include "i2c_mt6701.h"

#include "cli.h"

const char *err_too_many_args = "   Too many arguments";
const char *err_invalid_arg = "   Invalid argument";
const char *err_missing_arg = "   Missing argument";

i2c_master_dev_handle_t dev_handle;

MTEntry mttable[CONFIG_CNT];

StringView sv(const char *str) {
    return (StringView) {
        .data = str,
        .count = strlen(str),
    };
}

int sv_cmp(StringView *sv0, StringView *sv1) {
    size_t i = 0;
    if (sv0->count != sv1->count) return 0;
    while (i < sv0->count && i < sv1->count) {
        if (sv0->data[i] != sv1->data[i]) return 0;
        i++;
    }
    return 1;
}

int sv_scmp(StringView *sv0, const char *str) {
    StringView sv1 = sv(str);
    return sv_cmp(sv0, &sv1);
}

int sv_isnumeric(StringView *sv) {
    if (sv->count == 0) return 0;
    for (size_t i = 0; i < sv->count; i++) {
        if (!isdigit((unsigned char)sv->data[i])) {
            return 0;
        }
    }
    return 1;
}

int sv_isfloat(StringView *sv) {
    if (sv->count == 0) return 0;
    for (size_t i = 0; i < sv->count; i++) {
        if (!isdigit((unsigned char)sv->data[i]) && sv->data[i] != '.') {
            return 0;
        }
    }
    return 1;
}

long sv_tol(StringView *sv) {
    char buff[64];
    size_t n = sv->count < sizeof(buff) - 1 ? sv->count : sizeof(buff) - 1;

    memcpy(buff, sv->data, n);
    buff[n] = '\0';

    return strtol(buff, NULL, 10);
}

long sv_tolh(StringView *sv) {
    char buff[64];
    char *endptr;
    long out;
    size_t n = sv->count < sizeof(buff) - 1 ? sv->count : sizeof(buff) - 1;

    memcpy(buff, sv->data, n);
    buff[n] = '\0';

    out = strtol(buff, &endptr, 16);

    if ((endptr == buff) || ((out == LONG_MAX || out == LONG_MIN) && errno == ERANGE)) {
        return -1;
    }

    return out;
}

double sv_tod(StringView *sv) {
    char buff[64];
    size_t n = sv->count < sizeof(buff) - 1 ? sv->count : sizeof(buff) - 1;

    memcpy(buff, sv->data, n);
    buff[n] = '\0';

    return strtod(buff, NULL);
}

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

MTOpts lu_se(StringView *sv) {
    if (sv_scmp(sv, "sensor")) return SENSOR;
    else if (sv_scmp(sv, "EEPROM")) return EEPROM;
    else if (sv_scmp(sv, "UVW_MUX")) return UVW_MUX;
    else if (sv_scmp(sv, "ABZ_MUX")) return ABZ_MUX;
    else if (sv_scmp(sv, "DIR")) return SDIR;
    else if (sv_scmp(sv, "UVW_RES")) return UVW_RES;
    else if (sv_scmp(sv, "ABZ_RES")) return ABZ_RES;
    else if (sv_scmp(sv, "HYST")) return HYST;
    else if (sv_scmp(sv, "Z_PULSE_WIDTH")) return Z_PULSE_WIDTH;
    else if (sv_scmp(sv, "ZERO")) return ZERO;
    else if (sv_scmp(sv, "PWM_FREQ")) return PWM_FREQ;
    else if (sv_scmp(sv, "PWM_POL")) return PWM_POL;
    else if (sv_scmp(sv, "OUT_MODE")) return OUT_MODE;
    else if (sv_scmp(sv, "A_START")) return A_START;
    else if (sv_scmp(sv, "A_STOP")) return A_STOP;
    return 0;
}

void exec_help(StringView *argv, size_t argc) {
    StringView *option = NULL;
    size_t col;

    if (argc > 1) {
        printf("%s\n", err_too_many_args);
        return;
    }
    else if (argc > 0) {
        option = &argv[0];
    }

    if (!option) {
        col = 35;
        printpad("help", col);
        printf("Display this message\n");
        printpad("read <option> [--raw]", col);
        printf("Read encoder value (either sensor or EEPROM configuration)\n");
        printpad("monitor <sensor> <delay_ms> [--raw]", col);
        printf("Monitor encoder sensor\n");
        printpad("set <option> <value> [--raw]", col);
        printf("Set EEPROM configuration value\n");
        printpad("exit", col);
        printf("Quit program\n");
        printpad("quit", col);
        printf("Quit program\n");
        printf("\n   Use \"help [command]\" to display command-specific help\n");
    }
    else if (sv_scmp(option, "read")) {
        col = 10;
        printf(" Read the sensor value or the EEPROM configuration value.\n");
        printf(" Usage: read <option> [--raw]\n");
        printf(" Example: read ABZ_RES\n");
        printpad("option", col);
        printf("Sensor or EEPROM configuration option\n");
        printpad("--raw", col);
        printf("Display raw hex value\n");
        col = 15;
        printf("\n Available options:\n");
        printpad("sensor", col);
        printf("Angle position encoder sensor\n");
        printpad("EEPROM", col);
        printf("Display whole EEPROM\n");
        printpad("UVW_MUX", col);
        printf("UVW output type\n");
        printpad("ABZ_MUX", col);
        printf("ABZ output type\n");
        printpad("DIR", col);
        printf("Output rotation direction\n");
        printpad("UVW_RES", col);
        printf("UVW output resolution\n");
        printpad("ABZ_RES", col);
        printf("ABZ output resolution (PPR)\n");
        printpad("HYST", col);
        printf("Hysteresis filter parameter\n");
        printpad("Z_PULSE_WIDTH", col);
        printf("Z pulse width\n");
        printpad("ZERO", col);
        printf("Zero-degree position\n");
        printpad("PWM_FREQ", col);
        printf("PWM frame frequency\n");
        printpad("PWM_POL", col);
        printf("PWM polarity\n");
        printpad("OUT_MODE", col);
        printf("Out pin mode\n");
        printpad("A_START", col);
        printf("Start-point of analog output\n");
        printpad("A_STOP", col);
        printf("Stop-point of analog output\n");
    }
    else if (sv_scmp(option, "monitor")) {
        col = 10;
        printf(" Monitor sensor value continiously. Press Enter to stop.\n");
        printf(" Usage: monitor <sensor> <delay_ms> [--raw]\n");
        printf(" Example: monitor sensor 500\n");
        printpad("sensor", col);
        printf("Angle position encoder sensor\n");
        printpad("--raw", col);
        printf("Display raw hex value\n");
        col = 15;
        printf("\n Available sensors:\n");
        printpad("sensor", col);
        printf("Angle position encoder sensor\n");
    }
    else if (sv_scmp(option, "set")) {
        col = 10;
        printf(" Set EEPROM configuration value.\n");
        printf(" Usage: set <option> <value> [--raw]\n");
        printf(" Example: set A_START 180.5\n");
        printf(" Example: set HYST 0x04 --raw\n");
        printf(" Example: set OUT_MODE 0\n");
        printpad("option", col);
        printf("EEPROM configuration option\n");
        printpad("value", col);
        printf("Value to set. If value is out of range, it will be truncated or rounded to the nearest valid value. For some options, like OUT_MODE, only raw set is available (i.e. number)\n");
        printpad("--raw", col);
        printf("Set value in raw hex withour processing.\n");
        col = 15;
        printf("\n Available options:\n");
        printpad("UVW_MUX", col);
        printf("UVW output type (UVW, -A-B-Z)\n");
        printpad("ABZ_MUX", col);
        printf("ABZ output type (ABZ, UVW)\n");
        printpad("DIR", col);
        printf("Output rotation direction (CCW, CW)\n");
        printpad("UVW_RES", col);
        printf("UVW output resolution (1,2..,15,16)\n");
        printpad("ABZ_RES", col);
        printf("ABZ output resolution (PPR) (1,2..,1023,1024)\n");
        printpad("HYST", col);
        printf("Hysteresis filter parameter (1,2,4,8,0,0.25,0.5,1)\n");
        printpad("Z_PULSE_WIDTH", col);
        printf("Z pulse width (1,2,4,8,12,16,180,1)\n");
        printpad("ZERO", col);
        printf("Zero-degree position (0.0,0.088,..,359.824,359.912)\n");
        printpad("PWM_FREQ", col);
        printf("PWM frame frequency (994.4,497.2)\n");
        printpad("PWM_POL", col);
        printf("PWM polarity (high,low)\n");
        printpad("OUT_MODE", col);
        printf("Out pin mode (analog,PWM)\n");
        printpad("A_START", col);
        printf("Start-point of analog output (0.0,0.088,..,359.824,359.912)\n");
        printpad("A_STOP", col);
        printf("Stop-point of analog output (0.0,0.088,..,359.824,359.912)\n");
    }
}

void exec_read(StringView *argv, size_t argc) {
    StringView *opt = NULL;
    int raw = 0;

    if (argc == 0) {
        printf("%s: %s\n", err_missing_arg, "<option>");
        return;
    }
    else if (argc > 2) {
        printf("%s\n", err_too_many_args);
        return;
    }
    
    opt = &argv[0];
    if (argc == 2) {
        if (sv_scmp(&argv[1], "--raw")) raw = 1;
        else {
            printf("%s: %.*s\n", err_invalid_arg, argv[1].count, argv[1].data);
            return;
        }
    }

    if (raw) {
        MTOpts mtopt = lu_se(opt);
        if (mtopt == ERR_INVALID_OPT) {
            printf("%s: %.*s\n", err_invalid_arg, opt->count, opt->data);
            return;
        }
        printf("   0x%04X\n", (uint16_t)mttable[lu_se(opt)].read_raw(dev_handle));
    }
    else {
        MTOpts mtopt = lu_se(opt);
        if (mtopt == ERR_INVALID_OPT) {
            printf("%s: %.*s\n", err_invalid_arg, opt->count, opt->data);
            return;
        }
        printf("   %f.3\n", mttable[lu_se(opt)].read(dev_handle));
    }
}

void exec_monitor(StringView *argv, size_t argc) {
    StringView *opt = NULL;
    long delay_ms = 500;
    int raw = 0;
    uint16_t (*read_raw_func)(i2c_master_dev_handle_t) = NULL;
    double (*read_func)(i2c_master_dev_handle_t) = NULL;

    if (argc < 1) {
        printf("%s: %s\n", err_missing_arg, "<sensor>");
        return;
    }
    else if (argc > 3) {
        printf("%s\n", err_too_many_args);
        return;
    }
    
    opt = &argv[0];
    for (size_t i = 1; i < argc; i++) {
        if (sv_scmp(&argv[i], "--raw")) {
            raw = 1;
        }
        else if (sv_isnumeric(&argv[i])) {
            delay_ms = sv_tol(&argv[i]);
        }
        else {
            printf("%s: %.*s\n", err_invalid_arg, argv[i].count, argv[i].data);
            return;
        }
    }

    if (sv_scmp(opt, "sensor")) {
        if (raw) {
            read_raw_func = &mt6701_sensor_read_raw;
        }
        else {
            read_func = &mt6701_sensor_read;
        }
    }
    else {
        printf("%s: %.*s\n", err_invalid_arg, opt->count, opt->data);
        return;
    }

    while (1) {
        int c = getchar();
        if (c == '\n' || c == '\r') {
            break;
        }

        // read_raw_func();
        if (raw) {
            printf("   0x%04X\n", read_raw_func(dev_handle));
        }
        else {
            printf("   %f\n", read_func(dev_handle));
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void exec_set(StringView *argv, size_t argc) {
    StringView *opt = NULL;
    StringView *val = NULL;
    StringView slice = {0};
    int raw = 0;
    double vald = 0;

    if (argc < 1) {
        printf("%s: <option>\n", err_missing_arg);
        return;
    }
    else if (argc > 3) {
        printf("%s\n", err_too_many_args);
        return;
    }

    opt = &argv[0];
    
    if (argc > 1) {
        val = &argv[1];
    }

    if (argc == 3) {
        if (sv_scmp(&argv[2], "--raw")) {
            raw = 1;
        }
        else {
            printf("%s: %.*s\n", err_invalid_arg, argv[2].count, argv[2].data);
            return;
        }
    }

    if (sv_scmp(opt, "apply")) {
        for (size_t i = 0; i < CONFIG_CNT; i++) {
            SetBuff *ewbo = &mttable[i].ewb;
            if (ewbo->mod == 1) {
                MTOpts mtopt = lu_se(opt);
                if (raw) {
                    if (mtopt == ERR_INVALID_OPT) {
                        printf("%s: %.*s\n", err_invalid_arg, opt->count, opt->data);
                        return;
                    }
                    mttable[lu_se(opt)].write_raw(dev_handle, (uint16_t)ewbo->v);
                }
                else {
                    if (mtopt == ERR_INVALID_OPT) {
                        printf("%s: %.*s\n", err_invalid_arg, opt->count, opt->data);
                        return;
                    }
                    mttable[lu_se(opt)].write_raw(dev_handle, ewbo->v);
                }
            }
        }
        return;
    }

    if (!val) {
        printf("%s: <value>\n", err_missing_arg);
        return;
    }

    slice = *val;
    if (raw && slice.count > 2) {
        slice.count = 2;
        if (sv_scmp(&slice, "0x")) {
            val->data = val->data + 2;
            val->count = val->count - 2;
            printf("%d\n", val->count);
            vald = (double)sv_tolh(val);
            if (vald < 0) {
                printf("%s: %.*s\n", err_invalid_arg, val->count, val->data);
                return;
            }
        }
        else {
            printf("%s: %.*s\n", err_invalid_arg, val->count, val->data);
            return;
        }
    }
    else if (sv_isfloat(val)) {
        vald = sv_tod(val);
    }
    else {
        printf("%s: %.*s\n", err_invalid_arg, val->count, val->data);
        return;
    }

    MTOpts mtopt = lu_se(opt);
    mttable[mtopt].ewb = (SetBuff){
        .v = vald,
        .raw = raw,
        .mod = 1,
    };
}

void exec_quit(StringView *argv, size_t argc) {
    exit(0);
}

void parse(const char *str, Command *cmdv, size_t cmdc) {
    static const char *ws = " \t";
    StringView *a;
    StringView argv[16];
    size_t argc = 0;
    void (*exec_func)(StringView *, size_t) = NULL;
    StringView cmd =  {
        .data = str,
        .count = strcspn(str, ws),
    };
    str += cmd.count;

    while (*str) {
        str += strspn(str, ws);
        if (!*str) break;
        if (argc > 16) {
            printf("   Exceeded argument limit: 16\n");
            return;
        }

        a = &argv[argc];
        a->data = str;
        a->count = strcspn(str, ws);

        str += a->count;
        argc++;
    }

    for (size_t i = 0; i < cmdc; i++) {
        if (sv_cmp(&cmdv[i].name, &cmd)) {
            exec_func = cmdv[i].exec_func;
            break;
        }
    }

    if (!exec_func) {
        printf("   Invalid command: %.*s\n\n", cmd.count, cmd.data);
        printf("   Use \"help\" for a list of available commands\n");
    }
    else {
        exec_func(argv, argc);
    }
}

void start_cli(i2c_master_dev_handle_t adev_handle) {
    Command cmdv[] = {
        {
            .name = sv("help"),
            .exec_func = &exec_help,
        },
        {
            .name = sv("read"),
            .exec_func = &exec_read,
        },
        {
            .name = sv("monitor"),
            .exec_func = &exec_monitor,
        },
        {
            .name = sv("set"),
            .exec_func = &exec_set,
        },
        {
            .name = sv("exit"),
            .exec_func = &exec_quit,
        },
        {
            .name = sv("quit"),
            .exec_func = &exec_quit,
        },
    };
    size_t cmdc = sizeof(cmdv) / sizeof(cmdv[0]);

    mttable[UVW_MUX] = (MTEntry){
        .name = sv("UVW_MUX"),
        .read_raw = &mt6701_uvw_mux_read_raw,
        .read = &mt6701_uvw_mux_read,
        .write_raw = &mt6701_uvw_mux_write_raw,
        .write = &mt6701_uvw_mux_write,
    };
    mttable[ABZ_MUX] = (MTEntry){
        .name = sv("ABZ_MUX"),
        .read_raw = &mt6701_abz_mux_read_raw,
        .read = &mt6701_abz_mux_read,
        .write_raw = &mt6701_abz_mux_write_raw,
        .write = &mt6701_abz_mux_write,
    };
    mttable[SDIR] = (MTEntry){
        .name = sv("DIR"),
        .read_raw = &mt6701_dir_read_raw,
        .read = &mt6701_dir_read,
        .write_raw = &mt6701_dir_write_raw,
        .write = &mt6701_dir_write,
    };
    mttable[UVW_RES] = (MTEntry){
        .name = sv("UVW_RES"),
        .read_raw = &mt6701_uvw_res_read_raw,
        .read = &mt6701_uvw_res_read,
        .write_raw = &mt6701_uvw_res_write_raw,
        .write = &mt6701_uvw_res_write,
    };
    mttable[ABZ_RES] = (MTEntry){
        .name = sv("ABZ_RES"),
        .read_raw = &mt6701_abz_res_read_raw,
        .read = &mt6701_abz_res_read,
        .write_raw = &mt6701_abz_res_write_raw,
        .write = &mt6701_abz_res_write,
    };
    mttable[HYST] = (MTEntry){
        .name = sv("HYST"),
        .read_raw = &mt6701_hyst_read_raw,
        .read = &mt6701_hyst_read,
        .write_raw = &mt6701_hyst_write_raw,
        .write = &mt6701_hyst_write,
    };
    mttable[Z_PULSE_WIDTH] = (MTEntry){
        .name = sv("Z_PULSE_WIDTH"),
        .read_raw = &mt6701_z_pulse_width_read_raw,
        .read = &mt6701_z_pulse_width_read,
        .write_raw = &mt6701_z_pulse_width_write_raw,
        .write = &mt6701_z_pulse_width_write,
    };
    mttable[ZERO] = (MTEntry){
        .name = sv("ZERO"),
        .read_raw = &mt6701_zero_read_raw,
        .read = &mt6701_zero_read,
        .write_raw = &mt6701_zero_write_raw,
        .write = &mt6701_zero_write,
    };
    mttable[PWM_FREQ] = (MTEntry){
        .name = sv("PWM_FREQ"),
        .read_raw = &mt6701_pwm_freq_read_raw,
        .read = &mt6701_pwm_freq_read,
        .write_raw = &mt6701_pwm_freq_write_raw,
        .write = &mt6701_pwm_freq_write,
    };
    mttable[PWM_POL] = (MTEntry){
        .name = sv("PWM_POL"),
        .read_raw = &mt6701_pwm_pol_read_raw,
        .read = &mt6701_pwm_pol_read,
        .write_raw = &mt6701_pwm_pol_write_raw,
        .write = &mt6701_pwm_pol_write,
    };
    mttable[OUT_MODE] = (MTEntry){
        .name = sv("OUT_MODE"),
        .read_raw = &mt6701_out_mode_read_raw,
        .read = &mt6701_out_mode_read,
        .write_raw = &mt6701_out_mode_write_raw,
        .write = &mt6701_out_mode_write,
    };
    mttable[A_START] = (MTEntry){
        .name = sv("A_START"),
        .read_raw = &mt6701_a_start_read_raw,
        .read = &mt6701_a_start_read,
        .write_raw = &mt6701_a_start_write_raw,
        .write = &mt6701_a_start_write,
    };
    mttable[A_STOP] = (MTEntry){
        .name = sv("A_STOP"),
        .read_raw = &mt6701_a_stop_read_raw,
        .read = &mt6701_a_stop_read,
        .write_raw = &mt6701_a_stop_write_raw,
        .write = &mt6701_a_stop_write,
    };
    mttable[SENSOR] = (MTEntry){
        .name = sv("SENSOR"),
        .read_raw = &mt6701_sensor_read_raw,
        .read = &mt6701_sensor_read,
    };

    char buff[256];
    size_t cap = 0;
    size_t len = 0;

    dev_handle = adev_handle;

    for (size_t i = 0; i < CONFIG_CNT; i++) {
        mttable[i].ewb = (SetBuff){0};
    }

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    printf("[mt6701]> ");

    while (1) {
        int c = getchar();
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == '\r' || c == '\n') {
            putchar('\n');
            buff[len] = '\0';
            if (len > 0) {
                parse(buff, cmdv, cmdc);
            }
            len = 0;
            printf("[mt6701]> ");
            continue;
        }
        if (c == '\b' || c == 127) {
            if (len > 0) {
                len--;
                printf("\b \b");
            }
            continue;
        }
        if (len < sizeof(buff) - 1) {
            buff[len++] = (char)c;
            putchar(c);
        }
    }
}
