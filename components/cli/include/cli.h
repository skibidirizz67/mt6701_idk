#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <ctype.h>
#include <limits.h>
#include "driver/i2c_types.h"

typedef struct {
    const char *data;
    size_t count;
} StringView;

typedef struct {
    StringView name;
    void (*exec_func)(StringView *, size_t);
} Command;

typedef enum {
    UVW_MUX,
    ABZ_MUX,
    SDIR,
    UVW_RES,
    ABZ_RES,
    HYST,
    Z_PULSE_WIDTH,
    ZERO,
    PWM_FREQ,
    PWM_POL,
    OUT_MODE,
    A_START,
    A_STOP,
    SENSOR,
    EEPROM,
    CONFIG_CNT,
    ERR_INVALID_OPT,
} MTOpts;

typedef struct {
    double v;
    int raw, mod;
} SetBuff;

typedef struct {
    StringView name;
    SetBuff ewb;

    uint16_t (*read_raw)(i2c_master_dev_handle_t);
    double (*read)(i2c_master_dev_handle_t);

    void (*write)(i2c_master_dev_handle_t, double);
    void (*write_raw)(i2c_master_dev_handle_t, uint16_t);
} MTEntry;

void start_cli(i2c_master_dev_handle_t adev_handle);