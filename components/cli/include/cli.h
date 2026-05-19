#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <ctype.h>
#include <limits.h>
#include "driver/i2c_types.h"

#define CLI_MAX_STR_LEN 256
#define CLI_MAX_TOKENS 16
#define CLI_RAW_FLAG (1u << 0)

#define ERR_CLI_TOO_MANY_ARG "   Too many arguments"
#define ERR_CLI_INVALID_ARG "   Invalid argument"
#define ERR_CLI_MISSING_ARG "   Missing argument"
#define ERR_CLI_INVALID_CMD "   Invalid command"
#define ERR_CLI_INVALID_OPT "   Invalid option"

typedef struct {
    const char *data;
    size_t count;
} StringView;

typedef enum {
    OPT_UVW_MUX,
    OPT_ABZ_MUX,
    OPT_DIR,
    OPT_UVW_RES,
    OPT_ABZ_RES,
    OPT_HYST,
    OPT_Z_PULSE_WIDTH,
    OPT_ZERO,
    OPT_PWM_FREQ,
    OPT_PWM_POL,
    OPT_OUT_MODE,
    OPT_A_START,
    OPT_A_STOP,
    OPT_SENSOR,
    OPT_EEPROM,
    OPT_COUNT
} MTOpts;

typedef enum {
    CMD_HELP,
    CMD_READ,
    CMD_MONITOR,
    CMD_SET,
    CMD_PENDING,
    CMD_EXIT,
    CMD_QUIT,
    CMD_COUNT
} CmdId;

typedef struct {
    double val;
    bool raw;
    bool modified;
} SetBuff;

typedef struct {
    StringView name;
    uint16_t (*read_raw)(i2c_master_dev_handle_t);
    double (*read)(i2c_master_dev_handle_t);
    void (*write_raw)(i2c_master_dev_handle_t, uint16_t);
    void (*write)(i2c_master_dev_handle_t, double);
    bool readable, writable;
} MTEntry;

struct CliContext;

typedef struct {
    StringView name;
    CmdId id;
    void (*exec_fn)(struct CliContext *, StringView *, size_t, uint8_t);
    void (*help_fn)(void);
} Command;

typedef struct CliContext {
    i2c_master_dev_handle_t dev_handle;
    MTEntry table[OPT_COUNT];
    SetBuff pending[OPT_COUNT];
    Command cmdv[CMD_COUNT];
} CliContext;

typedef struct {
    CmdId id;
    StringView argv[CLI_MAX_TOKENS];
    size_t argc;
    unsigned flags;
} ParsedCommand;

void start_cli(i2c_master_dev_handle_t adev_handle);
