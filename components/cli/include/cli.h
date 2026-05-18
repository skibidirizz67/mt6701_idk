
typedef struct {
    const char *data;
    size_t count;
} StringView;

typedef struct {
    StringView name;
    void (*exec_func)(StringView *, size_t);
} Command;

void start_cli(i2c_master_dev_handle_t adev_handle);