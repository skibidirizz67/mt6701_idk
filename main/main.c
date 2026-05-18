#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "cli.h"
#include "i2c_mt6701.h"

void app_main(void) {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_mt6701_init(&bus_handle, &dev_handle);

    start_cli(dev_handle);

    i2c_mt6701_terminate(&dev_handle, &bus_handle);
}