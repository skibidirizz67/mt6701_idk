#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <stddef.h>
#include <stdint.h>

#define I2C_MASTER_SCL_IO 16
#define I2C_MASTER_SDA_IO 17
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TIMEOUT_MS 1000

#define MT6701_ADDR 0x06

static const char* TAG = "MT6701";

static esp_err_t mt6701_reg_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle) {
    i2c_master_bus_config_t bus_cfg= {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MT6701_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_cfg, dev_handle));
    ESP_ERROR_CHECK(i2c_master_probe(*bus_handle, MT6701_ADDR, I2C_MASTER_TIMEOUT_MS));
}

uint16_t mt6701_abz_res_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x30;
    uint8_t data[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 2));

    uint16_t resolution = (((uint16_t)(data[0] & 0x03) << 8) | data[1]);
    resolution += 1;

    return resolution;
}

void app_main(void) {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    
    uint16_t resolution = mt6701_abz_res_read(dev_handle);
    ESP_LOGI(TAG, "%d", resolution);

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
}