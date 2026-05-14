#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <stddef.h>
#include <stdint.h>

#define I2C_MASTER_SCL_IO 16
#define I2C_MASTER_SDA_IO 17
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TIMEOUT_MS 1000

#define MT6701_ADDR 0x06

#define MT6701_EEPROM_WRITE_WAIT_MS 650

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

uint8_t mt6701_uvw_mux_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x25;
    uint8_t data[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 1));

    // XXXXXXXD
    uint8_t uvw_mux = data[0] & 0x01;

    return uvw_mux;
}

uint8_t mt6701_abz_mux_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x29;
    uint8_t data[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 1));

    // XXXXXXDX
    uint8_t abz_mux = (data[0] & 0x02) >> 1;

    return abz_mux;
}

uint8_t mt6701_dir_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x29;
    uint8_t data[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 1));

    // XDXXXXXX
    uint8_t dir = (data[0] & 0x40) >> 6;

    return dir;
}

uint8_t mt6701_uvw_res_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x30;
    uint8_t data[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 1));

    // XXXXDDDD
    uint8_t uvw_res = data[0] & 0x0F;
    uvw_res += 1;

    return uvw_res;
}

uint16_t mt6701_abz_res_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x30;
    uint8_t data[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 2));

    // DDXXXXXX DDDDDDDD
    uint16_t abz_res = (((uint16_t)(data[0] & 0x03) << 8) | data[1]);
    abz_res += 1;

    return abz_res;
}

double mt6701_hyst_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x32;
    uint8_t data[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 3));

    // XXXXXXXD XXXXXXXX XXXXXXDD
    uint8_t hyst_code = ((data[0] & 0x01) << 2) | (data[2] & 0x03);
    
    double hyst = -1.0;
    switch (hyst_code) {
        case 0:
            hyst = 1.0;
            break;
        case 1:
            hyst = 2.0;
            break;
        case 2:
            hyst = 4.0;
            break;
        case 3:
            hyst = 8.0;
            break;
        case 4:
            hyst = 0.0;
            break;
        case 5:
            hyst = 0.25;
            break;
        case 6:
            hyst = 0.5;
            break;
        case 7:
            hyst = 1.0;
            break;
    }

    return hyst;
}

uint8_t mt6701_z_pulse_width_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x32;
    uint8_t data[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 1));

    // XXXXDDDX
    uint8_t zpw_code = (data[0] & 0x0E) >> 1;
    
    uint8_t zpw = 0;
    switch (zpw_code) {
        case 0:
            zpw = 1;
            break;
        case 1:
            zpw = 2;
            break;
        case 2:
            zpw = 4;
            break;
        case 3:
            zpw = 8;
            break;
        case 4:
            zpw = 12;
            break;
        case 5:
            zpw = 16;
            break;
        case 6:
            zpw = 180;
            break;
        case 7:
            zpw = 1;
            break;
    }

    return zpw;
}

double mt6701_zero_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x32;
    uint8_t data[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 2));

    // DDDDXXXX DDDDDDDD
    double zero = ((data[0] & 0xF0) >> 4) | data[1];
    zero = zero * 0.078;

    return zero;
}

uint8_t mt6701_pwm_freq_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x38;
    uint8_t data[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 1));

    // XXXXXXXD
    uint8_t pwm_freq = data[0] & 0x01;

    return pwm_freq;
}

uint8_t mt6701_pwm_pol_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x38;
    uint8_t data[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 1));

    // XXXXXXDX
    uint8_t pwm_pol = data[0] & 0x02;

    return pwm_pol;
}

uint8_t mt6701_out_mode_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x38;
    uint8_t data[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 1));

    // XXXXXDXX
    uint8_t out_mode = data[0] & 0x04;

    return out_mode;
}

double mt6701_a_stop_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x3E;
    uint8_t data[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 3));

    // XXXXDDDD XXXXXXXX DDDDDDDD
    double a_stop = ((data[0] & 0x0F) << 8) | data[2];
    a_stop = a_stop * 0.078;

    return a_stop;
}

double mt6701_a_start_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x3E;
    uint8_t data[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data, 2));

    // DDDDXXXX DDDDDDDD
    double a_start = ((data[0] & 0xF0) >> 4) | data[1];
    a_start = a_start * 0.078;

    return a_start;
}

void mt6701_log_eeprom(i2c_master_dev_handle_t dev_handle) {
    ESP_LOGI(TAG,
        "\n=== EEPROM ===\nUVW_MUX: %u\nABZ_MUX: %u\nDIR: %u\nUVW_RES: %u\nABZ_RES: %u\nHYST: %.2f\nZ_PULSE_WIDTH: %u\nZERO: %.3f\nPWM_FREQ: %u\nPWM_POL: %u\nOUT_MODE: %u\nA_STOP_READ: %.3f\nA_START_READ: %.3f",
        mt6701_uvw_mux_read(dev_handle),
        mt6701_abz_mux_read(dev_handle),
        mt6701_dir_read(dev_handle),
        mt6701_uvw_res_read(dev_handle),
        mt6701_abz_res_read(dev_handle),
        mt6701_hyst_read(dev_handle),
        mt6701_z_pulse_width_read(dev_handle),
        mt6701_zero_read(dev_handle),
        mt6701_pwm_freq_read(dev_handle),
        mt6701_pwm_pol_read(dev_handle),
        mt6701_out_mode_read(dev_handle),
        mt6701_a_stop_read(dev_handle),
        mt6701_a_start_read(dev_handle)
    );
}

void mt6701_eeprom_write_wait() {
    vTaskDelay(pdMS_TO_TICKS(MT6701_EEPROM_WRITE_WAIT_MS));
}

void mt6701_reg_write(i2c_master_dev_handle_t dev_handle, uint8_t *data, size_t len) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(dev_handle, data, len, I2C_MASTER_TIMEOUT_MS));
    
    uint8_t prog_key[2] = {0x09, 0xB3};
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(dev_handle, prog_key, 2, I2C_MASTER_TIMEOUT_MS));
    uint8_t prog_cmd[2] = {0x0A, 0x05};
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(dev_handle, prog_cmd, 2, I2C_MASTER_TIMEOUT_MS));

    mt6701_eeprom_write_wait();
}

void mt6701_abz_res_write(i2c_master_dev_handle_t dev_handle, uint16_t data) {
    uint8_t reg_addr = 0x30;
    uint8_t data_old[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, data_old, 2));

    data = data - 1;
    uint8_t ms_byte = (uint8_t)(data >> 8);
    uint8_t ls_byte = (uint8_t)(data & 0xFF);

    uint8_t data_new[3] = {
        reg_addr,
        ms_byte,
        ls_byte
    };

    mt6701_reg_write(dev_handle, data_new, 3);
}

void app_main(void) {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);

    mt6701_log_eeprom(dev_handle);

    //uint16_t res = mt6701_abz_res_read(dev_handle);
    //ESP_LOGI(TAG, "%u", res);
    //
    //mt6701_eeprom_write_wait();
    //
    //mt6701_abz_res_write(dev_handle, (uint16_t)1024);

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
}