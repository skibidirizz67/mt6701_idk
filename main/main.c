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

static esp_err_t mt6701_reg_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *buff, size_t size) {
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, buff, size, I2C_MASTER_TIMEOUT_MS);
}

void mt6701_eeprom_write_wait() {
    vTaskDelay(pdMS_TO_TICKS(MT6701_EEPROM_WRITE_WAIT_MS));
}

void mt6701_reg_write(i2c_master_dev_handle_t dev_handle, uint8_t *buff, size_t size) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(dev_handle, buff, size, I2C_MASTER_TIMEOUT_MS));
    
    uint8_t prog_key[2] = {0x09, 0xB3};
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(dev_handle, prog_key, 2, I2C_MASTER_TIMEOUT_MS));
    uint8_t prog_cmd[2] = {0x0A, 0x05};
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(dev_handle, prog_cmd, 2, I2C_MASTER_TIMEOUT_MS));

    mt6701_eeprom_write_wait();
}

uint8_t mt6701_uvw_mux_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x25;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff, 1));

    // XXXXXXXD
    uint8_t out = buff[0] & 0x01;

    return out;
}

void mt6701_uvw_mux_write(i2c_master_dev_handle_t dev_handle, uint8_t value) {
    uint8_t reg_addr = 0x25;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff + 1, 1));

    buff[0] = reg_addr;
    // XXXXXXXD
    buff[1] = (buff[1] & 0xFE) | (value & 0x01);

    mt6701_reg_write(dev_handle, buff, 2);
}

uint8_t mt6701_abz_mux_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x29;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff, 1));

    // XXXXXXDX
    uint8_t out = (buff[0] & 0x02) >> 1;

    return out;
}

void mt6701_abz_mux_write(i2c_master_dev_handle_t dev_handle, uint8_t value) {
    uint8_t reg_addr = 0x29;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff + 1, 1));

    buff[0] = reg_addr;
    // XXXXXXDX
    buff[1] = (buff[1] & 0xFD) | (value & 0x02);

    mt6701_reg_write(dev_handle, buff, 2);
}

uint8_t mt6701_dir_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x29;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff, 1));

    // XDXXXXXX
    uint8_t out = (buff[0] & 0x40) >> 6;

    return out;
}

void mt6701_dir_write(i2c_master_dev_handle_t dev_handle, uint8_t value) {
    uint8_t reg_addr = 0x29;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff + 1, 1));

    buff[0] = reg_addr;
    // XDXXXXXX
    buff[1] = (buff[1] & 0xBF) | ((value & 0x40) << 6);

    mt6701_reg_write(dev_handle, buff, 2);
}

uint8_t mt6701_uvw_res_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x30;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff, 1));

    // XXXXDDDD
    uint8_t out = buff[0] & 0x0F;
    out = out + 1;

    return out;
}

void mt6701_uvw_res_write(i2c_master_dev_handle_t dev_handle, uint8_t value) {
    uint8_t reg_addr = 0x30;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff + 1, 1));

    buff[0] = reg_addr;
    // XXXXDDDD
    buff[1] = (buff[1] & 0xF0) | (value & 0x0F);

    mt6701_reg_write(dev_handle, buff, 2);
}

uint16_t mt6701_abz_res_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x30;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff, 2));

    // DDXXXXXX DDDDDDDD
    uint16_t out = (((uint16_t)(buff[0] & 0x03) << 8) | buff[1]);
    out = out + 1;

    return out;
}

void mt6701_abz_res_write(i2c_master_dev_handle_t dev_handle, uint16_t value) {
    uint8_t reg_addr = 0x30;
    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff + 1, 2));

    value = value - 1;

    buff[0] = reg_addr;
    // DDXXXXXX DDDDDDDD
    buff[1] = (buff[1] & 0xFC) | (uint8_t)(((value >> 8) & 0xFF) & 0x03);
    buff[2] = (uint8_t)(value & 0xFF);

    mt6701_reg_write(dev_handle, buff, 3);
}

double mt6701_hyst_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x32;
    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff, 3));

    // XXXXXXXD XXXXXXXX XXXXXXDD
    uint8_t code = ((buff[0] & 0x01) << 2) | (buff[2] & 0x03);
    
    double out = 0.0;
    switch (code) {
        case 0:
            out = 1.0;
            break;
        case 1:
            out = 2.0;
            break;
        case 2:
            out = 4.0;
            break;
        case 3:
            out = 8.0;
            break;
        case 4:
            out = 0.0;
            break;
        case 5:
            out = 0.25;
            break;
        case 6:
            out = 0.5;
            break;
        case 7:
            out = 1.0;
            break;
    }

    return out;
}

// write is split in order to not wear-out unused middle byte
void mt6701_hyst_write(i2c_master_dev_handle_t dev_handle, double value) {
    uint8_t reg_addr = 0x32;
    uint8_t buff[4] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff + 1, 3));

    uint8_t code = 0;

    if (value == 1.0) code = 0;
    else if (value == 2.0) code = 1;
    else if (value == 4.0) code = 2;
    else if (value == 8.0) code = 3;
    else if (value == 0.0) code = 4;
    else if (value == 0.25) code = 5;
    else if (value == 0.5) code = 6;
    else if (value == 1.0) code = 7;

    buff[0] = reg_addr;
    // XXXXXXXD
    buff[1] = (buff[1] & 0xFE) | ((code >> 2) & 0x01);

    mt6701_reg_write(dev_handle, buff, 2);

    buff[0] = reg_addr + 2;
    // XXXXXXDD
    buff[1] = (buff[3] & 0xFC) | (code & 0x03);

    mt6701_reg_write(dev_handle, buff, 2);
}

uint8_t mt6701_z_pulse_width_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x32;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff, 1));

    // XXXXDDDX
    uint8_t code = (buff[0] & 0x0E) >> 1;
    
    uint8_t out = 0;
    switch (code) {
        case 0:
            out = 1;
            break;
        case 1:
            out = 2;
            break;
        case 2:
            out = 4;
            break;
        case 3:
            out = 8;
            break;
        case 4:
            out = 12;
            break;
        case 5:
            out = 16;
            break;
        case 6:
            out = 180;
            break;
        case 7:
            out = 1;
            break;
    }

    return out;
}

void mt6701_z_pulse_width_write(i2c_master_dev_handle_t dev_handle, uint8_t value) {
    uint8_t reg_addr = 0x32;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff + 1, 1));

    uint8_t code = 0;

    if (value == 1) code = 0;
    else if (value == 2) code = 1;
    else if (value == 4) code = 2;
    else if (value == 8) code = 3;
    else if (value == 12) code = 4;
    else if (value == 16) code = 5;
    else if (value == 180) code = 6;
    else if (value == 1) code = 7;

    buff[0] = reg_addr;
    // XXXXDDDX
    buff[1] = (buff[1] & 0xF1) | ((code << 1) & 0x0E);

    mt6701_reg_write(dev_handle, buff, 2);
}

double mt6701_zero_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x32;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff, 2));

    // DDDDXXXX DDDDDDDD
    uint16_t code = ((uint16_t)(buff[0] & 0xF0) >> 4) | buff[1];
    
    double out = code * 0.078;

    return out;
}

void mt6701_zero_write(i2c_master_dev_handle_t dev_handle, double value) {
    uint8_t reg_addr = 0x32;
    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff + 1, 2));

    uint16_t code = (uint16_t)(value / 0.078);

    buff[0] = reg_addr;
    // DDDDXXXX DDDDDDDD
    buff[1] = (buff[1] & 0x3F) | (uint8_t)(((code & 0xFF) << 4) & 0xF0); // #TODO remove 0xFF as redundant
    buff[2] = (uint8_t)(code & 0xFF);

    mt6701_reg_write(dev_handle, buff, 3);
}

uint8_t mt6701_pwm_freq_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x38;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff, 1));

    // XXXXXXXD
    uint8_t out = buff[0] & 0x01;

    return out;
}

void mt6701_pwm_freq_write(i2c_master_dev_handle_t dev_handle, uint8_t value) {
    uint8_t reg_addr = 0x38;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff + 1, 1));

    buff[0] = reg_addr;
    // XXXXXXXD
    buff[1] = (buff[1] & 0xFE) | (value & 0x01);

    mt6701_reg_write(dev_handle, buff, 2);
}

uint8_t mt6701_pwm_pol_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x38;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff, 1));

    // XXXXXXDX
    uint8_t out = buff[0] & 0x02;

    return out;
}

void mt6701_pwm_pol_write(i2c_master_dev_handle_t dev_handle, uint8_t value) {
    uint8_t reg_addr = 0x38;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff + 1, 1));

    buff[0] = reg_addr;
    // XXXXXXDX
    buff[1] = (buff[1] & 0xFD) | (value & 0x02);

    mt6701_reg_write(dev_handle, buff, 2);
}

uint8_t mt6701_out_mode_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x38;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff, 1));

    // XXXXXDXX
    uint8_t out = buff[0] & 0x04;

    return out;
}

void mt6701_out_mode_write(i2c_master_dev_handle_t dev_handle, uint8_t value) {
    uint8_t reg_addr = 0x38;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff + 1, 1));

    buff[0] = reg_addr;
    // XXXXXDXX
    buff[1] = (buff[1] & 0xFB) | (value & 0x04);

    mt6701_reg_write(dev_handle, buff, 2);
}

double mt6701_a_stop_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x3E;
    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff, 3));

    // XXXXDDDD XXXXXXXX DDDDDDDD
    uint16_t code = ((uint8_t)(buff[0] & 0x0F) << 8) | buff[2];

    double out = code * 0.078;

    return out;
}

// write is split in order to not wear-out unused middle byte
void mt6701_a_stop_write(i2c_master_dev_handle_t dev_handle, double value) {
    uint8_t reg_addr = 0x3E;
    uint8_t buff[4] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff + 1, 3));

    uint16_t code = (uint16_t)(value / 0.078);

    buff[0] = reg_addr;
    // XXXXDDDD
    buff[1] = (buff[1] & 0xF0) | ((code >> 8) & 0x0F);

    mt6701_reg_write(dev_handle, buff, 2);

    buff[0] = reg_addr + 2;
    // DDDDDDDD
    buff[1] = buff[3];

    mt6701_reg_write(dev_handle, buff, 2);
}

double mt6701_a_start_read(i2c_master_dev_handle_t dev_handle) {
    uint8_t reg_addr = 0x3E;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff, 2));

    uint16_t code = ((uint16_t)(buff[0] & 0xF0) >> 4) | buff[1];

    // DDDDXXXX DDDDDDDD
    double out = code * 0.078;

    return out;
}

void mt6701_a_start_write(i2c_master_dev_handle_t dev_handle, double value) {
    uint8_t reg_addr = 0x3E;
    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(dev_handle, reg_addr, buff + 1, 2));

    uint16_t code = (uint16_t)(value / 0.078);

    buff[1] = (buff[1] & 0xF0) | ((code >> 8) & 0x0F);

    buff[0] = reg_addr;
    // DDDDXXXX DDDDDDDD
    buff[1] = (buff[1] & 0x0F) | (uint8_t)((code << 4) & 0xF0);
    buff[2] = (uint8_t)(code & 0xFF);

    mt6701_reg_write(dev_handle, buff, 3);
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

void app_main(void) {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);

    mt6701_log_eeprom(dev_handle);

    mt6701_eeprom_write_wait();

    mt6701_a_stop_write(dev_handle, 69.69);

    mt6701_eeprom_write_wait();

    mt6701_log_eeprom(dev_handle);

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
}