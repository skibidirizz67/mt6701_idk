#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "i2c_control.h"

#define I2C_MASTER_SCL_IO 16
#define I2C_MASTER_SDA_IO 17
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TIMEOUT_MS 1000

#define MT6701_ADDR 0x06
#define INA228_ADDR 0x45

#define MT6701_EEPROM_WRITE_WAIT_MS 650

#define CURRENT_LSB 0.000391
#define ADCRANGE 1

static const char *TAG_MT6701 = "MT6701";
static const char *TAG_INA228 = "WINA228";

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t mt6701_handle;
i2c_master_dev_handle_t ina228_handle;

void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *mt6701_handle, i2c_master_dev_handle_t *ina228_handle) {
    i2c_master_bus_config_t bus_cfg= {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, bus_handle));

    i2c_device_config_t mt6701_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MT6701_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &mt6701_cfg, mt6701_handle));
    ESP_ERROR_CHECK(i2c_master_probe(*bus_handle, MT6701_ADDR, I2C_MASTER_TIMEOUT_MS));

    i2c_device_config_t ina228_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA228_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .scl_wait_us = 0,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &ina228_cfg, ina228_handle));
    ESP_ERROR_CHECK(i2c_master_probe(*bus_handle, INA228_ADDR, I2C_MASTER_TIMEOUT_MS));
}

void i2c_init() {
    i2c_master_init(&bus_handle, &mt6701_handle, &ina228_handle);
}

/**************mt6701-specific**************/

esp_err_t mt6701_reg_read(uint8_t reg_addr, uint8_t *buff, size_t size) {
    return i2c_master_transmit_receive(mt6701_handle, &reg_addr, 1, buff, size, I2C_MASTER_TIMEOUT_MS);
}

void mt6701_eeprom_write_wait() {
    vTaskDelay(pdMS_TO_TICKS(MT6701_EEPROM_WRITE_WAIT_MS));
}

void mt6701_reg_write(uint8_t *buff, size_t size) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(mt6701_handle, buff, size, I2C_MASTER_TIMEOUT_MS));
    
    uint8_t prog_key[2] = {0x09, 0xB3};
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(mt6701_handle, prog_key, 2, I2C_MASTER_TIMEOUT_MS));
    uint8_t prog_cmd[2] = {0x0A, 0x05};
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(mt6701_handle, prog_cmd, 2, I2C_MASTER_TIMEOUT_MS));

    mt6701_eeprom_write_wait();
}

uint16_t mt6701_sensor_read_raw() {
    uint8_t reg_addr = 0x03;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 2));
    
    // DDDDDDDD XXDDDDDD
    uint16_t code = ((uint16_t)buff[0] << 6) | (buff[1] & 0x3F);

    return (uint16_t)code;
}
double mt6701_sensor_read() {
    uint16_t code = mt6701_sensor_read_raw();

    double angle = code / 16384.0 * 360.0;

    return angle;
}

uint16_t mt6701_uvw_mux_read_raw() {
    uint8_t reg_addr = 0x25;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 1));

    // XXXXXXXD
    uint8_t out = buff[0] & 0x01;

    return (uint16_t)out;
}
double mt6701_uvw_mux_read() {
    return mt6701_uvw_mux_read_raw();
}

// UVW_MUX (Address 0x25[7])
// UVW_MUX register contains the configuration data of UVW output type.
// UVW_MUX UVW Output Type(Only for QFN Package)
// 0x0 UVW
// 0x1 -A-B-Z
void mt6701_uvw_mux_write_raw(uint16_t value) {
    uint8_t reg_addr = 0x25;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff + 1, 1));
    uint8_t svalue = (uint8_t)value;

    buff[0] = reg_addr;
    // XXXXXXXD
    buff[1] = (buff[1] & 0xFE) | (svalue & 0x01);

    mt6701_reg_write(buff, 2);
}
void mt6701_uvw_mux_write(double value) {
    mt6701_uvw_mux_write_raw(value);
}

uint16_t mt6701_abz_mux_read_raw() {
    uint8_t reg_addr = 0x29;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 1));

    // XXXXXXDX
    uint8_t out = (buff[0] & 0x02) >> 1;

    return (uint16_t)out;
}
double mt6701_abz_mux_read() {
    return mt6701_abz_mux_read_raw();
}

// ABZ_MUX (Address 0x29[6])
// ABZ_MUX register contains data of ABZ output type.
// ABZ_MUX ABZ Output Type
// 0x0 ABZ
// 0x1 UVW
void mt6701_abz_mux_write_raw(uint16_t value) {
    uint8_t reg_addr = 0x29;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff + 1, 1));
    uint8_t svalue = (uint8_t)value;

    buff[0] = reg_addr;
    // XXXXXXDX
    buff[1] = (buff[1] & 0xFD) | (svalue & 0x02);

    mt6701_reg_write(buff, 2);
}
void mt6701_abz_mux_write(double value) {
    mt6701_abz_mux_write_raw(value);
}

uint16_t mt6701_dir_read_raw() {
    uint8_t reg_addr = 0x29;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 1));

    // XDXXXXXX
    uint8_t out = (buff[0] & 0x40) >> 6;

    return (uint16_t)out;
}
double mt6701_dir_read() {
    return mt6701_dir_read_raw();
}

// DIR(Address 0x29[1])
// DIR register contains the configuration data of output rotation direction
// DIR Output Direction
// 0x0 CCW
// 0x1 CW
void mt6701_dir_write_raw(uint16_t value) {
    uint8_t reg_addr = 0x29;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff + 1, 1));
    uint8_t svalue = (uint8_t)value;

    buff[0] = reg_addr;
    // XDXXXXXX
    buff[1] = (buff[1] & 0xBF) | ((svalue & 0x40) << 6);

    mt6701_reg_write(buff, 2);
}
void mt6701_dir_write(double value) { // #TODO: FIX
    mt6701_dir_write_raw(value);
}

uint16_t mt6701_uvw_res_read_raw() {
    uint8_t reg_addr = 0x30;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 1));

    // XXXXDDDD
    uint8_t out = buff[0] & 0x0F;

    return (uint16_t)out;
}
double mt6701_uvw_res_read() {
    uint8_t out = mt6701_uvw_res_read_raw();
    out = out + 1;

    return (double)out;
}

// UVW_RES[3:0] (Address 0x30[7:4])
// UVW_RES register contains the configuration data of UVW output resolution (Pole-Paris).
// Reg. UVW_RES<3:0> UVW Output Pole Pairs
// 0x0 1
// 0x1 2
// 0x2 3
// ……. …….
// 0xD 14
// 0xE 15
// 0xF 16
void mt6701_uvw_res_write_raw(uint16_t value) {
    uint8_t reg_addr = 0x30;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff + 1, 1));
    uint8_t svalue = (uint8_t)value;

    buff[0] = reg_addr;
    // XXXXDDDD
    buff[1] = (buff[1] & 0xF0) | (svalue & 0x0F);

    mt6701_reg_write(buff, 2);
}
void mt6701_uvw_res_write(double value) {
    mt6701_uvw_res_write_raw(value - 1);
}

uint16_t mt6701_abz_res_read_raw() {
    uint8_t reg_addr = 0x30;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 2));

    // DDXXXXXX DDDDDDDD
    uint16_t out = (((uint16_t)(buff[0] & 0x03) << 8) | buff[1]);

    return (uint16_t)out;
}
double mt6701_abz_res_read() {
    uint16_t out = mt6701_abz_res_read_raw();
    out = out + 1;

    return (double)out;
}

// ABZ_RES[9:0] (Address 0x30[1:0] & 0x31[7:0])
// ABZ_RES register contains the configuration data of ABZ output resolution (PPR).
// Reg. ABZ_RES<9:0> ABZ Resolution (Pulse per Round)
// 0x000 1
// 0x001 2
// 0x002 3
// ……. …….
// 0x3FD 1022
// 0x3FE 1023
// 0x3FF 1024
void mt6701_abz_res_write_raw(uint16_t value) {
    uint8_t reg_addr = 0x30;
    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff + 1, 2));
    uint16_t svalue = (uint16_t)value;

    buff[0] = reg_addr;
    // DDXXXXXX DDDDDDDD
    buff[1] = (buff[1] & 0xFC) | (uint8_t)(((svalue >> 8) & 0xFF) & 0x03);
    buff[2] = (uint8_t)(svalue & 0xFF);

    mt6701_reg_write(buff, 3);
}
void mt6701_abz_res_write(double value) {
    mt6701_abz_res_write_raw(value - 1);
}

uint16_t mt6701_hyst_read_raw() {
    uint8_t reg_addr = 0x32;
    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 3));

    // XXXXXXXD XXXXXXXX XXXXXXDD
    uint8_t code = ((buff[0] & 0x01) << 2) | (buff[2] & 0x03);
    
    return (uint16_t)code;
}
double mt6701_hyst_read() {
    uint8_t code = mt6701_hyst_read_raw();
    
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

    return (double)out;
}

// HYST[2:0] (Address 0x32[7] & 0x34[7:6])
// HYST register contains the configuration data of hysteresis filter parameter.
// HYST Hysteresis (LSB)
// 0x0 1
// 0x1 2
// 0x2 4
// 0x3 8
// 0x4 0
// 0x5 0.25
// 0x6 0.5
// 0x7 1
// write is split in order to not wear-out unused middle byte
void mt6701_hyst_write_raw(uint16_t code) {
    uint8_t reg_addr = 0x32;
    uint8_t buff[4] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff + 1, 3));
    uint8_t scode = (uint8_t)code;

    buff[0] = reg_addr;
    // XXXXXXXD
    buff[1] = (buff[1] & 0xFE) | ((scode >> 2) & 0x01);

    mt6701_reg_write(buff, 2);

    buff[0] = reg_addr + 2;
    // XXXXXXDD
    buff[1] = (buff[3] & 0xFC) | (scode & 0x03);

    mt6701_reg_write(buff, 2);
}
void mt6701_hyst_write(double value) {
    uint8_t code = 0;

    if (value == 1.0) code = 0;
    else if (value == 2.0) code = 1;
    else if (value == 4.0) code = 2;
    else if (value == 8.0) code = 3;
    else if (value == 0.0) code = 4;
    else if (value == 0.25) code = 5;
    else if (value == 0.5) code = 6;
    else if (value == 1.0) code = 7;

    mt6701_hyst_write_raw(code);
}

uint16_t mt6701_z_pulse_width_read_raw() {
    uint8_t reg_addr = 0x32;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 1));

    // XXXXDDDX
    uint8_t code = (buff[0] & 0x0E) >> 1;

    return (uint16_t)code;
}
double mt6701_z_pulse_width_read() {
    uint8_t code = mt6701_z_pulse_width_read_raw();

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

    return (double)out;
}

// Z_PULSE_WIDTH[2:0] (Address 0x32[6:4])
// Z_PULSE_WIDTH register contains the configuration data of Z pulse width (Fig.9 & Fig.10 )
// HYST Z Pulse Width
// 0x0 1 LSB
// 0x1 2 LSB
// 0x2 4 LSB
// 0x3 8 LSB
// 0x4 12 LSB
// 0x5 16 LSB
// 0x6 180°
// 0x7 1 LSB
void mt6701_z_pulse_width_write_raw(uint16_t code) {
    uint8_t reg_addr = 0x32;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff + 1, 1));
    uint8_t scode = (uint8_t)code;

    buff[0] = reg_addr;
    // XXXXDDDX
    buff[1] = (buff[1] & 0xF1) | ((scode << 1) & 0x0E);

    mt6701_reg_write(buff, 2);
}
void mt6701_z_pulse_width_write(double value) {
    uint8_t code = 0;

    if (value == 1) code = 0;
    else if (value == 2) code = 1;
    else if (value == 4) code = 2;
    else if (value == 8) code = 3;
    else if (value == 12) code = 4;
    else if (value == 16) code = 5;
    else if (value == 180) code = 6;
    else if (value == 1) code = 7;

    mt6701_z_pulse_width_write_raw(code);
}

uint16_t mt6701_zero_read_raw() {
    uint8_t reg_addr = 0x32;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 2));

    // DDDDXXXX DDDDDDDD
    uint16_t code = ((uint16_t)(buff[0] & 0xF0) >> 4) | buff[1];

    return (uint16_t)code;
}
double mt6701_zero_read() {
    uint16_t code = mt6701_zero_read_raw();
    
    double out = code * 0.078;

    return out;
}

// ZERO[11:0] (Address 0x32[3:0] & 0x33[7:0])
// ZERO register contains the configuration data of zero-degree position.
// ZERO Zero Degree Position
// 0x000 0°
// 0x001 0.088°
// 0x002 0.176°
// … …
// 0xFFD 359.736°
// 0xFFE 359.824°
// 0xFFF 359.912°
void mt6701_zero_write_raw(uint16_t code) {
    uint8_t reg_addr = 0x32;
    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff + 1, 2));
    uint16_t scode = (uint16_t)code;

    buff[0] = reg_addr;
    // DDDDXXXX DDDDDDDD
    buff[1] = (buff[1] & 0x3F) | (uint8_t)((scode << 4) & 0xF0);
    buff[2] = (uint8_t)(scode & 0xFF);

    mt6701_reg_write(buff, 3);
}
void mt6701_zero_write(double value) {
    uint16_t code = (uint16_t)(value / 0.078);

    mt6701_zero_write_raw(code);
}

uint16_t mt6701_pwm_freq_read_raw() {
    uint8_t reg_addr = 0x38;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 1));

    // XXXXXXXD
    uint8_t out = buff[0] & 0x01;

    return (uint16_t)out;
}
double mt6701_pwm_freq_read() {
    uint8_t code = mt6701_pwm_freq_read_raw();

    double out = 0;
    switch (code) {
        case 0:
            out = 994.4;
            break;
        case 1:
            out = 497.2;
            break;
    }

    return (double)out;
}

// PWM_FREQ (Address 0x38[7])
// PWM_FREQ register contains the configuration data of PWM frame frequency
// PWM_FREQ PWM Frame Frequency
// 0x0 994.4 Hz
// 0x1 497.2 Hz
void mt6701_pwm_freq_write_raw(uint16_t code) {
    uint8_t reg_addr = 0x38;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff + 1, 1));
    uint8_t scode = (uint8_t)code;

    buff[0] = reg_addr;
    // XXXXXXXD
    buff[1] = (buff[1] & 0xFE) | (scode & 0x01);

    mt6701_reg_write(buff, 2);
}
void mt6701_pwm_freq_write(double value) {
    uint8_t code = 0;

    if (value == 994.4) code = 0;
    else if (value == 497.2) code = 1;

    mt6701_pwm_freq_write_raw(code);
}

uint16_t mt6701_pwm_pol_read_raw() {
    uint8_t reg_addr = 0x38;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 1));

    // XXXXXXDX
    uint8_t out = buff[0] & 0x02;

    return (uint16_t)out;
}
double mt6701_pwm_pol_read() {
    return mt6701_pwm_pol_read_raw();
}

// PWM_POL (Address 0x38[6])
// PWM_POL register contains data of PWM polarity.
// PWM_POL PWM Polarity
// 0x0 High Level Valid
// 0x1 Low Level Valid
void mt6701_pwm_pol_write_raw(uint16_t value) {
    uint8_t reg_addr = 0x38;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff + 1, 1));
    uint8_t svalue = (uint8_t)value;

    buff[0] = reg_addr;
    // XXXXXXDX
    buff[1] = (buff[1] & 0xFD) | (svalue & 0x02);

    mt6701_reg_write(buff, 2);
}
void mt6701_pwm_pol_write(double value) {
    mt6701_pwm_pol_write_raw(value);
}

uint16_t mt6701_out_mode_read_raw() {
    uint8_t reg_addr = 0x38;
    uint8_t buff[1] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 1));

    // XXXXXDXX
    uint8_t out = buff[0] & 0x04;

    return (uint16_t)out;
} 
double mt6701_out_mode_read() {
    return mt6701_out_mode_read_raw();
}

// OUT_MODE(Address 0x38[5])
// OUT_MODE register contains the configuration data of ‘Out’ Pin Mode
// OUT_MODE ‘Out’ Pin Mode
// 0x0 Analog Output
// 0x1 PWM Output
void mt6701_out_mode_write_raw(uint16_t value) {
    uint8_t reg_addr = 0x38;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff + 1, 1));
    uint8_t svalue = (uint8_t)value;

    buff[0] = reg_addr;
    // XXXXXDXX
    buff[1] = (buff[1] & 0xFB) | (svalue & 0x04);

    mt6701_reg_write(buff, 2);
}
void mt6701_out_mode_write(double value) {
    mt6701_out_mode_write_raw(value);
}

uint16_t mt6701_a_stop_read_raw() {
    uint8_t reg_addr = 0x3E;
    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 3));

    // XXXXDDDD XXXXXXXX DDDDDDDD
    uint16_t code = ((uint8_t)(buff[0] & 0x0F) << 8) | buff[2];

    return (uint16_t)code;

}
double mt6701_a_stop_read() {
    uint16_t code = mt6701_a_stop_read_raw();

    double out = code * 0.078;

    return out;
}

// A_STOP[11:0] (Address 0x3E[7:4] & 0x40[7:0])
// A_STOP register contains the configuration data of stop-point of analog output (Fig.15)
// A_STOP Analog/PWM Stop Angle
// 0x000 0°
// 0x001 0.088°
// 0x002 0.176°
// … …
// 0xFFD 359.736°
// 0xFFE 359.824°
// 0xFFF 359.912°
// write is split in order to not wear-out unused middle byte
void mt6701_a_stop_write_raw(uint16_t code) {
    uint8_t reg_addr = 0x3E;
    uint8_t buff[4] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff + 1, 3));
    uint16_t scode = (uint16_t)code;

    buff[0] = reg_addr;
    // XXXXDDDD
    buff[1] = (buff[1] & 0xF0) | ((scode >> 8) & 0x0F);

    mt6701_reg_write(buff, 2);

    buff[0] = reg_addr + 2;
    // DDDDDDDD
    buff[1] = buff[3]; // #TODO: possible bug

    mt6701_reg_write(buff, 2);
}
void mt6701_a_stop_write(double value) {
    uint16_t code = (uint16_t)(value / 0.078);

    mt6701_a_stop_write_raw(code);
}

uint16_t mt6701_a_start_read_raw() {
    uint8_t reg_addr = 0x3E;
    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff, 2));

    uint16_t code = ((uint16_t)(buff[0] & 0xF0) >> 4) | buff[1];

    return (uint16_t)code;
}
double mt6701_a_start_read() {
    uint16_t code = mt6701_a_start_read_raw();

    // DDDDXXXX DDDDDDDD
    double out = code * 0.078;

    return (double)out;
}

// A_START[11:0] (Address 0x3E[3:0] & 0x3F[7:0])
// A_START register contains the configuration data of start-point of analog output (Fig.15)
// A_START Analog/PWM Start Angle
// 0x000 0°
// 0x001 0.088°
// 0x002 0.176°
// … …
// 0xFFD 359.736°
// 0xFFE 359.824°
// 0xFFF 359.912
void mt6701_a_start_write_raw(uint16_t code) {
    uint8_t reg_addr = 0x3E;
    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(mt6701_reg_read(reg_addr, buff + 1, 2));
    uint8_t scode = (uint8_t)code;
    
    buff[1] = (buff[1] & 0xF0) | ((scode >> 8) & 0x0F);

    buff[0] = reg_addr;
    // DDDDXXXX DDDDDDDD
    buff[1] = (buff[1] & 0x0F) | (uint8_t)((scode << 4) & 0xF0);
    buff[2] = (uint8_t)(scode & 0xFF);

    mt6701_reg_write(buff, 3);
}
void mt6701_a_start_write(double value) {
    uint16_t code = (uint16_t)(value / 0.078);

    mt6701_a_start_write_raw(code);
}

uint16_t mt6701_log_eeprom_raw() {
    ESP_LOGI(TAG_MT6701,
        "\n=== EEPROM ===\nUVW_MUX: 0x%04X\nABZ_MUX: 0x%04X\nDIR: 0x%04X\n"
        "UVW_RES: 0x%04X\nABZ_RES: 0x%04X\nHYST: 0x%04X\nZ_PULSE_WIDTH: 0x%04X\n"
        "ZERO: 0x%04X\nPWM_FREQ: 0x%04X\nPWM_POL: 0x%04X\nOUT_MODE: 0x%04X\n"
        "A_STOP_READ: 0x%04X\nA_START_READ: 0x%04X",
        mt6701_uvw_mux_read_raw(),
        mt6701_abz_mux_read_raw(),
        mt6701_dir_read_raw(),
        mt6701_uvw_res_read_raw(),
        mt6701_abz_res_read_raw(),
        mt6701_hyst_read_raw(),
        mt6701_z_pulse_width_read_raw(),
        mt6701_zero_read_raw(),
        mt6701_pwm_freq_read_raw(),
        mt6701_pwm_pol_read_raw(),
        mt6701_out_mode_read_raw(),
        mt6701_a_stop_read_raw(),
        mt6701_a_start_read_raw()
    );
    return 0;
}
double mt6701_log_eeprom() {
    ESP_LOGI(TAG_MT6701,
        "\n=== EEPROM ===\nUVW_MUX: 0x%04X\nABZ_MUX: 0x%04X\nDIR: 0x%04X\n"
        "UVW_RES: 0x%04X\nABZ_RES: 0x%04X\nHYST: 0x%04X\nZ_PULSE_WIDTH: 0x%04X\n"
        "ZERO: 0x%04X\nPWM_FREQ: 0x%04X\nPWM_POL: 0x%04X\nOUT_MODE: 0x%04X\n"
        "A_STOP_READ: 0x%04X\nA_START_READ: 0x%04X",
        mt6701_uvw_mux_read(),
        mt6701_abz_mux_read(),
        mt6701_dir_read(),
        mt6701_uvw_res_read(),
        mt6701_abz_res_read(),
        mt6701_hyst_read(),
        mt6701_z_pulse_width_read(),
        mt6701_zero_read(),
        mt6701_pwm_freq_read(),
        mt6701_pwm_pol_read(),
        mt6701_out_mode_read(),
        mt6701_a_stop_read(),
        mt6701_a_start_read()
    );
    return 0.0;
}

void i2c_mt6701_terminate() {
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(mt6701_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
}

/**************mt6701-specific**************/

/**************ina228-specific**************/

esp_err_t ina228_reg_read(uint8_t reg_addr, uint8_t *buff, size_t size) {
    return i2c_master_transmit_receive(ina228_handle, &reg_addr, 1, buff, size, I2C_MASTER_TIMEOUT_MS);
}

void ina228_reg_write(uint8_t *buff, size_t size) {
    return;
}

uint16_t ina228_read_shunt_cal() {
    uint8_t reg_addr = 0x02;

    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(ina228_reg_read(reg_addr, buff, 2));

    // TODO: find out why shift by 2 is needed in order to get the corect value
    // update: turns out it was multiplication by 4 for ADCRANGE = 1
    uint16_t out = ((uint16_t)buff[0] << 8 | (buff[1] & 0x7FFF));
    if (ADCRANGE) out >>= 2;

    return out;
}

double ina228_get_current_lsb() {
    uint16_t shunt_cal = ina228_read_shunt_cal();

    double out = shunt_cal / (13107.2 * 1e6 * 0.0002);

    return out;
}

double ina228_get_max_expected_current() {
    double current_lsb = ina228_get_current_lsb();

    // 2^19 == 524288
    double out = current_lsb * 524288;

    return out;
}

int32_t ina228_read_vshunt_raw() {
    uint8_t reg_addr = 0x04;

    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(ina228_reg_read(reg_addr, buff, 3));

    // 0        1        2
    // DDDDDDDD DDDDDDDD DDDDXXXX
    uint32_t raw = ((int32_t)buff[0] << 16) | ((int32_t)buff[1] << 8) | ((int32_t)buff[2]);
    raw >>= 4;

    int32_t out = ((int32_t)(raw << 12)) >> 12;

    return out;
}
double ina228_read_vshunt() {
    return ina228_read_vshunt_raw() * (ADCRANGE? 78.125e-6 : 312.5e-6);
}

int32_t ina228_read_vbus_raw() {
    uint8_t reg_addr = 0x05;

    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(ina228_reg_read(reg_addr, buff, 3));

    // 0        1        2
    // DDDDDDDD DDDDDDDD DDDDXXXX
    uint32_t raw = ((int32_t)buff[0] << 16) | ((int32_t)buff[1] << 8) | ((int32_t)buff[2]);
    raw >>= 4;

    int32_t out = ((int32_t)(raw << 12)) >> 12;

    return out;
}
double ina228_read_vbus() {
    return ina228_read_vbus_raw() * 195.3125e-6;
}

int32_t ina228_read_current_raw() {
    uint8_t reg_addr = 0x07;

    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(ina228_reg_read(reg_addr, buff, 3));

    // 0        1        2
    // DDDDDDDD DDDDDDDD DDDDXXXX
    uint32_t raw = ((int32_t)buff[0] << 16) | ((int32_t)buff[1] << 8) | ((int32_t)buff[2]);
    raw >>= 4;

    int32_t out = ((int32_t)(raw << 12)) >> 12;

    return out;
}
double ina228_read_current() {
    return ina228_read_current_raw() * CURRENT_LSB;
}

uint16_t ina228_read_dietemp_raw() {
    uint8_t reg_addr = 0x06;

    uint8_t buff[2] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(ina228_reg_read(reg_addr, buff, 2));

    // 0        1      
    // DDDDDDDD DDDDDDDD
    uint16_t out = ((int32_t)buff[0] << 8) | ((int32_t)buff[1]);

    return out;
}
double ina228_read_dietemp() {
    return ina228_read_dietemp_raw() * 7.8125e-3;
}

uint32_t ina228_read_power_raw() {
    uint8_t reg_addr = 0x08;

    uint8_t buff[3] = {0};
    ESP_ERROR_CHECK_WITHOUT_ABORT(ina228_reg_read(reg_addr, buff, 3));

    // 0        1        2
    // DDDDDDDD DDDDDDDD DDDDDDDD
    uint32_t out = ((int32_t)buff[0] << 16) | ((int32_t)buff[1] << 8) | ((int32_t)buff[2]);

    return out;
}
double ina228_read_power() {
    return ina228_read_power_raw() * (ADCRANGE? 78.125e-6 : 312.5e-6);
}

/**************ina228-specific**************/
