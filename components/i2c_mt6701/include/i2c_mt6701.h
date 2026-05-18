#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static esp_err_t mt6701_reg_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *buff, size_t size);
void mt6701_eeprom_write_wait();
void mt6701_reg_write(i2c_master_dev_handle_t dev_handle, uint8_t *buff, size_t size);
uint8_t mt6701_uvw_mux_read(i2c_master_dev_handle_t dev_handle);
void mt6701_uvw_mux_write(i2c_master_dev_handle_t dev_handle, uint8_t value);
uint8_t mt6701_abz_mux_read(i2c_master_dev_handle_t dev_handle);
void mt6701_abz_mux_write(i2c_master_dev_handle_t dev_handle, uint8_t value);
uint8_t mt6701_dir_read(i2c_master_dev_handle_t dev_handle);
void mt6701_dir_write(i2c_master_dev_handle_t dev_handle, uint8_t value);
uint8_t mt6701_uvw_res_read(i2c_master_dev_handle_t dev_handle);
void mt6701_uvw_res_write(i2c_master_dev_handle_t dev_handle, uint8_t value);
uint16_t mt6701_abz_res_read(i2c_master_dev_handle_t dev_handle);
void mt6701_abz_res_write(i2c_master_dev_handle_t dev_handle, uint16_t value);
double mt6701_hyst_read(i2c_master_dev_handle_t dev_handle);
void mt6701_hyst_write(i2c_master_dev_handle_t dev_handle, double value);
uint8_t mt6701_z_pulse_width_read(i2c_master_dev_handle_t dev_handle);
void mt6701_z_pulse_width_write(i2c_master_dev_handle_t dev_handle, uint8_t value);
double mt6701_zero_read(i2c_master_dev_handle_t dev_handle);
void mt6701_zero_write(i2c_master_dev_handle_t dev_handle, double value);
uint8_t mt6701_pwm_freq_read(i2c_master_dev_handle_t dev_handle);
void mt6701_pwm_freq_write(i2c_master_dev_handle_t dev_handle, uint8_t value);
uint8_t mt6701_pwm_pol_read(i2c_master_dev_handle_t dev_handle);
void mt6701_pwm_pol_write(i2c_master_dev_handle_t dev_handle, uint8_t value);
uint8_t mt6701_out_mode_read(i2c_master_dev_handle_t dev_handle);
void mt6701_out_mode_write(i2c_master_dev_handle_t dev_handle, uint8_t value);
double mt6701_a_stop_read(i2c_master_dev_handle_t dev_handle);
void mt6701_a_stop_write(i2c_master_dev_handle_t dev_handle, double value);
double mt6701_a_start_read(i2c_master_dev_handle_t dev_handle);
void mt6701_a_start_write(i2c_master_dev_handle_t dev_handle, double value);
void mt6701_log_eeprom(i2c_master_dev_handle_t dev_handle);
void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);
void i2c_mt6701_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);
void i2c_mt6701_terminate(i2c_master_dev_handle_t *dev_handle, i2c_master_bus_handle_t *bus_handle);