#include "driver/i2c_types.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdio.h>


void i2c_init();
void i2c_mt6701_terminate();

/**************mt6701-specific**************/

uint16_t mt6701_sensor_read_raw();
double mt6701_sensor_read();

uint16_t mt6701_uvw_mux_read_raw();
void mt6701_uvw_mux_write_raw(uint16_t value);
double mt6701_uvw_mux_read();
void mt6701_uvw_mux_write(double value);

uint16_t mt6701_abz_mux_read_raw();
void mt6701_abz_mux_write_raw(uint16_t value);
double mt6701_abz_mux_read();
void mt6701_abz_mux_write(double value);

uint16_t mt6701_dir_read_raw();
void mt6701_dir_write_raw(uint16_t value);
double mt6701_dir_read();
void mt6701_dir_write(double value);

uint16_t mt6701_uvw_res_read_raw();
void mt6701_uvw_res_write_raw(uint16_t value);
double mt6701_uvw_res_read();
void mt6701_uvw_res_write(double value);

uint16_t mt6701_abz_res_read_raw();
void mt6701_abz_res_write_raw(uint16_t value);
double mt6701_abz_res_read();
void mt6701_abz_res_write(double value);

uint16_t mt6701_hyst_read_raw();
void mt6701_hyst_write_raw(uint16_t value);
double mt6701_hyst_read();
void mt6701_hyst_write(double value);

uint16_t mt6701_z_pulse_width_read_raw();
void mt6701_z_pulse_width_write_raw(uint16_t value);
double mt6701_z_pulse_width_read();
void mt6701_z_pulse_width_write(double value);

uint16_t mt6701_zero_read_raw();
void mt6701_zero_write_raw(uint16_t value);
double mt6701_zero_read();
void mt6701_zero_write(double value);

uint16_t mt6701_pwm_freq_read_raw();
void mt6701_pwm_freq_write_raw(uint16_t value);
double mt6701_pwm_freq_read();
void mt6701_pwm_freq_write(double value);

uint16_t mt6701_pwm_pol_read_raw();
void mt6701_pwm_pol_write_raw(uint16_t value);
double mt6701_pwm_pol_read();
void mt6701_pwm_pol_write(double value);

uint16_t mt6701_out_mode_read_raw();
void mt6701_out_mode_write_raw(uint16_t value);
double mt6701_out_mode_read();
void mt6701_out_mode_write(double value);

uint16_t mt6701_a_stop_read_raw();
void mt6701_a_stop_write_raw(uint16_t value);
double mt6701_a_stop_read();
void mt6701_a_stop_write(double value);

uint16_t mt6701_a_start_read_raw();
void mt6701_a_start_write_raw(uint16_t value);
double mt6701_a_start_read();
void mt6701_a_start_write(double value);

uint16_t mt6701_log_eeprom_raw();
double mt6701_log_eeprom();

/**************mt6701-specific**************/

/**************ina228-specific**************/

uint16_t ina228_read_shunt_cal();
double ina228_get_current_lsb();
double ina228_get_max_expected_current();

int32_t ina228_read_vshunt_raw();
double ina228_read_vshunt();

int32_t ina228_read_vbus_raw();
double ina228_read_vbus();

int32_t ina228_read_current_raw();
double ina228_read_current();

uint16_t ina228_read_dietemp_raw();
double ina228_read_dietemp();

uint32_t ina228_read_power_raw();
double ina228_read_power();

/**************ina228-specific**************/
