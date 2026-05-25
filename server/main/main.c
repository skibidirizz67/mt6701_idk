#include "esp_log.h"
#include "driver/uart.h"
#include "esp_log_buffer.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"
#include "soc/clk_tree_defs.h"
#include "i2c_mt6701.h"

#include "shared.h"

static const uart_port_t UART_PORT = UART_NUM_2;

static QueueHandle_t uart_queue;
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

// encodes the packet, computes checksum and writes it to UART buffer
// returns true if success, false if failed
bool server_send_packet(const Packet *p) {
	uint8_t buf[MAX_PKT_LEN];
	size_t buf_len = encode_packet(p, buf, MAX_PKT_LEN);
	if (!buf_len) return false;

	uint16_t cksum = crc16_ccitt(buf, buf_len - 2);
	buf[buf_len - 2] = (uint8_t)(cksum >> 8);
	buf[buf_len - 1] = (uint8_t)(cksum & 0xFF);

	uart_write_bytes(UART_PORT, buf, buf_len);
	return true;
}

uint16_t server_get_reg_value(uint8_t reg) {
    switch (reg) {
        case UVW_MUX:
            return mt6701_uvw_mux_read_raw(dev_handle);
        case ABZ_MUX:
            return mt6701_abz_mux_read_raw(dev_handle);
        case DIR:
            return mt6701_dir_read_raw(dev_handle);
        case UVW_RES:
            return mt6701_uvw_res_read_raw(dev_handle);
        case ABZ_RES:
            return mt6701_abz_res_read_raw(dev_handle);
        case HYST:
            return mt6701_hyst_read_raw(dev_handle);
        case Z_PULSE_WIDTH:
            return mt6701_z_pulse_width_read_raw(dev_handle);
        case ZERO:
            return mt6701_zero_read_raw(dev_handle);
        case PWM_FREQ:
            return mt6701_pwm_freq_read_raw(dev_handle);
        case PWM_POL:
            return mt6701_pwm_pol_read_raw(dev_handle);
        case OUT_MODE:
            return mt6701_out_mode_read_raw(dev_handle);
        case A_START:
            return mt6701_a_start_read_raw(dev_handle);
        case A_STOP:
            return mt6701_a_stop_read_raw(dev_handle);
    }
    return 0;
}

void server_set_reg_value(uint8_t reg, uint16_t val) {
    switch (reg) {
        case UVW_MUX:
            mt6701_uvw_mux_write_raw(dev_handle, val);
            break;
        case ABZ_MUX:
            mt6701_abz_mux_write_raw(dev_handle, val);
            break;
        case DIR:
            mt6701_dir_write_raw(dev_handle, val);
            break;
        case UVW_RES:
            mt6701_uvw_res_write_raw(dev_handle, val);
            break;
        case ABZ_RES:
            mt6701_abz_res_write_raw(dev_handle, val);
            break;
        case HYST:
            mt6701_hyst_write_raw(dev_handle, val);
            break;
        case Z_PULSE_WIDTH:
            mt6701_z_pulse_width_write_raw(dev_handle, val);
            break;
        case ZERO:
            mt6701_zero_write_raw(dev_handle, val);
            break;
        case PWM_FREQ:
            mt6701_pwm_freq_write_raw(dev_handle, val);
            break;
        case PWM_POL:
            mt6701_pwm_pol_write_raw(dev_handle, val);
            break;
        case OUT_MODE:
            mt6701_out_mode_write_raw(dev_handle, val);
            break;
        case A_START:
            mt6701_a_start_write_raw(dev_handle, val);
            break;
        case A_STOP:
            mt6701_a_stop_write_raw(dev_handle, val);
            break;
    }
}

// handles received packets; invalid packets are ignored
void server_handle_packet(const Packet *p) {
	if (!p || p->hdr != PKT_HDR || !check_packet_crc(p) || (p->len > 0 && !p->pld)) return;
	
    uint8_t reg;
    uint16_t val;
    uint8_t pld[3];
	Packet r = { .hdr = PKT_HDR };

	switch (p->cmd) {
		case READ_SENSOR:
            val = mt6701_sensor_read_raw(dev_handle);
			pld[0] = (uint8_t)(val >> 8);
			pld[1] = (uint8_t)(val & 0xFF);

			r.cmd = READ_SENSOR;
			r.len = 2;
			r.pld = &pld[0];

			server_send_packet(&r);
			break;
        case READ_CONFIG:
            reg = p->pld[0];
            val = server_get_reg_value(reg);
            pld[0] = reg;
            pld[1] = (uint8_t)(val >> 8);
			pld[2] = (uint8_t)(val & 0xFF);

            r.cmd = READ_CONFIG;
            r.len = 3;
            r.pld = &pld[0];

            server_send_packet(&r);
            break;
        case WRITE_CONFIG:
            reg = p->pld[0];
            val = ((uint16_t)(p->pld[1]) << 8) | p->pld[2];

            server_set_reg_value(reg, val);
	}
}

// FreeRTOS task to watch incoming data stream and parse it
static void uart_event_task(void *pvParameters) {
    uart_event_t e;
    uint8_t buf_arr[tx_rx_buf_len];
    uint8_t *buf = &buf_arr[0];
    size_t buf_len = 0;
    // busy looping is fine since xQueueReceive will block and wait for an item to receive (TickType_t)portMAX_DELAY)
    while (1) {
        if (xQueueReceive(uart_queue, (void*)&e, (TickType_t)portMAX_DELAY)) {
            if (e.type != UART_DATA) continue;

            ESP_ERROR_CHECK_WITHOUT_ABORT(uart_get_buffered_data_len(UART_PORT, &buf_len));
            if (buf_len < MIN_PKT_LEN) continue;
			buf = &buf_arr[0];
            buf_len = uart_read_bytes(UART_PORT, buf, tx_rx_buf_len, pdMS_TO_TICKS(100));
            // while buffer contains enough data for potential packet, try to parse it
            while (buf_len >= MIN_PKT_LEN) {
                Packet p;
                size_t decoded_len;
                // if not a valid packet, advance pointer by 1 byte and try again
                if (buf[0] != PKT_HDR || !(decoded_len = decode_packet(buf, buf_len, &p))) {
                    buf++;
                    buf_len--;
                    continue;
                }

                server_handle_packet(&p);

                buf += decoded_len;
                buf_len -= decoded_len;
            }
        }
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, tx_rx_buf_len, tx_rx_buf_len, 16, &uart_queue, 0));

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, 22, 23, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    i2c_mt6701_init(&bus_handle, &dev_handle);

    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, configMAX_PRIORITIES / 2, NULL);
}