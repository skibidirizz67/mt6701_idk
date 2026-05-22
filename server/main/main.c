#include "esp_log.h"
#include "driver/uart.h"
#include "esp_log_buffer.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"
#include "soc/clk_tree_defs.h"
#include "i2c_mt6701.h"

#include "shared.h"

static const uart_port_t UART_PORT = UART_NUM_0;

QueueHandle_t uart_queue;
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

uint16_t read_sensor_stub() {
	return 0x03FF;
}

bool server_send_packet(const Packet *p) {
	uint8_t buf[MAX_PKT_LEN];
	size_t buf_len = encode_packet(p, buf, MAX_PKT_LEN);
	if (!buf_len) return false;

	uint16_t cksum = crc16_ccitt(buf, buf_len - 2);
	buf[buf_len - 2] = (uint8_t)(cksum >> 8);
	buf[buf_len - 1] = (uint8_t)(cksum & 0xFF);

	int sizeb = uart_write_bytes(UART_PORT, buf, buf_len);
	ESP_LOGI("sizeb", "%d", sizeb);

	return true;
}

void server_handle_packet(const Packet *p) {
	if (!p || p->hdr != PKT_HDR || !check_packet_crc(p) || (p->len > 0 && !p->pld)) return;
	
	Packet r = {
		.hdr = PKT_HDR,
	};

	switch (p->cmd) {
		case READ_SENSOR:
			r.cmd = READ_SENSOR;
			r.len = 2;

			uint16_t angle = mt6701_sensor_read_raw(dev_handle);
			uint8_t buf[2] = {
				(uint8_t)(angle >> 8),
				(uint8_t)(angle & 0xFF),
			};
			r.pld = &buf[0];

			server_send_packet(&r);
			break;
	}
}

static void uart_event_task(void *pvParameters) {
    uart_event_t e;
    uint8_t buf_arr[128];
    uint8_t *buf = &buf_arr[0];
    size_t buf_len = 0;
    while (1) {
        if (xQueueReceive(uart_queue, (void*)&e, (TickType_t)portMAX_DELAY)) {
            if (e.type != UART_DATA) continue;

            ESP_ERROR_CHECK_WITHOUT_ABORT(uart_get_buffered_data_len(UART_PORT, &buf_len));
            if (buf_len < 5) continue;
			buf = &buf_arr[0];
            buf_len = uart_read_bytes(UART_PORT, buf, buf_len, pdMS_TO_TICKS(100));
            while (buf_len >= 5) {
                Packet p;
                size_t decoded_len;
                if (buf[0] != PKT_HDR || !(decoded_len = decode_packet(buf, buf_len, &p))) {
                    buf++;
                    buf_len--;
                    continue;
                }

                server_handle_packet(&p);

                // check header, if wrong decr and continue, parse packet, cmd, pldlen, pld, crc, copy from cli command handler also enum
                buf += decoded_len;
                buf_len -= decoded_len;
            }
        }
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 2048, 2048, 20, &uart_queue, 0));

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    i2c_mt6701_init(&bus_handle, &dev_handle);

    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, configMAX_PRIORITIES / 2, NULL);
}