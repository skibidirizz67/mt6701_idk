#include "esp_log.h"
#include "driver/uart.h"
#include "esp_log_buffer.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"
#include "soc/clk_tree_defs.h"
#include "i2c_mt6701.h"

static const uart_port_t UART_PORT = UART_NUM_0;
static const uint8_t PKT_HDR = 0xAA;
static const uint8_t MIN_PKT_LEN = 5;
static const uint8_t MAX_PKT_LEN = 8;

typedef struct {
	uint8_t hdr;
	uint8_t cmd;
	uint8_t len;
	uint8_t *pld;
	uint16_t crc;
} Packet;

typedef enum {
	READ_SENSOR,
	READ_CONFIG,
	WRITE_CONFIG,
} Cmds;

QueueHandle_t uart_queue;
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

static const uint16_t crc16_tab[] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};
uint16_t crc16_ccitt(const uint8_t *buf, size_t buf_len) {
    uint16_t cksum = 0xFFFF;
	for (size_t i = 0; i < buf_len; i++) {
		cksum = crc16_tab[((cksum>>8) ^ *buf++) & 0xFF] ^ (cksum << 8);
	}
	return cksum;
}

bool check_crc(const uint8_t *buf, size_t buf_len) {
	if (!buf || buf_len < 3) return false;
	return (((uint16_t)buf[buf_len - 2] << 8) | buf[buf_len - 1]) ==
		crc16_ccitt(buf, buf_len - 2);
}

size_t encode_packet(const Packet *p, uint8_t *buf, size_t buf_len) {
	if (!p || !buf) return 0;
	if (buf_len < (MIN_PKT_LEN + p->len)) return 0;

	buf[0] = p->hdr;
	buf[1] = p->cmd;
	buf[2] = p->len;
	if (p->len > 0) memcpy(buf + 3, p->pld, p->len);
	buf += 3 + p->len;
	buf[0] = (uint8_t)(p->crc >> 8);
	buf[1] = (uint8_t)(p->crc & 0xFF);

	return MIN_PKT_LEN + p->len;
}

size_t decode_packet(uint8_t *buf, size_t buf_len, Packet *p) {
	if (!buf || !p) return 0;
	if (buf_len < MIN_PKT_LEN || buf_len < (MIN_PKT_LEN + buf[2])) return 0;

	p->hdr = buf[0];
	p->cmd = buf[1];
	p->len = buf[2];
	if (p->len > 0) p->pld = buf + 3;
	buf += 3 + p->len;
	p->crc = ((uint16_t)buf[0] << 8) | buf[1];

	return MIN_PKT_LEN + p->len;
}

bool check_packet_crc(const Packet *p) {
	if (!p) return false;
	uint8_t buf[MAX_PKT_LEN];
	uint8_t buf_len = encode_packet(p, buf, MAX_PKT_LEN);
	if (buf_len == 0) return false;
	return check_crc(buf, buf_len);
}

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