#include "esp_log.h"
#include "driver/uart.h"
#include "esp_log_buffer.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"
#include "soc/clk_tree_defs.h"

#define UART_PORT_NUM UART_NUM_2
#define UART_RX_BUFF_SIZE 1024

#define PACKET_HEADER 0xAA55

typedef struct {
    uint16_t hdr;
    uint8_t cmd;
    uint8_t seq;
    uint8_t rsp;
    uint8_t len;
    uint8_t *pld;
    uint16_t crc;
} Packet;

typedef struct {
    Packet *txq[16]; // send queue
    Packet *rsq[16]; // response queue
    Packet *rxq[16]; // response queue
    uint8_t txqs; // send queue size
    uint8_t rsqs; // response queue size
    uint8_t rxqs; // response queue size
    uint8_t sqc; // sequence counter;
} ProtocolCtx;

typedef enum {
    PING,
    SREAD,
    MSTART,
    MSTOP = 0x04,
    CREAD,
    CWRITE,
    REBOOT,
    RSP_MASK = 0x80,
    PING_ = 0x80,
    SREAD_,
    MSTART_,
    MDATA_,
    MSTOP_,
    CREAD_,
    CWRITE_,
    REBOOT_,
} CMDs;

typedef enum {
    OK,
    ERR,
    BAD_CMD,
    BAD_REG,
    BAD_VAL,
} RSPs;

typedef enum {
    UVW_MUX,
    ABZ_MUX,
    DIR,
    UVW_RES,
    ABZ_RES,
    HYST,
    Z_PULSE_WIDTH,
    ZERO,
    PWM_FREQ,
    PWM_POL,
    OUT_MODE,
    A_START,
    A_STOP,
} CREGs;

const char *TAG = "uart_test";

ProtocolCtx ctx;

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
uint16_t crc16_ccitt(uint16_t init, const uint8_t *buff, size_t len) {
    uint16_t cksum = init;
	for (int i = 0;  i < len;  i++)
		cksum = crc16_tab[((cksum>>8) ^ *buff++) & 0xff] ^ (cksum << 8);
	return cksum;
}

bool packet_append_crc(uint8_t *buff, size_t len) {
    if (len < 3) return false;
    uint16_t cksum = crc16_ccitt(0xFFFF, buff, len - 2);
    buff[len - 2] = (uint8_t)(cksum >> 8);
    buff[len - 1] = (uint8_t)(cksum & 0XFF);
    return true;
}

bool packet_check_crc(uint8_t *buff, size_t len) {
    uint16_t cksum_a = crc16_ccitt(0xFFFF, buff, len - 2);
    uint16_t cksum_b = ((uint16_t)buff[len - 2] << 8) | buff[len - 1];
    return cksum_a == cksum_b;
}

size_t encode_packet(Packet p, uint8_t *out, size_t buff_len) {
    if (!out) return 0;
    if (buff_len < 8) return 0;

    out[0] = (uint8_t)(p.hdr >> 8);
    out[1] = (uint8_t)(p.hdr & 0xFF);
    out[2] = p.cmd;
    out[3] = p.seq;
    out[4] = p.rsp;
    out[5] = p.len;

    if (p.len > 0 && p.pld) {
        memcpy(out + 6, p.pld, p.len);
    }
    
    size_t out_len = 6 + p.len + 2;

    out[out_len - 2] = (uint8_t)(p.crc >> 8);
    out[out_len - 1] = (uint8_t)(p.crc & 0xFF);

    return out_len;
}

bool decode_packet(uint8_t *buff, size_t buff_len, Packet *out) {
    if (!buff || !out) return false;
    if (buff_len < 8) return false;

    *out = (Packet){0};

    out->hdr = ((uint16_t)buff[0] << 8) | buff[1];
    out->cmd = buff[2];
    out->seq = buff[3];
    out->rsp = buff[4];
    out->len = buff[5];

    if (out->len > 0) {
        if (buff_len - out->len < 8) return false;
        out->pld = buff + 6;
    }
    else out->pld = NULL;

    out->crc = ((uint16_t)buff[buff_len - 2] << 8) | buff[buff_len - 1];

    return true;
}

void packet_frame(ProtocolCtx *ctx, uint8_t *buff, size_t len) {
    if (len < 4) return;

    buff[0] = (uint8_t)(PACKET_HEADER >> 8);
    buff[1] = (uint8_t)(PACKET_HEADER & 0xFF);

    packet_append_crc(buff, len);
}

void struct_packet_frame(Packet *p) {
    p->hdr = 0xAA55;
    
    uint8_t tmp[11];
    size_t tmp_len = 11;

    tmp_len = encode_packet(*p, tmp, tmp_len);
    if (tmp_len == 0) return;

    p->crc = crc16_ccitt(0xFFFF, tmp, tmp_len - 2);
}

void rx_task(void *arg) {
    uint8_t buff[UART_RX_BUFF_SIZE];
    while (1) {
        int buff_len = uart_read_bytes(UART_PORT_NUM, buff, UART_RX_BUFF_SIZE, pdMS_TO_TICKS(1000));
        while (buff_len >= 8) {
            if (ctx.rxqs >= 16) {
                ESP_LOGE(TAG, "RX queue is full, can't add new");
                continue;
            }

            ctx.rxq[ctx.rxqs] = &(Packet){0};
            if (!decode_packet(buff, buff_len, ctx.rxq[ctx.rxqs])) {
                ESP_LOGE(TAG, "Failed to decode received packet");
                continue;
            }

            ESP_LOGI(TAG, "RX task received packet", ctx.rxq[ctx.rxqs]->seq);
            buff_len -= 8 + ctx.rxq[ctx.rxqs]->len;
            ctx.rxqs++;
        }
    }
}

void tx_task(void *arg) {
    uint8_t buff[11];
    while (1) {
        if (ctx.txqs > 0) {
            size_t buff_len = encode_packet(*ctx.txq[ctx.txqs - 1], buff, 11);
            if (buff_len < 1) {
                ESP_LOGE(TAG, "Invalid packet in queue, skipping");
                ctx.txqs--;
                continue;
            }
            packet_frame(&ctx, buff, buff_len);

            if (ctx.rsqs >= 16) {
                ESP_LOGE(TAG, "Response queue is full, can't add new");
            }
            else {
                ctx.rsq[ctx.rsqs] = ctx.txq[ctx.txqs - 1];
                ctx.rsqs++;
            }

            uart_write_bytes(UART_PORT_NUM, buff, buff_len);
            ESP_LOGI(TAG, "TX task sent packet", ctx.txq[ctx.txqs - 1]->seq);

            ctx.txqs--;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


void print_packet(Packet p) {
    ESP_LOGI(TAG, "\nPacket:\nHDR: 0x%04X\nCMD: 0x%02X\nSEQ: 0x%02x\nRSP: 0x%02X\nLEN: 0x%02X\nCRC: 0x%04x\n",
        p.hdr, p.cmd, p.seq, p.rsp, p.len, p.crc);
}

void app_main(void) {
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_RX_BUFF_SIZE * 2, 0, 0, NULL, 0));

    uart_config_t uart_config = {
        .baud_rate = 115200, 
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, 4, 5, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ctx = (ProtocolCtx){0};

    xTaskCreate(rx_task, "uart_rx_task", 3072, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 3072, NULL, configMAX_PRIORITIES - 2, NULL);

    Packet p0 = {0};
    Packet p1 = {0};
    Packet p2 = {0};
    
    p0.seq = ctx.sqc++;
    p1.seq = ctx.sqc++;
    p2.seq = ctx.sqc++;

    ctx.txq[ctx.txqs++] = &p0;
    ctx.txq[ctx.txqs++] = &p1;
    ctx.txq[ctx.txqs++] = &p2;


    while (1) {
        if (ctx.rxqs > 0) {
            Packet *p = ctx.rxq[ctx.rxqs - 1];
            if (p->cmd & RSP_MASK) {
                for (int i = 0; i < ctx.rsqs; i++) {
                    if (ctx.rsq[i]->seq != p->seq) continue;
                    ESP_LOGI(TAG, "Received response for packet %d:", p->seq);
                    print_packet(*p);
                    ctx.rsqs--;
                    ctx.rxqs--;
                }
            }
            else {
                if (p->cmd == PING) {
                    ESP_LOGI(TAG, "Received ping packet, sending response");
                    print_packet(*p);

                    ctx.txq[ctx.txqs] = &(*p);
                    ctx.txq[ctx.txqs]->cmd = PING_;
                    ctx.txqs++;
                    ctx.rxqs--;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}