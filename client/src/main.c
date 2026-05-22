#include <stdio.h>
#include <termios.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "shared.h"

bool client_send_packet(int fd, const Packet *p) {
	uint8_t buf[MAX_PKT_LEN];
	size_t buf_len = encode_packet(p, buf, MAX_PKT_LEN);
	if (!buf_len) return false;

	uint16_t cksum = crc16_ccitt(buf, buf_len - 2);
	buf[buf_len - 2] = (uint8_t)(cksum >> 8);
	buf[buf_len - 1] = (uint8_t)(cksum & 0xFF);

	ssize_t n = write(fd, buf, buf_len);
    if (n < 0) perror("write");

	return true;
}

int open_serial(const char *path, speed_t baud_rate) {
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, baud_rate);
    cfsetospeed(&tty, baud_rate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CLOCAL | CREAD;

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 1;

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
    return fd;
}

double conver_hyst(uint16_t val) {
    switch (val) {
        case 0:
            return 1.0;
        case 1:
            return 2.0;
        case 2:
            return 4.0;
        case 3:
            return 8.0;
        case 4:
            return 0.0;
        case 5:
            return 0.25;
        case 6:
            return 0.5;
        case 7:
            return 1.0;
    }
    return 0;
}

double convert_z_pulse_width(uint16_t val) {
    switch (val) {
        case 0:
            return 1;
        case 1:
            return 2;
        case 2:
            return 4;
        case 3:
            return 8;
        case 4:
            return 12;
        case 5:
            return 16;
        case 6:
            return 180;
        case 7:
            return 1;
    }
    return 0;
}

double client_convert_reg_value(uint8_t reg, uint16_t val) {
    switch (reg) {
        case UVW_MUX:
            return (double)val;
        case ABZ_MUX:
            return (double)val;
        case DIR:
            return (double)val;
        case UVW_RES:
            return (double)(val + 1);
        case ABZ_RES:
            return (double)(val + 1);
        case HYST:
            return conver_hyst(val);
        case Z_PULSE_WIDTH:
            return convert_z_pulse_width(val);
        case ZERO:
            return val * 0.078;
        case PWM_FREQ:
            return val? 994.4 : 497.2;;
        case PWM_POL:
            return (double)val;
        case OUT_MODE:
            return (double)val;
        case A_START:
            return val * 0.078;
        case A_STOP:
            return val * 0.078;
    }
    return 0;
}

void client_handle_packet(const Packet *p) {
	if (!p || p->hdr != PKT_HDR || !check_packet_crc(p) || (p->len > 0 && !p->pld)) return;
	
    uint8_t reg;
    uint16_t code;
    double val;

	switch (p->cmd) {
		case READ_SENSOR:
		    code = ((uint16_t)(p->pld[0]) << 8) | p->pld[1];
            val = code / 16384.0 * 360.0;
            printf("CLIENT RECEIVED ANGLE: %.3f (0x%04X)\n", val, code);
		    break;
        case READ_CONFIG:
            reg = p->pld[0];
		    code = ((uint16_t)(p->pld[1]) << 8) | p->pld[2];
            val = client_convert_reg_value(reg, code);
            printf("CLIENT RECEIVED REG_VALUE of 0x%02X: %.3f (0x%04X)\n", reg, val, code);
		    break;
	}
}

int main(void) {
    int fd = open_serial("/dev/ttyUSB0", B115200);
    if (fd < 0) { perror("open_serial"); return 1; }

    uint8_t buf_arr[4096];
    uint8_t *buf = NULL;
    uint8_t pld[3];
    ssize_t buf_len = 0;
    Packet p = { .hdr = PKT_HDR };

    while (1) {
        int c = getchar();
        if (c == '\n') {
            printf ("Trying to receive packets..\n");
            buf = &buf_arr[0];
            buf_len = read(fd, buf, 4069);
            if (buf_len < 5) continue;

            while (buf_len >= 5) {
                Packet r;
                size_t decoded_len;
                if (buf[0] != PKT_HDR || !(decoded_len = decode_packet(buf, buf_len, &r))) {
                    buf++;
                    buf_len--;
                    continue;
                }
                client_handle_packet(&r);

                buf += decoded_len;
                buf_len -= decoded_len;
            }
        }
        else if (c == 'a') {
            printf ("Trying to send angle read request..\n");

	        p.cmd = READ_SENSOR,
            p.len = 0;

            client_send_packet(fd, &p);
        }
        else if (c == 'r') {
            printf ("Trying to send ABZ_RES read request..\n");
            pld[0] = ABZ_RES;

	        p.cmd = READ_CONFIG;
            p.len = 1;
	        p.pld = &pld[0];

            client_send_packet(fd, &p);
        }
        else if (c == 'u') {
            printf ("Trying to send UVW_MUX read request..\n");
            pld[0] = UVW_MUX;

	        p.cmd = READ_CONFIG;
            p.len = 1;
	        p.pld = &pld[0];

            client_send_packet(fd, &p);
        }
    }
    close(fd);
    return 0;
}