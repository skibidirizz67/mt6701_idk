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

void client_handle_packet(const Packet *p) {
	if (!p || p->hdr != PKT_HDR || !check_packet_crc(p) || (p->len > 0 && !p->pld)) return;
	
    uint16_t code;
    double angle;

	switch (p->cmd) {
		case READ_SENSOR:
		    code = ((uint16_t)p->pld[0] << 6) | (p->pld[1] & 0x3F);
            angle = code / 16384.0 * 360.0;
            printf("CLIENT RECEIVED ANGLE: %.3f (0x%04X)\n", angle, code);
		    break;
	}
}

int main(void) {
    int fd = open_serial("/dev/ttyUSB0", B115200);
    if (fd < 0) { perror("open_serial"); return 1; }

    uint8_t buf_arr[4096];
    uint8_t *buf = NULL;
    ssize_t buf_len = 0;
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
            printf ("Trying to send angle read packet..\n");

	        Packet p = {
	        	.hdr = PKT_HDR,
                .cmd = READ_SENSOR,
                .len = 0,
	        };

            client_send_packet(fd, &p);
        }
    }
    close(fd);
    return 0;
}