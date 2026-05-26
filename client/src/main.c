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

// encodes the packet, computes checksum and writes it into provided file descriptor (fd) using unistd.h write function
// returns true if success, false if failed
bool client_send_packet(int fd, const Packet *p) {
	uint8_t buf[MAX_PKT_LEN];
	size_t buf_len = encode_packet(p, buf, MAX_PKT_LEN);
	if (!buf_len) return false;

	uint16_t cksum = crc16_ccitt(buf, buf_len - 2);
	buf[buf_len - 2] = (uint8_t)(cksum >> 8);
	buf[buf_len - 1] = (uint8_t)(cksum & 0xFF);

	ssize_t n = write(fd, buf, buf_len);
    if (n < 0) {
        perror("write");
        return false;
    }

	return true;
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

// very ugly way of converting values, i know
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

// handles received packets; invalid packets are ignored
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

// tries open serial communication with board using termios
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

#define CLI_MAX_STR_LEN 256
#define CLI_MAX_TOKENS 16
#define CLI_RAW_FLAG (1u << 0)

typedef struct {
    const char *data;
    size_t count;
} StringView;

StringView sv(const char *str) {
    return (StringView) {
        .data = str,
        .count = strlen(str),
    };
}

bool sv_cmp(StringView *a, StringView *b) {
    return (a->count == b->count) && memcmp(a->data, b->data, a->count) == 0;
}

bool sv_scmp(StringView *a, const char *b) {
    return (a->count == strlen(b)) && memcmp(a->data, b, a->count) == 0;
}

bool sv_starts_with(StringView *a, const char *prefix) {
    size_t plen = strlen(prefix);
    return a->count >= plen && memcmp(a->data, prefix, plen) == 0;
}

bool sv_tolh(StringView *sv, long *out) {
    if (!out || sv->count == 0 || sv->count >= 64) return false;
    char buff[64];
    memcpy(buff, sv->data, sv->count);
    buff[sv->count] = '\0';
    errno = 0;
    char *end = NULL;
    unsigned long v = strtoul(buff, &end, 16);
    if (errno != 0 || end == buff || *end != '\0') return false;
    *out = v;
    return true;
}

int tokenize(char *str, StringView *argv, size_t *argc) {
    static const char *ws = " \t";
    *argc = 0;

    while (*str) {
        str += strspn(str, ws);
        if (!*str) break;
        if (*argc >= CLI_MAX_TOKENS) {
            printf("   Exceeded argument limit: %d\n", CLI_MAX_TOKENS);
            return -1;
        }

        argv[*argc].data = str;
        argv[*argc].count = strcspn(str, ws);
        str += argv[*argc].count;
        (*argc)++;
    }

    return 0;
}

int main(void) {
    int fd = open_serial("/dev/ttyUSB0", B115200);
    if (fd < 0) { perror("open_serial"); return 1; }

    char *line = NULL;
    size_t n = 0;

    uint8_t buf_arr[4096];
    uint8_t *buf = NULL;
    uint8_t pld[3];
    ssize_t buf_len = 0;
    Packet p = { .hdr = PKT_HDR };

    while (1) {
        printf("[mt6701]> ");
        fflush(stdout);

        ssize_t r = getline(&line, &n, stdin);
        if (r == -1) {
            if (feof(stdin)) break;
            clearerr(stdin);
            continue;
        }

        while (r > 0 && (line[r-1] == '\n' || line[r-1] == '\r')) {
            line[--r] = '\0';
        }
        if (r == 0) {
            printf ("Trying to receive packets..\n");
            buf = &buf_arr[0];
            buf_len = read(fd, buf, 4069);
            if (buf_len < 5) continue;

            // while buffer contains enough data for potential packet, try to parse it
            while (buf_len >= 5) {
                Packet r;
                size_t decoded_len;
                // if not a valid packet, advance pointer by 1 byte and try again
                if (buf[0] != PKT_HDR || !(decoded_len = decode_packet(buf, buf_len, &r))) {
                    buf++;
                    buf_len--;
                    continue;
                }
                client_handle_packet(&r);

                buf += decoded_len;
                buf_len -= decoded_len;
            }
            continue;
        }

        StringView argv[CLI_MAX_TOKENS];
        size_t argc = 0;
        if (tokenize(line, argv, &argc) == 0 && argc > 0) {
            StringView *cmd = &argv[0];
            if (sv_scmp(cmd, "ang")) {
	            p.cmd = READ_SENSOR,
                p.len = 0;

                printf("Sending packet...\n");
                client_send_packet(fd, &p);
            }
            else if (sv_scmp(cmd, "reg")) {
                if (argc < 2) continue;
                long reg;
                if (!sv_tolh(&argv[1], &reg)) continue;
                pld[0] = (uint8_t)reg;
                p.cmd = READ_CONFIG,
                p.len = 1;
	            p.pld = &pld[0];

                printf("Sending packet...\n");
                client_send_packet(fd, &p);
            }
            else if (sv_scmp(cmd, "wreg")) {
                if (argc < 3) continue;
                long reg, val;
                if (!sv_tolh(&argv[1], &reg)) continue;
                if (!sv_tolh(&argv[2], &val)) continue;
                pld[0] = (uint8_t)reg;
                pld[1] = (uint8_t)(val >> 8);
                pld[2] = (uint8_t)(val & 0xFF);
                p.cmd = WRITE_CONFIG,
                p.len = 3;
	            p.pld = &pld[0];

                printf("Sending packet...\n");
                client_send_packet(fd, &p);
            }
        }
    }

    free(line);
    close(fd);
    return 0;
}