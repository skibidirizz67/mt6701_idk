#include "shared.h"

uint16_t crc16_ccitt(const uint8_t *buf, size_t buf_len) {
    uint16_t cksum = 0xFFFF;
	for (size_t i = 0; i < buf_len; i++) {
		cksum = crc16_tab[((cksum>>8) ^ *buf++) & 0xFF] ^ (cksum << 8);
	}
	return cksum;
}

// compare values in last 2 bytes of a buffer to a computed checksum
bool check_crc(const uint8_t *buf, size_t buf_len) {
	if (!buf || buf_len < 3) return false;
	return (((uint16_t)buf[buf_len - 2] << 8) | buf[buf_len - 1]) ==
		crc16_ccitt(buf, buf_len - 2);
}

/* turn Packet struct into stream of bytes
 * Parameters:
 *  const Packet *p -- pointer to packet to encode
 *  uint8_t *buf    -- buffer to write bytes into
 *  size_t buf_len  -- size of the buf
 * Returns: on success - number of bytes, written into buffer; on failure - zero
*/
size_t encode_packet(const Packet *p, uint8_t *buf, size_t buf_len) {
	if (!p || !buf) return 0;
	if (buf_len < (size_t)(MIN_PKT_LEN + p->len)) return 0;

	buf[0] = p->hdr;
	buf[1] = p->cmd;
	buf[2] = p->len;
	if (p->len > 0) memcpy(buf + 3, p->pld, p->len);
	buf += 3 + p->len;
	buf[0] = (uint8_t)(p->crc >> 8);
	buf[1] = (uint8_t)(p->crc & 0xFF);

	return MIN_PKT_LEN + p->len;
}

/* turn stream of bytes into Packet struct
 * Parameters:
 *  uint8_t *buf   -- buffer with stream of bytes to decode
 *  size_t buf_len -- size of the buf
 *  Packet *p      -- packet to write data into
 * Returns: on success - size of the Packet int bytes; on failure - zero
*/
size_t decode_packet(uint8_t *buf, size_t buf_len, Packet *p) {
	if (!buf || !p) return 0;
	if (buf_len < MIN_PKT_LEN || buf_len < (size_t)(MIN_PKT_LEN + buf[2])) return 0;

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