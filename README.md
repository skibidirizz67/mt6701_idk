# Protocol 

Baud rate: 115200
Configuration: 8N1 (8 data bits, no parity, 1 stop bit)
Flow control: none

## Packet layout

```c
typedef struct Packet {
	uint8_t hdr;  // header / magic byte / SOF 
	uint8_t cmd;  // packet type / command
	uint8_t len;  // length of payload in bytes
	uint8_t *pld; // payload (e.g. config register and data to write)
	uint16_t crc; // CRC16-CCITT-FALSE checksum
} Packet;
```

## Header: 0x55

## Commands

Available commands / packet types and their description

### READ_SENSOR (0x00)

If received by server, treated like request to get encoder angle position data.
Should not contain any payload.


If recived by client, treated like angle position data.
Should contain 2-byte payload in form of unprocessed angle data.

### READ_CONFIG (0x01)

If received by server, treated like request to get encoder configuration.
Should contain 1-byte payload in form of a configuration option.
Following options are available:

 * UVW_MUX          (0x00)
 * ABZ_MUX          (0x01)
 * DIR              (0x02)
 * UVW_RES          (0x03)
 * ABZ_RES          (0x04)
 * HYST             (0x05)
 * Z_PULSE_WIDTH    (0x06)
 * ZERO             (0x07)
 * PWM_FREQ         (0x08)
 * PWM_POL          (0x09)
 * OUT_MODE         (0x0A)
 * A_START          (0x0B)
 * A_STOP           (0x0C)


If received by client, treated like configuration option information.
Should contain 3-byte payload in form of a configuration option (1 byte) and a value (2 bytes).

### WRITE_CONFIG (0x02)

If received by server, treated like command to modify encoder configuration.
Should contain 3-byte payload in form of a configuration option (1 byte) and a value (2 bytes).
Out-of-range values will be automatically truncated by i2c_mt6701 server component


Ignored by client.