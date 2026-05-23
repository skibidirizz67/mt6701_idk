# Protocol 

 * Baud rate: 115200
 * Configuration: 8N1 (8 data bits, no parity, 1 stop bit)
 * Flow control: none

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

### Header: 0x55

## Commands

Available commands / packet types and their description

### READ_SENSOR (0x00)

If received by server, treated like request to get encoder angle position data.  
Should not contain any payload.  
Server should respond with angle data.  


If recived by client, treated like angle position data.  
Should contain 2-byte payload in form of unprocessed angle data.  
Client should not respond in any way (just print out the value).  

### READ_CONFIG (0x01)

If received by server, treated like request to get encoder configuration.  
Should contain 1-byte payload in form of a configuration option.  
Server should respond with the same configuration option and it's value.  
Following options are available:

```
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
```  


If received by client, treated like configuration option information.  
Should contain 3-byte payload in form of a configuration option (1 byte) and a value (2 bytes).  
Client should not respond in any way (just print out the value).  

### WRITE_CONFIG (0x02)

If received by server, treated like command to modify encoder configuration.  
Should contain 3-byte payload in form of a configuration option (1 byte) and a value (2 bytes).  
Out-of-range values will be automatically truncated by i2c_mt6701 server component.  
Server should not respond in any way (just update config).  


Ignored by client.

# Other

## Server

Server is an ESP32 board connected to MT6701 magnet encoder (I2C).  
It constantly listents for incoming packets on UART port 0 (the USB one) and responds accordingly; Tx is GPIO 1 and Rx is GPIO 3 (default)  

## Client

Client is any machine that can run the client code (probably only Linux distros) and is connected to a Server over UART.  
Connection is established using termios library, and communication is done using standard unistd.h functions.  
Client is controlled by user using terminal. On command, client reads the file descripitor and parses any packets available, or sends request packets.  
USB port is hard-coded to /dev/ttyUSB0  

### Client commands:

 * \n (newline) -- read file descriptor and parse any available packets; prints out information like angle
 * a -- send request to get angle sensor value (you should spam \n a few times to get the response)
 * r -- send request to get ABZ_RES config value (you should spam \n a few times to get the response)
 * u -- send request to get UVW_MUX config value (you should spam \n a few times to get the response)
 * h -- send request to get HYST config value (you should spam \n a few times to get the response)
 * H -- send request to modify HYST config value (the new value is hardcoded to be 0x05)
