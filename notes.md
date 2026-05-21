=== UART binary protocol sketch ===

Baud rate: 115200
Parameters: 8N1
Flow control: no flow control #TODO investigate
Header: 0xAA55
Checksum: CRC16-CCITT-FALSE; poly 0x1021; init 0xFFFF; xorout 0x0000; no reflection
Bit order: LSB first

Layout:
[HDR][CMD][SEQ][RSP][LEN][PLD][CRC]

Field description
name    bytes   description
===========================
HDR     2       packet header (0xAA55)
CMD     1       command id / packet type id
SEQ     1       packet sequence number (0-255); response should have the same SEQ as request
RSP     1       response code
LEN     1       payload size (0-255)
PLD     LEN     payload, contents determined by CMD
CRC     2       CRC16-CCITT-FALSE checksum

Request commands table:
code    name    description             PLD len     PLD
=======================================================
0x00    PING    ping                    0           -
0x01    SREAD   read sensor             0           -
0x02    MSTART  start monitoring sensor 2           sampling rate in Hz | count of packets
0x03    -       reserved                -           -
0x04    MSTOP   stop monitoring sensor  0           -
0x05    CREAD   read EEPROM (config)    1           register to read
0x06    CWRITE  write EEPROM (config)   1+value     register to write | value to write
0x07    REBOOT  reboot encoder          0           -
P.S.  EEPROM value length ranges from 1 to 2 bytes depending on register
#TODO determine upper limit of sampling rate

Response commands table:
code    name    description         PLD len     PLD
===================================================
0x80    PING_   ping                0           -
0x81    SREAD_  read sensor         2           sensor value
0x82    MSTART_ start monitor ACK   2           sampling rate in Hz | count of packets
0x83    MDATA_  monitor data        2           sensor value
0x84    MSTOP_  stop monitor ACK    0           -   
0x85    CREAD_  read EEPROM         1+value     read register | read value
0x86    CWRITE_ write EEPROM        1+value     modified register | written value
0x87    REBOOT_  reboot encoder     0           -
P.S.  EEPROM value length ranges from 1 to 2 bytes depending on register
PP.S. In case of a bad response code, payload can be different
3P.S. Reboot encoder response is sent AFTER reboot is complete
4P.S. all MDATARs should have the same SEQ as the initial request

Response / Status codes table:
code    name        description
===============================
0x00    OK          operation succesfull
0x01    ERR         generic error
0x02    BAD_CMD     invalid command code
0x03    BAD_REG     invalid register in the payload
0x04    BAD_VAL     invalid value to write in the payload
P.S. Request should always have OK status code (but responser doesn't check it)

EEPROM / config register list:
code    name            len     valid values
============================================
0x00    UVW_MUX         1       0x00   - 0x01
0x01    ABZ_MUX         1       0x00   - 0x01
0x02    DIR             1       0x00   - 0x01
0x03    UVW_RES         1       0x00   - 0x0F
0x04    ABZ_RES         2       0x0000 - 0x03FF
0x05    HYST            1       0x00   - 0x07
0x06    Z_PULSE_WIDTH   1       0x00   - 0x07
0x07    ZERO            2       0x0000 - 0x0FFF
0x08    PWM_FREQ        1       0x00   - 0x01
0x09    PWM_POL         1       0x00   - 0x01
0x0A    OUT_MODE        1       0x00   - 0x01
0x0B    A_START         2       0x0000 - 0x0FFF
0x0C    A_STOP          2       0x0000 - 0x0FFF
P.S.  Returned values are encoded and should be decoded in-place
PP.S. For detailed description and value tables refer to MT6701 encoder manual

Request packet examples:
HDR     CMD     SEQ     RSP     LEN     PLD         CRC     Description
=======================================================================
0xAA55  0x00    0x00    0x00    0x00    -           0x5F5F  ping
0xAA55  0x01    0x05    0x00    0x00    -           0xC21B  request to get sensor value
0xAA55  0x02    0x69    0x00    0x02    0x0AFF      0x1020  request to start monitoring sensor with sampling rate of 10 Hz and 256 packets in total (i.e. the stream will continue for 25.6 seconds if no MSTOP is sent)
0xAA55  0x04    0xAB    0x00    0x00    -           0xD8C3  request to stop monitoring sensor
0xAA55  0x05    0xFF    0x00    0x01    0x01        0xBFFE  request to read ABZ_MUX value
0xAA55  0x05    0x8C    0x00    0x01    0x05        0x26D3  request to read HYST value
0xAA55  0x05    0xE4    0x00    0x01    0x0B        0x1B0C  request to read A_START value
0xAA55  0x06    0x1F    0x00    0x02    0x0201      0x41FA  request to write value 0x01 to DIR
0xAA55  0x06    0x7D    0x00    0x03    0x0401FF    0xAC59  request to write value 0x01FF to ABZ_RES
0xAA55  0x07    0x30    0x00    0x00    -           0xCBD7  request to reboot encoder
Wrong request packet examples:
=======================================================================
0xAA69  0x01    0x05    0x00    0x00    -           0x45DE  request to get sensor value with wrong HDR
0xAA55  0x05    0x8C    0x00    0x01    0x00        0x7676  request to read UVW_MUX value
0xAA55  0x3C    0x00    0x00    0x00    -           0x3C84  request with invalid command code
0xAA55  0x05    0x8C    0x00    0x01    0xAB        0x72F7  request to read wrong value
P.S. If requestor receives wrong response or no response, they just retry

Matching response examples:
HDR     CMD     SEQ     RSP     LEN     PLD         CRC     Description
=======================================================================
0xAA55  0x80    0x00    0x00    0x00    -           0x8267  ping-back on request
0xAA55  0x81    0x05    0x00    0x02    0x00D2      0x8213  requested sensor value
0xAA55  0x82    0x69    0x00    0x02    0x0AFF      0xC400  acknowledgement of request to start monitoring sensor with sampling rate of 10 Hz and 256 packets
0xAA55  0x83    0x69    0x00    0x02    0x025C      0x8D80  monitoring sensor value
0xAA55  0x83    0x69    0x00    0x02    0x1BF0      0x400D  monitoring sensor value
0xAA55  0x84    0xAB    0x00    0x00    -           0x05FB  acknowledgement of request to stop monitoring sensor
0xAA55  0x85    0xFF    0x00    0x01    0x00        0x8D0F  requested ABZ_MUX value
0xAA55  0x85    0x8C    0x00    0x01    0x03        0x64C5  requested HYST value
0xAA55  0x85    0xE4    0x00    0x02    0x01FA      0x83B4  requested A_START value
0xAA55  0x86    0x1F    0x00    0x02    0x0201      0x95DA  successfull write of value 0x01 to DIR
0xAA55  0x86    0x7D    0x00    0x03    0x0401FF    0x07A0  successfull write of value 0x01FF to ABZ_RES
0xAA55  0x87    0x30    0x00    0x00    -           0x16EF  encoder rebooted succesfully
Matching response on wrong request packet examples:
=======================================================================
-       -       -       -       -       -           -       will be ignored because of wrong header
-       -       -       -       -       -           -       ignored because received checksum didn't match the data
0xAA55  0xBC    0x00    0x02    0x00    -           0x87DE  requested command is invalid
0xAA55  0x85    0x8C    0x03    0x01    0xAB        0x0977  requested register to read is invalid
0xAA55  0x86    0x7D    0x04    0x03    0x04FF3B    0x27A0  requested to write value out of bounds to ABZ_RES
0xAA55  0x81    0x05    0x01    0x00    -           0x2C12  couldn't get sensor value for some reason
0xAA55  0x87    0x30    0x01    0x00    -           0x25DE  couldn't reboot the encoder for some reason



