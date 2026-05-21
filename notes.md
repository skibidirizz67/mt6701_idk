=== UART binary protocol sketch ===

Baud rate: 115200
Parameters: 8N1
Flow control: no flow control #TODO investigate
Header: 0xAA55
Checksum: CRC16-CCITT-FALSE; poly 0x1021; init 0xFFFF; xorout 0x0000; no reflection

Layout:
[HDR][CMD][CRC]

Field description
name    bytes   description
===========================
HDR     2       packet header (0xAA55)
CMD     1       command id / packet type id
CRC     2       CRC16-CCITT-FALSE checksum

Request commands table:
code    name    description
===========================
0x01    SREAD   read sensor

Response commands table:
code    name    description
===========================    
0x81    SREAD_  read sensor 

Request packet examples:
HDR     CMD     CRC     Description
=======================================================================
0xAA55  0x01    0xC21B  request to get sensor value

Matching response examples:
HDR     CMD     CRC     Description
=======================================================================
0xAA55  0x81   0x8213  requested sensor value


