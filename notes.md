=== UART binary protocol sketch ===

Parameters: 8N1

Layout:
[HDR][CMD][RSP][LEN][PLD][CRC]

Description of each chunk:
name    bytes   description
===========================
HDR     1       packet header (0x69)
CMD     1       command id
RSP     1       response code
LEN     1       payload size (from 0 to 255)
PLD     LEN     payload, contents determined by CMD
CRC     2       CRC16-CCITT checksum

sequence / id
rsp
ack