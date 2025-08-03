# Slave170 - RS485 Protocol 170 Slave
Slave170 is a firmware that acts as a slave in my 'Protocol 170'. It responds to master requests and signals status via an LED and the RS485 module.
Currently used as a slave for my master.
Called "Protocol170" because the START_BYTE for the SLAVES is 0xAA... 170!

## Requirements

- Arduino MEGA (ATmega2560)  / Arduino UNO / ESP32
- Arduino-CLI or Arduino IDE  
- Connection to UART (Serial1) / Serial (and other pins)
- MAX485 (RS485) Module (5V Use case!)

## Configuration
- `SLAVE_ID`: 0-254 possible IDs `(0xFF = Broadcast)`
- `BAUDRATE`: Default 9600

## Bahavior & LED Status
| State       | Type    | Meaning                                  |
| ----------- | ------- | ---------------------------------------- |
| **IDLE**    | OK      | Waiting for request (ready to operate)   |
| **TIMEOUT** | WARNING | No pings received recently               |
| **SUCCESS** | OK      | Request received and answered correctly  |
| **ERROR**   | ERROR   | Invalid frame, CRC error, ...            |
  
## Troubleshooting
- **No LED-activity**: Check BAUDRATE and connections to Serial1 (also verify if any delays in the program).
- **Always ERROR**: Check test frame, verify correct CRC calculation, ...
- **MAKE SURE**: Always sync internal params with master etc.
- **WARNING**: There are no plausibility checks for if the requested pin exists, ... etc.

## Debugging
- `#define DEBUG_MODE`: Enabled debugging, Serial baud 9600 will be used to transmit some logs. (Some controllers use the same Serial ports to communicate w the module I use, If uploading code fails, remove pin connection of TX/RX!)

### TO SLAVE (from MASTER)
- `START_BYTE` is always `0xAA` (TO SLAVE)
- `FRAME_TYPE` is always `0x10` (FRAME_TYPE_REQUEST)
- Always contains the `6-bytes` payload header as below
  
```
[START_BYTE][FRAME_TYPE][LENGTH]
[REQUEST_ID_HI][REQUEST_ID_LO][SLAVE_ID][REQUEST_TYPE][OPERATION][MODE][...]
[CRC_LO][CRC_HI]
```
[HEADER] = 3 bytes
- 1b `START_BYTE`: 0xAA OR 0x7E (`0xAA = to slave` and `0x7E = to master`)
- 1b `FRAME_TYPE`: 0x10 OR 0x20 (`0x10 = FRAME_TYPE_REQUEST` (always by master) and `0x20 = FRAME_TYPE_RESPONSE` (always by slave))
- 1b `LENGTH`: <until-max-length> (All the following bytes)

[PAYLOAD HEADER] = 6 bytes
- 2b `REQUEST_ID`: 0-65535 (Set by master, always 0 if invoked by slave)
- 1b `SLAVE_ID`: 0-254 (Desired slave, `0xFF`/`255` = Broadcast)
- 1b `REQUEST_TYPE`: XMASTER (Request is by master) = 0x01, XSLAVE (Request is by slave) = 0x02, XERROR (Request is an error by slave) = 0x03
- 1b `OPERATION`: XPING = 0x01, XREAD = 0x02, XWRITE = 0x03 (All available operations)
- 1b `MODE`: XDIGITAL = 0x01, XANALOG = 0x02 (All available modes. Used for master requests)

[PAYLOAD] = 0-MAX bytes
- variable

[END / CRC] = 2 bytes
- 1b `CRC LOW`
- 1b `CRC HIGH`

### TO MASTER (from SLAVE)
- `START_BYTE` is always `0xAA` (TO SLAVE)
- `FRAME_TYPE` is always `0x10` (FRAME_TYPE_REQUEST)
- Always contains the `6-bytes` payload header as below
  
```
[START_BYTE][FRAME_TYPE][LENGTH]
[REQUEST_ID_HI][REQUEST_ID_LO][SLAVE_ID][REQUEST_TYPE][OPERATION][MODE][...]
[CRC_LO][CRC_HI]
```
[HEADER] = 3 bytes
- 1b `START_BYTE`: 0xAA OR 0x7E (`0xAA = to slave` and `0x7E = to master`)
- 1b `FRAME_TYPE`: 0x10 OR 0x20 (`0x10 = FRAME_TYPE_REQUEST` (always by master) and `0x20 = FRAME_TYPE_RESPONSE` (always by slave))
- 1b `LENGTH`: <until-max-length> (All the following bytes)

[PAYLOAD HEADER] = 6 bytes
- 2b `REQUEST_ID`: 0-65535 (Set by master, always 0 if invoked by slave)
- 1b `SLAVE_ID`: 0-254 (Desired slave, `0xFF`/`255` = Broadcast)
- 1b `REQUEST_TYPE`: XMASTER (Request is by master) = 0x01, XSLAVE (Request is by slave) = 0x02, XERROR (Request is an error by slave) = 0x03
- 1b `OPERATION`: XPING = 0x01, XREAD = 0x02, XWRITE = 0x03 (All available operations)
- 1b `MODE`: XDIGITAL = 0x01, XANALOG = 0x02 (All available modes. Used for master requests)

[PAYLOAD] = 0-MAX bytes
- variable

[END / CRC] = 2 bytes
- 1b `CRC LOW`
- 1b `CRC HIGH`
