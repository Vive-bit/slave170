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
- `BAUDRATE`: Any (currently `38400`)

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
- `#define DEBUG_MODE`: Enabled debugging, Serial baud 38400 will be used to transmit some logs. (Some controllers use the same Serial ports to communicate w the module I use, If uploading code fails, remove pin connection of TX/RX!)

## Frame to SLAVE (from MASTER)
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

[PAYLOAD] = 0...MAX bytes
- variable

[END / CRC] = 2 bytes
- 1b `CRC LOW`
- 1b `CRC HIGH`

# Frames
## Frame to MASTER (from SLAVE)
- `START_BYTE` is always `0x7E` (TO MASTER)
- `FRAME_TYPE` is always `0x20` (FRAME_TYPE_RESPONSE)
- Always contains the `4-bytes` payload header as below
  
```
[START_BYTE][FRAME_TYPE][LENGTH]
[REQUEST_ID_HI][REQUEST_ID_LO][SLAVE_ID][REQUEST_TYPE][...]
[CRC_LO][CRC_HI]
```
[HEADER] = 3 bytes
- 1b `START_BYTE`: 0xAA OR 0x7E (`0xAA = to slave` and `0x7E = to master`)
- 1b `FRAME_TYPE`: 0x10 OR 0x20 (`0x10 = FRAME_TYPE_REQUEST` (always by master) and `0x20 = FRAME_TYPE_RESPONSE` (always by slave))
- 1b `LENGTH`: <until-max-length> (All the following bytes)

[PAYLOAD HEADER] = 4 bytes
- 2b `REQUEST_ID`: 0-65535 (Set by master, always 0 if invoked by slave)
- 1b `SLAVE_ID`: 0-254 (Desired slave, `0xFF`/`255` = Broadcast)
- 1b `REQUEST_TYPE`: XSLAVE (Request is by slave) = 0x02, XERROR (Request is an error by slave) = 0x03

[PAYLOAD] = 0...MAX bytes
- variable

[END / CRC] = 2 bytes
- 1b `CRC LOW`
- 1b `CRC HIGH`

# Available Operation types
## `XPING`: Mainly used to ensure life of slave. Returns:
```
[VERSION][RUNTIME_LO][RUNTIME_HI]
```
[PAYLOAD] = 3 bytes
- 1b `VERSION`: Current slave version as hex
- 2b `RUNTIME`: Active time since program startup (Format: Minutes, maximum results as `0xFFFF`)

## `XREAD`: Current value of a pin. Returns:
```
[VALUE_LO][VALUE_HI]
```
[PAYLOAD] = 2 bytes
- 2b `VALUE`: Current pin value, if the `Mode` from the payload header is invalid, returns `0`

## `XWRITE`: Writes a new value to a pin. Returns:
```
[NEW_VALUE_LO][NEW_VALUE_HI]
```
[PAYLOAD] = 2 bytes
- 2b `NEW_VALUE_HI`: Returns the written value

# Send-delay
- RS485 Half-Duplex `only 1 Participant sends at a time`
- Master sends request `DE >> TX`
- Slave can only send if Master is listening -> `wait for master DE >> RX`
- Sending too fast -> ` Data collission / Startbyte gone / CRC Error`

## Calculation
BIT_US      = 1_000_000 / BAUDRATE
BYTE_US     = BIT_US * 10 `10 = (Start + 8 Data bytes + Stop)`
Delay_US    = (frameLen + BYTE_OFFSET) * BYTE_US + FUDGE_US

## Parameters
frameLen    = actual length of the recieved frame `Start/Payload header/Payload/CRC`
BYTE_OFFSET = Security-Bytes for FIFO + DE/RE Switch
FUDGE_US    = Set value for Interrupt-Latency / OS-Jitter

## Example @38400 Baud
BIT_US  = 26 µs
BYTE_US = 260 µs
frameLen = 11 Bytes
BYTE_OFFSET = 7
FUDGE_US = 60000
Delay_US = (11 + 7) * 260 + 60000 ≈ 64680 µs (~64.7 ms)

## Higher baudrate -> less Bit-time -> shorter waiting time
9600 Baud ≈ ~79 ms
38400 Baud ≈ ~65 ms
115200 Baud ≈ ~61 ms

- Too short waiting time -> `Packet loss!!!`
- Waiting time too high -> `wasted protocol latency potential`
