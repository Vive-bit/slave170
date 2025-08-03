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
