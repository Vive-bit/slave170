// ARDUINO UNO

#include <Arduino.h>

// SLAVE_ID //
static constexpr uint8_t SLAVE_ID = 0x01;
//////////////

static constexpr uint8_t START_BYTE = 0x7E;
static constexpr uint8_t FRAME_TYPE_REQUEST = 0x10;
static constexpr uint8_t FRAME_TYPE_RESPONSE = 0x20;
static constexpr uint8_t FRAME_LENGTH_REQUEST = 13;
static constexpr uint8_t FRAME_LENGTH_RESPONSE = 10;
static constexpr uint8_t STATUS_LED_PIN = 4;
static constexpr uint8_t DE_RE_PIN = 2;

const unsigned long IDLE_INTERVAL = 1500;
const unsigned long TIMEOUT_INTERVAL = 450;
const unsigned long ERROR_INTERVAL = 3000;
const unsigned long SUCCESS_INTERVAL = 100;

static constexpr unsigned long MAX_ON_TIME = 30000;
const unsigned long LAST_PING_TIMEOUT = 13000; // master sends every 10s
static constexpr unsigned long BAUDRATE = 9600; // master

enum BlinkState { IDLE, TIMEOUT, ERROR, SUCCESS };
// ENUMS: HANDSHAKE IMPORTANT //
enum Operation : uint8_t {
    XPING = 0x01,
    XREAD = 0x02,
    XWRITE = 0x03
};
enum Mode : uint8_t {
    XDIGITAL = 0x01,
    XANALOG = 0x02
};
enum RequestType : uint8_t {
    XGET = 0x01,
    XSEND = 0x02
};
/////////////////////////////////

struct Expiry {
  uint8_t pin;
  unsigned long offTime;
};
Expiry expiries[48]; // tracking up to 48 pins 
uint8_t expiryCount = 0;

// CRC16
uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

// LED
BlinkState currentBlinkState = IDLE;
unsigned long lastToggle = 0;
unsigned long modeEndTime = 0;
unsigned long currentBlinkInterval = TIMEOUT_INTERVAL;
unsigned long lastPingTime = 0;

void setCurrentBlinkState(BlinkState state) {
    currentBlinkState = state;

    switch (state) {
        case IDLE: currentBlinkInterval = IDLE_INTERVAL; break;
        case TIMEOUT: currentBlinkInterval = TIMEOUT_INTERVAL; break;
        case ERROR: currentBlinkInterval = ERROR_INTERVAL; break;
        case SUCCESS: currentBlinkInterval = SUCCESS_INTERVAL; break;
    }

    digitalWrite(STATUS_LED_PIN, LOW);
    lastToggle = millis();
    modeEndTime = (state == ERROR || state == SUCCESS) ? lastToggle + currentBlinkInterval : 0;
}

void setTransmitMode(bool enable) {
    digitalWrite(DE_RE_PIN, enable ? HIGH : LOW);
}

void sendResponseFrame(uint16_t request_id, uint16_t data_value) {
    uint8_t frame[FRAME_LENGTH_RESPONSE];

    frame[0] = START_BYTE;
    frame[1] = FRAME_TYPE_RESPONSE;
    frame[2] = (request_id >> 8) & 0xFF;
    frame[3] = request_id & 0xFF;
    frame[4] = SLAVE_ID;
    frame[5] = XSEND;
    frame[6] = (data_value >> 8) & 0xFF;
    frame[7] = data_value & 0xFF;

    uint16_t crc = crc16(frame, 8);
    frame[8] = crc & 0xFF;
    frame[9] = (crc >> 8) & 0xFF;

    delay(20);
    setTransmitMode(true);
    Serial.write(frame, FRAME_LENGTH_RESPONSE);
    Serial.flush();
    setTransmitMode(false);
}

void updateBlink() {
  unsigned long now = millis();

  // no ping? go status TIMEOUT
  if (currentBlinkState == IDLE && (now - lastPingTime) >= LAST_PING_TIMEOUT) {
    setCurrentBlinkState(TIMEOUT);
  }
  
  // fallback SUCCESS and ERROR to IDLE
  if ((currentBlinkState == ERROR || currentBlinkState == SUCCESS) && (now >= modeEndTime + (currentBlinkInterval >> 3))) {
    setCurrentBlinkState(IDLE);
  }

  // LED toggle
  if (now - lastToggle >= currentBlinkInterval) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    lastToggle = now;
  }
}

bool handleRequest(const uint8_t* frame) {
  uint16_t request_id = (frame[2] << 8) | frame[3];
  RequestType reqType = static_cast<RequestType>(frame[5]);
  Operation op = static_cast<Operation>(frame[6]);
  Mode mode = static_cast<Mode>(frame[7]);
  uint16_t new_state = (frame[8] << 8) | frame[9];
  uint8_t gpio_id = frame[10];

  if (op == XPING) {
    lastPingTime = millis();
  }

  if (reqType != XGET) {
    setCurrentBlinkState(ERROR);
    return false;
  }
  if (gpio_id == DE_RE_PIN || gpio_id == STATUS_LED_PIN) {
    setCurrentBlinkState(ERROR);
    return false;
  }

  switch (op) {
    case XPING:
      sendResponseFrame(request_id, 0x01);
      setCurrentBlinkState(SUCCESS);
      return true;

    case XREAD: {
      uint16_t val = (mode == XDIGITAL) ? digitalRead(gpio_id) : (mode == XANALOG)  ? analogRead(gpio_id)  : 0;
      if (mode != XDIGITAL && mode != XANALOG) {
        setCurrentBlinkState(ERROR);
        return false;
      }
      sendResponseFrame(request_id, val);
      setCurrentBlinkState(SUCCESS);
      return true;
    }

    case XWRITE:
      if (mode == XDIGITAL) {
        digitalWrite(gpio_id, new_state ? HIGH : LOW);

        if (new_state) {
            bool found = false;
            for (uint8_t i = 0; i < expiryCount; i++) {
                if (expiries[i].pin == gpio_id) {
                    expiries[i].offTime = millis() + MAX_ON_TIME;
                    found = true;
                    break;
                }
            }
            if (!found && expiryCount < sizeof(expiries)/sizeof(expiries[0])) {
                expiries[expiryCount++] = { gpio_id, millis() + MAX_ON_TIME };
            }
        } else {
            for (uint8_t i = 0; i < expiryCount; i++) {
                if (expiries[i].pin == gpio_id) {
                    expiries[i] = expiries[--expiryCount];
                    break;
                }
              }
          }
      }
      else if (mode == XANALOG) analogWrite(gpio_id, new_state);
      else {
        setCurrentBlinkState(ERROR);
        return false;

      }
      sendResponseFrame(request_id, new_state);
      setCurrentBlinkState(SUCCESS);
      return true;

    default:
      setCurrentBlinkState(ERROR);
      return false;
  }
}

void autoOffCheck() {
    unsigned long now = millis();
    for (int8_t i = expiryCount - 1; i >= 0; i--) {
        if ((long)(now - expiries[i].offTime) >= 0) {
            digitalWrite(expiries[i].pin, LOW);
            expiries[i] = expiries[--expiryCount];
            setCurrentBlinkState(SUCCESS);
        }
    }
}

void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(200);
    digitalWrite(STATUS_LED_PIN, LOW);
    
    pinMode(DE_RE_PIN, OUTPUT);
    digitalWrite(DE_RE_PIN, LOW);

    for (uint8_t i = 0; i < SLAVE_ID; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(80);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(80);
    }

    Serial.begin(BAUDRATE);
    // delay(100);
    // lastPingTime = millis(); // we enter timeout directly
}

void loop() {
    updateBlink();
    autoOffCheck();
    
    while (Serial.available() && Serial.peek() != START_BYTE) {
        Serial.read();
    }
    if (Serial.available() >= FRAME_LENGTH_REQUEST) {
        uint8_t frame[FRAME_LENGTH_REQUEST];
        Serial.readBytes(frame, FRAME_LENGTH_REQUEST);

        if (frame[0] != START_BYTE || frame[1] != FRAME_TYPE_REQUEST || frame[4] != SLAVE_ID) {
            setCurrentBlinkState(ERROR);
        } else {
            uint16_t crc_recv = (frame[12] << 8) | frame[11];
            if (crc16(frame, (FRAME_LENGTH_REQUEST-2)) != crc_recv) {
                setCurrentBlinkState(ERROR);
            } else {
                handleRequest(frame);
            }
        }
    }
}
