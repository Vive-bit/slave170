#include <Arduino.h>

// ==== SLAVE ID ====
static constexpr uint8_t SLAVE_ID = 0x01;

// ==== Pins (ESP32 sicher) ====
static constexpr uint8_t STATUS_LED_PIN = 2;   // onboard-LED (GPIO2)
static constexpr uint8_t DE_RE_PIN = 4;        // RS485 DE/RE

static constexpr uint8_t UART_TX = 16;         // DI → MAX485 DI
static constexpr uint8_t UART_RX = 17;         // RO ← MAX485 RO

// ==== UART + Timing ====
static constexpr unsigned long BAUDRATE = 9600;
static constexpr unsigned long IDLE_INTERVAL = 1500;
static constexpr unsigned long TIMEOUT_INTERVAL = 450;
static constexpr unsigned long ERROR_INTERVAL = 3000;
static constexpr unsigned long SUCCESS_INTERVAL = 100;
static constexpr unsigned long LAST_PING_TIMEOUT = 13000;

// ==== Frame Constants ====
static constexpr uint8_t START_BYTE = 0x7E;
static constexpr uint8_t FRAME_TYPE_REQUEST = 0x10;
static constexpr uint8_t FRAME_TYPE_RESPONSE = 0x20;
static constexpr uint8_t FRAME_LENGTH_REQUEST = 13;
static constexpr uint8_t FRAME_LENGTH_RESPONSE = 10;

// ==== ENUMS ====
enum BlinkState { IDLE, TIMEOUT, ERROR, SUCCESS };
enum Operation : uint8_t { XPING = 0x01, XREAD = 0x02, XWRITE = 0x03 };
enum Mode : uint8_t { XDIGITAL = 0x01, XANALOG = 0x02 };
enum RequestType : uint8_t { XGET = 0x01, XSEND = 0x02 };

// ==== State ====
BlinkState currentBlinkState = IDLE;
unsigned long lastToggle = 0;
unsigned long modeEndTime = 0;
unsigned long currentBlinkInterval = TIMEOUT_INTERVAL;
unsigned long lastPingTime = 0;

// ==== CRC16 ====
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

// ==== PWM Ersatz für analogWrite ====
void analogWriteESP32(uint8_t pin, uint8_t val) {
    static bool initialized = false;
    if (!initialized) {
        ledcSetup(0, 5000, 8);
        ledcAttachPin(pin, 0);
        initialized = true;
    }
    ledcWrite(0, val);
}

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

void updateBlink() {
    unsigned long now = millis();

    if (currentBlinkState == IDLE && (now - lastPingTime) >= LAST_PING_TIMEOUT)
        setCurrentBlinkState(TIMEOUT);

    if ((currentBlinkState == ERROR || currentBlinkState == SUCCESS) && (now >= modeEndTime + (currentBlinkInterval >> 3)))
        setCurrentBlinkState(IDLE);

    if (now - lastToggle >= currentBlinkInterval) {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        lastToggle = now;
    }
}

// ==== RS485 ====
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
    Serial1.write(frame, FRAME_LENGTH_RESPONSE);
    Serial1.flush();
    setTransmitMode(false);
}

bool handleRequest(const uint8_t* frame) {
    uint16_t request_id = (frame[2] << 8) | frame[3];
    RequestType reqType = static_cast<RequestType>(frame[5]);
    Operation op = static_cast<Operation>(frame[6]);
    Mode mode = static_cast<Mode>(frame[7]);
    uint16_t new_state = (frame[8] << 8) | frame[9];
    uint8_t gpio_id = frame[10];

    if (op == XPING)
        lastPingTime = millis();

    if (reqType != XGET || gpio_id == DE_RE_PIN || gpio_id == STATUS_LED_PIN) {
        setCurrentBlinkState(ERROR);
        return false;
    }

    switch (op) {
        case XPING:
            sendResponseFrame(request_id, 0x01);
            setCurrentBlinkState(SUCCESS);
            return true;

        case XREAD: {
            uint16_t val = (mode == XDIGITAL) ? digitalRead(gpio_id) :
                            (mode == XANALOG) ? analogRead(gpio_id) : 0;
            if (mode != XDIGITAL && mode != XANALOG) {
                setCurrentBlinkState(ERROR);
                return false;
            }
            sendResponseFrame(request_id, val);
            setCurrentBlinkState(SUCCESS);
            return true;
        }

        case XWRITE:
            if (mode == XDIGITAL)
                digitalWrite(gpio_id, new_state ? HIGH : LOW);
            else if (mode == XANALOG)
                analogWriteESP32(gpio_id, new_state);
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

void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(200);
    digitalWrite(STATUS_LED_PIN, LOW);

    pinMode(DE_RE_PIN, OUTPUT);
    digitalWrite(DE_RE_PIN, LOW);

    for (uint8_t i = 0; i < SLAVE_ID; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH); delay(80);
        digitalWrite(STATUS_LED_PIN, LOW); delay(80);
    }

    Serial1.begin(BAUDRATE, SERIAL_8N1, UART_RX, UART_TX);
}

void loop() {
    updateBlink();

    while (Serial1.available() && Serial1.peek() != START_BYTE) {
        Serial1.read();
    }

    if (Serial1.available() >= FRAME_LENGTH_REQUEST) {
        uint8_t frame[FRAME_LENGTH_REQUEST];
        Serial1.readBytes(frame, FRAME_LENGTH_REQUEST);

        if (frame[0] != START_BYTE || frame[1] != FRAME_TYPE_REQUEST || frame[4] != SLAVE_ID) {
            setCurrentBlinkState(ERROR);
        } else {
            uint16_t crc_recv = (frame[12] << 8) | frame[11];
            if (crc16(frame, FRAME_LENGTH_REQUEST - 2) != crc_recv) {
                setCurrentBlinkState(ERROR);
            } else {
                handleRequest(frame);
            }
        }
    }
}
