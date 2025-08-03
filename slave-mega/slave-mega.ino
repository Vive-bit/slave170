// ARDUINO MEGA

#include <Arduino.h>

// SLAVE_CONFIG
static constexpr uint8_t SLAVE_ID = 0x01;
static constexpr uint8_t STATUS_LED_PIN = 4;
static constexpr uint8_t DE_RE_PIN = 2;
static constexpr unsigned long BAUDRATE = 9600;
static constexpr unsigned long LOG_BAUDRATE = 9600;

// PROTOCOL_HEADER
static constexpr uint8_t START_BYTE          = 0xAA;
static constexpr uint8_t START_BYTE_MASTER   = 0x7E;
static constexpr uint8_t FRAME_TYPE_REQUEST  = 0x10;
static constexpr uint8_t FRAME_TYPE_RESPONSE = 0x20;

// RUNTIME
enum BlinkState { IDLE, TIMEOUT, ERROR, SUCCESS };
enum Operation : uint8_t { XPING = 0x01, XREAD = 0x02, XWRITE = 0x03 };
enum Mode : uint8_t { XDIGITAL = 0x01, XANALOG = 0x02 };
enum RequestType : uint8_t { XMASTER = 0x01, XSLAVE = 0x02, XERROR = 0x03 };
enum ErrorType : uint8_t { XLENGTH_PING = 0x01, XLENGTH_READ = 0x02, XLENGTH_WRITE = 0x03, MISSING_OPERATION = 0x04, XLENGTH_MAX_FRAME = 0x05, XPING_TIMEOUT = 0x06, XTIMELIMIT_EXCEEDED = 0x07, XRESENDING_FRAME = 0x08 };

const unsigned long IDLE_INTERVAL = 1500;
const unsigned long TIMEOUT_INTERVAL = 450;
const unsigned long ERROR_INTERVAL = 3000;
const unsigned long SUCCESS_INTERVAL = 100;

// WATCHDOG
static constexpr unsigned long MAX_ON_TIME = 30000;
static constexpr unsigned long LAST_PING_TIMEOUT = 13000;

// RAM-Buffer
static constexpr uint8_t MAX_FRAME_LEN = 64;
static constexpr uint8_t MAX_RETRY_ENTRIES = 4;
static constexpr uint8_t DEFAULT_TEMP_MAX_FRAME_SIZE = 32;
static constexpr uint8_t MAX_AUTO_OFF_EXPIRY_RETRIES = 8;

///////////////////////////////////////////////////////////////////////////////

// Auto-off struct
struct Expiry { uint8_t pin; unsigned long offTime; };
Expiry expiries[MAX_AUTO_OFF_EXPIRY_RETRIES];
uint8_t expiryCount = 0;

// Retry-buffer
struct RetryEntry {
  uint8_t data[MAX_FRAME_LEN];
  uint8_t len, retries;
  unsigned long ts;
};
RetryEntry retryBuf[MAX_RETRY_ENTRIES];
uint8_t retryCount = 0;

// State machine
enum RxState { RS_WAIT, RS_FRAME, RS_LEN, RS_PAY, RS_CRC1, RS_CRC2 };
RxState rxState = RS_WAIT;
uint8_t rxLen = 0, rxBuf[MAX_FRAME_LEN], rxPos = 0;
uint16_t rxCrc = 0xFFFF;

// Blink-Status
BlinkState currentBlinkState = IDLE;
unsigned long lastToggle = 0;
unsigned long currentBlinkInterval = TIMEOUT_INTERVAL;
unsigned long modeEndTime = 0;
unsigned long lastPingTime = 0;

// CRC16
void crcUpdate(uint16_t* crc, uint8_t b) {
  *crc ^= b;
  for (uint8_t i = 0; i < 8; i++)
    *crc = (*crc & 1) ? (*crc >> 1) ^ 0xA001 : (*crc >> 1);
}

// LED blink
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
  modeEndTime = (state == ERROR || state == SUCCESS) ? lastToggle + currentBlinkInterval : 0;;
}

// DE/RE
void setTransmitMode(bool en) {
  digitalWrite(DE_RE_PIN, en ? HIGH : LOW);
}

// Initializing
void protocolBegin() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(DE_RE_PIN, OUTPUT);
  Serial1.begin(BAUDRATE);

#ifdef DEBUG_MODE
  Serial.begin(LOG_BAUDRATE);
#endif

  // reset RAM explicitly
  memset(expiries, 0, sizeof(expiries));
  expiryCount = 0;
  memset(retryBuf, 0, sizeof(retryBuf));
  retryCount = 0;

  // SlaveId blinker
  for (uint8_t i = 0; i < SLAVE_ID; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH); delay(80);
    digitalWrite(STATUS_LED_PIN, LOW);  delay(80);
  }

  setCurrentBlinkState(TIMEOUT); // we enter timeout directly
}

// Debug
void debugLog(const char* msg) {
#ifdef DEBUG_MODE
  Serial.println(msg);
#endif
}

// Error event
void sendErrorResponse(uint8_t errorCode, uint16_t reqId = 0) {
  uint8_t frame[DEFAULT_TEMP_MAX_FRAME_SIZE];
  uint8_t header[] = { byte(reqId >> 8), byte(reqId), SLAVE_ID, XERROR };
  uint8_t payload[] = { errorCode };

  uint8_t len = buildFrame(frame, START_BYTE_MASTER, FRAME_TYPE_RESPONSE, header, sizeof(header), payload, sizeof(payload));
  if (len > 0) sendFrame(frame, len);
}

// Auto-off
void autoOffCheck() {
  unsigned long now = millis();
  for (int8_t i = expiryCount - 1; i >= 0; i--) {
    if (now >= expiries[i].offTime) {
      digitalWrite(expiries[i].pin, LOW);
      expiries[i] = expiries[--expiryCount];
      setCurrentBlinkState(ERROR);
      sendErrorResponse(XTIMELIMIT_EXCEEDED); 
      debugLog("REQUEST - Pin active for too long!");
    }
  }
}

// Retries
void processRetries() {
  for (int8_t i = retryCount - 1; i >= 0; i--) {
    auto& e = retryBuf[i];
    if ((millis() - e.ts >= 500) && (e.retries < 3)) {
      sendFrame(e.data, e.len);
      e.retries++;
      e.ts = millis();
    }
    if (e.retries >= 3) {
      retryBuf[i] = retryBuf[--retryCount];
      setCurrentBlinkState(ERROR);
      sendErrorResponse(XRESENDING_FRAME); 
      debugLog("REQUEST - Resending frame!");
    }
  }
}

// Blink-Updater and Ping-Timeout
void updateBlink() {
  unsigned long now = millis();

  // no ping? go status TIMEOUT
  if ((currentBlinkState == IDLE) && (now - lastPingTime >= LAST_PING_TIMEOUT)) { setCurrentBlinkState(TIMEOUT); sendErrorResponse(XPING_TIMEOUT); debugLog("REQUEST - Missing ping requests by master -> TIMEOUT STATUS"); }

  // fallback SUCCESS and ERROR to IDLE
  if (((currentBlinkState == ERROR) || (currentBlinkState == SUCCESS))
      && (now >= modeEndTime + (currentBlinkInterval >> 3)))
    setCurrentBlinkState(IDLE);

  // LED toggle
  if (now - lastToggle >= currentBlinkInterval) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    lastToggle = now;
  }
}

// Send
void sendFrame(const uint8_t* frame, uint8_t len) {
  delay(10);
  setTransmitMode(true);
  Serial1.write(frame, len);
  Serial1.flush();
  setTransmitMode(false);
  debugLog("REQUEST - Frame sent");
}

// Build frame
uint8_t buildFrame(
    uint8_t* dest,
    uint8_t start, uint8_t type,
    const uint8_t* header, uint8_t header_len,
    const uint8_t* payload, uint8_t payload_len)
{
    uint8_t total_len = 3 + header_len + payload_len + 2; // 3 = header, payload header, payload, crc
    if (total_len > DEFAULT_TEMP_MAX_FRAME_SIZE) return 0;
    dest[0] = start;
    dest[1] = type;
    dest[2] = header_len + payload_len + 2; // LEN = payloads + CRC

    for (uint8_t i = 0; i < header_len; i++)
        dest[3 + i] = header[i];
    for (uint8_t i = 0; i < payload_len; i++)
        dest[3 + header_len + i] = payload[i];

    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < 3 + header_len + payload_len; i++)
        crcUpdate(&crc, dest[i]);
    dest[3 + header_len + payload_len] = crc & 0xFF;
    dest[3 + header_len + payload_len + 1] = crc >> 8;

    return total_len;
}

// Request handler
bool handleRequest(const uint8_t* f) {
  if (!f) return false;

  // payload header
  uint16_t reqId = (uint16_t(f[0]) << 8) | f[1];
  uint8_t slave = f[2];
  RequestType type = RequestType(f[3]);
  Operation op  = Operation(f[4]);
  Mode mode     = Mode(f[5]);
  
  if (type != XMASTER) {debugLog("REQUEST - Not by a master"); return false;}

  switch (op) {
    case XPING:
      if (rxLen < 6) { sendErrorResponse(XLENGTH_PING, reqId); setCurrentBlinkState(ERROR); debugLog("REQUEST - Missing payload length for ping type"); return false; }
      break;
    case XREAD:
      if (rxLen < 7) { sendErrorResponse(XLENGTH_READ, reqId); setCurrentBlinkState(ERROR); debugLog("REQUEST - Missing payload length for read type"); return false; }
      break;
    case XWRITE:
      if (rxLen < 9) { sendErrorResponse(XLENGTH_WRITE, reqId); setCurrentBlinkState(ERROR); debugLog("REQUEST - Missing payload length for write type"); return false; }
      break;
    default:
      sendErrorResponse(MISSING_OPERATION, reqId); setCurrentBlinkState(ERROR); debugLog("REQUEST - Missing operation"); return false; break;
  }

  uint8_t frame[DEFAULT_TEMP_MAX_FRAME_SIZE];
  uint8_t header[] = { byte(reqId >> 8), byte(reqId), SLAVE_ID, XSLAVE };

  switch (op) {
    case XPING: {
      lastPingTime = millis();
      uint8_t payload[] = { };
      uint8_t len = buildFrame(frame, START_BYTE_MASTER, FRAME_TYPE_RESPONSE, header, sizeof(header), payload, sizeof(payload));
      if (len > 0) sendFrame(frame, len);
      setCurrentBlinkState(SUCCESS);
      debugLog("REQUEST - Ping OK");
      return true;
    }
    
    case XREAD: {
      uint8_t pin = f[6];
      uint16_t out = (mode == XDIGITAL) ? digitalRead(pin) : (mode == XANALOG) ? analogRead(pin) : 0;
      uint8_t payload[] = { byte(out >> 8), byte(out) };
      uint8_t len = buildFrame(frame, START_BYTE_MASTER, FRAME_TYPE_RESPONSE, header, sizeof(header), payload, sizeof(payload));
      if (len > 0) sendFrame(frame, len);
      setCurrentBlinkState(SUCCESS);
      debugLog("REQUEST - Read OK");
      return true;
    }
    
    case XWRITE: {
      uint16_t val = (uint16_t(f[6]) << 8) | f[7];
      uint8_t pin = f[8];
      if (mode == XDIGITAL) {
        digitalWrite(pin, val ? HIGH : LOW);
        if (val) {
          if (expiryCount < sizeof(expiries) / sizeof(expiries[0])) expiries[expiryCount++] = { pin, millis() + MAX_ON_TIME };
        } else {
          for (uint8_t i = 0; i < expiryCount; i++) {
            if (expiries[i].pin == pin) {
              expiries[i] = expiries[--expiryCount]; break;
            }
          }
        }
      } else if (mode == XANALOG) {
        analogWrite(pin, val);
      } else {
        setCurrentBlinkState(ERROR);
        return false;
      }

      uint8_t payload[] = { byte(val >> 8), byte(val) };
      uint8_t len = buildFrame(frame, START_BYTE_MASTER, FRAME_TYPE_RESPONSE, header, sizeof(header), payload, sizeof(payload));
      if (len > 0) sendFrame(frame, len);
      setCurrentBlinkState(SUCCESS);
      debugLog("REQUEST - Write OK");
      return true;
    }
    
    default:
      return false;
  }
}

void feedByte(uint8_t b) {
  switch (rxState) {
    case RS_WAIT:
      if (b == START_BYTE) { rxState = RS_FRAME; rxPos = 0; rxCrc = 0xFFFF; crcUpdate(&rxCrc, b); }
      break;
    case RS_FRAME:
      if (b == FRAME_TYPE_REQUEST) { rxState = RS_LEN; crcUpdate(&rxCrc, b); } else { rxState = RS_WAIT; }
      break;
    case RS_LEN:
      rxLen = b; rxPos = 0;
      crcUpdate(&rxCrc, b);
      rxState = (rxLen > 4 && rxLen <= MAX_FRAME_LEN) ? RS_PAY : RS_WAIT;
      break;
    case RS_PAY:
      if (rxPos < rxLen - 2 && rxPos < MAX_FRAME_LENGTH) { rxBuf[rxPos++] = b; crcUpdate(&rxCrc, b); } else {debugLog("REQUEST - Exceeded MAX_FRAME_LEN!"); sendErrorResponse(XLENGTH_MAX_FRAME); rxState = RS_WAIT; }
      if (rxPos >= rxLen - 2) {
          rxState = RS_CRC1;
      }
      break;
    case RS_CRC1:
      //rxCrc = (rxCrc & 0xFF00) | b; 
      crcUpdate(&rxCrc, b);
      rxState = RS_CRC2;
      break;
    case RS_CRC2:
      crcUpdate(&rxCrc, b);
      //rxCrc = (rxCrc & 0x00FF) | (uint16_t(b) << 8);

      if (rxCrc == 0) {
        if (rxBuf[2] == SLAVE_ID || rxBuf[2] == 0xFF) {if (!handleRequest(rxBuf)) debugLog("REQUEST - Error occured!");}
        else setCurrentBlinkState(ERROR);
      } else {setCurrentBlinkState(ERROR); debugLog("REQUEST - CRC error!");}
      rxState = RS_WAIT;
      break;
  }
}

void setup() { protocolBegin(); }
void loop() {
  processRetries();
  updateBlink();
  autoOffCheck();
  while (Serial1.available()) feedByte(Serial1.read());
}
