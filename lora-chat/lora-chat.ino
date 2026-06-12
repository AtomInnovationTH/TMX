// LightTracker LoRa Chat — transceiver firmware
//
// One sketch for both boards. Bridges a browser (Web Serial, NDJSON lines over
// USB CDC @ 115200) to a LoRa point-to-point text chat (SX1262).
//
// Serial protocol (newline-delimited JSON):
//   browser -> board : {"t":"ping"}
//                      {"t":"name","v":"Joi"}
//                      {"t":"join"}
//                      {"t":"tx","id":7,"text":"hello"}
//   board -> browser : {"t":"ready","fw":"1.0","freq":923.2,"name":"Joi"}
//                      {"t":"rx","from":"Mai","text":"hi","rssi":-42,"snr":12.5}
//                      {"t":"join","from":"Mai","rssi":-42,"snr":12.5}
//                      {"t":"sent","id":7}        radio TX completed
//                      {"t":"ack","id":7}         remote board confirmed receipt
//                      {"t":"err","msg":"..."}
//
// Air frame (binary, explicit header + CRC, max ~217 bytes):
//   [0]   0xC4  magic
//   [1]   0x01  protocol version
//   [2]   type: 0=chat 1=join 2=ack    (bit7 set = 12-byte position trailer)
//   [3:4] senderId (random per power-on, little-endian)
//   [5:6] msgId (little-endian)
//   [7]   nameLen (<=16)
//   [8..] name bytes, then payload:
//           chat: UTF-8 text (<=180 bytes)
//           join: empty
//           ack : [origSenderId:2][origMsgId:2]
//   trailer (when type bit7 set, appended after payload):
//           [lat:i32 deg*1e7][lon:i32 deg*1e7][alt:i16 m][sats:u8][batt:u8 V*20]
//
// Radio settings are the field-verified LightTracker tracker settings:
// 923.2 MHz / SF8 / BW125 / CR4-5 / syncWord 0x12 / CRC on / 16 dBm.

#include <RadioLib.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "SparkFun_Ublox_Arduino_Library.h"

// ---------- Hardware (LightTracker-B) ----------
// SX1262: NSS 8, DIO1 3, NRST 9, BUSY 2
SX1262 lora = new Module(8, 3, 9, 2);

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR     0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool displayOK = false;

// GPS (u-blox via I2C) + battery sense — same wiring as the tracker firmware
#define GpsPwr  12
#define BattPin A5
SFE_UBLOX_GPS myGPS;
bool gpsOK = false;

// ---------- Radio settings (must match on both boards) ----------
static const float    LORA_FREQ      = 923.2f; // AS923 (Thailand 920-925 MHz)
static const float    LORA_BW        = 125.0f; // kHz
static const uint8_t  LORA_SF        = 8;      // SF7 = 2x faster, less range
static const uint8_t  LORA_CR        = 5;      // 4/5
static const uint8_t  LORA_SYNCWORD  = 0x12;   // private network
static const int8_t   LORA_POWER     = 16;     // dBm (AS923 limit)
static const uint16_t LORA_PREAMBLE  = 8;

// ---------- Protocol ----------
static const uint8_t PROTO_MAGIC   = 0xC4;
static const uint8_t PROTO_VERSION = 0x01;
enum MsgType : uint8_t { TYPE_CHAT = 0, TYPE_JOIN = 1, TYPE_ACK = 2 };
static const uint8_t TYPE_MASK     = 0x7F;
static const uint8_t FLAG_HAS_POS  = 0x80;   // bit7 of type byte

static const uint8_t  MAX_NAME    = 16;
static const uint8_t  MAX_TEXT    = 180;
static const size_t   HDR_LEN     = 8;
static const size_t   TRAILER_LEN = 12;
static const size_t   MAX_FRAME   = HDR_LEN + MAX_NAME + MAX_TEXT + TRAILER_LEN;

static const uint32_t ACK_TIMEOUT_MS = 2500;
static const uint8_t  MAX_RETRIES    = 1;     // one retransmit if no ack
static const uint8_t  CAD_ATTEMPTS   = 5;     // listen-before-talk tries

static const uint32_t GPS_POLL_MS    = 2000;  // read GPS/battery every 2 s
static const uint32_t GPS_REPORT_MS  = 5000;  // own-position event to browser

// ---------- State ----------
volatile bool receivedFlag    = false;
volatile bool enableInterrupt = true;

uint16_t myId = 0;          // random per power-on
uint16_t txMsgId = 0;       // increments per outgoing frame
char     myName[MAX_NAME + 1] = "anon";

// Pending-ack tracking (one in flight; chat is human-speed)
bool     pendingActive = false;
uint16_t pendingMsgId  = 0;   // air msgId
long     pendingWebId  = -1;  // browser's "id" for status events
uint8_t  pendingFrame[MAX_FRAME];
size_t   pendingLen    = 0;
uint32_t pendingSentAt = 0;
uint8_t  pendingRetries = 0;

// Dedupe ring of recently seen (senderId, msgId)
static const uint8_t DEDUPE_N = 8;
uint32_t dedupeRing[DEDUPE_N] = {0};
uint8_t  dedupeIdx = 0;

// Serial line buffer
static const size_t LINE_MAX = 360;
char   lineBuf[LINE_MAX];
size_t lineLen = 0;
bool   lineOverflow = false;

// Counters / OLED status
uint32_t rxCount = 0, txCount = 0;
char lastFrom[MAX_NAME + 1] = "";
char lastText[22] = "";          // truncated for OLED
float lastRSSI = 0, lastSNR = 0;

// Own position / battery (cached by pollGps)
int32_t  myLat = 0, myLon = 0;   // degrees * 1e7
int16_t  myAltM = 0;             // meters MSL
uint8_t  mySats = 0;             // 0 = no fix
uint8_t  myBatt = 0;             // volts * 20
uint32_t lastGpsPoll = 0, lastGpsReport = 0;

float readBattVolts() {
  const float R1 = 560000.0f, R2 = 100000.0f;
  float v = (analogRead(BattPin) + analogRead(BattPin) + analogRead(BattPin)) / 3.0f;
  v = (v * 3.3f) / 1024.0f;
  v = v / (R2 / (R1 + R2));
  if (v < 0 || v > 12.7f) v = 0;
  return v;
}

void pollGps() {
  myBatt = (uint8_t)(readBattVolts() * 20.0f);
  if (!gpsOK) return;
  if (myGPS.getPVT(100) && myGPS.getFixType() != 0 && myGPS.getSIV() > 3) {
    myLat  = myGPS.getLatitude();
    myLon  = myGPS.getLongitude();
    long altMM = myGPS.getAltitudeMSL();
    myAltM = (int16_t)constrain(altMM / 1000, -32768L, 32767L);
    mySats = myGPS.getSIV();
  } else {
    mySats = 0;   // stale fix -> mark invalid, keep last lat/lon for OLED
  }
}

// Own-position event for the browser (it computes distance/bearing locally)
void emitGps() {
  StaticJsonDocument<224> d;
  d["t"] = "gps";
  d["fix"] = (mySats > 0);
  d["sats"] = mySats;
  if (mySats > 0) {
    d["lat"] = serialized(String(myLat / 1e7, 7));
    d["lon"] = serialized(String(myLon / 1e7, 7));
    d["alt"] = myAltM;
  }
  d["batt"] = serialized(String(myBatt / 20.0f, 2));
  serializeJson(d, SerialUSB);
  SerialUSB.println();
}

size_t appendTrailer(uint8_t* buf, size_t len) {
  int32_t lat = (mySats > 0) ? myLat : 0;
  int32_t lon = (mySats > 0) ? myLon : 0;
  memcpy(buf + len, &lat, 4);
  memcpy(buf + len + 4, &lon, 4);
  memcpy(buf + len + 8, &myAltM, 2);
  buf[len + 10] = mySats;
  buf[len + 11] = myBatt;
  return len + TRAILER_LEN;
}

// ---------- ISR ----------
void onDio1(void) {
  if (!enableInterrupt) return;
  receivedFlag = true;
}

// ---------- JSON out helpers ----------
void emitReady() {
  StaticJsonDocument<160> d;
  d["t"] = "ready";
  d["fw"] = "1.1";
  d["freq"] = LORA_FREQ;
  d["name"] = myName;
  serializeJson(d, SerialUSB);
  SerialUSB.println();
}

void emitErr(const char* msg) {
  StaticJsonDocument<128> d;
  d["t"] = "err";
  d["msg"] = msg;
  serializeJson(d, SerialUSB);
  SerialUSB.println();
}

void emitStatus(const char* t, long id) {
  StaticJsonDocument<64> d;
  d["t"] = t;
  d["id"] = id;
  serializeJson(d, SerialUSB);
  SerialUSB.println();
}

// ---------- OLED ----------
void refreshDisplay() {
  if (!displayOK) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(0, 0);
  display.println(F("LoRa Chat 923.2MHz"));
  display.print(F("Me: ")); display.println(myName);
  display.print(F("TX:")); display.print(txCount);
  display.print(F(" RX:")); display.print(rxCount);
  display.print(F(" S:")); display.print(mySats);
  display.print(F(" ")); display.print(myBatt / 20.0f, 1); display.println(F("V"));
  if (lastFrom[0]) {
    display.print(F("From ")); display.print(lastFrom);
    display.print(F(" ")); display.print((int)lastRSSI); display.println(F("dBm"));
    display.println(lastText);
  } else {
    display.println(F("Waiting for peers..."));
  }
  display.display();
}

// ---------- Dedupe ----------
bool seenBefore(uint16_t sender, uint16_t msgId) {
  uint32_t key = ((uint32_t)sender << 16) | msgId;
  for (uint8_t i = 0; i < DEDUPE_N; i++) {
    if (dedupeRing[i] == key) return true;
  }
  dedupeRing[dedupeIdx] = key;
  dedupeIdx = (dedupeIdx + 1) % DEDUPE_N;
  return false;
}

// ---------- Radio TX ----------
// Listen-before-talk + transmit, then return to receive mode.
// Returns RadioLib status code.
int16_t transmitFrame(const uint8_t* frame, size_t len) {
  enableInterrupt = false;

  // CSMA: check channel, back off randomly if busy
  for (uint8_t i = 0; i < CAD_ATTEMPTS; i++) {
    int16_t scan = lora.scanChannel();
    if (scan == CHANNEL_FREE) break;
    delay(random(50, 400));
  }

  int16_t state = lora.transmit((uint8_t*)frame, len);

  receivedFlag = false;        // clear any IRQ raised by TX-done/CAD
  lora.startReceive();
  enableInterrupt = true;

  if (state == ERR_NONE) txCount++;
  return state;
}

size_t buildFrame(uint8_t* buf, uint8_t type, uint16_t msgId,
                  const uint8_t* payload, size_t payloadLen, bool withPos) {
  uint8_t nameLen = strlen(myName);
  buf[0] = PROTO_MAGIC;
  buf[1] = PROTO_VERSION;
  buf[2] = withPos ? (type | FLAG_HAS_POS) : type;
  buf[3] = myId & 0xFF;
  buf[4] = myId >> 8;
  buf[5] = msgId & 0xFF;
  buf[6] = msgId >> 8;
  buf[7] = nameLen;
  memcpy(buf + HDR_LEN, myName, nameLen);
  if (payload && payloadLen) memcpy(buf + HDR_LEN + nameLen, payload, payloadLen);
  size_t len = HDR_LEN + nameLen + payloadLen;
  if (withPos) len = appendTrailer(buf, len);
  return len;
}

void sendAck(uint16_t origSender, uint16_t origMsgId) {
  // Small random delay so the original sender is back in receive mode
  delay(random(30, 120));
  uint8_t payload[4] = {
    (uint8_t)(origSender & 0xFF), (uint8_t)(origSender >> 8),
    (uint8_t)(origMsgId & 0xFF), (uint8_t)(origMsgId >> 8)
  };
  uint8_t frame[MAX_FRAME];
  size_t len = buildFrame(frame, TYPE_ACK, ++txMsgId, payload, sizeof(payload), false);
  transmitFrame(frame, len);
}

// ---------- Radio RX ----------
void handleRadioPacket() {
  enableInterrupt = false;
  receivedFlag = false;

  uint8_t buf[256];
  size_t len = lora.getPacketLength();
  if (len > sizeof(buf)) len = sizeof(buf);
  int16_t state = lora.readData(buf, len);
  float rssi = lora.getRSSI();
  float snr  = lora.getSNR();

  lora.startReceive();
  enableInterrupt = true;

  if (state != ERR_NONE) return;                      // CRC error etc: drop
  if (len < HDR_LEN) return;                          // runt
  if (buf[0] != PROTO_MAGIC || buf[1] != PROTO_VERSION) return; // not ours

  uint8_t  rawType = buf[2];
  uint8_t  type    = rawType & TYPE_MASK;
  bool     hasPos  = (rawType & FLAG_HAS_POS) != 0;
  uint16_t sender  = buf[3] | (buf[4] << 8);
  uint16_t msgId   = buf[5] | (buf[6] << 8);
  uint8_t  nameLen = buf[7];
  if (sender == myId) return;                         // self-echo guard
  if (nameLen > MAX_NAME || HDR_LEN + nameLen > len) return;
  if (hasPos && len < HDR_LEN + nameLen + TRAILER_LEN) return;

  char from[MAX_NAME + 1];
  memcpy(from, buf + HDR_LEN, nameLen);
  from[nameLen] = '\0';

  const uint8_t* payload = buf + HDR_LEN + nameLen;
  size_t payloadLen = len - HDR_LEN - nameLen - (hasPos ? TRAILER_LEN : 0);

  // Peer position trailer (last 12 bytes of the frame)
  int32_t pLat = 0, pLon = 0; int16_t pAlt = 0; uint8_t pSats = 0, pBatt = 0;
  if (hasPos) {
    const uint8_t* tr = buf + len - TRAILER_LEN;
    memcpy(&pLat, tr, 4);
    memcpy(&pLon, tr + 4, 4);
    memcpy(&pAlt, tr + 8, 2);
    pSats = tr[10];
    pBatt = tr[11];
  }

  if (type == TYPE_ACK) {
    if (payloadLen < 4) return;
    uint16_t origSender = payload[0] | (payload[1] << 8);
    uint16_t origMsgId  = payload[2] | (payload[3] << 8);
    if (pendingActive && origSender == myId && origMsgId == pendingMsgId) {
      pendingActive = false;
      emitStatus("ack", pendingWebId);
    }
    return;
  }

  bool duplicate = seenBefore(sender, msgId);

  if (type == TYPE_CHAT) {
    // Always ack (the previous ack may have been lost), but only show once
    sendAck(sender, msgId);
    if (duplicate) return;

    char text[MAX_TEXT + 1];
    size_t tlen = payloadLen > MAX_TEXT ? MAX_TEXT : payloadLen;
    memcpy(text, payload, tlen);
    text[tlen] = '\0';

    StaticJsonDocument<512> d;
    d["t"] = "rx";
    d["from"] = from;
    d["text"] = text;
    d["rssi"] = serialized(String(rssi, 1));
    d["snr"]  = serialized(String(snr, 1));
    if (hasPos) {
      JsonObject pos = d.createNestedObject("pos");
      pos["sats"] = pSats;
      if (pSats > 0) {
        pos["lat"] = serialized(String(pLat / 1e7, 7));
        pos["lon"] = serialized(String(pLon / 1e7, 7));
        pos["alt"] = pAlt;
      }
      pos["batt"] = serialized(String(pBatt / 20.0f, 2));
    }
    serializeJson(d, SerialUSB);
    SerialUSB.println();

    rxCount++;
    strncpy(lastFrom, from, MAX_NAME); lastFrom[MAX_NAME] = '\0';
    strncpy(lastText, text, sizeof(lastText) - 1); lastText[sizeof(lastText) - 1] = '\0';
    lastRSSI = rssi; lastSNR = snr;
    refreshDisplay();

  } else if (type == TYPE_JOIN) {
    if (duplicate) return;
    StaticJsonDocument<320> d;
    d["t"] = "join";
    d["from"] = from;
    d["rssi"] = serialized(String(rssi, 1));
    d["snr"]  = serialized(String(snr, 1));
    if (hasPos) {
      JsonObject pos = d.createNestedObject("pos");
      pos["sats"] = pSats;
      if (pSats > 0) {
        pos["lat"] = serialized(String(pLat / 1e7, 7));
        pos["lon"] = serialized(String(pLon / 1e7, 7));
        pos["alt"] = pAlt;
      }
      pos["batt"] = serialized(String(pBatt / 20.0f, 2));
    }
    serializeJson(d, SerialUSB);
    SerialUSB.println();

    strncpy(lastFrom, from, MAX_NAME); lastFrom[MAX_NAME] = '\0';
    strncpy(lastText, "(joined)", sizeof(lastText) - 1);
    lastRSSI = rssi; lastSNR = snr;
    refreshDisplay();
  }
}

// ---------- Serial command handling ----------
void handleLine(const char* line) {
  StaticJsonDocument<512> d;
  DeserializationError e = deserializeJson(d, line);
  if (e) { emitErr("bad json"); return; }
  const char* t = d["t"];
  if (!t) { emitErr("missing t"); return; }

  if (strcmp(t, "ping") == 0) {
    emitReady();

  } else if (strcmp(t, "name") == 0) {
    const char* v = d["v"];
    if (!v || !v[0]) { emitErr("missing name"); return; }
    strncpy(myName, v, MAX_NAME);
    myName[MAX_NAME] = '\0';
    emitReady();
    refreshDisplay();

  } else if (strcmp(t, "join") == 0) {
    uint8_t frame[MAX_FRAME];
    size_t len = buildFrame(frame, TYPE_JOIN, ++txMsgId, nullptr, 0, true);
    int16_t state = transmitFrame(frame, len);
    if (state != ERR_NONE) emitErr("radio tx failed");
    refreshDisplay();

  } else if (strcmp(t, "tx") == 0) {
    const char* text = d["text"];
    long webId = d["id"] | -1L;
    if (!text || !text[0]) { emitErr("empty text"); return; }
    size_t tlen = strlen(text);
    if (tlen > MAX_TEXT) { emitErr("text too long"); return; }
    if (pendingActive) {
      // Previous message still waiting on ack; give up on its ack
      pendingActive = false;
    }
    uint8_t frame[MAX_FRAME];
    size_t len = buildFrame(frame, TYPE_CHAT, ++txMsgId,
                            (const uint8_t*)text, tlen, true);
    int16_t state = transmitFrame(frame, len);
    if (state == ERR_NONE) {
      emitStatus("sent", webId);
      pendingActive  = true;
      pendingMsgId   = txMsgId;
      pendingWebId   = webId;
      memcpy(pendingFrame, frame, len);
      pendingLen     = len;
      pendingSentAt  = millis();
      pendingRetries = 0;
      refreshDisplay();
    } else {
      emitErr("radio tx failed");
    }

  } else {
    emitErr("unknown cmd");
  }
}

void pollSerial() {
  while (SerialUSB.available()) {
    char c = (char)SerialUSB.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0 && !lineOverflow) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
      } else if (lineOverflow) {
        emitErr("line too long");
      }
      lineLen = 0;
      lineOverflow = false;
    } else if (lineLen < LINE_MAX - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }
}

void checkAckRetry() {
  if (!pendingActive) return;
  if (millis() - pendingSentAt < ACK_TIMEOUT_MS) return;
  if (pendingRetries < MAX_RETRIES) {
    pendingRetries++;
    pendingSentAt = millis();
    transmitFrame(pendingFrame, pendingLen);   // same msgId -> remote dedupes
  } else {
    pendingActive = false;                     // delivered-unknown; stays "sent"
  }
}

// ---------- Setup / loop ----------
void setup() {
  SerialUSB.begin(115200);

  // Seed randomness from floating analog pins + clock
  randomSeed(analogRead(A0) ^ ((uint32_t)analogRead(A2) << 10) ^ micros());
  myId = (uint16_t)random(1, 0xFFFF);

  // GPS power + I2C bus (shared with OLED)
  pinMode(GpsPwr, OUTPUT);
  digitalWrite(GpsPwr, LOW);   // GPS on
  delay(100);
  Wire.begin();
  Wire.setClock(400000);

  // OLED (optional hardware — non-fatal if absent)
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    displayOK = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(1);
    display.setCursor(0, 0);
    display.println(F("LoRa Chat starting.."));
    display.display();
  }

  // GPS (optional — chat works without a fix or without the module)
  if (myGPS.begin()) {
    gpsOK = true;
    myGPS.setUART1Output(0);          // UBX over I2C only, no NMEA noise
    myGPS.setUART2Output(0);
    myGPS.setI2COutput(COM_TYPE_UBX);
    myGPS.setNavigationFrequency(2);
  }

  // Radio — field-verified LightTracker settings
  int16_t state = lora.begin();
  bool ok = (state == ERR_NONE)
    && lora.setFrequency(LORA_FREQ, true) == ERR_NONE
    && lora.setBandwidth(LORA_BW) == ERR_NONE
    && lora.setSpreadingFactor(LORA_SF) == ERR_NONE
    && lora.setCodingRate(LORA_CR) == ERR_NONE
    && lora.setSyncWord(LORA_SYNCWORD) == ERR_NONE
    && lora.setCurrentLimit(140) == ERR_NONE
    && lora.setOutputPower(LORA_POWER) == ERR_NONE
    && lora.setPreambleLength(LORA_PREAMBLE) == ERR_NONE
    && lora.setCRC(1) == ERR_NONE;

  if (!ok) {
    // Radio is mandatory: report forever so the browser shows a clear error
    while (true) {
      emitErr("radio init failed");
      delay(2000);
    }
  }

  lora.setDio1Action(onDio1);
  lora.startReceive();

  pollGps();
  emitReady();
  emitGps();
  refreshDisplay();
}

void loop() {
  if (receivedFlag) handleRadioPacket();
  pollSerial();
  checkAckRetry();

  uint32_t now = millis();
  if (now - lastGpsPoll >= GPS_POLL_MS) {
    lastGpsPoll = now;
    pollGps();
  }
  if (now - lastGpsReport >= GPS_REPORT_MS) {
    lastGpsReport = now;
    emitGps();
    refreshDisplay();
  }
}
