// LightTracker LoRa Chat — transceiver firmware
//
// One sketch for both boards. Bridges a browser (Web Serial, NDJSON lines over
// USB CDC @ 115200) to a LoRa point-to-point text chat (SX1262).
//
// Serial protocol (newline-delimited JSON):
//   browser -> board : {"t":"ping"}
//                      {"t":"name","v":"Joi"}
//                      {"t":"client","v":"Chrome 126 · macOS · desktop"}
//                      {"t":"join"}
//                      {"t":"tx","id":7,"text":"hello"}
//   board -> browser : {"t":"ready","fw":"1.8","freq":923.2,"name":"Joi","id":4321}
//                      {"t":"rx","from":"Mai","id":2048,"text":"hi","rssi":-42,"snr":12.5}
//                      {"t":"join","from":"Mai","id":2048,"rssi":-42,"snr":12.5,
//                       "client":"Edge · Windows · desktop"}
//                      {"t":"beacon","from":"Mai","id":2048,"rssi":-42,"snr":12.5,
//                       "pos":{...},"ext":{"speed":4,"course":120,"hacc":4,"fixType":3,
//                       "temp":31,"press":1008.4,"hdg":210},
//                       "nb":[{"id":4321,"rssi":-55},{"id":7777,"rssi":-88}]}
//                       id  = sender's per-boot senderId (fw 1.7+; lets the app
//                             map names<->ids for mesh trilateration).
//                       nb  = neighbours the sender currently hears + their RSSI
//                             (fw 1.7+), so one connected board can recover the
//                             whole pairwise link matrix, not just its own row.
//                       ext.temp/press/hdg = board environment (fw 1.8+): BMP180
//                             board temp °C + pressure hPa, magnetic heading deg
//                             (each omitted when the sensor is absent/invalid).
//                      {"t":"gps",...,"sats":11,"siv":14,"speed":4.2,
//                       "course":118,"fixType":3,"hacc":3.8,"pdop":1.6,
//                       "utc":"2026-06-13T06:40:12Z","temp":31.2,"press":1008.4,
//                       "hdg":210,"up":120,"noise":-104.5,"tx":3,"rx":2,"csave":47}
//                       sats=used in fix (0=no fix); siv=satellites in view.
//                       temp=board °C, press=hPa (BMP180, fw 1.8+, omitted if
//                       absent); hdg=magnetic heading deg (fw 1.8+, omitted until
//                       the compass self-calibrates by being rotated once).
//                      {"t":"sent","id":7}        radio TX completed
//                      {"t":"ack","id":7}         remote board confirmed receipt
//                      {"t":"err","msg":"..."}
//
// Air frame (binary, explicit header + CRC, max ~217 bytes):
//   [0]   0xC4  magic
//   [1]   0x01  protocol version
//   [2]   type: 0=chat 1=join 2=ack 3=beacon
//         bit6 (0x40) set = chat payload is smaz-compressed (fw 1.4+)
//         bit7 (0x80) set = 12-byte position trailer
//   [3:4] senderId (random per power-on, little-endian)
//   [5:6] msgId (little-endian)
//   [7]   nameLen (<=16)
//   [8..] name bytes, then payload:
//           chat  : UTF-8 text (<=180 bytes), optionally smaz-compressed when
//                   the bit6 flag is set (TX uses it only when it shrinks the
//                   payload; RX decompresses before emitting plaintext). Thai/
//                   emoji and other incompressible text fall back to raw.
//           join  : compact client/app profile string (fw 1.5+, <=40 bytes),
//                   smaz-compressed when bit6 set; empty when none. Older boards
//                   (<=1.4) ignore these extra bytes (they only read the trailer).
//           ack   : [origSenderId:2][origMsgId:2]
//           beacon: [speed:u8 km/h capped][course:u8 deg/2][hacc:u8 m capped]
//                   [fixType:u8]  (fw 1.3+; v1.2 parses the trailer from the
//                   frame end and ignores these bytes; no ack; sent ~60 s when
//                   fix held; unknown to fw <=1.1, which ignores type 3).
//                   fw 1.5+ also sends ONE beacon on boot (every board) as a
//                   presence/power ping, and labeled boards beacon autonomously.
//                   fw 1.7+ appends a neighbour block after the 4 ext bytes:
//                   [count:u8][ {senderId:2 LE}{rssi:i8} x count ] (up to 6,
//                   freshest first). Older boards read only the 4 ext bytes and
//                   ignore the rest; new boards decode it into the `nb` event.
//                   fw 1.8+ appends 4 environment bytes AFTER the neighbour
//                   block: [temp:i8 °C (-128=invalid)][press:u16 LE hPa*10
//                   (0=invalid)][hdg:u8 deg/2 (255=invalid)]. Receivers parse
//                   them only when payloadLen >= nbEnd+4, so 1.7<->1.8 fleets
//                   mix cleanly (env data is just absent from a 1.7 sender).
//                   fw 1.8+ also beacons every ~60 s once joined WHETHER OR NOT
//                   a GPS fix is held (presence heartbeat: name+battery+temp
//                   still report with sats=0), so silence is meaningful.
//   trailer (when type bit7 set, appended after payload):
//           [lat:i32 deg*1e7][lon:i32 deg*1e7][alt:i16 m][sats:u8][batt:u8 V*20]
//
// Radio settings are the field-verified LightTracker tracker settings:
// 923.2 MHz / SF8 / BW125 / CR4-5 / syncWord 0x12 / CRC on / 16 dBm.

#include <RadioLib.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include "smaz.h"

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

// ---------- Onboard environment sensors (fw 1.8) ----------
// BMP180 barometer/thermometer (I2C 0x77) — board temperature + pressure. The
// original tracker firmware reads it via this same vendored library. Reading is
// blocking (~10-30 ms) but happens only in the 2 s poll, so it never touches an
// ACK window. Non-fatal: if the chip is absent, bmpOK stays false and the temp/
// pressure fields are simply omitted (same pattern as gpsOK/displayOK).
Adafruit_BMP085 bmp;
bool     bmpOK = false;
int16_t  myTempC10 = 0;      // board temperature, °C * 10 (valid when bmpOK)
uint32_t myPressPa = 0;      // barometric pressure, Pa (valid when bmpOK)

// LSM303DLHC magnetometer (I2C 0x1E) — an approximate magnetic heading. There is
// no vendored driver, so this is a tiny raw-Wire implementation (see magInit/
// magRead). Hard-iron offset is auto-calibrated in RAM only: we track the min/max
// of each axis as the board is turned, and the heading is only reported once the
// board has been rotated enough this power-on (spread > threshold). No tilt
// compensation, no declination — it is labelled "magnetic · approx" in the UI.
static const uint8_t MAG_ADDR = 0x1E;
bool     magOK = false;
int16_t  magMinX = 32767, magMaxX = -32768;
int16_t  magMinY = 32767, magMaxY = -32768;
uint16_t myHdgDeg = 0;       // magnetic heading, degrees (valid when hdgValid)
bool     hdgValid = false;
static const int16_t MAG_CAL_SPREAD = 150;   // per-axis min/max span needed to trust the heading

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
enum MsgType : uint8_t { TYPE_CHAT = 0, TYPE_JOIN = 1, TYPE_ACK = 2, TYPE_BEACON = 3 };
static const uint8_t TYPE_MASK       = 0x3F;   // was 0x7F (types are 0..3)
static const uint8_t FLAG_COMPRESSED = 0x40;   // bit6: chat payload is smaz-compressed
static const uint8_t FLAG_HAS_POS    = 0x80;   // bit7 of type byte

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
static const uint32_t BEACON_MS      = 60000; // position beacon period (+ jitter)

// ---------- State ----------
volatile bool receivedFlag    = false;
volatile bool enableInterrupt = true;

uint16_t myId = 0;          // random per power-on
uint16_t txMsgId = 0;       // increments per outgoing frame
// Build-time board label. Defaults to "anon" (browser sets the chat name).
// Build a labeled board with: --build-property build.extra_flags=-DDEVICE_NAME='"solar"'
// A non-default DEVICE_NAME is treated as a fixed identity: the board always
// broadcasts it and ignores browser name-overrides (see handleLine "name").
#ifndef DEVICE_NAME
#define DEVICE_NAME "anon"
#endif
char     myName[MAX_NAME + 1] = DEVICE_NAME;
const bool nameLocked = (strcmp(DEVICE_NAME, "anon") != 0);
// Compact client/app profile (browser+OS+kind or app/fw version) sent once in the
// JOIN payload so peers can show "what the other side is chatting from". Local
// until the browser provides it; never anything more sensitive than this string.
static const uint8_t MAX_CLIENT = 40;
char     myClient[MAX_CLIENT + 1] = "";

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

// ---------- Neighbour RSSI table (mesh trilateration, fw 1.7) ----------
// We remember the last RSSI at which we heard each peer, and rebroadcast a small
// slice of this table in every beacon. A single USB-connected board then sees
// not just "how far is each peer from me" (its own RX RSSI) but the whole
// pairwise link matrix, which is what the web app needs to lay out a relative
// constellation (and anchor it once any node has GPS).
static const uint8_t  NB_MAX        = 8;       // peers tracked locally
static const uint8_t  NB_REPORT_MAX = 6;       // peers attached to one beacon
static const uint32_t NB_STALE_MS   = 300000;  // forget a peer after 5 min silence
struct Neighbor { uint16_t id; int8_t rssi; uint32_t seen; };
Neighbor nbTable[NB_MAX];
uint8_t  nbCount = 0;

void noteNeighbor(uint16_t id, float rssi) {
  int8_t r = (int8_t)constrain((long)lroundf(rssi), -128L, 127L);  // dBm fits i8
  uint32_t now = millis();
  for (uint8_t i = 0; i < nbCount; i++) {
    if (nbTable[i].id == id) { nbTable[i].rssi = r; nbTable[i].seen = now; return; }
  }
  if (nbCount < NB_MAX) {
    nbTable[nbCount++] = { id, r, now };
  } else {                                  // evict the stalest entry
    uint8_t oldest = 0;
    for (uint8_t i = 1; i < NB_MAX; i++) if (nbTable[i].seen < nbTable[oldest].seen) oldest = i;
    nbTable[oldest] = { id, r, now };
  }
}

// Serial line buffer
static const size_t LINE_MAX = 360;
char   lineBuf[LINE_MAX];
size_t lineLen = 0;
bool   lineOverflow = false;

// Counters / OLED status
uint32_t rxCount = 0, txCount = 0;
uint32_t bytesRaw = 0, bytesAir = 0;   // chat-text bytes before/after smaz (csave %)
char lastFrom[MAX_NAME + 1] = "";
char lastText[22] = "";          // truncated for OLED
float lastRSSI = 0, lastSNR = 0;

// Own position / battery / motion (cached by pollGps — single getPVT poll)
int32_t  myLat = 0, myLon = 0;   // degrees * 1e7
int16_t  myAltM = 0;             // meters MSL
uint8_t  mySats = 0;             // satellites used in the current valid fix (0 = no fix)
uint8_t  mySiv = 0;              // satellites in view / tracked (independent of fix)
uint8_t  myBatt = 0;             // volts * 20
int32_t  mySpeedMms = 0;         // ground speed, mm/s
uint16_t myCourseDeg = 0;        // course over ground, degrees
uint8_t  myFixType = 0;          // 0=none 2=2D 3=3D
uint32_t myHaccMm = 0;           // horizontal accuracy estimate, mm
uint16_t myPdop = 0;             // pDOP * 100
uint16_t myYear = 0;             // UTC date/time (valid when myTimeOk)
uint8_t  myMonth = 0, myDay = 0, myHour = 0, myMin = 0, mySec = 0;
bool     myTimeOk = false;
uint32_t lastGpsPoll = 0, lastGpsReport = 0;
uint32_t lastPvtOk = 0;          // last successful getPVT (for bounded staleness)
static const uint32_t FIX_STALE_MS = 15000;  // give up on a held fix after this

// Position beacon: starts after the browser joins; jittered so boards desync
bool     hasJoined = false;
uint32_t nextBeaconAt = 0;

float readBattVolts() {
  const float R1 = 560000.0f, R2 = 100000.0f;
  float v = (analogRead(BattPin) + analogRead(BattPin) + analogRead(BattPin)) / 3.0f;
  v = (v * 3.3f) / 1024.0f;
  v = v / (R2 / (R1 + R2));
  if (v < 0 || v > 12.7f) v = 0;
  return v;
}

// ---------- LSM303DLHC magnetometer (raw Wire, fw 1.8) ----------
// The mag block sits at its own I2C address (0x1E); registers of interest:
//   CRA_REG_M 0x00 = 0x10 -> 15 Hz output      CRB_REG_M 0x01 = 0x20 -> ±1.3 Ga
//   MR_REG_M  0x02 = 0x00 -> continuous mode    IRA_REG_M 0x0A reads 0x48 ('H')
// Data at 0x03 is big-endian in the quirky X, Z, Y order.
bool magInit() {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);                                  // IRA_REG_M identity register
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((int)MAG_ADDR, 1) != 1) return false;
  if (Wire.read() != 0x48) return false;             // not an LSM303DLHC mag
  const uint8_t cfg[3][2] = { {0x00, 0x10}, {0x01, 0x20}, {0x02, 0x00} };
  for (uint8_t i = 0; i < 3; i++) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(cfg[i][0]); Wire.write(cfg[i][1]);
    if (Wire.endTransmission() != 0) return false;
  }
  return true;
}

// Read one sample, feed the running hard-iron min/max, and recompute the heading.
// hdgValid flips true only once the board has been rotated enough this power-on
// (both axes have swept > MAG_CAL_SPREAD counts), so a still board never emits a
// bogus fixed heading.
void magRead() {
  if (!magOK) return;
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x03);                                  // OUT_X_H_M (auto-increments)
  if (Wire.endTransmission() != 0) return;
  if (Wire.requestFrom((int)MAG_ADDR, 6) != 6) return;
  uint8_t b[6];
  for (uint8_t i = 0; i < 6; i++) b[i] = Wire.read();   // read in a defined order
  int16_t x = (int16_t)((b[0] << 8) | b[1]);            // X (H,L); b[2..3]=Z, unused
  int16_t y = (int16_t)((b[4] << 8) | b[5]);            // Y (H,L). No tilt comp, so Z
                                                        // is intentionally not decoded.
  if (x < magMinX) magMinX = x; if (x > magMaxX) magMaxX = x;
  if (y < magMinY) magMinY = y; if (y > magMaxY) magMaxY = y;
  if (magMaxX - magMinX < MAG_CAL_SPREAD || magMaxY - magMinY < MAG_CAL_SPREAD) {
    hdgValid = false;
    return;
  }
  float xc = x - (magMinX + magMaxX) / 2.0f;         // hard-iron corrected
  float yc = y - (magMinY + magMaxY) / 2.0f;
  float h = atan2f(yc, xc) * 57.29578f;              // rad -> deg
  if (h < 0) h += 360.0f;
  myHdgDeg = (uint16_t)(h + 0.5f) % 360;
  hdgValid = true;
}

void pollGps() {
  myBatt = (uint8_t)(readBattVolts() * 20.0f);
  if (bmpOK) {
    myTempC10 = (int16_t)lroundf(bmp.readTemperature() * 10.0f);
    myPressPa = (uint32_t)bmp.readPressure();        // Pa
  }
  magRead();
  if (!gpsOK) return;
  if (!myGPS.getPVT(250)) {
    // Transient I2C/poll timeout: tolerate a few missed polls (keep the last
    // fixType/siv/position so the browser doesn't flicker). But if the module
    // stays unresponsive past FIX_STALE_MS — dead, unplugged, or a solar board
    // brown-out loop — give up the held fix so we stop emitting/beaconing a
    // frozen position. A real loss via a successful poll is handled below.
    if (lastPvtOk && millis() - lastPvtOk > FIX_STALE_MS) {
      mySats = 0;
      myFixType = 0;
    }
    return;
  }
  lastPvtOk = millis();
  // One PVT poll; every getter below reads the cached message (no extra I2C)
  myFixType   = myGPS.getFixType();
  mySiv       = myGPS.getSIV();                     // tracked, regardless of fix
  mySpeedMms  = myGPS.getGroundSpeed();             // mm/s
  long crs    = myGPS.getHeading() / 100000L;       // deg*1e-5 -> deg
  myCourseDeg = (uint16_t)(((crs % 360) + 360) % 360);
  myHaccMm    = (uint32_t)myGPS.getHorizontalAccEst();
  myPdop      = myGPS.getPDOP();                    // *0.01
  myTimeOk    = myGPS.getDateValid() && myGPS.getTimeValid();
  if (myTimeOk) {
    myYear = myGPS.getYear();  myMonth = myGPS.getMonth(); myDay = myGPS.getDay();
    myHour = myGPS.getHour();  myMin = myGPS.getMinute();  mySec = myGPS.getSecond();
  }
  // u-blox already encodes fix quality in fixType; require a 3D fix. (Dropping
  // the old `siv > 3` gate kills the flicker that produced false "fix lost".)
  if (myFixType >= 3) {
    myLat  = myGPS.getLatitude();
    myLon  = myGPS.getLongitude();
    long altMM = myGPS.getAltitudeMSL();
    myAltM = (int16_t)constrain(altMM / 1000, -32768L, 32767L);
    mySats = mySiv;        // sats used in this valid fix
  } else {
    mySats = 0;
  }
}

// Own-position + board telemetry event for the browser
void emitGps() {
  StaticJsonDocument<640> d;   // rich fix + fw1.8 temp/press/hdg fields
  d["t"] = "gps";
  d["fix"] = (mySats > 0);
  d["sats"] = mySats;
  d["siv"] = mySiv;
  d["fixType"] = myFixType;
  if (mySats > 0) {
    d["lat"] = serialized(String(myLat / 1e7, 7));
    d["lon"] = serialized(String(myLon / 1e7, 7));
    d["alt"] = myAltM;
    d["speed"]  = serialized(String(mySpeedMms * 0.0036f, 1));   // mm/s -> km/h
    d["course"] = myCourseDeg;
    d["hacc"]   = serialized(String(myHaccMm / 1000.0f, 1));     // mm -> m
    d["pdop"]   = serialized(String(myPdop * 0.01f, 1));
  }
  if (myTimeOk) {
    char utc[24];
    snprintf(utc, sizeof(utc), "%04u-%02u-%02uT%02u:%02u:%02uZ",
             myYear, myMonth, myDay, myHour, myMin, mySec);
    d["utc"] = utc;
  }
  if (bmpOK) {
    d["temp"]  = serialized(String(myTempC10 / 10.0f, 1));   // board °C
    d["press"] = serialized(String(myPressPa / 100.0f, 1));  // Pa -> hPa
  }
  if (hdgValid) d["hdg"] = myHdgDeg;                          // magnetic heading, deg
  d["batt"] = serialized(String(myBatt / 20.0f, 2));
  d["up"] = millis() / 1000;
  d["noise"] = serialized(String(lora.getRSSIInst(), 1));   // idle-RX noise floor
  d["tx"] = txCount;
  d["rx"] = rxCount;
  if (bytesRaw > 0) d["csave"] = (uint8_t)((bytesRaw - bytesAir) * 100 / bytesRaw);
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
  d["fw"] = "1.8";
  d["freq"] = LORA_FREQ;
  d["name"] = myName;
  d["id"] = myId;
  if (nameLocked) d["locked"] = true;
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
  display.print(F(" ")); display.print(myBatt / 20.0f, 1); display.print(F("V"));
  if (bmpOK) { display.print(F(" ")); display.print(myTempC10 / 10); display.print(F("C")); }
  display.println();
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
                  const uint8_t* payload, size_t payloadLen, bool withPos,
                  bool compressed) {
  uint8_t nameLen = strlen(myName);
  buf[0] = PROTO_MAGIC;
  buf[1] = PROTO_VERSION;
  buf[2] = type | (withPos ? FLAG_HAS_POS : 0) | (compressed ? FLAG_COMPRESSED : 0);
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
  size_t len = buildFrame(frame, TYPE_ACK, ++txMsgId, payload, sizeof(payload), false, false);
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
  bool     compressed = (rawType & FLAG_COMPRESSED) != 0;
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
  noteNeighbor(sender, rssi);   // track link quality for mesh trilateration (fw 1.7)

  if (type == TYPE_CHAT) {
    // Always ack (the previous ack may have been lost), but only show once
    sendAck(sender, msgId);
    if (duplicate) return;

    char text[MAX_TEXT + 1];
    if (compressed) {
      int dlen = smaz_decompress((const char*)payload, (int)payloadLen, text, MAX_TEXT);
      if (dlen < 0 || dlen > MAX_TEXT) return;   // corrupt/oversized -> drop
      text[dlen] = '\0';
    } else {
      size_t tlen = payloadLen > MAX_TEXT ? MAX_TEXT : payloadLen;
      memcpy(text, payload, tlen);
      text[tlen] = '\0';
    }

    StaticJsonDocument<512> d;
    d["t"] = "rx";
    d["from"] = from;
    d["id"] = sender;
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
    d["id"] = sender;
    d["rssi"] = serialized(String(rssi, 1));
    d["snr"]  = serialized(String(snr, 1));
    // Optional client/app profile carried in the JOIN payload (fw 1.5+).
    char client[MAX_CLIENT + 1];
    if (payloadLen > 0) {
      if (compressed) {
        int dl = smaz_decompress((const char*)payload, (int)payloadLen, client, MAX_CLIENT);
        if (dl > 0 && dl <= MAX_CLIENT) { client[dl] = '\0'; d["client"] = client; }
      } else {
        size_t cl = payloadLen > MAX_CLIENT ? MAX_CLIENT : payloadLen;
        memcpy(client, payload, cl); client[cl] = '\0';
        d["client"] = client;
      }
    }
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

  } else if (type == TYPE_BEACON) {
    if (duplicate || !hasPos) return;
    StaticJsonDocument<896> d;   // pos + ext (incl. fw1.8 env) + up to 6 nb entries
    d["t"] = "beacon";
    d["from"] = from;
    d["id"] = sender;
    d["rssi"] = serialized(String(rssi, 1));
    d["snr"]  = serialized(String(snr, 1));
    JsonObject pos = d.createNestedObject("pos");
    pos["sats"] = pSats;
    if (pSats > 0) {
      pos["lat"] = serialized(String(pLat / 1e7, 7));
      pos["lon"] = serialized(String(pLon / 1e7, 7));
      pos["alt"] = pAlt;
    }
    pos["batt"] = serialized(String(pBatt / 20.0f, 2));
    // fw 1.7+ neighbour block sits after the 4 ext bytes; fw 1.8+ env bytes sit
    // after the nb block. Compute that boundary ONCE and reuse it for both the
    // env decode and the nb-array decode so the two offsets can never drift.
    uint8_t nbCountRx = 0;
    size_t  nbEnd = 5;                 // first byte after the nb block
    bool    haveNb = false;
    if (payloadLen >= 5) {
      nbCountRx = payload[4];
      if (nbCountRx > 0 && payloadLen >= (size_t)(5 + nbCountRx * 3)) {
        nbEnd = 5 + nbCountRx * 3;
        haveNb = true;
      }
    }
    if (payloadLen >= 4) {            // fw 1.3+ extended beacon payload
      JsonObject ext = d.createNestedObject("ext");
      ext["speed"]   = payload[0];          // km/h
      ext["course"]  = payload[1] * 2;      // deg
      ext["hacc"]    = payload[2];          // m
      ext["fixType"] = payload[3];
      // fw 1.8+ environment bytes after the nb block. Only decode when the nb
      // block is well-formed (or absent) AND the frame is long enough; a 1.7
      // sender simply has no bytes there → fields omitted.
      if ((haveNb || nbCountRx == 0) && payloadLen >= nbEnd + 4) {
        const uint8_t* ev = payload + nbEnd;
        int8_t   t   = (int8_t)ev[0];
        uint16_t p10 = ev[1] | (ev[2] << 8);
        uint8_t  h   = ev[3];
        if (t != -128) ext["temp"]  = t;                               // °C
        if (p10 != 0)  ext["press"] = serialized(String(p10 / 10.0f, 1)); // hPa
        if (h != 255)  ext["hdg"]   = (int)h * 2;                       // deg
      }
    }
    // fw 1.7+ neighbour block: [count][ {id:2}{rssi:i8} x count ]
    if (haveNb) {
      JsonArray arr = d.createNestedArray("nb");
      for (uint8_t i = 0; i < nbCountRx; i++) {
        const uint8_t* e = payload + 5 + i * 3;
        JsonObject o = arr.createNestedObject();
        o["id"]   = (uint16_t)(e[0] | (e[1] << 8));
        o["rssi"] = (int8_t)e[2];
      }
    }
    if (d.overflowed()) return;   // dropped fields would corrupt mesh/telemetry
    serializeJson(d, SerialUSB);
    SerialUSB.println();
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
    if (!nameLocked) {            // a labeled board keeps its fixed identity
      strncpy(myName, v, MAX_NAME);
      myName[MAX_NAME] = '\0';
    }
    emitReady();
    refreshDisplay();

  } else if (strcmp(t, "client") == 0) {
    const char* v = d["v"];
    if (!v) { emitErr("missing client"); return; }
    strncpy(myClient, v, MAX_CLIENT);
    myClient[MAX_CLIENT] = '\0';
    // stored; transmitted as the JOIN payload on the next join

  } else if (strcmp(t, "join") == 0) {
    // Carry the compact client profile as the JOIN payload (smaz when smaller,
    // so on-air size never exceeds raw; empty when the browser hasn't sent one).
    uint8_t frame[MAX_FRAME];
    size_t clen = strlen(myClient);
    bool comp = false;
    const uint8_t* pl = (const uint8_t*)myClient;
    size_t plLen = clen;
    uint8_t cbuf[MAX_CLIENT * 2 + 16];
    if (clen) {
      int z = smaz_compress(myClient, (int)clen, (char*)cbuf, sizeof(cbuf));
      if (z > 0 && z < (int)clen) { pl = cbuf; plLen = (size_t)z; comp = true; }
    }
    size_t len = buildFrame(frame, TYPE_JOIN, ++txMsgId, pl, plLen, true, comp);
    int16_t state = transmitFrame(frame, len);
    if (state != ERR_NONE) emitErr("radio tx failed");
    hasJoined = true;                               // start periodic beacons
    nextBeaconAt = millis() + BEACON_MS + random(0, 8000);
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
    // smaz the text; use the compressed form only when it's actually smaller,
    // so the on-air payload is never larger than the raw text. Thai/emoji and
    // other incompressible input fall back to raw. cbuf is oversized for smaz's
    // worst case (it never bounds-checks the verbatim-escape writes).
    uint8_t cbuf[MAX_TEXT * 2 + 16];
    int clen = smaz_compress(text, (int)tlen, (char*)cbuf, sizeof(cbuf));
    const uint8_t* payload;
    size_t payloadLen;
    bool compressed;
    if (clen > 0 && clen < (int)tlen) {
      payload = cbuf; payloadLen = (size_t)clen; compressed = true;
    } else {
      payload = (const uint8_t*)text; payloadLen = tlen; compressed = false;
    }
    bytesRaw += tlen;          // running totals -> csave % on the gps event
    bytesAir += payloadLen;
    uint8_t frame[MAX_FRAME];
    size_t len = buildFrame(frame, TYPE_CHAT, ++txMsgId,
                            payload, payloadLen, true, compressed);
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

// Broadcast one position beacon: name + battery + position (if a fix is held) +
// motion ext + a slice of our neighbour-RSSI table (fw 1.7). Used for the boot
// ping and the periodic beacon.
void sendBeacon() {
  long kmh = mySpeedMms > 0 ? (mySpeedMms * 9L) / 2500L : 0;   // mm/s -> km/h
  uint32_t haccM = myHaccMm / 1000;
  uint8_t pl[4 + 1 + NB_REPORT_MAX * 3 + 4];   // ext + nb block + fw1.8 env bytes
  pl[0] = (uint8_t)(kmh > 255 ? 255 : kmh);
  pl[1] = (uint8_t)(myCourseDeg / 2);
  pl[2] = (uint8_t)(haccM > 255 ? 255 : haccM);
  pl[3] = myFixType;
  // Append up to NB_REPORT_MAX fresh neighbours: [count][ {id:2 LE}{rssi:i8} ].
  uint8_t n = 0;
  size_t off = 5;
  uint32_t now = millis();
  for (uint8_t i = 0; i < nbCount && n < NB_REPORT_MAX; i++) {
    if (now - nbTable[i].seen > NB_STALE_MS) continue;
    pl[off++] = nbTable[i].id & 0xFF;
    pl[off++] = nbTable[i].id >> 8;
    pl[off++] = (uint8_t)nbTable[i].rssi;   // i8 dBm as raw byte
    n++;
  }
  pl[4] = n;
  // fw 1.8 environment trailer: board temp, pressure, magnetic heading. Sentinels
  // (-128 / 0 / 255) mark "sensor absent or not yet calibrated"; a 1.7 receiver
  // never reads these bytes, a 1.8 receiver skips the sentinels.
  int8_t tByte = bmpOK ? (int8_t)constrain((myTempC10 + (myTempC10 >= 0 ? 5 : -5)) / 10,
                                           -127L, 127L)
                       : (int8_t)-128;
  uint16_t p10 = 0;
  if (bmpOK) { uint32_t v = (myPressPa + 5) / 10; p10 = v > 65535 ? 65535 : (uint16_t)v; }
  uint8_t hByte = hdgValid ? (uint8_t)(myHdgDeg / 2) : 255;
  pl[off++] = (uint8_t)tByte;
  pl[off++] = p10 & 0xFF;
  pl[off++] = p10 >> 8;
  pl[off++] = hByte;
  uint8_t frame[MAX_FRAME];
  size_t len = buildFrame(frame, TYPE_BEACON, ++txMsgId, pl, off, true, false);
  transmitFrame(frame, len);
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

  // Environment sensors (optional — fw 1.8). Both are non-fatal: a missing chip
  // just omits its telemetry fields, exactly like gpsOK/displayOK.
  bmpOK = bmp.begin();                // BMP180 temp/pressure (I2C 0x77)
  magOK = magInit();                  // LSM303DLHC magnetometer (I2C 0x1E)

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

  // One-time presence ping on boot, for EVERY board (chat handset or tracker).
  // Announces name + battery (+ position if already locked) so a listener can
  // confirm the board powered up — e.g. a solar board booting on marginal power
  // (repeated boot pings = it's brown-out rebooting). Best-effort, CAD-gated.
  sendBeacon();

  // Standalone (labeled) board: no browser will ever send "join", so start
  // beaconing autonomously. It announces its name + battery right away (so you
  // can confirm it's solar-powered and alive) and adds position once it gets a
  // GPS lock. A browser-driven board still waits for an explicit join.
  if (nameLocked) {
    hasJoined = true;
    nextBeaconAt = millis() + 10000;   // first beacon ~10 s after boot
  }
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

  // Periodic presence beacon (fw 1.8). Once joined, EVERY board beacons ~60 s
  // whether or not it holds a GPS fix — the trailer still carries name, battery
  // and (fw 1.8) temperature with sats=0, so a fixless board is still "alive" on
  // the roster and going silent becomes a trustworthy signal. Position is added
  // whenever a fix is held. Never while an ack is pending (don't trample a chat
  // retransmit window). transmitFrame does CAD listen-before-talk; jitter
  // desyncs two boards.
  if (hasJoined && !pendingActive && now >= nextBeaconAt) {
    nextBeaconAt = now + BEACON_MS + random(0, 8000);
    sendBeacon();
  }
}
