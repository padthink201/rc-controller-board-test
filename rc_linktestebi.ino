// =====================================================================
//  ESP-NOW LINK TEST  --  LONG RANGE MODE ON
//
//  Flash this SAME file onto BOTH ESP32 boards. Nothing to edit, no MAC
//  addresses to type in. Each board broadcasts a counter 20 times a
//  second and reports whatever it hears back from the other one.
//
//  Board: ESP32 Dev Module.  Arduino-ESP32 core 2.0.x or 3.x.
//  Serial Monitor at 115200.
//
//  LONG RANGE MODE
//    LINK_USE_LR puts the radio on Espressif's long-range PHY: a much
//    slower, much tougher modulation that trades data rate for
//    distance. It is proprietary - ESP32s can only talk it to other
//    ESP32s - which is fine, because that is all we have.
//
//    BOTH boards must run with the same LINK_USE_LR setting. One on
//    and one off is not a weak link, it is no link at all, and it
//    looks exactly like a broken board. If nothing works, check this
//    first.
//
//    While LR is on, these boards cannot talk ordinary WiFi at all -
//    no access points, no OTA. We don't need any of that.
//
//  WHAT YOU SHOULD SEE
//    Each board prints its own MAC address at boot - write both down,
//    the real sketches want them. After a second or two each should
//    report the other at "loss 0%". The onboard blue LED is SOLID
//    while the link is alive and goes OUT when it dies, so you can
//    walk one board away and watch for the range limit without
//    staring at a laptop.
//
//  RANGE TEST
//    Power one board from a USB power bank, leave the other on the
//    laptop, and walk away. The summary line tracks the worst RSSI it
//    has seen and how many times the link has dropped, so you can walk
//    out, walk back, and read off what happened.
//
//    Hold the board up, not down by your leg - your body blocks 2.4
//    GHz badly. Keep both boards off the ground and away from metal.
//    The antenna is the little zigzag at the end of the board: leave
//    it sticking out past everything else, vertical, nowhere near the
//    battery or the ESC.
// =====================================================================
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// Keep these the same as the real sketches, so a good result here
// actually means something for the real thing.
#define LINK_CHANNEL     1      // 1..13
#define LINK_USE_LR      1      // 1 = long range. BOTH boards or neither.

#define SEND_PERIOD_MS   50     // 20 packets per second
#define REPORT_MS        1000
#define LINK_TIMEOUT_MS  500
#define PIN_LED          2      // onboard blue LED

static const uint8_t MAGIC = 0x5A;
static uint8_t BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  id;      // last byte of the sender's MAC, so the boards differ
  uint32_t seq;
} Ping;

static uint8_t  myId  = 0;
static uint32_t txSeq = 0;

// written by the radio callback, read by loop()
static volatile uint32_t rxWindow = 0;
static volatile uint32_t lastRxMs = 0;
static volatile uint32_t peerSeq  = 0;
static volatile int8_t   peerRssi = 0;
static uint8_t  peerMac[6] = { 0 };

static bool     seenPeer   = false;
static bool     wasAlive   = false;
static int8_t   worstRssi  = 0;
static uint32_t dropouts   = 0;
static uint32_t tSend = 0, tReport = 0;

// ---------------------------------------------------------------------
// The radio callback's signature changed between core 2.x and 3.x, so
// there are two complete versions of it, each calling the same handler.
// Do not merge them back into one function with the #if inside the
// signature - the Arduino IDE's prototype generator cannot parse that.
void handlePing(const uint8_t *src, const uint8_t *data, int len, int8_t rssi) {
  if (len != (int)sizeof(Ping)) return;
  Ping p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != MAGIC) return;
  if (p.id == myId) return;              // ignore anything that looks like us

  if (src) memcpy(peerMac, src, 6);
  peerRssi = rssi;
  peerSeq  = p.seq;
  lastRxMs = millis();
  rxWindow++;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  handlePing(info ? info->src_addr : NULL, data, len,
             (info && info->rx_ctrl) ? (int8_t)info->rx_ctrl->rssi : 0);
}
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  handlePing(mac, data, len, 0);
}
#endif

static void halt(const char *msg) {
  Serial.println(msg);
  while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  delay(400);

  Serial.println(F("\n========== ESP-NOW LINK TEST =========="));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  esp_wifi_set_ps(WIFI_PS_NONE);          // stop the radio dozing off

#if LINK_USE_LR
  // LR only. Dropping 11b/g/n means every frame goes out on the
  // long-range PHY, and the board stops being able to talk normal
  // WiFi - which we don't want anyway.
  if (esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR) != ESP_OK)
    Serial.println(F("!! could not enable long-range mode"));
  Serial.println(F("mode       : LONG RANGE (both boards must match)"));
#else
  Serial.println(F("mode       : normal 802.11"));
#endif

  esp_wifi_set_channel(LINK_CHANNEL, WIFI_SECOND_CHAN_NONE);

  uint8_t m[6];
  WiFi.macAddress(m);
  myId = m[5];
  Serial.print(F("MY MAC     : ")); Serial.println(WiFi.macAddress());
  Serial.printf("paste this : { 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X };\n",
                m[0], m[1], m[2], m[3], m[4], m[5]);
  Serial.printf("channel    : %d\n", LINK_CHANNEL);

  if (esp_now_init() != ESP_OK) halt("esp_now_init FAILED - press reset");
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t pi;
  memset(&pi, 0, sizeof(pi));
  memcpy(pi.peer_addr, BROADCAST, 6);
  pi.channel = LINK_CHANNEL;
  pi.encrypt = false;
  if (esp_now_add_peer(&pi) != ESP_OK) halt("add_peer FAILED - press reset");


  Serial.println(F("sending. flash this same sketch onto the other board.\n"));
}

// ---------------------------------------------------------------------
void loop() {
  uint32_t now = millis();

  if (now - tSend >= SEND_PERIOD_MS) {
    tSend = now;
    Ping p;
    p.magic = MAGIC;
    p.id    = myId;
    p.seq   = ++txSeq;
    esp_now_send(BROADCAST, (uint8_t *)&p, sizeof(p));
  }

  bool alive = (lastRxMs != 0) && (now - lastRxMs < LINK_TIMEOUT_MS);
  digitalWrite(PIN_LED, alive ? HIGH : LOW);

  if (alive) {
    if (!seenPeer) {
      seenPeer = true;
      Serial.printf(">>> FIRST CONTACT with %02X:%02X:%02X:%02X:%02X:%02X\n",
                    peerMac[0], peerMac[1], peerMac[2],
                    peerMac[3], peerMac[4], peerMac[5]);
    }
    if (worstRssi == 0 || peerRssi < worstRssi) worstRssi = peerRssi;
  }
  if (wasAlive && !alive) { dropouts++; Serial.println(F("--- link lost ---")); }
  if (!wasAlive && alive && seenPeer) Serial.println(F("--- link back ---"));
  wasAlive = alive;

  if (now - tReport >= REPORT_MS) {
    tReport = now;
    uint32_t got = rxWindow;
    rxWindow = 0;

    const uint32_t expected = REPORT_MS / SEND_PERIOD_MS;
    if (got > expected) got = expected;
    uint32_t loss = 100 - (got * 100 / expected);

    if (!alive) {
      Serial.printf("sent %-6lu   ** nothing heard **            drops %lu\n",
                    (unsigned long)txSeq, (unsigned long)dropouts);
    } else {
      Serial.printf("sent %-6lu   %02X:%02X:%02X:%02X:%02X:%02X  "
                    "got %2lu/%lu  loss %3lu%%  rssi %4d (worst %4d)  drops %lu\n",
                    (unsigned long)txSeq,
                    peerMac[0], peerMac[1], peerMac[2],
                    peerMac[3], peerMac[4], peerMac[5],
                    (unsigned long)got, (unsigned long)expected,
                    (unsigned long)loss, (int)peerRssi, (int)worstRssi,
                    (unsigned long)dropouts);
    }
  }

  delay(1);
}
