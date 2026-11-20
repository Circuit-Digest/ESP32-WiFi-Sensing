// WiFi motion detection - receiver v8 (ESP-NOW mesh, automatic peer discovery)
// Router as transmitter (ping -> CSI). Board-to-board sensing via ESP-NOW
// beacons: boards find each other automatically, no MAC addresses to type.
// Arduino-ESP32 core 3.x. Keep webpage.h in the same folder as this file.

#include "esp_idf_version.h"
#include "lwip/ip_addr.h"
#include "ping/ping_sock.h"
#include <WebServer.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

// ---------- EDIT THESE ----------
const char *WIFI_SSID = "Semicon ACT";
const char *WIFI_PASS = "cracksen1605";
const char *NAME = "XIAO"; // unique name per board
// Only needed on the board whose web page you open: the OTHER boards' IPs.
const char *PEER_IPS = "192.168.0.2,192.168.0.3,192.168.0.24";
// --------------------------------

uint8_t TX_MAC[6] = {0}; // router MAC - filled from WiFi.BSSID()

#define NSUB 64  // subcarriers used
#define WIN 50   // sliding window = 50 packets (~1 s at 50 pps)
#define STEP 5   // recompute every STEP packets
#define EVMAX 30 // motion events remembered

const uint32_t HOLD_MS = 3000; // keep "motion" on for 3 s after last trigger
const uint32_t CAL_MS = 15000; // calibration time (keep room empty)

// Beacon payload: lets boards recognise each other (ignores other ESP-NOW
// users)
static const uint8_t BEACON_MAGIC[4] = {'W', 'M', 'S', '1'};

// ---- Router CSI state ----
float buf[WIN][NSUB];
int widx = 0, filled = 0, sinceCalc = 0;

volatile float smoothMetric = 0, threshold = 0.05f, calMax = 0;
volatile float baseline = 0.03f, sensWifi = 1.15f, sensEn = 1.35f;
volatile bool calibrating = true, motion = false;
volatile uint32_t pktCount = 0, pps = 0, motionUntil = 0, calStart = 0;

// ---- ESP-NOW per-peer CSI state (filled automatically) ----
#define EN_WIN 30  // window for board-to-board links
#define MAXPEERS 8 // up to 8 other boards
struct PeerState {
  uint8_t mac[6];
  float buf[EN_WIN][NSUB];
  int widx, filled, step;
  float smooth, threshold, calMax, baseline;
  bool motion, calibrating;
  uint32_t motionUntil, calStart, pktCount;
  float lastRatio; // smooth / threshold, shown on the web page
};
PeerState peerSt[MAXPEERS];
volatile int peerStCount = 0;

// ---- Motion event log ----
struct Ev {
  uint32_t start, dur;
  float peak;
};
Ev evs[EVMAX];
volatile uint32_t evN = 0;
uint32_t evStart = 0;
float evPeak = 0;
bool inEv = false;

// ---- Recent samples for the live graph ----
#define SMAX 64
struct Sm {
  uint32_t t;
  float v;
};
Sm sms[SMAX];
volatile uint32_t smN = 0;

WebServer server(80);

#include "webpage.h"

// Add a board to the peer list the first time we hear its beacon
static void addPeerIfNew(const uint8_t *mac) {
  int n = peerStCount;
  for (int p = 0; p < n; p++)
    if (memcmp(peerSt[p].mac, mac, 6) == 0)
      return;
  if (n >= MAXPEERS)
    return;
  PeerState &ps = peerSt[n];
  memset(&ps, 0, sizeof(ps));
  memcpy(ps.mac, mac, 6);
  ps.baseline = 0.03f;
  ps.calibrating = true;
  ps.calStart = millis();
  peerStCount = n + 1; // publish last, after the slot is fully set up
}

// Process one packet's CSI from a peer board into its own sliding window
static void processPeerPacket(PeerState &ps, const int8_t *d, int len) {
  if (len / 2 < NSUB)
    return;
  float a[NSUB], sum = 0;
  for (int k = 0; k < NSUB; k++) {
    float im = d[2 * k], re = d[2 * k + 1];
    a[k] = sqrtf(re * re + im * im);
    sum += a[k];
  }
  float mean = sum / NSUB;
  if (mean < 1)
    return;
  for (int k = 0; k < NSUB; k++)
    ps.buf[ps.widx][k] = a[k] / mean;
  ps.widx = (ps.widx + 1) % EN_WIN;
  if (ps.filled < EN_WIN)
    ps.filled++;
  ps.pktCount++;

  if (++ps.step < STEP || ps.filled < EN_WIN)
    return;
  ps.step = 0;

  float tot = 0;
  int cnt = 0;
  for (int k = 0; k < NSUB; k++) {
    float m = 0;
    for (int i = 0; i < EN_WIN; i++)
      m += ps.buf[i][k];
    m /= EN_WIN;
    if (m < 0.2f)
      continue;
    float v = 0;
    for (int i = 0; i < EN_WIN; i++) {
      float dd = ps.buf[i][k] - m;
      v += dd * dd;
    }
    tot += sqrtf(v / EN_WIN);
    cnt++;
  }
  float mt = cnt ? tot / cnt : 0;
  ps.smooth = 0.6f * ps.smooth + 0.4f * mt;

  uint32_t now = millis();
  if (ps.calibrating) {
    if (ps.smooth > ps.calMax)
      ps.calMax = ps.smooth;
    if (now - ps.calStart > CAL_MS) {
      ps.baseline = ps.calMax;
      ps.threshold = ps.baseline * sensEn + 0.003f;
      ps.calibrating = false;
    }
    ps.motion = false;
  } else {
    ps.lastRatio = ps.smooth / ps.threshold;
    if (ps.smooth > ps.threshold)
      ps.motionUntil = now + HOLD_MS;
    ps.motion = (int32_t)(ps.motionUntil - now) > 0;
  }
}

// CSI callback - runs for every received packet (router replies or peer
// beacons)
void csiCb(void *ctx, wifi_csi_info_t *info) {
  if (!info || !info->buf)
    return;
  if (info->len / 2 < NSUB)
    return;

  // --- Router path ---
  if (memcmp(info->mac, TX_MAC, 6) == 0) {
    const int8_t *d = info->buf;
    float a[NSUB], sum = 0;
    for (int k = 0; k < NSUB; k++) {
      float im = d[2 * k], re = d[2 * k + 1];
      a[k] = sqrtf(re * re + im * im);
      sum += a[k];
    }
    float mean = sum / NSUB;
    if (mean < 1)
      return;
    for (int k = 0; k < NSUB; k++)
      buf[widx][k] = a[k] / mean;
    widx = (widx + 1) % WIN;
    if (filled < WIN)
      filled++;
    pktCount++;

    if (++sinceCalc < STEP || filled < WIN)
      return;
    sinceCalc = 0;

    float tot = 0;
    int cnt = 0;
    for (int k = 0; k < NSUB; k++) {
      float m = 0;
      for (int i = 0; i < WIN; i++)
        m += buf[i][k];
      m /= WIN;
      if (m < 0.2f)
        continue;
      float v = 0;
      for (int i = 0; i < WIN; i++) {
        float dd = buf[i][k] - m;
        v += dd * dd;
      }
      tot += sqrtf(v / WIN);
      cnt++;
    }
    float mt = cnt ? tot / cnt : 0;
    smoothMetric = 0.6f * smoothMetric + 0.4f * mt;

    uint32_t now = millis();
    if (calibrating) {
      if (smoothMetric > calMax)
        calMax = smoothMetric;
      if (now - calStart > CAL_MS) {
        baseline = calMax;
        threshold = baseline * sensWifi + 0.003f;
        calibrating = false;
      }
      motion = false;
      inEv = false;
    } else {
      float ratio = smoothMetric / threshold;
      if (smoothMetric > threshold)
        motionUntil = now + HOLD_MS;
      bool m = (int32_t)(motionUntil - now) > 0;
      if (m && !inEv) {
        inEv = true;
        evStart = now;
        evPeak = ratio;
      }
      if (m && inEv && ratio > evPeak)
        evPeak = ratio;
      if (!m && inEv) {
        inEv = false;
        int32_t dur = (int32_t)((motionUntil - HOLD_MS) - evStart);
        if (dur < 0)
          dur = 0;
        Ev &e = evs[evN % EVMAX];
        e.start = evStart;
        e.dur = (uint32_t)dur;
        e.peak = evPeak;
        evN++;
      }
      motion = m;
    }
    Sm &q = sms[smN % SMAX];
    q.t = now;
    q.v = calibrating ? 0 : smoothMetric / threshold;
    smN++;
    return;
  }

  // --- Peer board path (ESP-NOW beacons) ---
  int n = peerStCount;
  for (int p = 0; p < n; p++) {
    if (memcmp(info->mac, peerSt[p].mac, 6) == 0) {
      processPeerPacket(peerSt[p], info->buf, info->len);
      return;
    }
  }
}

// ESP-NOW receive callback: learn who the other boards are
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data,
                  int len) {
  if (len == 4 && memcmp(data, BEACON_MAGIC, 4) == 0)
    addPeerIfNew(info->src_addr);
}

// Ping the router ~50 times a second so CSI keeps flowing
void startPing() {
  ip_addr_t target;
  ipaddr_aton(WiFi.gatewayIP().toString().c_str(), &target);
  esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
  cfg.target_addr = target;
  cfg.count = ESP_PING_COUNT_INFINITE;
  cfg.interval_ms = 20;
  cfg.timeout_ms = 1000;
  cfg.data_size = 1;
  cfg.task_stack_size = 3072;
  esp_ping_callbacks_t cbs = {};
  esp_ping_handle_t h;
  esp_ping_new_session(&cfg, &cbs, &h);
  esp_ping_start(h);
}

// Broadcast a beacon at 20 Hz so the other boards can measure CSI from us
void espNowBeaconTask(void *pv) {
  static const uint8_t broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  for (;;) {
    esp_now_send(broadcast, BEACON_MAGIC, 4);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ---------------------------------------------------------------------
// HTTP handlers
// ---------------------------------------------------------------------
void sendCors() { server.sendHeader("Access-Control-Allow-Origin", "*"); }

// Send large PROGMEM strings in 1 KB chunks so they never exceed the lwIP TCP
// send buffer
static void sendProgmemChunks(const char *data) {
  size_t len = strlen_P(data);
  const size_t CHUNK = 1024;
  for (size_t i = 0; i < len; i += CHUNK) {
    size_t n = (len - i < CHUNK) ? (len - i) : CHUNK;
    server.sendContent_P(data + i, n);
  }
}

void sendPage() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8", "");
  sendProgmemChunks(HTML_HEAD);
  String peerLine = "const PEER=\"" + String(PEER_IPS) + "\";\n";
  server.sendContent(peerLine);
  sendProgmemChunks(HTML_TAIL);
  server.sendContent("");
}

// /espnow -> what THIS board measures from each other board
void sendEspNow() {
  String s = "[";
  int n = peerStCount;
  for (int p = 0; p < n; p++) {
    if (p)
      s += ",";
    char mac[18];
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             peerSt[p].mac[0], peerSt[p].mac[1], peerSt[p].mac[2],
             peerSt[p].mac[3], peerSt[p].mac[4], peerSt[p].mac[5]);
    s += "{\"mac\":\"";
    s += mac;
    s += "\",\"motion\":";
    s += peerSt[p].motion ? "true" : "false";
    s += ",\"calib\":";
    s += peerSt[p].calibrating ? "true" : "false";
    s += ",\"ratio\":";
    s += String(peerSt[p].lastRatio, 2);
    s += ",\"pkt\":";
    s += String(peerSt[p].pktCount);
    s += "}";
  }
  s += "]";
  sendCors();
  server.send(200, "application/json", s);
}

void setup() {
  Serial.begin(115200);
  delay(1500); // give ESP32-S3 USB CDC time to connect to PC serial monitor

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  WiFi.setSleep(false);

  memcpy(TX_MAC, WiFi.BSSID(), 6);
  Serial.println("\nIP : " + WiFi.localIP().toString() + "   channel " +
                 String(WiFi.channel()));
  Serial.println("MAC: " + WiFi.macAddress() + "   Router: " + WiFi.BSSIDstr());

  // --- CSI ---
  wifi_csi_config_t csiCfg = {};
  csiCfg.lltf_en = true;
  csiCfg.htltf_en = false;
  csiCfg.stbc_htltf2_en = false;
  csiCfg.ltf_merge_en = false;
  csiCfg.channel_filter_en = false;
  csiCfg.manu_scale = false;
  esp_wifi_set_csi_config(&csiCfg);
  esp_wifi_set_csi_rx_cb(csiCb, NULL);
  esp_wifi_set_csi(true);

  // --- ESP-NOW ---
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowRecv);

    esp_now_peer_info_t bcast = {};
    memset(bcast.peer_addr, 0xFF, 6);
    bcast.channel = 0; // use the channel the router is on
    bcast.ifidx = WIFI_IF_STA;
    bcast.encrypt = false;
    esp_now_add_peer(&bcast);

    // ESP-NOW defaults to a 1 Mbps legacy rate, and the chip gives NO CSI for
    // those frames. Send as 802.11n MCS0 so the receiving boards get CSI.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_now_rate_config_t rc = {};
    rc.phymode = WIFI_PHY_MODE_HT20;
    rc.rate = WIFI_PHY_RATE_MCS0_LGI;
    rc.ersu = false;
    rc.dcm = false;
    esp_now_set_peer_rate_config(bcast.peer_addr, &rc);
#else
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_MCS0_LGI);
#endif

    xTaskCreatePinnedToCore(espNowBeaconTask, "enBeacon", 4096, NULL, 1, NULL,
                            0);
    Serial.println("ESP-NOW ready. Beacons on; other boards are discovered "
                   "automatically.");
  } else {
    Serial.println("ESP-NOW init failed - running router-only mode.");
  }

  startPing();
  calStart = millis();

  // --- Web server routes ---
  server.on("/", sendPage);
  server.on("/espnow", sendEspNow);

  server.on("/status", []() {
    uint32_t n = smN;
    uint32_t since =
        server.hasArg("since") ? (uint32_t)server.arg("since").toInt() : n;
    if (since > n)
      since = 0;
    if (n - since > SMAX)
      since = n - SMAX;
    String s;
    s.reserve(900);
    s += "{\"name\":\"";
    s += NAME;
    s += "\",\"mac\":\"";
    s += WiFi.macAddress();
    s += "\",\"motion\":";
    s += motion ? "true" : "false";
    s += ",\"calibrating\":";
    s += calibrating ? "true" : "false";
    s += ",\"metric\":";
    s += String((float)smoothMetric, 4);
    s += ",\"threshold\":";
    s += String((float)threshold, 4);
    s += ",\"pps\":";
    s += String((unsigned)pps);
    s += ",\"rssi\":";
    s += String(WiFi.RSSI());
    s += ",\"up\":";
    s += String(millis());
    s += ",\"n\":";
    s += String(n);
    s += ",\"enPeers\":";
    s += String((int)peerStCount);
    s += ",\"s\":[";
    for (uint32_t i = since; i < n; i++) {
      Sm q = sms[i % SMAX];
      if (i > since)
        s += ",";
      s += "[" + String(q.t) + "," + String(q.v, 2) + "]";
    }
    s += "]}";
    sendCors();
    server.send(200, "application/json", s);
  });

  server.on("/events", []() {
    uint32_t n = evN;
    int cnt = n < EVMAX ? (int)n : EVMAX;
    String s =
        "{\"up\":" + String(millis()) + ",\"n\":" + String(n) + ",\"ev\":[";
    for (int i = 0; i < cnt; i++) {
      Ev e = evs[(n - cnt + i) % EVMAX];
      if (i)
        s += ",";
      s += "{\"start\":" + String(e.start) + ",\"dur\":" + String(e.dur) +
           ",\"peak\":" + String(e.peak, 2) + "}";
    }
    s += "]}";
    sendCors();
    server.send(200, "application/json", s);
  });

  server.on("/calibrate", []() {
    calMax = 0;
    calStart = millis();
    calibrating = true;
    int n = peerStCount;
    for (int p = 0; p < n; p++) { // also recalibrate the board-to-board links
      peerSt[p].calMax = 0;
      peerSt[p].calStart = millis();
      peerSt[p].calibrating = true;
    }
    sendCors();
    server.send(200, "text/plain", "ok");
  });

  server.on("/sens", []() {
    String type = server.hasArg("type") ? server.arg("type") : "";
    float val = server.hasArg("v") ? server.arg("v").toFloat() : 0.0f;
    if (server.hasArg("wifi")) {
      sensWifi = server.arg("wifi").toFloat();
      threshold = baseline * sensWifi + 0.003f;
    }
    if (server.hasArg("en")) {
      sensEn = server.arg("en").toFloat();
      int n = peerStCount;
      for (int p = 0; p < n; p++)
        peerSt[p].threshold = peerSt[p].baseline * sensEn + 0.003f;
    }
    if (type == "wifi" && val > 0) {
      sensWifi = val;
      threshold = baseline * sensWifi + 0.003f;
    } else if (type == "en" && val > 0) {
      sensEn = val;
      int n = peerStCount;
      for (int p = 0; p < n; p++)
        peerSt[p].threshold = peerSt[p].baseline * sensEn + 0.003f;
    } else if (server.hasArg("v") && type == "") {
      sensWifi = val;
      sensEn = val;
      threshold = baseline * sensWifi + 0.003f;
      int n = peerStCount;
      for (int p = 0; p < n; p++)
        peerSt[p].threshold = peerSt[p].baseline * sensEn + 0.003f;
    }
    sendCors();
    server.send(200, "text/plain", "ok");
  });

  server.begin();
}

void loop() {
  server.handleClient();
  static uint32_t lastT = 0, lastCnt = 0;
  if (millis() - lastT >= 1000) {
    lastT = millis();
    pps = pktCount - lastCnt;
    lastCnt = pktCount;
    Serial.printf("pps=%u metric=%.4f thr=%.4f motion=%d events=%u peers=%d",
                  (unsigned)pps, (float)smoothMetric, (float)threshold,
                  (int)motion, (unsigned)evN, (int)peerStCount);
    int n = peerStCount;
    for (int p = 0; p < n; p++) {
      char mac[7];
      snprintf(mac, sizeof(mac), "%02X%02X%02X", peerSt[p].mac[3],
               peerSt[p].mac[4], peerSt[p].mac[5]);
      Serial.printf("  EN[%s] pkts=%u ratio=%.2f(%s)", mac,
                    (unsigned)peerSt[p].pktCount, peerSt[p].lastRatio,
                    peerSt[p].motion ? "MOV" : "---");
    }
    Serial.println();
  }
}
