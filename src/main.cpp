/**
 * BeePlan gateway — ESP-NOW v1/v2 RX, flash spool, REST uplink.
 */
#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <cstring>
#include <esp_now.h>
#include <esp_random.h>
#include <esp_wifi.h>
#include <time.h>

#include "beeplan_io.h"
#include "beeplan_espnow.h"
#include "config.h"
#include "envelope_v2.h"

namespace {

constexpr size_t kRxQueueCap = 512;
constexpr uint32_t kHeartbeatIntervalMs = 15U * 60U * 1000U;
constexpr uint32_t kDrainIntervalMs = 30U * 1000U;
constexpr size_t kMaxSpoolLinesPerBatch = 32;
constexpr size_t kMaxSamplesPerBatch = 96;
constexpr char kSpoolPath[] = "/beeplan_spool.jsonl";
constexpr char kSpoolTmpPath[] = "/beeplan_spool.tmp";
/** Мягкий лимит строк spool; при превышении — отбрасываем старые. */
constexpr uint16_t kSpoolSoftMaxLines = 400;
/** Запас свободного места для перезаписи spool (байт). */
constexpr size_t kSpoolCompactReserveBytes = 32768;
constexpr char kPrefsNamespace[] = "beeplan";
constexpr char kPrefsGwReportSeq[] = "gw_report_seq";

Preferences g_prefs;

struct EdgeWakeConfig {
  char public_id[33]{};
  uint16_t wake_interval_sec = 0;
};

constexpr size_t kMaxEdgeWakeConfigs = 100;
EdgeWakeConfig g_edge_wake_configs[kMaxEdgeWakeConfigs];
size_t g_edge_wake_count = 0;

void load_edge_wake_configs(const JsonDocument& doc) {
  g_edge_wake_count = 0;
  JsonArrayConst arr = doc["edge_devices"].as<JsonArrayConst>();
  if (arr.isNull()) {
    return;
  }
  for (JsonObjectConst item : arr) {
    if (g_edge_wake_count >= kMaxEdgeWakeConfigs) {
      break;
    }
    const char* pid = item["public_id"] | "";
    if (pid[0] == '\0') {
      continue;
    }
    EdgeWakeConfig& slot = g_edge_wake_configs[g_edge_wake_count++];
    strncpy(slot.public_id, pid, sizeof(slot.public_id) - 1);
    const int wake = item["wake_interval_sec"] | 3600;
    if (wake >= 10 && wake <= 86400) {
      slot.wake_interval_sec = static_cast<uint16_t>(wake);
    } else {
      slot.wake_interval_sec = 3600;
    }
  }
}

uint16_t wake_interval_for_public_id(const char* device_id) {
  if (device_id == nullptr) {
    return 0;
  }
  for (size_t i = 0; i < g_edge_wake_count; ++i) {
    if (strcmp(g_edge_wake_configs[i].public_id, device_id) == 0) {
      return g_edge_wake_configs[i].wake_interval_sec;
    }
  }
  return 0;
}

struct RxV1Item {
  EnvelopeV1 envelope;
};

struct RxV2Item {
  ReportFrameV2 frame;
  uint8_t src_mac[6];
  /** RSSI приёма пакета на gateway (dBm); -127 если недоступен. */
  int8_t rx_rssi_dbm;
};

enum class RxKind : uint8_t { V1, V2 };

struct RxItem {
  RxKind kind;
  union {
    RxV1Item v1;
    RxV2Item v2;
  };
};

RxItem g_rx_queue[kRxQueueCap];
volatile uint16_t g_rx_head = 0;
volatile uint16_t g_rx_tail = 0;
portMUX_TYPE g_rx_mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t g_last_heartbeat_ms = 0;
uint32_t g_last_drain_ms = 0;
uint32_t g_drain_backoff_ms = kDrainIntervalMs;
uint16_t g_spool_pending_count = 0;

/** ESP-IDF 4.x ESP-NOW recv_cb не отдаёт RSSI — кэш из promiscuous mode. */
constexpr size_t kRssiCacheSlots = 8;
constexpr uint32_t kRssiCacheTtlMs = 500;

struct RssiCacheSlot {
  uint8_t mac[6]{};
  int8_t rssi_dbm = -127;
  uint32_t seen_ms = 0;
};

RssiCacheSlot g_rssi_cache[kRssiCacheSlots];
portMUX_TYPE g_rssi_mux = portMUX_INITIALIZER_UNLOCKED;

bool mac_equal(const uint8_t* a, const uint8_t* b) {
  return memcmp(a, b, 6) == 0;
}

bool rssi_dbm_valid(int8_t rssi_dbm) {
  return rssi_dbm < 0 && rssi_dbm > -120;
}

void rssi_cache_store(const uint8_t* mac, int8_t rssi_dbm) {
  if (mac == nullptr || !rssi_dbm_valid(rssi_dbm)) {
    return;
  }
  const uint32_t now = millis();
  portENTER_CRITICAL(&g_rssi_mux);
  for (auto& slot : g_rssi_cache) {
    if (mac_equal(slot.mac, mac)) {
      slot.rssi_dbm = rssi_dbm;
      slot.seen_ms = now;
      portEXIT_CRITICAL(&g_rssi_mux);
      return;
    }
  }
  size_t replace_idx = 0;
  uint32_t oldest_ms = UINT32_MAX;
  for (size_t i = 0; i < kRssiCacheSlots; ++i) {
    if (g_rssi_cache[i].seen_ms == 0) {
      replace_idx = i;
      oldest_ms = 0;
      break;
    }
    if (g_rssi_cache[i].seen_ms < oldest_ms) {
      oldest_ms = g_rssi_cache[i].seen_ms;
      replace_idx = i;
    }
  }
  memcpy(g_rssi_cache[replace_idx].mac, mac, 6);
  g_rssi_cache[replace_idx].rssi_dbm = rssi_dbm;
  g_rssi_cache[replace_idx].seen_ms = now;
  portEXIT_CRITICAL(&g_rssi_mux);
}

int8_t rssi_cache_lookup(const uint8_t* mac) {
  if (mac == nullptr) {
    return -127;
  }
  const uint32_t now = millis();
  portENTER_CRITICAL(&g_rssi_mux);
  for (const auto& slot : g_rssi_cache) {
    if (slot.seen_ms != 0 && mac_equal(slot.mac, mac) && (now - slot.seen_ms) <= kRssiCacheTtlMs) {
      const int8_t rssi = slot.rssi_dbm;
      portEXIT_CRITICAL(&g_rssi_mux);
      return rssi;
    }
  }
  portEXIT_CRITICAL(&g_rssi_mux);
  return -127;
}

void promiscuous_rx_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (buf == nullptr) {
    return;
  }
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) {
    return;
  }
  auto* pkt = static_cast<wifi_promiscuous_pkt_t*>(buf);
  const uint16_t sig_len = pkt->rx_ctrl.sig_len;
  if (sig_len < 16 || sig_len > 250) {
    return;
  }
  const uint8_t* src = pkt->payload + 10;
  rssi_cache_store(src, static_cast<int8_t>(pkt->rx_ctrl.rssi));
}

bool rssi_promiscuous_begin() {
  if (esp_wifi_set_promiscuous(false) != ESP_OK) {
    return false;
  }
  if (esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb) != ESP_OK) {
    return false;
  }
  return esp_wifi_set_promiscuous(true) == ESP_OK;
}

int8_t resolve_rx_rssi_dbm(const uint8_t* mac, int8_t callback_rssi_dbm) {
  if (rssi_dbm_valid(callback_rssi_dbm)) {
    return callback_rssi_dbm;
  }
  const int8_t cached = rssi_cache_lookup(mac);
  if (rssi_dbm_valid(cached)) {
    return cached;
  }
  return -127;
}

bool rx_queue_push(const RxItem& item) {
  portENTER_CRITICAL(&g_rx_mux);
  const uint16_t next = static_cast<uint16_t>((g_rx_head + 1) % kRxQueueCap);
  if (next == g_rx_tail) {
    portEXIT_CRITICAL(&g_rx_mux);
    BEE_SERIAL.println("ESP-NOW rx queue full — dropped packet");
    return false;
  }
  g_rx_queue[g_rx_head] = item;
  g_rx_head = next;
  portEXIT_CRITICAL(&g_rx_mux);
  return true;
}

bool rx_queue_pop(RxItem& out) {
  portENTER_CRITICAL(&g_rx_mux);
  if (g_rx_tail == g_rx_head) {
    portEXIT_CRITICAL(&g_rx_mux);
    return false;
  }
  out = g_rx_queue[g_rx_tail];
  g_rx_tail = static_cast<uint16_t>((g_rx_tail + 1) % kRxQueueCap);
  portEXIT_CRITICAL(&g_rx_mux);
  return true;
}

bool ensure_edge_peer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) {
    return true;
  }
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = WiFi.channel();
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  return esp_now_add_peer(&peer) == ESP_OK;
}

bool send_ack_v2(const ReportFrameV2& frame, const uint8_t* dst_mac) {
  if (!ensure_edge_peer(dst_mac)) {
    return false;
  }
  AckFrameV2 ack{};
  ack.magic = kBeeplanMagicAck;
  ack.proto_version = kBeeplanProtoV2;
  ack.ack_seq = frame.seq;
  memset(ack.device_id, 0, sizeof(ack.device_id));
  strncpy(ack.device_id, frame.device_id, sizeof(ack.device_id) - 1);
  const time_t now = time(nullptr);
  ack.gateway_unix_ts =
      now > static_cast<time_t>(kMinValidUnixTs) ? static_cast<uint32_t>(now) : 0U;
  ack.proto_version = kBeeplanProtoAckV3;
  ack.wake_interval_sec = wake_interval_for_public_id(frame.device_id);
  return esp_now_send(dst_mac, reinterpret_cast<uint8_t*>(&ack), sizeof(ack)) == ESP_OK;
}

bool is_channel_probe(const EnvelopeV1& e) {
  return e.unix_ts == 0 && e.metric == 0 && e.i16_a == 0 && e.i16_b == 0;
}

void handle_v1(const uint8_t* data, int len) {
  if (len != static_cast<int>(sizeof(EnvelopeV1))) {
    return;
  }
  EnvelopeV1 tmp{};
  memcpy(&tmp, data, sizeof(tmp));
  if (tmp.magic != kBeeplanMagicV1) {
    return;
  }
  if (is_channel_probe(tmp)) {
    return;
  }
  RxItem item{};
  item.kind = RxKind::V1;
  item.v1.envelope = tmp;
  if (rx_queue_push(item)) {
    BEE_SERIAL.printf("ESP-NOW v1 rx id=%s metric=%u\n", tmp.device_id,
                      static_cast<unsigned>(tmp.metric));
  }
}

void handle_v2(const uint8_t* mac, const uint8_t* data, int len, int8_t rx_rssi_dbm = -127) {
  if (len != static_cast<int>(sizeof(ReportFrameV2))) {
    return;
  }
  ReportFrameV2 tmp{};
  memcpy(&tmp, data, sizeof(tmp));
  if (tmp.magic != kBeeplanMagicV2 || tmp.proto_version != kBeeplanProtoV2) {
    return;
  }
  // ACK сразу — edge ждёт ответ ~300 мс после отправки.
  send_ack_v2(tmp, mac);

  RxItem item{};
  item.kind = RxKind::V2;
  item.v2.frame = tmp;
  item.v2.rx_rssi_dbm = rx_rssi_dbm;
  memcpy(item.v2.src_mac, mac, 6);
  if (rx_queue_push(item)) {
    BEE_SERIAL.printf("ESP-NOW v2 rx id=%s seq=%u rssi=%d\n", tmp.device_id,
                      static_cast<unsigned>(tmp.seq), static_cast<int>(rx_rssi_dbm));
  }
}

void on_recv_legacy(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < 4) {
    return;
  }
  uint32_t magic = 0;
  memcpy(&magic, data, sizeof(magic));
  if (magic == kBeeplanMagicV2) {
    const int8_t rx_rssi = resolve_rx_rssi_dbm(mac, -127);
    handle_v2(mac, data, len, rx_rssi);
    return;
  }
  handle_v1(data, len);
}

#if BEEPLAN_ESPNOW_V3
void on_recv_v3(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* mac = info ? info->src_addr : nullptr;
  uint8_t zero[6] = {};
  int8_t callback_rssi = -127;
  if (info != nullptr && info->rx_ctrl != nullptr) {
    callback_rssi = info->rx_ctrl->rssi;
  }
  if (len >= 4) {
    uint32_t magic = 0;
    memcpy(&magic, data, sizeof(magic));
    if (magic == kBeeplanMagicV2) {
      const int8_t rx_rssi = resolve_rx_rssi_dbm(mac ? mac : zero, callback_rssi);
      handle_v2(mac ? mac : zero, data, len, rx_rssi);
      return;
    }
  }
  on_recv_legacy(mac ? mac : zero, data, len);
}
#endif

bool wifi_connect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < 60; ++i) {
    if (WiFi.status() == WL_CONNECTED) {
      return true;
    }
    delay(500);
  }
  return false;
}

void sync_time_utc() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  for (int i = 0; i < 100; ++i) {
    time_t now = time(nullptr);
    if (now > 1700000000) {
      return;
    }
    delay(200);
  }
}

String format_iso_utc(time_t t) {
  struct tm tm{};
  gmtime_r(&t, &tm);
  char buf[40];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return String(buf);
}

String decode_firmware_version(uint16_t major_minor, uint16_t patch) {
  const int maj = (major_minor >> 8) & 0xff;
  const int min = major_minor & 0xff;
  char buf[16];
  snprintf(buf, sizeof(buf), "%d.%d.%d", maj, min, patch & 0xffff);
  return String(buf);
}

/** battery_x100 ≤ 500 — напряжение (V×100); иначе legacy-процент×100. */
float decode_battery_volts(int16_t battery_x100) {
  if (battery_x100 <= 0) {
    return 0.0f;
  }
  if (battery_x100 <= 500) {
    return battery_x100 / 100.0f;
  }
  float pct = battery_x100 / 100.0f;
  if (pct > 100.0f) {
    pct = 100.0f;
  }
  return 3.30f + (pct / 100.0f) * 0.90f;
}

int8_t gateway_wifi_rssi_dbm() {
  if (WiFi.status() != WL_CONNECTED) {
    return -127;
  }
  int8_t rssi = static_cast<int8_t>(WiFi.RSSI());
  if (rssi >= 0) {
    return -127;
  }
  if (rssi < -120) {
    return -120;
  }
  return rssi;
}

#if defined(BEEPLAN_GATEWAY_BATTERY_ADC_PIN) && (BEEPLAN_GATEWAY_BATTERY_ADC_PIN >= 0)
constexpr int kGwBatAdcPin = BEEPLAN_GATEWAY_BATTERY_ADC_PIN;
constexpr int kGwBatSampleCount = 8;
/** Делитель 100k/100k (как на T-Energy). */
constexpr float kGwBatDividerRatio = 2.0f;
bool g_gw_bat_adc_ready = false;

void gateway_battery_adc_init() {
  if (g_gw_bat_adc_ready) {
    return;
  }
  analogSetPinAttenuation(kGwBatAdcPin, ADC_11db);
  pinMode(kGwBatAdcPin, INPUT);
  g_gw_bat_adc_ready = true;
}

bool gateway_battery_volts(float& out) {
  gateway_battery_adc_init();
  uint32_t sum_mv = 0;
  for (int i = 0; i < kGwBatSampleCount; ++i) {
    sum_mv += static_cast<uint32_t>(analogReadMilliVolts(kGwBatAdcPin));
    delay(1);
  }
  out = static_cast<float>(sum_mv) / static_cast<float>(kGwBatSampleCount) / 1000.0f;
  out *= kGwBatDividerRatio;
  if (out < 2.8f || out > 4.35f) {
    return false;
  }
  out = roundf(out * 100.0f) / 100.0f;
  return true;
}
#else
bool gateway_battery_volts(float& out) {
  (void)out;
  return false;
}
#endif

bool fill_gateway_batch_status(JsonObject gw) {
  bool any = false;
  const int8_t rssi = gateway_wifi_rssi_dbm();
  if (rssi_dbm_valid(rssi)) {
    gw["signal_dbm"] = rssi;
    any = true;
  }
  float volts = 0.0f;
  if (gateway_battery_volts(volts)) {
    gw["battery_volts"] = volts;
    any = true;
  }
  return any;
}

void append_gateway_batch_status(JsonDocument& batch) {
  JsonObject gw = batch["gateway"].to<JsonObject>();
  if (!fill_gateway_batch_status(gw)) {
    batch.remove("gateway");
  }
}

bool append_gateway_batch_to_body(String& body_out) {
  JsonDocument gw_doc;
  JsonObject gw = gw_doc.to<JsonObject>();
  if (!fill_gateway_batch_status(gw)) {
    return false;
  }
  body_out += ",\"gateway\":";
  String gw_frag;
  serializeJson(gw, gw_frag);
  body_out += gw_frag;
  return true;
}

bool spool_init() {
  if (!LittleFS.begin(true)) {
    BEE_SERIAL.println("WARN: LittleFS mount failed — spool disabled");
    return false;
  }
  if (!LittleFS.exists(kSpoolPath)) {
    File f = LittleFS.open(kSpoolPath, "w");
    if (f) {
      f.close();
    }
  }
  return true;
}

void spool_recount() {
  g_spool_pending_count = 0;
  if (!LittleFS.exists(kSpoolPath)) {
    return;
  }
  File f = LittleFS.open(kSpoolPath, "r");
  if (!f) {
    return;
  }
  while (f.available()) {
    if (f.readStringUntil('\n').length() > 2) {
      ++g_spool_pending_count;
    }
  }
  f.close();
}

size_t littlefs_free_bytes() {
  return LittleFS.totalBytes() - LittleFS.usedBytes();
}

size_t spool_file_bytes() {
  if (!LittleFS.exists(kSpoolPath)) {
    return 0;
  }
  File f = LittleFS.open(kSpoolPath, "r");
  if (!f) {
    return 0;
  }
  const size_t sz = f.size();
  f.close();
  return sz;
}

bool spool_has_compact_space() {
  return littlefs_free_bytes() > spool_file_bytes() + kSpoolCompactReserveBytes;
}

void spool_clear_all(const char* reason) {
  BEE_SERIAL.printf("WARN: spool cleared (%s), was %u lines\n", reason,
                    static_cast<unsigned>(g_spool_pending_count));
  LittleFS.remove(kSpoolPath);
  LittleFS.remove(kSpoolTmpPath);
  spool_init();
  g_spool_pending_count = 0;
}

bool spool_drop_head_lines(size_t lines_to_drop) {
  if (lines_to_drop == 0 || !LittleFS.exists(kSpoolPath)) {
    return true;
  }
  if (!spool_has_compact_space()) {
    spool_clear_all("LittleFS full");
    return true;
  }

  File in = LittleFS.open(kSpoolPath, "r");
  if (!in) {
    return false;
  }
  File out = LittleFS.open(kSpoolTmpPath, "w");
  if (!out) {
    in.close();
    spool_clear_all("spool tmp open failed");
    return true;
  }

  size_t dropped = 0;
  while (in.available()) {
    String line = in.readStringUntil('\n');
    line.trim();
    if (line.length() < 3) {
      continue;
    }
    if (dropped < lines_to_drop) {
      ++dropped;
      continue;
    }
    out.println(line);
  }
  in.close();
  out.flush();
  out.close();

  if (dropped == 0) {
    LittleFS.remove(kSpoolTmpPath);
    return false;
  }

  LittleFS.remove(kSpoolPath);
  if (!LittleFS.rename(kSpoolTmpPath, kSpoolPath)) {
    spool_clear_all("spool rename failed");
    return false;
  }
  spool_recount();
  return true;
}

void spool_trim_if_needed() {
  if (g_spool_pending_count <= kSpoolSoftMaxLines && littlefs_free_bytes() > kSpoolCompactReserveBytes) {
    return;
  }
  const size_t drop = g_spool_pending_count > 0 ? g_spool_pending_count / 2 : 0;
  if (drop == 0 || !spool_drop_head_lines(drop)) {
    spool_clear_all("spool pressure");
  } else {
    BEE_SERIAL.printf("WARN: spool trimmed to %u lines (free=%u B)\n",
                      static_cast<unsigned>(g_spool_pending_count),
                      static_cast<unsigned>(littlefs_free_bytes()));
  }
}

bool spool_append_line(const String& line) {
  if (g_spool_pending_count >= kSpoolSoftMaxLines || littlefs_free_bytes() < kSpoolCompactReserveBytes) {
    spool_trim_if_needed();
  }
  File f = LittleFS.open(kSpoolPath, "a");
  if (!f) {
    spool_trim_if_needed();
    f = LittleFS.open(kSpoolPath, "a");
    if (!f) {
      return false;
    }
  }
  f.println(line);
  f.flush();
  f.close();
  ++g_spool_pending_count;
  return true;
}

bool post_heartbeat() {
  WiFiClient client;
  HTTPClient http;
  String url = String(API_BASE_URL) + "/v1/concentrators/heartbeat";
  if (!http.begin(client, url)) {
    return false;
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", String("Bearer ") + INGEST_TOKEN);

  JsonDocument doc;
  doc["mac"] = WiFi.macAddress();
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["wifi_channel"] = WiFi.channel();
  doc["spool_pending_count"] = g_spool_pending_count;
  const int8_t rssi = gateway_wifi_rssi_dbm();
  if (rssi_dbm_valid(rssi)) {
    doc["signal_dbm"] = rssi;
  }
  String body;
  serializeJson(doc, body);

  const int code = http.POST(body);
  BEEPLAN_LOG("POST /v1/concentrators/heartbeat -> %d spool=%u ch=%d\n", code,
              static_cast<unsigned>(g_spool_pending_count), WiFi.channel());
  if (code >= 200 && code < 300) {
    JsonDocument resp;
    const String resp_body = http.getString();
    if (deserializeJson(resp, resp_body) == DeserializationError::Ok) {
      load_edge_wake_configs(resp);
      BEEPLAN_LOG("heartbeat edge configs=%u\n", static_cast<unsigned>(g_edge_wake_count));
    }
  }
  http.end();
  return code >= 200 && code < 300;
}

void send_heartbeat_with_retry() {
  for (int i = 0; i < 20; ++i) {
    if (post_heartbeat()) {
      return;
    }
    delay(3000);
  }
  BEE_SERIAL.println("heartbeat failed after retries");
}

bool post_batch_body(const String& body, size_t sample_count, JsonArray& accepted_ids) {
  if (sample_count == 0) {
    return true;
  }
  WiFiClient client;
  HTTPClient http;
  String url = String(API_BASE_URL) + "/v1/telemetry/batch";
  if (!http.begin(client, url)) {
    return false;
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", String("Bearer ") + INGEST_TOKEN);

  const int code = http.POST(body);
  BEE_SERIAL.printf("POST /v1/telemetry/batch -> %d samples=%u\n", code,
                    static_cast<unsigned>(sample_count));
  if (code < 200 || code >= 300) {
    BEE_SERIAL.println(http.getString());
    http.end();
    return false;
  }

  JsonDocument resp;
  const String resp_body = http.getString();
  http.end();
  if (deserializeJson(resp, resp_body) == DeserializationError::Ok &&
      resp["accepted_report_ids"].is<JsonArray>()) {
    for (JsonVariant v : resp["accepted_report_ids"].as<JsonArray>()) {
      accepted_ids.add(v.as<const char*>());
    }
  }
  return true;
}

bool post_batch(JsonDocument& batchRoot, JsonArray& accepted_ids) {
  if (!batchRoot["samples"].is<JsonArray>()) {
    batchRoot["samples"].to<JsonArray>();
  }
  JsonArray samples = batchRoot["samples"].as<JsonArray>();
  if (samples.size() == 0) {
    return true;
  }
  append_gateway_batch_status(batchRoot);
  String body;
  serializeJson(batchRoot, body);
  return post_batch_body(body, samples.size(), accepted_ids);
}

void append_v1_sample(JsonArray& samples, const EnvelopeV1& e) {
  time_t ts = static_cast<time_t>(e.unix_ts);
  if (ts < 1700000000) {
    ts = time(nullptr);
  }
  JsonObject s = samples.add<JsonObject>();
  s["device_public_id"] = String(e.device_id);
  s["ts"] = format_iso_utc(ts);
  switch (e.metric) {
    case 0:
      s["metric"] = "temperature_c";
      s["value"]["celsius"] = e.i16_a / 100.0f;
      break;
    case 1:
      s["metric"] = "relative_humidity";
      s["value"]["percent"] = e.i16_a / 100.0f;
      break;
    case 3:
      s["metric"] = "signal_level";
      s["value"]["dbm"] = e.i16_a;
      break;
    case 4:
      s["metric"] = "battery_voltage";
      s["value"]["volts"] = decode_battery_volts(e.i16_a);
      break;
    case 5:
      s["metric"] = "firmware_version";
      s["value"]["version"] = decode_firmware_version(static_cast<uint16_t>(e.i16_a),
                                                      static_cast<uint16_t>(e.i16_b));
      break;
    default:
      s["metric"] = "unknown";
      s["value"]["placeholder"] = e.i16_a;
      break;
  }
}

uint32_t next_gateway_report_seq() {
  g_prefs.begin(kPrefsNamespace, false);
  uint32_t seq = g_prefs.getUInt(kPrefsGwReportSeq, 0);
  ++seq;
  g_prefs.putUInt(kPrefsGwReportSeq, seq);
  g_prefs.end();
  return seq;
}

String make_report_id(const ReportFrameV2& frame) {
  char buf[128];
  const uint32_t ts = frame.unix_ts;
  if (ts >= kMinValidUnixTs) {
    snprintf(buf, sizeof(buf), "%s:%lu:%u", frame.device_id, static_cast<unsigned long>(ts),
             static_cast<unsigned>(frame.seq));
  } else if (ts > 0) {
    // Edge без NTP: unix_ts = NVS-счётчик отчётов на устройстве.
    snprintf(buf, sizeof(buf), "%s:nvs:%lu", frame.device_id, static_cast<unsigned long>(ts));
  } else {
    // Старая прошивка edge (unix_ts=0): монотонный счётчик gateway в NVS.
    const uint32_t gw_seq = next_gateway_report_seq();
    snprintf(buf, sizeof(buf), "%s:gw:%lu", frame.device_id, static_cast<unsigned long>(gw_seq));
  }
  return String(buf);
}

void append_v2_samples(JsonArray& samples, const ReportFrameV2& frame, const String& report_id,
                       int8_t rx_rssi_dbm = -127) {
  time_t ts = static_cast<time_t>(frame.unix_ts);
  if (ts < static_cast<time_t>(kMinValidUnixTs)) {
    ts = time(nullptr);
  }
  const String iso = format_iso_utc(ts);
  const String device_id = String(frame.device_id);

  if (frame.metrics_present & kMetricTemp) {
    JsonObject s = samples.add<JsonObject>();
    s["device_public_id"] = device_id;
    s["metric"] = "temperature_c";
    s["ts"] = iso;
    s["report_id"] = report_id;
    s["value"]["celsius"] = frame.temp_c_x100 / 100.0f;
  }
  if (frame.metrics_present & kMetricRh) {
    JsonObject s = samples.add<JsonObject>();
    s["device_public_id"] = device_id;
    s["metric"] = "relative_humidity";
    s["ts"] = iso;
    s["report_id"] = report_id;
    s["value"]["percent"] = frame.rh_x100 / 100.0f;
  }
  if (frame.metrics_present & kMetricSignal) {
    JsonObject s = samples.add<JsonObject>();
    s["device_public_id"] = device_id;
    s["metric"] = "signal_level";
    s["ts"] = iso;
    s["report_id"] = report_id;
    const int signal_dbm =
        rssi_dbm_valid(rx_rssi_dbm) ? rx_rssi_dbm
                                    : (rssi_dbm_valid(static_cast<int8_t>(frame.signal_dbm)) ? frame.signal_dbm
                                                                                             : rx_rssi_dbm);
    s["value"]["dbm"] = signal_dbm;
  }
  if (frame.metrics_present & kMetricBattery) {
    JsonObject s = samples.add<JsonObject>();
    s["device_public_id"] = device_id;
    s["metric"] = "battery_voltage";
    s["ts"] = iso;
    s["report_id"] = report_id;
    s["value"]["volts"] = decode_battery_volts(frame.battery_x100);
  }
  if (frame.metrics_present & kMetricFirmware) {
    JsonObject s = samples.add<JsonObject>();
    s["device_public_id"] = device_id;
    s["metric"] = "firmware_version";
    s["ts"] = iso;
    s["report_id"] = report_id;
    s["value"]["version"] = decode_firmware_version(frame.fw_major_minor, frame.fw_patch);
  }
}

String build_spool_line(const ReportFrameV2& frame, int8_t rx_rssi_dbm = -127) {
  JsonDocument doc;
  doc["report_id"] = make_report_id(frame);
  JsonArray samples = doc["samples"].to<JsonArray>();
  append_v2_samples(samples, frame, doc["report_id"].as<const char*>(), rx_rssi_dbm);
  String line;
  serializeJson(doc, line);
  return line;
}

void process_v2_item(const RxV2Item& item) {
  const String spool_line = build_spool_line(item.frame, item.rx_rssi_dbm);
  if (!spool_append_line(spool_line)) {
    BEE_SERIAL.println("WARN: spool append failed");
  }
}

void flush_ram_queue(JsonDocument& batch) {
  JsonArray samples = batch["samples"].to<JsonArray>();
  RxItem item{};
  while (rx_queue_pop(item)) {
    if (item.kind == RxKind::V1) {
      append_v1_sample(samples, item.v1.envelope);
    } else {
      append_v2_samples(samples, item.v2.frame, make_report_id(item.v2.frame), item.v2.rx_rssi_dbm);
    }
  }
}

bool drop_first_spool_line() {
  return spool_drop_head_lines(1);
}

bool read_first_spool_line(String& line_out) {
  line_out = "";
  if (!LittleFS.exists(kSpoolPath)) {
    return false;
  }
  File f = LittleFS.open(kSpoolPath, "r");
  if (!f) {
    return false;
  }
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() < 3) {
      continue;
    }
    line_out = line;
    break;
  }
  f.close();
  return line_out.length() >= 3;
}

void drop_corrupt_spool_head() {
  for (;;) {
    String line;
    if (!read_first_spool_line(line)) {
      return;
    }
    JsonDocument doc;
    if (deserializeJson(doc, line) == DeserializationError::Ok &&
        doc["samples"].is<JsonArray>() && doc["samples"].as<JsonArray>().size() > 0) {
      return;
    }
    BEE_SERIAL.println("WARN: drop corrupt spool head");
    drop_first_spool_line();
  }
}

/** Собирает один JSON batch из RAM-очереди и строк spool (без merge JsonDocument). */
bool build_uplink_body(String& body_out, JsonArray& report_ids_out, JsonDocument& ram_batch,
                       size_t& spool_lines_out, size_t& sample_count_out) {
  spool_lines_out = 0;
  sample_count_out = 0;
  body_out = "{\"samples\":[";
  bool first_sample = true;

  if (ram_batch["samples"].is<JsonArray>()) {
    for (JsonObject s : ram_batch["samples"].as<JsonArray>()) {
      if (!first_sample) {
        body_out += ',';
      }
      String frag;
      serializeJson(s, frag);
      body_out += frag;
      first_sample = false;
      ++sample_count_out;
    }
  }

  drop_corrupt_spool_head();
  if (!LittleFS.exists(kSpoolPath)) {
    if (sample_count_out == 0) {
      return false;
    }
    body_out += ']';
    append_gateway_batch_to_body(body_out);
    body_out += '}';
    return true;
  }
  File f = LittleFS.open(kSpoolPath, "r");
  if (!f) {
    if (sample_count_out == 0) {
      return false;
    }
    body_out += ']';
    append_gateway_batch_to_body(body_out);
    body_out += '}';
    return true;
  }

  while (f.available() && spool_lines_out < kMaxSpoolLinesPerBatch &&
         sample_count_out < kMaxSamplesPerBatch) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() < 3) {
      continue;
    }

    JsonDocument line_doc;
    const DeserializationError err = deserializeJson(line_doc, line);
    if (err) {
      BEE_SERIAL.printf("WARN: spool line parse: %s\n", err.c_str());
      break;
    }
    JsonArray line_samples = line_doc["samples"].as<JsonArray>();
    if (line_samples.size() == 0) {
      continue;
    }

    const char* rid = line_doc["report_id"] | "";
    if (rid[0] != '\0') {
      report_ids_out.add(rid);
    }
    for (JsonObject s : line_samples) {
      if (sample_count_out >= kMaxSamplesPerBatch) {
        break;
      }
      if (!first_sample) {
        body_out += ',';
      }
      String frag;
      serializeJson(s, frag);
      body_out += frag;
      first_sample = false;
      ++sample_count_out;
    }
    ++spool_lines_out;
  }
  f.close();
  if (sample_count_out == 0) {
    return false;
  }
  body_out += ']';
  append_gateway_batch_to_body(body_out);
  body_out += '}';
  return true;
}

void try_uplink() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  JsonDocument ram_batch;
  flush_ram_queue(ram_batch);

  JsonDocument report_ids_doc;
  JsonArray pending_report_ids = report_ids_doc.to<JsonArray>();

  String body;
  size_t spool_lines = 0;
  size_t sample_count = 0;
  if (!build_uplink_body(body, pending_report_ids, ram_batch, spool_lines, sample_count)) {
    g_drain_backoff_ms = kDrainIntervalMs;
    return;
  }

  JsonDocument accepted_doc;
  JsonArray accepted_ids = accepted_doc.to<JsonArray>();
  if (!post_batch_body(body, sample_count, accepted_ids)) {
    g_drain_backoff_ms = min<uint32_t>(g_drain_backoff_ms * 2, 15U * 60U * 1000U);
    return;
  }

  if (spool_lines > 0) {
    if (!spool_drop_head_lines(spool_lines)) {
      BEE_SERIAL.println("WARN: spool_drop_head failed");
    }
  }

  BEE_SERIAL.printf("drain ok: spool_lines=%u samples=%u pending=%u free=%u B\n",
                    static_cast<unsigned>(spool_lines), static_cast<unsigned>(sample_count),
                    static_cast<unsigned>(g_spool_pending_count),
                    static_cast<unsigned>(littlefs_free_bytes()));
  g_drain_backoff_ms = kDrainIntervalMs;
}

}  // namespace

void setup() {
  beeplan_led_init();
  beeplan_serial_begin();
  BEE_SERIAL.printf("BeePlan gateway %s\n", FIRMWARE_SERIAL_TAG);
  BEE_SERIAL.printf("sizeof(ReportFrameV2)=%u queue=%u\n",
                    static_cast<unsigned>(sizeof(ReportFrameV2)),
                    static_cast<unsigned>(kRxQueueCap));

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (!wifi_connect()) {
    BEE_SERIAL.println("WiFi failed — reboot after fixing credentials");
    return;
  }

  BEE_SERIAL.println(WiFi.localIP());
  BEE_SERIAL.printf("WiFi channel=%d (ESP-NOW uses this channel)\n", WiFi.channel());

  spool_init();
  spool_recount();
  if (g_spool_pending_count > kSpoolSoftMaxLines || !spool_has_compact_space()) {
    spool_trim_if_needed();
  }

  if (esp_now_init() != ESP_OK) {
    BEE_SERIAL.println("esp_now_init failed — halted");
    while (true) {
      beeplan_led_toggle();
      delay(150);
    }
  }
#if BEEPLAN_ESPNOW_V3
  if (!beeplan_register_recv_cb(on_recv_v3)) {
    BEE_SERIAL.println("recv_cb register failed — halted");
    while (true) {
      beeplan_led_toggle();
      delay(150);
    }
  }
#else
  if (!beeplan_register_recv_cb(on_recv_legacy)) {
    BEE_SERIAL.println("recv_cb register failed — halted");
    while (true) {
      beeplan_led_toggle();
      delay(150);
    }
  }
#endif
  if (rssi_promiscuous_begin()) {
    BEE_SERIAL.println("RSSI promiscuous mode on");
  } else {
    BEE_SERIAL.println("WARN: RSSI promiscuous mode failed");
  }

  randomSeed(esp_random());
  sync_time_utc();
  send_heartbeat_with_retry();
  g_last_heartbeat_ms = millis();
  g_last_drain_ms = millis();
}

void loop() {
  RxItem item{};
  while (rx_queue_pop(item)) {
    if (item.kind == RxKind::V2) {
      process_v2_item(item.v2);
    } else {
      JsonDocument batch;
      JsonArray samples = batch["samples"].to<JsonArray>();
      append_v1_sample(samples, item.v1.envelope);
      JsonDocument accepted;
      JsonArray accepted_ids = accepted.to<JsonArray>();
      if (WiFi.status() == WL_CONNECTED) {
        post_batch(batch, accepted_ids);
      }
    }
  }

  const uint32_t now_ms = millis();
  if (now_ms - g_last_heartbeat_ms >= kHeartbeatIntervalMs) {
    post_heartbeat();
    g_last_heartbeat_ms = now_ms;
  }

  if (now_ms - g_last_drain_ms >= g_drain_backoff_ms) {
    try_uplink();
    g_last_drain_ms = now_ms;
  }

  delay(50);
}
