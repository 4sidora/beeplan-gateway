/**
 * BeePlan gateway — ESP-NOW v1/v2 RX, flash spool, REST uplink.
 */
#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <cstring>
#include <esp_now.h>
#include <esp_random.h>
#include <time.h>

#include "beeplan_io.h"
#include "beeplan_espnow.h"
#include "config.h"
#include "envelope_v2.h"

namespace {

constexpr size_t kRxQueueCap = 512;
constexpr uint32_t kHeartbeatIntervalMs = 15U * 60U * 1000U;
constexpr uint32_t kDrainIntervalMs = 30U * 1000U;
constexpr char kSpoolPath[] = "/beeplan_spool.jsonl";

struct RxV1Item {
  EnvelopeV1 envelope;
};

struct RxV2Item {
  ReportFrameV2 frame;
  uint8_t src_mac[6];
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

void handle_v2(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != static_cast<int>(sizeof(ReportFrameV2))) {
    return;
  }
  ReportFrameV2 tmp{};
  memcpy(&tmp, data, sizeof(tmp));
  if (tmp.magic != kBeeplanMagicV2 || tmp.proto_version != kBeeplanProtoV2) {
    return;
  }
  RxItem item{};
  item.kind = RxKind::V2;
  item.v2.frame = tmp;
  memcpy(item.v2.src_mac, mac, 6);
  if (rx_queue_push(item)) {
    BEE_SERIAL.printf("ESP-NOW v2 rx id=%s seq=%u\n", tmp.device_id, static_cast<unsigned>(tmp.seq));
  }
}

void on_recv_legacy(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < 4) {
    return;
  }
  uint32_t magic = 0;
  memcpy(&magic, data, sizeof(magic));
  if (magic == kBeeplanMagicV2) {
    handle_v2(mac, data, len);
    return;
  }
  handle_v1(data, len);
}

#if BEEPLAN_ESPNOW_V3
void on_recv_v3(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* mac = info ? info->src_addr : nullptr;
  uint8_t zero[6] = {};
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

float gateway_battery_demo_percent() {
  static float pct = 91.0f;
  pct += static_cast<float>(random(-8, 9)) / 10.0f;
  if (pct < 10.0f) {
    pct = 98.0f;
  }
  if (pct > 100.0f) {
    pct = 100.0f;
  }
  return pct;
}

bool spool_init() {
  if (LittleFS.begin(true)) {
    return true;
  }
  BEE_SERIAL.println("WARN: LittleFS mount failed — spool disabled");
  return false;
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

bool spool_append_line(const String& line) {
  File f = LittleFS.open(kSpoolPath, "a");
  if (!f) {
    return false;
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
  if (WiFi.status() == WL_CONNECTED) {
    doc["signal_dbm"] = WiFi.RSSI();
  }
  doc["battery_percent"] = gateway_battery_demo_percent();
  String body;
  serializeJson(doc, body);

  const int code = http.POST(body);
  BEE_SERIAL.printf("POST /v1/concentrators/heartbeat -> %d spool=%u ch=%d\n", code,
                    static_cast<unsigned>(g_spool_pending_count), WiFi.channel());
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

bool post_batch(JsonDocument& batchRoot, JsonArray& accepted_ids) {
  if (!batchRoot["samples"].is<JsonArray>()) {
    return true;
  }
  JsonArray samples = batchRoot["samples"].as<JsonArray>();
  if (samples.size() == 0) {
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

  String body;
  serializeJson(batchRoot, body);

  const int code = http.POST(body);
  BEE_SERIAL.printf("POST /v1/telemetry/batch -> %d samples=%u\n", code,
                    static_cast<unsigned>(samples.size()));
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
      s["metric"] = "battery_percent";
      s["value"]["percent"] = e.i16_a / 100.0f;
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

String make_report_id(const ReportFrameV2& frame) {
  char buf[96];
  snprintf(buf, sizeof(buf), "%s:%lu:%u", frame.device_id,
           static_cast<unsigned long>(frame.unix_ts), static_cast<unsigned>(frame.seq));
  return String(buf);
}

void append_v2_samples(JsonArray& samples, const ReportFrameV2& frame, const String& report_id) {
  time_t ts = static_cast<time_t>(frame.unix_ts);
  if (ts < 1700000000) {
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
    s["value"]["dbm"] = frame.signal_dbm;
  }
  if (frame.metrics_present & kMetricBattery) {
    JsonObject s = samples.add<JsonObject>();
    s["device_public_id"] = device_id;
    s["metric"] = "battery_percent";
    s["ts"] = iso;
    s["report_id"] = report_id;
    s["value"]["percent"] = frame.battery_x100 / 100.0f;
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

String build_spool_line(const ReportFrameV2& frame) {
  JsonDocument doc;
  doc["report_id"] = make_report_id(frame);
  JsonArray samples = doc["samples"].to<JsonArray>();
  append_v2_samples(samples, frame, doc["report_id"].as<const char*>());
  String line;
  serializeJson(doc, line);
  return line;
}

void process_v2_item(const RxV2Item& item) {
  const String spool_line = build_spool_line(item.frame);
  if (!spool_append_line(spool_line)) {
    BEE_SERIAL.println("WARN: spool append failed");
  }
  send_ack_v2(item.frame, item.src_mac);
}

void flush_ram_queue(JsonDocument& batch) {
  JsonArray samples = batch["samples"].to<JsonArray>();
  RxItem item{};
  while (rx_queue_pop(item)) {
    if (item.kind == RxKind::V1) {
      append_v1_sample(samples, item.v1.envelope);
    } else {
      append_v2_samples(samples, item.v2.frame, make_report_id(item.v2.frame));
    }
  }
}

bool remove_accepted_from_spool(const JsonArray& accepted_ids) {
  if (!LittleFS.exists(kSpoolPath)) {
    return true;
  }
  File in = LittleFS.open(kSpoolPath, "r");
  if (!in) {
    return false;
  }
  File out = LittleFS.open("/beeplan_spool.tmp", "w");
  if (!out) {
    in.close();
    return false;
  }

  while (in.available()) {
    String line = in.readStringUntil('\n');
    line.trim();
    if (line.length() < 3) {
      continue;
    }
    JsonDocument doc;
    if (deserializeJson(doc, line) != DeserializationError::Ok) {
      out.println(line);
      continue;
    }
    const char* rid = doc["report_id"] | "";
    bool drop = false;
    for (JsonVariant v : accepted_ids) {
      if (v.as<String>() == rid) {
        drop = true;
        break;
      }
    }
    if (!drop) {
      out.println(line);
    }
  }
  in.close();
  out.close();
  LittleFS.remove(kSpoolPath);
  LittleFS.rename("/beeplan_spool.tmp", kSpoolPath);
  spool_recount();
  return true;
}

void drain_spool(JsonDocument& batch, JsonArray& report_ids_out) {
  if (!LittleFS.exists(kSpoolPath)) {
    return;
  }
  File f = LittleFS.open(kSpoolPath, "r");
  if (!f) {
    return;
  }
  JsonArray samples = batch["samples"].to<JsonArray>();
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() < 3) {
      continue;
    }
    JsonDocument doc;
    if (deserializeJson(doc, line) != DeserializationError::Ok) {
      continue;
    }
    const char* rid = doc["report_id"] | "";
    if (rid[0] != '\0') {
      report_ids_out.add(rid);
    }
    if (!doc["samples"].is<JsonArray>()) {
      continue;
    }
    for (JsonObject s : doc["samples"].as<JsonArray>()) {
      samples.add(s);
    }
  }
  f.close();
}

void try_uplink() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  JsonDocument batch;
  JsonDocument sent_ids_doc;
  JsonArray sent_report_ids = sent_ids_doc.to<JsonArray>();
  flush_ram_queue(batch);
  drain_spool(batch, sent_report_ids);

  JsonArray samples = batch["samples"].to<JsonArray>();
  if (samples.size() == 0) {
    g_drain_backoff_ms = kDrainIntervalMs;
    return;
  }

  JsonDocument accepted_doc;
  JsonArray accepted_ids = accepted_doc.to<JsonArray>();
  if (post_batch(batch, accepted_ids)) {
    if (accepted_ids.size() == 0 && sent_report_ids.size() > 0) {
      remove_accepted_from_spool(sent_report_ids);
    } else if (accepted_ids.size() > 0) {
      remove_accepted_from_spool(accepted_ids);
    }
    g_drain_backoff_ms = kDrainIntervalMs;
  } else {
    g_drain_backoff_ms = min<uint32_t>(g_drain_backoff_ms * 2, 15U * 60U * 1000U);
  }
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
