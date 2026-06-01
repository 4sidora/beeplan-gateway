/**
 * BeePlan gateway — приём ESP-NOW, батч в REST API по Wi-Fi.
 */
#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_now.h>
#include <time.h>

#include "beeplan_io.h"
#include "beeplan_espnow.h"
#include "config.h"

namespace {

constexpr uint32_t kMagic = 0x00BEEF01;

struct __attribute__((packed)) Envelope {
  uint32_t magic;
  uint8_t proto_version;
  char device_id[32];
  uint32_t unix_ts;
  uint8_t metric;
  int16_t i16_a;
  int16_t i16_b;
};

constexpr size_t kRxQueueCap = 32;
Envelope g_rx_queue[kRxQueueCap];
volatile uint16_t g_rx_head = 0;
volatile uint16_t g_rx_tail = 0;
portMUX_TYPE g_rx_mux = portMUX_INITIALIZER_UNLOCKED;

bool is_channel_probe(const Envelope& e) {
  return e.unix_ts == 0 && e.metric == 0 && e.i16_a == 0 && e.i16_b == 0;
}

bool rx_queue_push(const Envelope& e) {
  portENTER_CRITICAL(&g_rx_mux);
  const uint16_t next = static_cast<uint16_t>((g_rx_head + 1) % kRxQueueCap);
  if (next == g_rx_tail) {
    portEXIT_CRITICAL(&g_rx_mux);
    BEE_SERIAL.println("ESP-NOW rx queue full — dropped packet");
    return false;
  }
  g_rx_queue[g_rx_head] = e;
  g_rx_head = next;
  portEXIT_CRITICAL(&g_rx_mux);
  return true;
}

bool rx_queue_pop(Envelope& out) {
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

void on_recv_legacy(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
  if (len != static_cast<int>(sizeof(Envelope))) {
    BEE_SERIAL.printf("ESP-NOW bad len=%d want=%u\n", len, static_cast<unsigned>(sizeof(Envelope)));
    return;
  }
  Envelope tmp;
  memcpy(&tmp, data, sizeof(Envelope));
  if (tmp.magic != kMagic) {
    BEE_SERIAL.println("ESP-NOW bad magic");
    return;
  }
  if (is_channel_probe(tmp)) {
    return;
  }
  if (rx_queue_push(tmp)) {
    BEE_SERIAL.printf("ESP-NOW rx id=%s metric=%u\n", tmp.device_id, static_cast<unsigned>(tmp.metric));
  }
}

#if BEEPLAN_ESPNOW_V3
void on_recv_v3(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  (void)info;
  on_recv_legacy(nullptr, data, len);
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
  struct tm tm;
  gmtime_r(&t, &tm);
  char buf[40];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return String(buf);
}

const char* metric_name(uint8_t m) {
  switch (m) {
    case 0:
      return "temperature_c";
    case 1:
      return "relative_humidity";
    case 2:
      return "audio_features";
    default:
      return "unknown";
  }
}

bool post_heartbeat() {
  WiFiClient client;
  HTTPClient http;
  String url = String(API_BASE_URL) + "/v1/concentrators/heartbeat";
  if (!http.begin(client, url)) {
    BEE_SERIAL.println("heartbeat: http.begin failed");
    return false;
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", String("Bearer ") + INGEST_TOKEN);

  JsonDocument doc;
  doc["mac"] = WiFi.macAddress();
  doc["firmware_version"] = FIRMWARE_VERSION;
  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  BEE_SERIAL.printf("POST /v1/concentrators/heartbeat -> %d mac=%s\n", code, WiFi.macAddress().c_str());
  if (code < 200 || code >= 300) {
    BEE_SERIAL.println(http.getString());
    http.end();
    return false;
  }
  http.end();
  return true;
}

void send_heartbeat_with_retry() {
  BEE_SERIAL.printf("Gateway MAC: %s\n", WiFi.macAddress().c_str());
  for (int i = 0; i < 20; ++i) {
    if (post_heartbeat()) {
      return;
    }
    delay(3000);
  }
  BEE_SERIAL.println("heartbeat failed after retries");
}

bool post_batch(JsonDocument& batchRoot) {
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
    BEE_SERIAL.println("http.begin failed");
    return false;
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", String("Bearer ") + INGEST_TOKEN);

  String body;
  serializeJson(batchRoot, body);

  int code = http.POST(body);
  BEE_SERIAL.printf("POST /v1/telemetry/batch -> %d\n", code);
  if (code < 200 || code >= 300) {
    BEE_SERIAL.println(http.getString());
    http.end();
    return false;
  }
  http.end();
  return true;
}

void append_sample(JsonArray& samples, const Envelope& e) {
  time_t ts = static_cast<time_t>(e.unix_ts);
  if (ts < 1700000000) {
    ts = time(nullptr);
  }

  JsonObject s = samples.add<JsonObject>();
  s["device_public_id"] = String(e.device_id);
  s["metric"] = metric_name(e.metric);
  s["ts"] = format_iso_utc(ts);
  if (e.metric == 0) {
    JsonObject v = s["value"].to<JsonObject>();
    v["celsius"] = e.i16_a / 100.0f;
  } else if (e.metric == 1) {
    JsonObject v = s["value"].to<JsonObject>();
    v["percent"] = e.i16_a / 100.0f;
  } else {
    JsonObject v = s["value"].to<JsonObject>();
    v["placeholder"] = e.i16_a;
  }
}

}  // namespace

void setup() {
  beeplan_led_init();
  beeplan_serial_begin();
  BEE_SERIAL.printf("BeePlan gateway %s\n", FIRMWARE_SERIAL_TAG);
#if BEEPLAN_ESPNOW_V3
  BEE_SERIAL.println("ESP-NOW callbacks: IDF5/v3");
#else
  BEE_SERIAL.println("ESP-NOW callbacks: legacy");
#endif
  BEE_SERIAL.printf("sizeof(Envelope)=%u\n", static_cast<unsigned>(sizeof(Envelope)));

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (!wifi_connect()) {
    BEE_SERIAL.println("WiFi failed — reboot after fixing credentials");
    return;
  }

  BEE_SERIAL.println(WiFi.localIP());
  BEE_SERIAL.printf("WiFi channel=%d (ESP-NOW uses this channel)\n", WiFi.channel());

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

  sync_time_utc();
  send_heartbeat_with_retry();
}

void loop() {
  static JsonDocument batch;
  static JsonArray samples = batch["samples"].to<JsonArray>();
  static uint32_t last_flush = 0;

  Envelope e{};
  while (rx_queue_pop(e)) {
    append_sample(samples, e);
  }

  uint32_t now_ms = millis();
  bool due = (samples.size() >= 2) || (samples.size() > 0 && (now_ms - last_flush) > 5000U);

  if (WiFi.status() == WL_CONNECTED && due) {
    if (post_batch(batch)) {
      batch.clear();
      samples = batch["samples"].to<JsonArray>();
      last_flush = now_ms;
    }
  }

  delay(50);
}
