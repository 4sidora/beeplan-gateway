/**
 * BeePlan gateway — приём ESP-NOW, батч в REST API по Wi-Fi.
 */
#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_now.h>
#include <time.h>

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

volatile bool g_have_packet = false;
Envelope g_last{};

void on_recv(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
  if (len != static_cast<int>(sizeof(Envelope))) {
    return;
  }
  Envelope tmp;
  memcpy(&tmp, data, sizeof(Envelope));
  if (tmp.magic != kMagic) {
    return;
  }
  g_last = tmp;
  g_have_packet = true;
}

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
    Serial.println("heartbeat: http.begin failed");
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
  Serial.printf("POST /v1/concentrators/heartbeat -> %d mac=%s\n", code, WiFi.macAddress().c_str());
  if (code < 200 || code >= 300) {
    Serial.println(http.getString());
    http.end();
    return false;
  }
  http.end();
  return true;
}

void send_heartbeat_with_retry() {
  Serial.printf("Gateway MAC: %s\n", WiFi.macAddress().c_str());
  for (int i = 0; i < 20; ++i) {
    if (post_heartbeat()) {
      return;
    }
    delay(3000);
  }
  Serial.println("heartbeat failed after retries");
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
    Serial.println("http.begin failed");
    return false;
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", String("Bearer ") + INGEST_TOKEN);

  String body;
  serializeJson(batchRoot, body);

  int code = http.POST(body);
  Serial.printf("POST /v1/telemetry/batch -> %d\n", code);
  if (code < 200 || code >= 300) {
    Serial.println(http.getString());
    http.end();
    return false;
  }
  http.end();
  return true;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("BeePlan gateway");

  // ESP-NOW requires Wi-Fi stack to be up before esp_now_init().
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init failed");
    abort();
  }
  esp_now_register_recv_cb(on_recv);

  if (!wifi_connect()) {
    Serial.println("WiFi failed — uplink disabled until reboot");
  } else {
    Serial.println(WiFi.localIP());
    sync_time_utc();
    send_heartbeat_with_retry();
  }
}

void loop() {
  static JsonDocument batch;
  static JsonArray samples = batch["samples"].to<JsonArray>();
  static uint32_t last_flush = 0;

  if (g_have_packet) {
    g_have_packet = false;
    Envelope e = g_last;

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

  uint32_t now_ms = millis();
  bool due = (samples.size() >= 8) || (samples.size() > 0 && (now_ms - last_flush) > 30000U);

  if (WiFi.status() == WL_CONNECTED && due) {
    if (post_batch(batch)) {
      batch.clear();
      samples = batch["samples"].to<JsonArray>();
      last_flush = now_ms;
    }
  }

  delay(50);
}
