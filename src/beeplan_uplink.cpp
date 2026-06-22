#include "beeplan_uplink.h"

#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <sys/time.h>

#include "beeplan_io.h"
#include "config.h"

#ifndef UPLINK_MODE_WIFI
#define UPLINK_MODE_WIFI 0
#endif
#ifndef UPLINK_MODE_CELLULAR
#define UPLINK_MODE_CELLULAR 1
#endif

#if UPLINK_MODE == UPLINK_MODE_CELLULAR && defined(BEEPLAN_BOARD_TTGO_T_CALL)

#ifndef MODEM_TX
#define MODEM_TX 27
#endif
#ifndef MODEM_RX
#define MODEM_RX 26
#endif
#ifndef MODEM_PWRKEY
#define MODEM_PWRKEY 4
#endif
#ifndef MODEM_RST
#define MODEM_RST 5
#endif
#ifndef MODEM_POWER_ON
#define MODEM_POWER_ON 23
#endif

#define TINY_GSM_MODEM_SIM800
#include <HttpClient.h>
#include <TinyGsmClient.h>

HardwareSerial g_modem_serial(1);
TinyGsm g_modem(g_modem_serial);
bool g_cellular_ready = false;

#endif  // cellular board

namespace {

#if UPLINK_MODE == UPLINK_MODE_CELLULAR && defined(BEEPLAN_BOARD_TTGO_T_CALL)

void modem_power_on() {
  pinMode(MODEM_POWER_ON, OUTPUT);
  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  digitalWrite(MODEM_POWER_ON, HIGH);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(300);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(1000);
  digitalWrite(MODEM_PWRKEY, HIGH);
}

bool cellular_connect() {
  modem_power_on();
  g_modem_serial.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1000);

  BEE_SERIAL.println("Modem restart…");
  if (!g_modem.restart()) {
    BEE_SERIAL.println("Modem restart failed");
    return false;
  }

  BEE_SERIAL.println("Waiting for network…");
  if (!g_modem.waitForNetwork(120000L)) {
    BEE_SERIAL.println("Network registration failed");
    return false;
  }

  BEE_SERIAL.printf("Connecting GPRS APN=%s\n", CELLULAR_APN);
  const char* user = (CELLULAR_USER[0] != '\0') ? CELLULAR_USER : nullptr;
  const char* pass = (CELLULAR_PASS[0] != '\0') ? CELLULAR_PASS : nullptr;
  if (!g_modem.gprsConnect(CELLULAR_APN, user, pass)) {
    BEE_SERIAL.println("GPRS connect failed");
    return false;
  }

  BEE_SERIAL.println("GPRS connected");
  return true;
}

bool cellular_ensure_gprs() {
  if (g_modem.isGprsConnected()) {
    return true;
  }
  BEE_SERIAL.println("GPRS reconnect…");
  const char* user = (CELLULAR_USER[0] != '\0') ? CELLULAR_USER : nullptr;
  const char* pass = (CELLULAR_PASS[0] != '\0') ? CELLULAR_PASS : nullptr;
  return g_modem.gprsConnect(CELLULAR_APN, user, pass);
}

int8_t csq_to_dbm(int8_t csq) {
  if (csq <= 0 || csq == 99) {
    return -127;
  }
  return static_cast<int8_t>(-113 + 2 * csq);
}

bool parse_http_url(const String& url, String& host, uint16_t& port, String& path) {
  if (url.startsWith("https://")) {
    return false;
  }
  if (!url.startsWith("http://")) {
    return false;
  }
  const String rest = url.substring(7);
  const int slash = rest.indexOf('/');
  const String authority = (slash >= 0) ? rest.substring(0, slash) : rest;
  path = (slash >= 0) ? rest.substring(slash) : String("/");
  const int colon = authority.indexOf(':');
  if (colon >= 0) {
    host = authority.substring(0, colon);
    port = static_cast<uint16_t>(authority.substring(colon + 1).toInt());
    if (port == 0) {
      port = 80;
    }
  } else {
    host = authority;
    port = 80;
  }
  return host.length() > 0;
}

bool cellular_http_post(const String& url, const String& authorization_header, const String& body,
                        int& status_code, String& response_body, uint32_t timeout_ms) {
  if (url.startsWith("https://")) {
    BEE_SERIAL.println("cellular: https not supported — use http://api.beeplan.tech in firmware master");
    status_code = 0;
    response_body = "";
    return false;
  }
  if (!cellular_ensure_gprs()) {
    status_code = -1;
    response_body = "";
    return false;
  }

  String host;
  String path;
  uint16_t port = 80;
  if (!parse_http_url(url, host, port, path)) {
    status_code = -1;
    response_body = "";
    BEE_SERIAL.println("cellular HTTP: invalid url");
    return false;
  }

  TinyGsmClient client(g_modem);
  const uint32_t timeout = timeout_ms > 0 ? timeout_ms : 180000U;
  client.setTimeout(timeout / 4);
  HttpClient http(client, host.c_str(), port);
  http.setHttpResponseTimeout(timeout);

  const unsigned long t0 = millis();
  BEE_SERIAL.printf("cellular HTTP POST %u B -> %s\n", static_cast<unsigned>(body.length()),
                    url.c_str());

  http.beginRequest();
  http.post(path);
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Authorization", authorization_header);
  http.sendHeader("Connection", "close");
  http.sendHeader("Content-Length", body.length());
  http.beginBody();
  const char* data = body.c_str();
  size_t offset = 0;
  while (offset < body.length()) {
    const size_t chunk = min(static_cast<size_t>(128), body.length() - offset);
    http.write(reinterpret_cast<const uint8_t*>(data + offset), chunk);
    offset += chunk;
    delay(0);
  }
  http.endRequest();

  status_code = http.responseStatusCode();
  response_body = http.responseBody();
  client.stop();

  BEE_SERIAL.printf("cellular HTTP done %lu ms code=%d\n",
                    static_cast<unsigned long>(millis() - t0), status_code);

  if (status_code < 0) {
    BEE_SERIAL.printf("cellular HTTP error: %d url=%s\n", status_code, url.c_str());
  } else {
    BEEPLAN_LOG("cellular HTTP %d bytes=%u\n", status_code,
                static_cast<unsigned>(response_body.length()));
  }
  return status_code >= 200 && status_code < 300;
}

void cellular_sync_time() {
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  int second = 0;
  float timezone = 0;
  if (!g_modem.getNetworkTime(&year, &month, &day, &hour, &minute, &second, &timezone)) {
    BEE_SERIAL.println("cellular: network time unavailable");
    return;
  }
  struct tm tm_local{};
  tm_local.tm_year = year - 1900;
  tm_local.tm_mon = month - 1;
  tm_local.tm_mday = day;
  tm_local.tm_hour = hour;
  tm_local.tm_min = minute;
  tm_local.tm_sec = second;
  // TinyGSM: timezone уже в часах (поле ±zz из +CCLK — четверти часа, делится на 4 в библиотеке).
  const time_t wall = mktime(&tm_local);
  const int offset_sec = static_cast<int>(lroundf(timezone * 3600.0f));
  const time_t epoch = wall - offset_sec;
  if (epoch > 1700000000) {
    timeval tv{.tv_sec = epoch, .tv_usec = 0};
    settimeofday(&tv, nullptr);
    BEE_SERIAL.printf("cellular time synced UTC epoch=%ld (local %04d-%02d-%02d %02d:%02d:%02d tz_h=%.1f)\n",
                      static_cast<long>(epoch), year, month, day, hour, minute, second, timezone);
  }
}

#endif  // cellular

bool wifi_sta_connect() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < 60; ++i) {
    if (WiFi.status() == WL_CONNECTED) {
      return true;
    }
    delay(500);
  }
  return false;
}

void wifi_espnow_channel_only() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect();
  esp_wifi_set_channel(GATEWAY_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
}

void wifi_sync_time() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  for (int i = 0; i < 100; ++i) {
    time_t now = time(nullptr);
    if (now > 1700000000) {
      return;
    }
    delay(200);
  }
}

bool wifi_http_post(const String& url, const String& authorization_header, const String& body,
                    int& status_code, String& response_body, uint32_t timeout_ms) {
  WiFiClient client;
  HTTPClient http;
  if (timeout_ms > 0) {
    http.setTimeout(timeout_ms);
  }
  if (!http.begin(client, url)) {
    status_code = 0;
    response_body = "";
    return false;
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", authorization_header);
  status_code = http.POST(body);
  response_body = http.getString();
  http.end();
  return status_code >= 200 && status_code < 300;
}

}  // namespace

bool uplink_init() {
#if UPLINK_MODE == UPLINK_MODE_CELLULAR
#if defined(BEEPLAN_BOARD_TTGO_T_CALL)
  wifi_espnow_channel_only();
  BEE_SERIAL.printf("ESP-NOW channel=%d (cellular uplink)\n", GATEWAY_WIFI_CHANNEL);
  g_cellular_ready = cellular_connect();
  return g_cellular_ready;
#else
  BEE_SERIAL.println("Cellular uplink requires TTGO T-Call board");
  return false;
#endif
#else
  if (!wifi_sta_connect()) {
    return false;
  }
  BEE_SERIAL.println(WiFi.localIP());
  BEE_SERIAL.printf("WiFi channel=%d\n", WiFi.channel());
  return true;
#endif
}

bool uplink_ready() {
#if UPLINK_MODE == UPLINK_MODE_CELLULAR && defined(BEEPLAN_BOARD_TTGO_T_CALL)
  return g_cellular_ready && g_modem.isGprsConnected();
#else
  return WiFi.status() == WL_CONNECTED;
#endif
}

bool uplink_http_post(const String& url, const String& authorization_header, const String& body,
                      int& status_code, String& response_body, uint32_t timeout_ms) {
#if UPLINK_MODE == UPLINK_MODE_CELLULAR && defined(BEEPLAN_BOARD_TTGO_T_CALL)
  return cellular_http_post(url, authorization_header, body, status_code, response_body, timeout_ms);
#else
  return wifi_http_post(url, authorization_header, body, status_code, response_body, timeout_ms);
#endif
}

void uplink_sync_time() {
#if UPLINK_MODE == UPLINK_MODE_CELLULAR && defined(BEEPLAN_BOARD_TTGO_T_CALL)
  cellular_sync_time();
#else
  wifi_sync_time();
#endif
}

int8_t uplink_signal_dbm() {
#if UPLINK_MODE == UPLINK_MODE_CELLULAR && defined(BEEPLAN_BOARD_TTGO_T_CALL)
  return csq_to_dbm(static_cast<int8_t>(g_modem.getSignalQuality()));
#else
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
#endif
}

uint8_t gateway_wifi_channel() {
#if UPLINK_MODE == UPLINK_MODE_CELLULAR
  return GATEWAY_WIFI_CHANNEL;
#else
  return WiFi.channel();
#endif
}
