#include "beeplan_uplink.h"

#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "beeplan_io.h"
#include "config.h"

#ifndef UPLINK_MODE_WIFI
#define UPLINK_MODE_WIFI 0
#endif
#ifndef UPLINK_MODE_CELLULAR
#define UPLINK_MODE_CELLULAR 1
#endif

namespace {

struct ParsedUrl {
  bool https = false;
  String host;
  uint16_t port = 80;
  String path = "/";
};

bool parse_url(const String& url, ParsedUrl& out) {
  String rest = url;
  if (rest.startsWith("https://")) {
    out.https = true;
    out.port = 443;
    rest = rest.substring(8);
  } else if (rest.startsWith("http://")) {
    rest = rest.substring(7);
  } else {
    return false;
  }

  const int slash = rest.indexOf('/');
  String host_port = slash >= 0 ? rest.substring(0, slash) : rest;
  out.path = slash >= 0 ? rest.substring(slash) : "/";
  if (out.path.length() == 0) {
    out.path = "/";
  }

  const int colon = host_port.indexOf(':');
  if (colon >= 0) {
    out.host = host_port.substring(0, colon);
    out.port = static_cast<uint16_t>(host_port.substring(colon + 1).toInt());
  } else {
    out.host = host_port;
  }
  return out.host.length() > 0;
}

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
#define TINY_GSM_USE_SSL
#include <TinyGsmClient.h>
#include <HttpClient.h>

HardwareSerial g_modem_serial(1);
TinyGsm g_modem(g_modem_serial);
TinyGsmClientSecure g_cell_secure(g_modem);
bool g_cellular_ready = false;

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

int8_t csq_to_dbm(int8_t csq) {
  if (csq <= 0 || csq == 99) {
    return -127;
  }
  return static_cast<int8_t>(-113 + 2 * csq);
}

bool cellular_http_post(const ParsedUrl& parsed, const String& authorization_header, const String& body,
                        int& status_code, String& response_body) {
  const uint16_t port = parsed.port != 0 ? parsed.port : (parsed.https ? 443 : 80);
  HttpClient http(g_cell_secure, parsed.host.c_str(), port);
  http.beginRequest();
  http.post(parsed.path.c_str());
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Authorization", authorization_header);
  http.sendHeader("Connection", "close");
  http.beginBody();
  http.print(body);
  http.endRequest();
  status_code = http.responseStatusCode();
  response_body = "";
  while (http.available()) {
    response_body += static_cast<char>(http.read());
  }
  http.stop();
  return status_code >= 200 && status_code < 300;
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

bool wifi_http_post(const String& url, const String& authorization_header, const String& body,
                    int& status_code, String& response_body) {
  WiFiClient client;
  HTTPClient http;
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
                      int& status_code, String& response_body) {
#if UPLINK_MODE == UPLINK_MODE_CELLULAR && defined(BEEPLAN_BOARD_TTGO_T_CALL)
  ParsedUrl parsed;
  if (!parse_url(url, parsed)) {
    status_code = 0;
    response_body = "";
    return false;
  }
  return cellular_http_post(parsed, authorization_header, body, status_code, response_body);
#else
  return wifi_http_post(url, authorization_header, body, status_code, response_body);
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
