#include "beeplan_uplink.h"

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

WiFiClient g_wifi_client;

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
#include <TinyGsmClient.h>

HardwareSerial g_modem_serial(1);
TinyGsm g_modem(g_modem_serial);
TinyGsmClient g_cell_client(g_modem);
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

Client& uplink_http_client() {
#if UPLINK_MODE == UPLINK_MODE_CELLULAR && defined(BEEPLAN_BOARD_TTGO_T_CALL)
  return g_cell_client;
#else
  return g_wifi_client;
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
