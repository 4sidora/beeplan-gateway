#pragma once

#include <esp_idf_version.h>
#include <esp_now.h>
#include <esp_wifi.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#define BEEPLAN_ESPNOW_V3 1
#endif

/** 802.11 LR — увеличивает дальность ESP-NOW (ESP32 / ESP32-S3). */
inline bool beeplan_espnow_enable_lr() {
  const uint8_t proto = static_cast<uint8_t>(
      WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
  return esp_wifi_set_protocol(WIFI_IF_STA, proto) == ESP_OK;
}

#if BEEPLAN_ESPNOW_V3

inline bool beeplan_register_recv_cb(void (*cb)(const esp_now_recv_info_t*, const uint8_t*, int)) {
  return esp_now_register_recv_cb(cb) == ESP_OK;
}

#else

inline bool beeplan_register_recv_cb(void (*cb)(const uint8_t*, const uint8_t*, int)) {
  return esp_now_register_recv_cb(cb) == ESP_OK;
}

#endif
