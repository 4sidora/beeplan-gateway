#pragma once

#include <esp_idf_version.h>
#include <esp_now.h>

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#define BEEPLAN_ESPNOW_V3 1
#endif

#if BEEPLAN_ESPNOW_V3

inline bool beeplan_register_recv_cb(void (*cb)(const esp_now_recv_info_t*, const uint8_t*, int)) {
  return esp_now_register_recv_cb(cb) == ESP_OK;
}

#else

inline bool beeplan_register_recv_cb(void (*cb)(const uint8_t*, const uint8_t*, int)) {
  return esp_now_register_recv_cb(cb) == ESP_OK;
}

#endif
