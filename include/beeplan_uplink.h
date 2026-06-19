#pragma once

#include <Arduino.h>
#include <stdint.h>

/** Инициализация Wi‑Fi (для ESP-NOW) и канала передачи данных на API. */
bool uplink_init();

/** Готовность HTTP-канала (Wi‑Fi или GPRS). */
bool uplink_ready();

/**
 * POST JSON на полный URL. authorization_header — значение заголовка Authorization (например "Bearer …").
 * Возвращает true при HTTP 2xx; status_code и response_body заполняются всегда.
 */
bool uplink_http_post(const String& url, const String& authorization_header, const String& body,
                      int& status_code, String& response_body);

/** RSSI Wi‑Fi или CSQ модема (dBm, -127 если нет). */
int8_t uplink_signal_dbm();

/** Канал Wi‑Fi для ESP-NOW. */
uint8_t gateway_wifi_channel();
