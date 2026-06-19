#pragma once

#include <Client.h>
#include <stdint.h>

/** Инициализация Wi‑Fi (для ESP-NOW) и канала передачи данных на API. */
bool uplink_init();

/** Готовность HTTP-канала (Wi‑Fi или GPRS). */
bool uplink_ready();

/** Клиент для HTTPClient (WiFiClient или TinyGsmClient). */
Client& uplink_http_client();

/** RSSI Wi‑Fi или CSQ модема (dBm, -127 если нет). */
int8_t uplink_signal_dbm();

/** Канал Wi‑Fi для ESP-NOW. */
uint8_t gateway_wifi_channel();
