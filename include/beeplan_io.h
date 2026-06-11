#pragma once

#include <Arduino.h>

#include "config.h"

#if defined(CONFIG_IDF_TARGET_ESP32C3) && !ARDUINO_USB_CDC_ON_BOOT
#define BEE_SERIAL Serial0
#define BEE_SERIAL_BEGIN() BEE_SERIAL.begin(115200, SERIAL_8N1, 20, 21)
#else
#define BEE_SERIAL Serial
#define BEE_SERIAL_BEGIN() BEE_SERIAL.begin(115200)
#endif

#if BEEPLAN_DEBUG
#define BEEPLAN_LOG(...) BEE_SERIAL.printf(__VA_ARGS__)
#define BEEPLAN_LOGLN(msg) BEE_SERIAL.println(msg)
#else
#define BEEPLAN_LOG(...) ((void)0)
#define BEEPLAN_LOGLN(msg) ((void)0)
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3)
constexpr int kBoardLedPins[] = {12, 13};
#endif

inline void beeplan_led_init() {
#if defined(CONFIG_IDF_TARGET_ESP32C3)
  for (int pin : kBoardLedPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
#endif
}

inline void beeplan_led_toggle() {
#if defined(CONFIG_IDF_TARGET_ESP32C3)
  for (int pin : kBoardLedPins) {
    digitalWrite(pin, !digitalRead(pin));
  }
#endif
}

inline void beeplan_serial_begin() {
#if BEEPLAN_DEBUG
  BEE_SERIAL_BEGIN();
  delay(400);
  BEE_SERIAL.println();
  BEE_SERIAL.println();
#if defined(CONFIG_IDF_TARGET_ESP32C3) && ARDUINO_USB_CDC_ON_BOOT
  BEE_SERIAL.println("BeePlan: USB CDC console");
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  BEE_SERIAL.println("BeePlan: UART0 GPIO20/21 (CH343 / CORE-ESP32-C3)");
#else
  BEE_SERIAL.println("BeePlan: serial console");
#endif
  BEE_SERIAL.flush();
#endif
}
