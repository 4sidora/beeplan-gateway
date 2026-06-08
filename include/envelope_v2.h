#pragma once

#include <stdint.h>

/** ESP-NOW v1 — one metric per frame (legacy). */
constexpr uint32_t kBeeplanMagicV1 = 0x00BEEF01;
constexpr uint8_t kBeeplanProtoV1 = 1;

/** ESP-NOW v2 — aggregated hourly report. */
constexpr uint32_t kBeeplanMagicV2 = 0x00BEEF02;
constexpr uint8_t kBeeplanProtoV2 = 2;

/** Downlink ACK (gateway → edge). */
constexpr uint32_t kBeeplanMagicAck = 0x00BEEFAC;

constexpr uint8_t kFlagEmergency = 0x01;
constexpr uint8_t kFlagScheduled = 0x02;
constexpr uint8_t kFlagAudioIncluded = 0x04;

constexpr uint8_t kMetricTemp = 0x01;
constexpr uint8_t kMetricRh = 0x02;
constexpr uint8_t kMetricSignal = 0x04;
constexpr uint8_t kMetricBattery = 0x08;
constexpr uint8_t kMetricFirmware = 0x10;
constexpr uint8_t kMetricAudio = 0x20;

struct __attribute__((packed)) EnvelopeV1 {
  uint32_t magic;
  uint8_t proto_version;
  char device_id[32];
  uint32_t unix_ts;
  uint8_t metric;
  int16_t i16_a;
  int16_t i16_b;
};

struct __attribute__((packed)) ReportFrameV2 {
  uint32_t magic;
  uint8_t proto_version;
  uint8_t flags;
  uint8_t seq;
  uint8_t device_type;
  uint8_t metrics_present;
  char device_id[32];
  uint32_t unix_ts;
  int16_t temp_c_x100;
  int16_t rh_x100;
  int16_t signal_dbm;
  int16_t battery_x100;
  uint16_t fw_major_minor;
  uint16_t fw_patch;
  int16_t audio_rms_x1000;
  int16_t audio_peak_hz;
};

struct __attribute__((packed)) AckFrameV2 {
  uint32_t magic;
  uint8_t proto_version;
  uint8_t ack_seq;
  char device_id[32];
};

static_assert(sizeof(ReportFrameV2) <= 64, "ReportFrameV2 must stay compact");
