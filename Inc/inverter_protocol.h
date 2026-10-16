#ifndef INVERTER_PROTOCOL_H
#define INVERTER_PROTOCOL_H

#include <stdint.h>

/* Shared wire contract. Frequency is 0.01 Hz, current 0.1 A, time 0.1 s.
 * Power is watts; speed is nameplate rotor RPM. No LS power/slip encoding. */
#define INV_PROTOCOL_VERSION 0x0101U
#define INV_PROTOCOL_PHASE 1U
#define INV_HEARTBEAT_REGISTER 4U
#define INV_LINK_TIMEOUT_MS 100U
#define INV_CONFIG_BASE 0x0100U
#define INV_BEGIN_REGISTER 0x0020U
#define INV_SEQUENCE_REGISTER 0x0021U
#define INV_CRC_REGISTER 0x0022U
#define INV_COMMIT_REGISTER 0x0023U
#define INV_STATUS_REGISTER 0x0024U
#define INV_VERSION_REGISTER 0x0025U
#define INV_ACTIVE_SEQUENCE_REGISTER 0x0026U
#define INV_ERROR_REGISTER 0x0027U
#define INV_PHASE_REGISTER 0x0028U
#define INV_BEGIN_VALUE 0xB001U
#define INV_COMMIT_VALUE 0xC001U

enum InverterField {
  INV_POWER_W, INV_VOLTAGE_V, INV_CURRENT_X10_A, INV_RATED_RPM,
  INV_BASE_FREQ_X100_HZ, INV_AUTOTUNE, INV_MAX_FREQ_X100_HZ,
  INV_OPEN_HIGH, INV_OPEN_LOW, INV_OPEN_ACC, INV_OPEN_DEC, INV_OPEN_STOP,
  INV_CLOSE_HIGH, INV_CLOSE_LOW, INV_CLOSE_ACC, INV_CLOSE_DEC, INV_CLOSE_STOP,
  INV_POLES, INV_FIELD_COUNT
};
enum InverterStatus { INV_LOCKED, INV_RECEIVING, INV_READY, INV_REJECTED };
enum InverterError {
  INV_ERROR_NONE, INV_ERROR_BUSY, INV_ERROR_INCOMPLETE, INV_ERROR_CRC,
  INV_ERROR_RANGE, INV_ERROR_UNSUPPORTED, INV_ERROR_SESSION
};
typedef struct { uint16_t words[INV_FIELD_COUNT]; } InverterProfile;

static inline uint16_t InverterProfile_Crc(const InverterProfile *profile,
                                          uint16_t sequence) {
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0; i < INV_FIELD_COUNT + 2U; ++i) {
    uint16_t word = (i == 0U) ? INV_PROTOCOL_VERSION :
                    (i == 1U) ? sequence : profile->words[i - 2U];
    for (uint8_t b = 0; b < 2U; ++b) {
      crc ^= (b == 0U) ? (word >> 8) : (word & 0xFFU);
      for (uint8_t bit = 0; bit < 8U; ++bit)
        crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
    }
  }
  return crc;
}

static inline uint16_t InverterProfile_Validate(const InverterProfile *p) {
  const uint16_t *w = p->words;
  if (w[INV_POWER_W] == 0U || w[INV_POWER_W] > 2200U ||
      w[INV_VOLTAGE_V] < 100U || w[INV_VOLTAGE_V] > 230U ||
      w[INV_CURRENT_X10_A] < 1U || w[INV_CURRENT_X10_A] > 26U ||
      w[INV_BASE_FREQ_X100_HZ] < 1000U || w[INV_BASE_FREQ_X100_HZ] > 10000U ||
      w[INV_MAX_FREQ_X100_HZ] < w[INV_BASE_FREQ_X100_HZ] ||
      w[INV_MAX_FREQ_X100_HZ] > 10000U || w[INV_AUTOTUNE] != 0U ||
      (w[INV_POLES] != 2U && w[INV_POLES] != 4U && w[INV_POLES] != 6U))
    return INV_ERROR_RANGE;
  uint32_t synchronous_rpm = (120UL * w[INV_BASE_FREQ_X100_HZ]) / (100UL * w[INV_POLES]);
  if (w[INV_RATED_RPM] == 0U || w[INV_RATED_RPM] > synchronous_rpm ||
      w[INV_RATED_RPM] < synchronous_rpm / 2U)
    return INV_ERROR_RANGE;
  for (uint8_t i = INV_OPEN_HIGH; i <= INV_CLOSE_HIGH; i += 5U) {
    if (w[i + 1U] < 100U || w[i + 1U] > w[i] || w[i] > w[INV_MAX_FREQ_X100_HZ])
      return INV_ERROR_RANGE;
    for (uint8_t j = 2U; j <= 4U; ++j)
      if (w[i + j] < 1U || w[i + j] > 100U) return INV_ERROR_RANGE;
  }
  return INV_ERROR_NONE;
}

/* Phase 1 transports the profile but only authorizes the commissioned drive.
 * Motion and V/f parameter application will be introduced in later phases. */
static inline uint16_t InverterProfile_ValidatePhase1(const InverterProfile *p) {
  uint16_t error = InverterProfile_Validate(p);
  if (error != INV_ERROR_NONE) return error;
  const uint16_t *w = p->words;
  if (w[INV_POWER_W] != 370U || w[INV_VOLTAGE_V] != 230U ||
      w[INV_CURRENT_X10_A] != 26U || w[INV_POLES] != 2U ||
      w[INV_BASE_FREQ_X100_HZ] != 5000U || w[INV_MAX_FREQ_X100_HZ] != 5000U ||
      w[INV_OPEN_HIGH] != 5000U || w[INV_CLOSE_HIGH] != 5000U ||
      w[INV_OPEN_LOW] != 3500U || w[INV_CLOSE_LOW] != 3500U ||
      w[INV_OPEN_ACC] != 5U || w[INV_CLOSE_ACC] != 5U ||
      w[INV_OPEN_DEC] != 5U || w[INV_CLOSE_DEC] != 5U ||
      w[INV_OPEN_STOP] != 5U || w[INV_CLOSE_STOP] != 5U)
    return INV_ERROR_UNSUPPORTED;
  return INV_ERROR_NONE;
}

#endif
