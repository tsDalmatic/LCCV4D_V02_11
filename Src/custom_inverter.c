#include "custom_inverter.h"
#include "modbus.h"
#include "eprom.h"
#include "Defines.h"
#include "Global_Var.h"
extern void stop(void);

/* Separate block preserves settings used by the existing M100 products. */
#define PROFILE_EEPROM 0x0200U
#define PROFILE_MAGIC_ADDRESS 0x0224U
#define PROFILE_CRC_ADDRESS 0x0226U
#define PROFILE_MAGIC 0xC101U
#define HEARTBEAT_INTERVAL_MS 25U
#define STATUS_INTERVAL_MS 250U
static const InverterProfile baseline = {{370,230,26,3000,5000,0,5000,
                                         5000,3500,5,5,5,5000,3500,5,5,5,2}};
static InverterProfile profile;
volatile uint16_t CustomInverter_State;
volatile uint16_t CustomInverter_Error;
volatile uint8_t CustomInverter_RestartRequired;
volatile uint16_t CustomInverter_ReplyErrorCount;
volatile uint16_t CustomInverter_ReplyErrorStreak;
static uint16_t response, sequence, heartbeat;
static uint8_t step, field, waiting, initialized, enabled_before;
static uint32_t heartbeat_tick, response_tick, last_ack_tick;
static uint8_t suppress_service;
static uint8_t profile_ok, heartbeat_started;

static void save_word(uint16_t address, uint16_t value) {
  EE_write(address, (uint8_t)value);
  EE_write(address + 1U, (uint8_t)(value >> 8));
}
static uint16_t load_word(uint16_t address) {
  uint8_t low = EE_read(address);
  return (uint16_t)(low | ((uint16_t)EE_read(address + 1U) << 8));
}
void CustomInverter_RequestConfiguration(void) {
  step = field = waiting = 0U;
  CustomInverter_State = INV_LOCKED;
  CustomInverter_Error = INV_ERROR_NONE;
  heartbeat_started = 0U;
  last_ack_tick = HAL_GetTick();
  CustomInverter_ReplyErrorStreak = 0U;
  ++sequence; if (sequence == 0U) ++sequence;
  door_run_off;
}
void CustomInverter_Init(void) {
  if (e21_fail) {
    initialized=1U; profile_ok=0U;
    CustomInverter_State=INV_REJECTED; CustomInverter_Error=INV_ERROR_RANGE;
    return;
  }
  suppress_service = 1U;
  uint16_t magic = load_word(PROFILE_MAGIC_ADDRESS);
  if (magic != PROFILE_MAGIC) {
    profile = baseline;
    for (uint8_t i = 0; i < INV_FIELD_COUNT; ++i) save_word(PROFILE_EEPROM + 2U*i, profile.words[i]);
    save_word(PROFILE_CRC_ADDRESS, InverterProfile_Crc(&profile, 0U));
    /* Marker last: an interrupted first initialization is repeated next boot. */
    save_word(PROFILE_MAGIC_ADDRESS, PROFILE_MAGIC);
  } else {
    for (uint8_t i = 0; i < INV_FIELD_COUNT; ++i) profile.words[i] = load_word(PROFILE_EEPROM + 2U*i);
  }
  initialized = 1U; suppress_service = 0U;
  profile_ok = 1U;
  CustomInverter_RequestConfiguration();
  if (e21_fail || load_word(PROFILE_CRC_ADDRESS) != InverterProfile_Crc(&profile, 0U) ||
      InverterProfile_ValidatePhase1(&profile) != INV_ERROR_NONE) {
    CustomInverter_Error = INV_ERROR_RANGE; CustomInverter_State = INV_REJECTED;
    profile_ok = 0U;
  }
}
uint8_t CustomInverter_Ready(void) { return CustomInverter_State == INV_READY; }
uint16_t CustomInverter_GetMenu(uint8_t menu) {
  static const uint8_t menus[INV_FIELD_COUNT] = {100,101,102,103,104,105,109,110,111,112,113,114,120,121,122,123,124,0};
  for (uint8_t i=0; i<INV_FIELD_COUNT; ++i) if (menus[i] == menu) return profile.words[i];
  return 0U;
}
static void failed(void) {
  CustomInverter_RestartRequired = 1U;
  CustomInverter_RequestConfiguration();
  CustomInverter_Error = INV_ERROR_SESSION;
  e27_modbus = 1U;
  stop();
}
static void publish_ready(void) {
  if (CustomInverter_RestartRequired &&
      (stop_pb || stop_lid_pb || stop_disp_pb) &&
      !open_pb && !close_pb && !open_disp_pb && !close_disp_pb)
    CustomInverter_RestartRequired = 0U;
  e27_modbus = CustomInverter_RestartRequired;
  if (CustomInverter_RestartRequired) door_run_off;
}
static void restart_upload(void) {
  step = field = waiting = 0U;
  CustomInverter_State = INV_LOCKED;
  CustomInverter_Error = INV_ERROR_NONE;
  heartbeat_started = 0U;
  ++sequence; if (sequence == 0U) ++sequence;
  door_run_off;
}
/* All protocol exchanges use the existing FC03/FC06 driver, one outstanding request. */
void CustomInverter_Service(uint8_t enabled) {
  uint32_t now = HAL_GetTick();
  if (suppress_service || !initialized) return;
  if (!profile_ok) { CustomInverter_State=INV_REJECTED; e27_modbus=1U; door_run_off; return; }
  if (!enabled) {
    if (enabled_before) CustomInverter_RequestConfiguration();
    enabled_before = 0U; return;
  }
  enabled_before = 1U;
  if (CustomInverter_Ready()) publish_ready();
  if (!CustomInverter_Ready()) { e27_modbus = 1U; door_run_off; }
  if (CustomInverter_State == INV_REJECTED) return;
  HAL_StatusTypeDef state = getModbusState();
  if (state == HAL_BUSY) return;
  if (waiting) {
    uint8_t completed = waiting;
    waiting = 0U;
    if (state != HAL_OK) {
      if (CustomInverter_ReplyErrorCount != 0xFFFFU) ++CustomInverter_ReplyErrorCount;
      if (CustomInverter_ReplyErrorStreak != 0xFFFFU) ++CustomInverter_ReplyErrorStreak;
      heartbeat_started = 0U;
      if ((uint32_t)(now - last_ack_tick) >= INV_LINK_TIMEOUT_MS) { failed(); return; }
      if (!CustomInverter_Ready()) restart_upload();
      return;
    }
    last_ack_tick = now;
    CustomInverter_ReplyErrorStreak = 0U;
    if (completed == 1U && step == 1U && response != INV_PROTOCOL_VERSION) {
      CustomInverter_State = INV_REJECTED; CustomInverter_Error = INV_ERROR_UNSUPPORTED; return;
    }
    if (completed == 1U && step == 7U && response != INV_READY) { failed(); return; }
    if (completed == 1U && step == 8U && response != sequence) { failed(); return; }
    if (completed == 1U && step == 9U) {
      if (response != INV_READY) { failed(); return; }
      response_tick = now; CustomInverter_State = INV_READY;
      CustomInverter_Error = INV_ERROR_NONE; publish_ready();
    }
  }
  /* A reply may complete in the UART ISR while foreground work delays this
   * service call. Consume that reply before deciding its acknowledgement is
   * stale, otherwise a valid waiting reply can falsely latch E27. */
  if ((uint32_t)(now - last_ack_tick) >= INV_LINK_TIMEOUT_MS) { failed(); return; }
  /* Heartbeat writes receive acknowledgements too. They pre-empt no transaction. */
  if (!heartbeat_started || (uint32_t)(now - heartbeat_tick) >= HEARTBEAT_INTERVAL_MS) {
    if (setModbusParam((enum modbusParam)INV_HEARTBEAT_REGISTER, ++heartbeat) == HAL_OK) {
      heartbeat_tick = now; waiting = 2U;
      heartbeat_started = 1U;
    }
    return;
  }
  if (CustomInverter_Ready()) {
    if ((uint32_t)(now - response_tick) >= STATUS_INTERVAL_MS) {
      if (getModbusParam((enum modbusParam)INV_STATUS_REGISTER, &response) == HAL_OK) {
        step = 9U; waiting = 1U;
      }
    }
    return;
  }
  HAL_StatusTypeDef sent = HAL_ERROR;
  switch (step) {
    case 0: sent = getModbusParam((enum modbusParam)INV_VERSION_REGISTER, &response); if (sent == HAL_OK) step=1U; break;
    case 1: sent = setModbusParam((enum modbusParam)INV_BEGIN_REGISTER, INV_BEGIN_VALUE); if (sent == HAL_OK) step=2U; break;
    case 2:
      sent = setModbusParam((enum modbusParam)(INV_CONFIG_BASE + field), profile.words[field]);
      if (sent == HAL_OK && ++field == INV_FIELD_COUNT) step=3U;
      break;
    case 3: sent = setModbusParam((enum modbusParam)INV_SEQUENCE_REGISTER, sequence); if (sent == HAL_OK) step=4U; break;
    case 4: sent = setModbusParam((enum modbusParam)INV_CRC_REGISTER, InverterProfile_Crc(&profile,sequence)); if (sent == HAL_OK) step=5U; break;
    case 5: sent = setModbusParam((enum modbusParam)INV_COMMIT_REGISTER, INV_COMMIT_VALUE); if (sent == HAL_OK) step=6U; break;
    case 6: sent = getModbusParam((enum modbusParam)INV_STATUS_REGISTER,&response); if (sent == HAL_OK) step=7U; break;
    case 7: sent = getModbusParam((enum modbusParam)INV_ACTIVE_SEQUENCE_REGISTER,&response); if (sent == HAL_OK) step=8U; break;
    case 8:
      CustomInverter_State = INV_READY; publish_ready(); response_tick=now; return;
    default: failed(); return;
  }
  if (sent == HAL_OK) waiting=1U;
}
HAL_StatusTypeDef CustomInverter_CheckLink(void) {
  if (!initialized) CustomInverter_Init();
  if (!profile_ok) return HAL_ERROR;
  CustomInverter_RequestConfiguration();
  uint32_t started=HAL_GetTick();
  while (!CustomInverter_Ready() && CustomInverter_State != INV_REJECTED &&
         (uint32_t)(HAL_GetTick()-started) < 2000U) {
    CustomInverter_Service(1U);
  }
  return CustomInverter_Ready() ? HAL_OK : HAL_ERROR;
}
