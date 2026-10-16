#ifndef CUSTOM_INVERTER_H
#define CUSTOM_INVERTER_H
#include "inverter_protocol.h"
#include "stm32f0xx_hal.h"
#ifndef LCC_CUSTOM_INVERTER
#define LCC_CUSTOM_INVERTER 1
#endif
void CustomInverter_Init(void);
void CustomInverter_Service(uint8_t enabled);
void CustomInverter_RequestConfiguration(void);
HAL_StatusTypeDef CustomInverter_CheckLink(void);
uint8_t CustomInverter_Ready(void);
uint16_t CustomInverter_GetMenu(uint8_t menu);
extern volatile uint16_t CustomInverter_State;
extern volatile uint16_t CustomInverter_Error;
extern volatile uint8_t CustomInverter_RestartRequired;
extern volatile uint16_t CustomInverter_ReplyErrorCount;
extern volatile uint16_t CustomInverter_ReplyErrorStreak;
#endif
