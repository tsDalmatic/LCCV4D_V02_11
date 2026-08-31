/**
  ******************************************************************************
  * @file    main.h 
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    19-Jun-2017
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32F0xx_hal.h"

//#if defined (USE_STM32072B_EVAL)
//  #include "stm32072b_eval.h"
//  #include "stm32072b_eval_lcd.h"
//#else
//  #error define proper eval includes!!!
//#endif /* USE_STM32072B_EVAL */

#include <stdio.h>

#define USARTx  USART2

#define MAX_FLASH_LATENCY FLASH_LATENCY_1
#define __HAL_RCC_CLEAR_FLAG __HAL_RCC_CLEAR_RESET_FLAGS

/* Exported types ------------------------------------------------------------*/
/* Exported variableses ------------------------------------------------------*/
  extern uint32_t MyRAMCounter;
  extern uint32_t MyFLASHCounter;
/* Exported variableses ------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define   LED_VLM   GPIO_PIN_9
#define   LED_NVM   GPIO_PIN_10
#define   LED_ERR   GPIO_PIN_11
#define  BSP_LED_On(gpio_pin)  HAL_GPIO_WritePin(GPIOD, (gpio_pin), GPIO_PIN_RESET)
#define  BSP_LED_Off(gpio_pin) HAL_GPIO_WritePin(GPIOD, (gpio_pin), GPIO_PIN_SET)
#define  BSP_LED_Toggle(gpio_pin) HAL_GPIO_TogglePin(GPIOD, (gpio_pin))
#define  BSP_LED_Init   User_signal_Init
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void STL_StartUp(void);
void SystemInit (void);
void SystemClock_Config(void);
void StartUpClock_Config(void);
void User_signal_Init(uint16_t gpio_pin);
#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
  void __iar_data_init3(void);
#endif /* __IAR_SYSTEMS_ICC__ */
#ifdef __CC_ARM             /* KEIL Compiler */
extern void $Super$$main(void);  
#endif /* __CC_ARM */
#ifdef __GNUC__
int16_t __io_putchar(int16_t ch);
#endif /* __GNUC__ */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
