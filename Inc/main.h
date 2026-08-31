/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2022 STMicroelectronics
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
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DISPLAY_DATA_Pin GPIO_PIN_2
#define DISPLAY_DATA_GPIO_Port GPIOE
#define DISPLAY_CP_Pin GPIO_PIN_3
#define DISPLAY_CP_GPIO_Port GPIOE
#define OPTICAL_EDGE_Pin GPIO_PIN_4
#define OPTICAL_EDGE_GPIO_Port GPIOE
#define OPTICAL_EDGE_EXTI_IRQn EXTI4_15_IRQn
#define SAFETY_PHOTO_Pin GPIO_PIN_5
#define SAFETY_PHOTO_GPIO_Port GPIOE
#define DISPLAY_STR_Pin GPIO_PIN_6
#define DISPLAY_STR_GPIO_Port GPIOE
#define DIGIT_1_Pin GPIO_PIN_0
#define DIGIT_1_GPIO_Port GPIOC
#define DIGIT_2_Pin GPIO_PIN_1
#define DIGIT_2_GPIO_Port GPIOC
#define DIGIT_3_Pin GPIO_PIN_2
#define DIGIT_3_GPIO_Port GPIOC
#define DIGIT_4_Pin GPIO_PIN_3
#define DIGIT_4_GPIO_Port GPIOC
#define AN_PNE_EDGE_Pin GPIO_PIN_0
#define AN_PNE_EDGE_GPIO_Port GPIOA
#define USART2_DE_ENCODER_Pin GPIO_PIN_1
#define USART2_DE_ENCODER_GPIO_Port GPIOA
#define USART2_TX_ENCODER_Pin GPIO_PIN_2
#define USART2_TX_ENCODER_GPIO_Port GPIOA
#define USART2_RX_ENCODER_Pin GPIO_PIN_3
#define USART2_RX_ENCODER_GPIO_Port GPIOA
#define AN_WICKET_Pin GPIO_PIN_5
#define AN_WICKET_GPIO_Port GPIOA
#define FREE_IN1_Pin GPIO_PIN_6
#define FREE_IN1_GPIO_Port GPIOA
#define PULSE_SPEED_Pin GPIO_PIN_7
#define PULSE_SPEED_GPIO_Port GPIOA
#define FREE_IN2_Pin GPIO_PIN_4
#define FREE_IN2_GPIO_Port GPIOC
#define FREE_IN3_Pin GPIO_PIN_5
#define FREE_IN3_GPIO_Port GPIOC
#define LAMP_CLK_Pin GPIO_PIN_0
#define LAMP_CLK_GPIO_Port GPIOB
#define LAMP_DATA_Pin GPIO_PIN_1
#define LAMP_DATA_GPIO_Port GPIOB
#define EX_AUTO_Pin GPIO_PIN_2
#define EX_AUTO_GPIO_Port GPIOB
#define CLOSE_PB_Pin GPIO_PIN_8
#define CLOSE_PB_GPIO_Port GPIOE
#define OPEN_PB_Pin GPIO_PIN_9
#define OPEN_PB_GPIO_Port GPIOE
#define GO_FUNCTION_Pin GPIO_PIN_10
#define GO_FUNCTION_GPIO_Port GPIOE
#define UP_RUNNING_Pin GPIO_PIN_11
#define UP_RUNNING_GPIO_Port GPIOE
#define DOOR_UP_Pin GPIO_PIN_12
#define DOOR_UP_GPIO_Port GPIOE
#define STOP_PB_Pin GPIO_PIN_13
#define STOP_PB_GPIO_Port GPIOE
#define DOOR_DOWN_Pin GPIO_PIN_14
#define DOOR_DOWN_GPIO_Port GPIOE
#define STOP_LID_Pin GPIO_PIN_15
#define STOP_LID_GPIO_Port GPIOE
#define USART3_TX_OPTION_Pin GPIO_PIN_10
#define USART3_TX_OPTION_GPIO_Port GPIOB
#define USART3_RX_OPTION_Pin GPIO_PIN_11
#define USART3_RX_OPTION_GPIO_Port GPIOB
#define K1_CTRL_Pin GPIO_PIN_12
#define K1_CTRL_GPIO_Port GPIOB
#define EMER_STOP_Pin GPIO_PIN_13
#define EMER_STOP_GPIO_Port GPIOB
#define USART3_DE_OPTION_Pin GPIO_PIN_14
#define USART3_DE_OPTION_GPIO_Port GPIOB
#define SAFETY_TEST_Pin GPIO_PIN_8
#define SAFETY_TEST_GPIO_Port GPIOD
#define LED_LEDPAD_Pin GPIO_PIN_9
#define LED_LEDPAD_GPIO_Port GPIOD
#define SAFETY_CHAIN_Pin GPIO_PIN_10
#define SAFETY_CHAIN_GPIO_Port GPIOD
#define MON_PH1_3_Pin GPIO_PIN_12
#define MON_PH1_3_GPIO_Port GPIOD
#define MON_PH2_3_Pin GPIO_PIN_14
#define MON_PH2_3_GPIO_Port GPIOD
#define DIL_SW1_Pin GPIO_PIN_6
#define DIL_SW1_GPIO_Port GPIOC
#define DIL_SW2_Pin GPIO_PIN_7
#define DIL_SW2_GPIO_Port GPIOC
#define DIL_SW3_Pin GPIO_PIN_8
#define DIL_SW3_GPIO_Port GPIOC
#define I2C1_SCL_EEPROM_Pin GPIO_PIN_9
#define I2C1_SCL_EEPROM_GPIO_Port GPIOA
#define I2C1_SDA_EEPROM_Pin GPIO_PIN_10
#define I2C1_SDA_EEPROM_GPIO_Port GPIOA
#define USART4_DE_MODBUS_Pin GPIO_PIN_15
#define USART4_DE_MODBUS_GPIO_Port GPIOA
#define USART4_TX_MODBUS_Pin GPIO_PIN_10
#define USART4_TX_MODBUS_GPIO_Port GPIOC
#define USART4_RX_MODBUS_Pin GPIO_PIN_11
#define USART4_RX_MODBUS_GPIO_Port GPIOC
#define WIFI_GP3_Pin GPIO_PIN_1
#define WIFI_GP3_GPIO_Port GPIOD
#define WIFI_GP2_Pin GPIO_PIN_3
#define WIFI_GP2_GPIO_Port GPIOD
#define WIFI_GP1_Pin GPIO_PIN_4
#define WIFI_GP1_GPIO_Port GPIOD
#define AUX2_Pin GPIO_PIN_6
#define AUX2_GPIO_Port GPIOD
#define SPEED_STAGE_0_Pin GPIO_PIN_4
#define SPEED_STAGE_0_GPIO_Port GPIOB
#define DOOR_RUN_Pin GPIO_PIN_6
#define DOOR_RUN_GPIO_Port GPIOB
#define SPEED_STAGE_1_Pin GPIO_PIN_9
#define SPEED_STAGE_1_GPIO_Port GPIOB
#define AUX1_Pin GPIO_PIN_1
#define AUX1_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
