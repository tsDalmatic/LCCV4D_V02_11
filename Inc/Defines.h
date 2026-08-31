
#include "stm32f0xx_hal.h"
//
#define main_version 2 // Software version adjust. Change this when software is changed
#define sub_version 10  // Software version adjust. Change this when software is changed
#define spec_factory 0 // set this 1, 2 or other number if special factory settting is used. 
#define version 1      // 1 = Version with welding/inverter check with encoder or tacho made in encoder.c 26-01-2022
                       // 0 = without welding check
#define condoor 0      // 1 = CLS is learned on the floor - see encoder.c
                       // 0 = ClS is learned 3-5 cm from floor. 
#define para22_aut 0   //	0 = No automatic set of this value 22-11-2024
                       // 1 = Automatic set of this value by learning limit process to 25 or 50 value when Kostal enk. used
//
// a=target variable, b=bit number to act upon 0-n 
#define bit_set(a,b) (a |= (1<<(b)))
#define bit_clear(a,b) (a &= ~(1<<(b)))
#define bit_flip(a,b) (a ^= (1<<(b)))
#define bit_test(a,b) (((a)>>(b)) & 1)
#define true 1
#define false 0
// 
#define contactor HAL_GPIO_ReadPin(K1_CTRL_GPIO_Port, K1_CTRL_Pin)
#define contactor_on HAL_GPIO_WritePin(K1_CTRL_GPIO_Port, K1_CTRL_Pin, GPIO_PIN_SET)
#define contactor_off HAL_GPIO_WritePin(K1_CTRL_GPIO_Port, K1_CTRL_Pin, GPIO_PIN_RESET)
//
// tenary operator bruges - (betinget udtryk) used for phase invert function 03-03-2017
#define door_up (phase_inv ? HAL_GPIO_ReadPin(DOOR_DOWN_GPIO_Port, DOOR_DOWN_Pin) : \
                             HAL_GPIO_ReadPin(DOOR_UP_GPIO_Port, DOOR_UP_Pin))

#define door_up_on (phase_inv ? HAL_GPIO_WritePin(DOOR_DOWN_GPIO_Port, DOOR_DOWN_Pin, GPIO_PIN_SET) : \
                                HAL_GPIO_WritePin(DOOR_UP_GPIO_Port, DOOR_UP_Pin, GPIO_PIN_SET))

#define door_up_off (phase_inv ? HAL_GPIO_WritePin(DOOR_DOWN_GPIO_Port, DOOR_DOWN_Pin, GPIO_PIN_RESET) : \
                                 HAL_GPIO_WritePin(DOOR_UP_GPIO_Port, DOOR_UP_Pin, GPIO_PIN_RESET))
//
#define door_down (phase_inv ? HAL_GPIO_ReadPin(DOOR_UP_GPIO_Port, DOOR_UP_Pin) : \
                               HAL_GPIO_ReadPin(DOOR_DOWN_GPIO_Port, DOOR_DOWN_Pin))

#define door_down_on (phase_inv ? HAL_GPIO_WritePin(DOOR_UP_GPIO_Port, DOOR_UP_Pin, GPIO_PIN_SET) : \
                                  HAL_GPIO_WritePin(DOOR_DOWN_GPIO_Port, DOOR_DOWN_Pin, GPIO_PIN_SET))

#define door_down_off (phase_inv ? HAL_GPIO_WritePin(DOOR_UP_GPIO_Port, DOOR_UP_Pin, GPIO_PIN_RESET) : \
                                   HAL_GPIO_WritePin(DOOR_DOWN_GPIO_Port, DOOR_DOWN_Pin, GPIO_PIN_RESET))

//
#define speed_stage_1 HAL_GPIO_ReadPin(SPEED_STAGE_1_GPIO_Port, SPEED_STAGE_1_Pin)
#define speed_stage_1_on HAL_GPIO_WritePin(SPEED_STAGE_1_GPIO_Port, SPEED_STAGE_1_Pin, GPIO_PIN_SET)
#define speed_stage_1_off HAL_GPIO_WritePin(SPEED_STAGE_1_GPIO_Port, SPEED_STAGE_1_Pin, GPIO_PIN_RESET)
#define speed_stage_0 HAL_GPIO_ReadPin(SPEED_STAGE_0_GPIO_Port, SPEED_STAGE_0_Pin)
#define speed_stage_0_on HAL_GPIO_WritePin(SPEED_STAGE_0_GPIO_Port, SPEED_STAGE_0_Pin, GPIO_PIN_SET)
#define speed_stage_0_off HAL_GPIO_WritePin(SPEED_STAGE_0_GPIO_Port, SPEED_STAGE_0_Pin, GPIO_PIN_RESET)
#define door_run HAL_GPIO_ReadPin(DOOR_RUN_GPIO_Port, DOOR_RUN_Pin)
#define door_run_on HAL_GPIO_WritePin(DOOR_RUN_GPIO_Port, DOOR_RUN_Pin, GPIO_PIN_SET)
#define door_run_off HAL_GPIO_WritePin(DOOR_RUN_GPIO_Port, DOOR_RUN_Pin, GPIO_PIN_RESET)
#define emergency_out HAL_GPIO_ReadPin(EMERGENCY_OUT_GPIO_Port, EMERGENCY_OUT_Pin)
#define emergency_out_on HAL_GPIO_WritePin(EMERGENCY_OUT_GPIO_Port, EMERGENCY_OUT_Pin, GPIO_PIN_SET)
#define emergency_out_off HAL_GPIO_WritePin(EMERGENCY_OUT_GPIO_Port, EMERGENCY_OUT_Pin, GPIO_PIN_RESET)
//
#define option HAL_GPIO_ReadPin(OPTION_GPIO_Port, OPTION_Pin)
#define option_on HAL_GPIO_WritePin(OPTION_GPIO_Port, OPTION_Pin, GPIO_PIN_SET)
#define option_off HAL_GPIO_WritePin(OPTION_GPIO_Port, OPTION_Pin, GPIO_PIN_RESET)
//
#define safety_test_on HAL_GPIO_WritePin(SAFETY_TEST_GPIO_Port, SAFETY_TEST_Pin, GPIO_PIN_SET)
#define safety_test_off HAL_GPIO_WritePin(SAFETY_TEST_GPIO_Port, SAFETY_TEST_Pin, GPIO_PIN_RESET)
#define safety_test HAL_GPIO_ReadPin(SAFETY_TEST_GPIO_Port, SAFETY_TEST_Pin)
//
#define aux1_on HAL_GPIO_WritePin(AUX1_GPIO_Port, AUX1_Pin, GPIO_PIN_SET)
#define aux1_off HAL_GPIO_WritePin(AUX1_GPIO_Port, AUX1_Pin, GPIO_PIN_RESET)
#define aux1 HAL_GPIO_ReadPin(AUX1_GPIO_Port, AUX1_Pin)
//
#define aux2_on HAL_GPIO_WritePin(AUX2_GPIO_Port, AUX2_Pin, GPIO_PIN_SET)
#define aux2_off HAL_GPIO_WritePin(AUX2_GPIO_Port, AUX2_Pin, GPIO_PIN_RESET)
#define aux2 HAL_GPIO_ReadPin(AUX2_GPIO_Port, AUX2_Pin)
//
#define aux4_on HAL_GPIO_WritePin(LAMP_CLK_GPIO_Port, LAMP_CLK_Pin, GPIO_PIN_SET)
#define aux4_off HAL_GPIO_WritePin(LAMP_CLK_GPIO_Port, LAMP_CLK_Pin, GPIO_PIN_RESET)
#define aux4 HAL_GPIO_ReadPin(LAMP_CLK_GPIO_Port, LAMP_CLK_Pin)
//
#define aux5_on HAL_GPIO_WritePin(LAMP_DATA_GPIO_Port, LAMP_DATA_Pin, GPIO_PIN_SET)
#define aux5_off HAL_GPIO_WritePin(LAMP_DATA_GPIO_Port, LAMP_DATA_Pin, GPIO_PIN_RESET)
#define aux5 HAL_GPIO_ReadPin(LAMP_DATA_GPIO_Port, LAMP_DATA_Pin)
//
// Digit 1 to 4 are inverted to fit with old V7E SR software
#define digit_1_on HAL_GPIO_WritePin(DIGIT_4_GPIO_Port, DIGIT_4_Pin, GPIO_PIN_SET)
#define digit_1_off HAL_GPIO_WritePin(DIGIT_4_GPIO_Port, DIGIT_4_Pin, GPIO_PIN_RESET)
#define digit_1 HAL_GPIO_ReadPin(DIGIT_4_GPIO_Port, DIGIT_4_Pin)
//
#define digit_2_on HAL_GPIO_WritePin(DIGIT_3_GPIO_Port, DIGIT_3_Pin, GPIO_PIN_SET)
#define digit_2_off HAL_GPIO_WritePin(DIGIT_3_GPIO_Port, DIGIT_3_Pin, GPIO_PIN_RESET)
#define digit_2 HAL_GPIO_ReadPin(DIGIT_3_GPIO_Port, DIGIT_3_Pin)
//
#define digit_3_on HAL_GPIO_WritePin(DIGIT_2_GPIO_Port, DIGIT_2_Pin, GPIO_PIN_SET)
#define digit_3_off HAL_GPIO_WritePin(DIGIT_2_GPIO_Port, DIGIT_2_Pin, GPIO_PIN_RESET)
#define digit_3 HAL_GPIO_ReadPin(DIGIT_2_GPIO_Port, DIGIT_2_Pin)
//
#define digit_4_on HAL_GPIO_WritePin(DIGIT_1_GPIO_Port, DIGIT_1_Pin, GPIO_PIN_SET)
#define digit_4_off HAL_GPIO_WritePin(DIGIT_1_GPIO_Port, DIGIT_1_Pin, GPIO_PIN_RESET)
#define digit_4 HAL_GPIO_ReadPin(DIGIT_1_GPIO_Port, DIGIT_1_Pin)
//
#define disp_data_on HAL_GPIO_WritePin(DISPLAY_DATA_GPIO_Port, DISPLAY_DATA_Pin, GPIO_PIN_SET)
#define disp_data_off HAL_GPIO_WritePin(DISPLAY_DATA_GPIO_Port, DISPLAY_DATA_Pin, GPIO_PIN_RESET)
#define disp_data HAL_GPIO_ReadPin(DISPLAY_DATA_GPIO_Port, DISPLAY_DATA_Pin)
//
#define disp_clk_on HAL_GPIO_WritePin(DISPLAY_CP_GPIO_Port, DISPLAY_CP_Pin, GPIO_PIN_SET)
#define disp_clk_off HAL_GPIO_WritePin(DISPLAY_CP_GPIO_Port, DISPLAY_CP_Pin, GPIO_PIN_RESET)
#define disp_clk HAL_GPIO_ReadPin(DISPLAY_CP_GPIO_Port, DISPLAY_CP_Pin)
//
#define disp_str_on HAL_GPIO_WritePin(DISPLAY_STR_GPIO_Port, DISPLAY_STR_Pin, GPIO_PIN_SET)
#define disp_str_off HAL_GPIO_WritePin(DISPLAY_STR_GPIO_Port, DISPLAY_STR_Pin, GPIO_PIN_RESET)
#define disp_str HAL_GPIO_ReadPin(DISPLAY_STR_GPIO_Port, DISPLAY_STR_Pin)
//
#define led_error_on HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET)
#define led_error_off HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET)
#define led_error HAL_GPIO_ReadPin(LED_ERROR_GPIO_Port, LED_ERROR_Pin)
//
#define watchdog_on HAL_GPIO_WritePin(UP_RUNNING_GPIO_Port, UP_RUNNING_Pin, GPIO_PIN_SET)
#define watchdog_off HAL_GPIO_WritePin(UP_RUNNING_GPIO_Port, UP_RUNNING_Pin, GPIO_PIN_RESET)
#define tst_watchdog HAL_GPIO_ReadPin(UP_RUNNING_GPIO_Port, UP_RUNNING_Pin)
//
#define led_stop_on HAL_GPIO_WritePin(LED_STOP_GPIO_Port, LED_STOP_Pin, GPIO_PIN_SET)
#define led_stop_off HAL_GPIO_WritePin(LED_STOP_GPIO_Port, LED_STOP_Pin, GPIO_PIN_RESET)
#define led_stop HAL_GPIO_ReadPin(LED_STOP_GPIO_Port, LED_STOP_Pin)
//
#define led_e_stop_on HAL_GPIO_WritePin(LED_E_STOP_GPIO_Port, LED_E_STOP_Pin, GPIO_PIN_SET)
#define led_e_stop_off HAL_GPIO_WritePin(LED_E_STOP_GPIO_Port, LED_E_STOP_Pin, GPIO_PIN_RESET)
#define led_e_stop HAL_GPIO_ReadPin(LED_E_STOP_GPIO_Port, LED_E_STOP_Pin)
//
#define lamp_dat_on HAL_GPIO_WritePin(LAMP_DATA_GPIO_Port, LAMP_DATA_Pin, GPIO_PIN_SET)
#define lamp_dat_off HAL_GPIO_WritePin(LAMP_DATA_GPIO_Port, LAMP_DATA_Pin, GPIO_PIN_RESET)
//
#define led_pad_on HAL_GPIO_WritePin(LED_LEDPAD_GPIO_Port, LED_LEDPAD_Pin, GPIO_PIN_SET)
#define led_pad_off HAL_GPIO_WritePin(LED_LEDPAD_GPIO_Port, LED_LEDPAD_Pin, GPIO_PIN_RESET)
//
#define lamp_clk_on HAL_GPIO_WritePin(LAMP_CLK_GPIO_Port, LAMP_CLK_Pin, GPIO_PIN_SET)
#define lamp_clk_off HAL_GPIO_WritePin(LAMP_CLK_GPIO_Port, LAMP_CLK_Pin, GPIO_PIN_RESET)
//
#define lamp_dat HAL_GPIO_ReadPin(LAMP_DATA_GPIO_Port, LAMP_DATA_Pin)
//
#define usart2_de_man_on HAL_GPIO_WritePin(USART2_DE_MAN_GPIO_Port, USART2_DE_MAN_Pin, GPIO_PIN_SET)
#define usart2_de_man_off HAL_GPIO_WritePin(USART2_DE_MAN_GPIO_Port, USART2_DE_MAN_Pin, GPIO_PIN_RESET)
//
#define usart6_de_man_on HAL_GPIO_WritePin(USART6_DE_MAN_GPIO_Port, USART6_DE_MAN_Pin, GPIO_PIN_SET)
#define usart6_de_man_off HAL_GPIO_WritePin(USART6_DE_MAN_GPIO_Port, USART6_DE_MAN_Pin, GPIO_PIN_RESET)
//
#define sp_delay 120 // speed delay setup, 120 = 0.8 sec
#define opd_value_1 2 // opdate value interval for adaptive speed - normal 10 debug
#define t_dw 1     // 
#define t_8k2 2    // 
#define t_fraba 3  // 
#define t_lindab 4 //
#define autoclose_max 990 // highest autoclose value
//
#define indi_time 225 // 225 x 6.66mS = 1.5 sec.


