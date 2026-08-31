#include "stm32f0xx_hal.h"
#include <Defines.h>
#include "Global_Var.h"
#include "eprom.h"
#include "modbus.h"
#include "stm32fxx_STLmain.h"

#define kostal  0x01            // Kostal enkoder valgt
uint8_t turn_off_filter = 2;     // filter for turn off inverter safety
uint8_t stop_active_tb;				  // Testbit for push-button released
uint8_t push_inv_reset_time = 30; // filter for inverter power restart 14-02-2023
extern uint32_t position_old ;  // tidligere position til stop hvis position ikke ændrer sig
extern uint8_t wic_slack;       // filter testbit for wicket door and slack rope 
extern uint8_t e_type;					// selected Encoder type
extern uint8_t timeout_modbus;  // timeout for init modbus by powerup
extern uint16_t inv_timeout;     // inverter timeout for safety 850mS at power-up 22-11-2024
extern uint16_t powerup_tim;     // 22-11-2024
extern IWDG_HandleTypeDef hiwdg;
#include "stm32fxx_STLlib.h"
//
void restart_modbus() // 14-02-2023
 {
	timeout_modbus = 0; // 
	e27_modbus = 0; // 
	digit_1_on; // T9 OFF turn off LED display until right update is ready
	digit_2_on; // T8 OFF
	digit_3_on; // T7 OFF
	digit_4_on; // T5 OFF
	// Setup 5 sec period - temporary to prevent timeout by modbus init.
  IwdgHandle.Instance = IWDG;
  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_256;
  IwdgHandle.Init.Reload = 625; // changed from 32 
  if( HAL_IWDG_Init(&IwdgHandle) != HAL_OK )
    {
     FailSafePOR();
    }
    __HAL_IWDG_START(&IwdgHandle); 
	//
  for (int i=0; i<200; i++) // 2.0 sec. wait for inverter powerup. 1.9 sec. is minimum
	                          // for M100 before the inverter is ready. 
    {
     HAL_Delay(10); //  10mS steps
		}			
	HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout debug test 
	do
	{
	 timeout_modbus++;
	}
	while ((timeout_modbus < 4) && (initModbusParam() != HAL_OK)); // 14-02-2023  Max 3 init cycles
	//	
	if (timeout_modbus > 3) // more than 7.5 sec. with no answers
	 {
		e27_modbus = 1; // check modbus connection + new powerup // 
	 } 
	// Setup back to 256 mS period as in STLmain.c
  IwdgHandle.Instance = IWDG;
  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_256;
  IwdgHandle.Init.Reload = 32; // changed from 32 
  if( HAL_IWDG_Init(&IwdgHandle) != HAL_OK )
    {
     FailSafePOR();
    }
    __HAL_IWDG_START(&IwdgHandle); 
	// 
  inv_timeout = 1501; // prevent ecode1 fail stop after restart modbus		22-11-2024	
  }
//
void prepare_turn_off()
 {
  if (turn_off_filter == 0)
	 {
		door_up_off; // 28-03-2023 - up relay is used for power to inverter
	 }
	else
	 {
    turn_off_filter--;
		inv_timeout = 90; 
   }		
 }	
//
void push_button_test()
{
 if ((((stop_pb == 1) && (e_stop_pb == 0)) | (stop_lid_pb == 1) | (stop_disp_pb == 1)) &&
	 (stop_active_tb == 0) && (door_up == 0) && ((safety_test != 1) && (safety_after_time == 0))) // 14-02-2023
  {
	 if (push_inv_reset_time !=0) push_inv_reset_time--;
   if (push_inv_reset_time == 0)
	  {		 
	   door_up_on; // turn ON power to inverter.
     stop_active_tb = 1;
     e32_weld = 0; // clear this if this is set. it used with non-inverter use 14-02-2023			
     restart_modbus();		// 14-02-2023
	  }
  }
 else
  {
	 push_inv_reset_time = 30; // 100mS filter for reset	
   if ((stop_pb == 0) && (stop_lid_pb == 0) && (stop_disp_pb == 0))
    {
     stop_active_tb = 0; 
    }
   else stop_active_tb = 1; // one of these stop PB is not 0 - don't restart inverter		
  }
}	
 
// ***************************************************************
// ***************************************************************
void inverter_safety()
{
 push_button_test();
//	
 EE_read(EE_limit_ready); // update ecode2 in case of not answers from encoder 25-11-2024
 if (temp == 255) ecode2 = 1;
 else ecode2 = 0;	
//	
 if (run_prog == 0)
  {
   contactor_off; // not in use  - debug with contactor
	 door_down_off; // not in use
   if ((e_stop_pb == 1) | (e21_fail == 1) | (ecode7 == 1) | ((ecode1 == 1) && (ecode2 == 0)) | (ecode3 == 1)) // 21-11-2024
	  {
		 if ((ecode3 == 1) && (inv_timeout > 10))
		  {
       inv_timeout = 10; // reduce reactiontime to 2 x 33 mS to a special short time for this fail (movement but should be stopped)
		  }			 
     if (inv_timeout == 0)
		  {
			 prepare_turn_off();	
			}
		 else
		  {
       inv_timeout--;
		  }			 
    }			
   else
	  {
     if (powerup_tim == 0) inv_timeout = 90; // On power-up shall not be preset
     turn_off_filter = 2; 					 
	  }
  }	 
 else
  {
		
  }	 
}

