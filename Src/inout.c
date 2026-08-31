#include "stm32f0xx_hal.h"
#include "Global_Var.h"
#include <eprom.h>
#include "Defines.h"

#define   SAFETY_INPUT_SAMPLES         2 // 6.66mS with 3.3mS main cycle, 2 result in 10mS filter at 6.66mS main cycle
#define ORDINARY_INPUT_SAMPLES         9 // 30mS with 3.33mS main cycle
#define EXTENDED_INPUT_SAMPLES        15 // 50mS with 3.33mS main cycle
#define   DIP_SW_INPUT_SAMPLES       120 // 200 with 2mS main cycle
#define   DIP_SW_INPUT_SAMPLES_MAX   120 // 200 with 2mS main cycle
#define   H_OLS_INPUT_SAMPLES        60  // 100 with 2mS main cycle

uint8_t ft_dip_sw3         = 120; // long filter for this reset at power up 11-11-2016
int32_t htest = 0;               // debug test af hard fault
//
// Filter timers (preset to 1 in order
// to update immediately upon power on)
static uint8_t ft_e_stop_pb  = 24; // 40 with 2 mS main cycle
static uint8_t ft_stop_lid_pb  = 24;
static uint8_t ft_stop_safety_chain  = 24; 
//static uint8_t ft_thermo_stop = 24; //22-12-2021 No thermo input in this control
static uint8_t ft_kip_pb   = 60; // moved to inout.c 10-01-2022
	  
uint8_t ft_close_pb = 12;
static uint8_t ft_close_disp_pb = 12;  
	  
static uint8_t ft_ex_auto   = 12;	// not in use yet but preparied for it
static uint8_t ft_ols   = 9;	
static uint8_t ft_cls   = 9;			
static uint8_t ft_dil_sw1  = 50; // long filter  at power up
static uint8_t ft_dil_sw2  = 50; //
static uint8_t ft_dil_sw3  = 50; // 
static uint8_t ft_free_1  = 50; // 	 03-11-2021
static uint8_t ft_free_2  = 50; // 03-11-2021
static uint8_t ft_free_3  = 50; // 03-11-2021
//  
static uint8_t ft_photo1   = 12;
//static uint8_t ft_photo2  = 12; // not available on LCC V4D
static uint8_t ft_dw8k2     = 12;
//static uint8_t ft_ex_dw8k2  = 12; // 10-11-2021
static uint8_t ft_fraba   = 3;
// static uint8_t s_out_tim  = 0; // safety output pulse width 
static uint8_t ft_mon_ph1 = 24; //
static uint8_t ft_mon_ph2 = 24; //
static uint8_t wic_timer = 15; // 12-11-2021
uint8_t mon_ph1; // contactor monitor for welding check
uint8_t mon_ph2; // contactor monitor for welding check
//
extern uint8_t push_buttons;
extern uint8_t gdv1;           // 16-12-2021
//
extern uint8_t WD_activated;   // 1 = Wicket Door activated. 10-11-2021
extern uint8_t SC_activated;   // 1 = slack cable activated. 10-11-2021
extern uint8_t Safety_2_short; // 1 = safety_2 terminals are short-circuit  10-11-2021 
uint8_t wic_slack = 0;         // filter testbit for wicket door and slack rope 11-01-2022
extern uint8_t dock_53_active; // controlbit for active dock leveler control function 24-11-2021
extern uint8_t dock_54_active; // controlbit for active dock leveler control with half open function 23-06-2021
uint8_t h_ols;                 // 1/2 open limit switch after sw-filter 02-12-2021 when free_1 input is used
uint8_t ex_auto;               // external autoclose turn on/off switch - not in use yet but prepaired
uint8_t dock_h_input = 0;      // shared h_ols input and  dock half input 23-06-2021
uint8_t thermo_stop;           // new seperate thermo input 22-12-2021
//
//uint8_t temp; // temp declare is already made 11-10-2017
//*******************************************************************	
void Delay_1uS() // at 48 MHz
{
 for (uint8_t i=0; i<16; i++)
  {
  }		
}
//*******************************************************************

void read_inout()
 {
 // STOP Emergency Button sampling (input active low)
  if (e_stop_pb != HAL_GPIO_ReadPin(EMER_STOP_GPIO_Port, EMER_STOP_Pin) || !--ft_e_stop_pb) 
	 {
    if (!ft_e_stop_pb)
      e_stop_pb = !e_stop_pb;
    if (e_stop_pb)
      ft_e_stop_pb = EXTENDED_INPUT_SAMPLES;
    else
      ft_e_stop_pb = ORDINARY_INPUT_SAMPLES;
   }
	//if (e_stop_pb == 1) led_e_stop_on; // 10-12-2021 10-03-2022 no stop led on this board
  //else led_e_stop_off;	 
// STOP Button sampling (input active low)
  if (stop_pb != HAL_GPIO_ReadPin(STOP_PB_GPIO_Port, STOP_PB_Pin) || !--ft_stop_pb) // 02-11-2021 
	 {
    if (!ft_stop_pb)
      stop_pb = !stop_pb;
    if (stop_pb)
      ft_stop_pb = EXTENDED_INPUT_SAMPLES;
    else
      ft_stop_pb = ORDINARY_INPUT_SAMPLES;
   }
	// if (stop_pb == 1) led_stop_on; disabled  ved debug
	// else led_stop_off;
	 
	 // STOP LID Button sampling (input active high)
  if (stop_lid_pb == HAL_GPIO_ReadPin(STOP_LID_GPIO_Port, STOP_LID_Pin) || !--ft_stop_lid_pb) 
	 {
    if (!ft_stop_lid_pb)
      stop_lid_pb = !stop_lid_pb;
    if (stop_lid_pb)
      ft_stop_lid_pb = EXTENDED_INPUT_SAMPLES;
    else
      ft_stop_lid_pb = ORDINARY_INPUT_SAMPLES;
   }
	//if ((stop_pb == 1) || (stop_lid_pb)) led_stop_on; // 10-12-2021 10-03-2022 no stop led on this board
  //else led_stop_off;	
  //
 	// STOP SAFETY CHAIN sampling (input active high)
  if (stop_safety_chain == HAL_GPIO_ReadPin(SAFETY_CHAIN_GPIO_Port, SAFETY_CHAIN_Pin) || !--ft_stop_safety_chain) 
	 {
    if (!ft_stop_safety_chain)
      stop_safety_chain = !stop_safety_chain;
    if (stop_safety_chain)
      ft_stop_safety_chain = EXTENDED_INPUT_SAMPLES;
    else
      ft_stop_safety_chain = ORDINARY_INPUT_SAMPLES;
   } 
	// Stop, open and close push-button on display unit GDV1 (active high)
	if (gdv1 == 1)
	 {		
	  if ((stop_disp_pb == bit_test(push_buttons,2)) || !--ft_stop_disp_pb) 
	   {
      if (!ft_stop_disp_pb) stop_disp_pb = !stop_disp_pb;
      if (stop_lid_pb) ft_stop_disp_pb = EXTENDED_INPUT_SAMPLES;
      else ft_stop_disp_pb = ORDINARY_INPUT_SAMPLES;
     } 
    // OPEN Button on display unit (input active high)
	  if ((open_disp_pb == bit_test(push_buttons,0)) || !--ft_open_disp_pb) 
	   {
		  if (!ft_open_disp_pb) open_disp_pb = !open_disp_pb;
      ft_open_disp_pb = ORDINARY_INPUT_SAMPLES;
     } 
    // CLOSE Button on display unit (input active high)
    if ((close_disp_pb == bit_test(push_buttons,1)) || !--ft_close_disp_pb)
 	   {
      if (!ft_close_disp_pb) close_disp_pb = !close_disp_pb;
      ft_close_disp_pb = ORDINARY_INPUT_SAMPLES;
     } 
   }
	else
	 {
    stop_disp_pb = 0;  // GDV1 not in use
		open_disp_pb = 0;
    close_disp_pb = 0;		 
	 }		
	 // OPEN Button sampling (input active high)
  if (open_pb == HAL_GPIO_ReadPin(OPEN_PB_GPIO_Port, OPEN_PB_Pin) || !--ft_open_pb) 
	 {
    if (!ft_open_pb)
      open_pb = !open_pb;
    ft_open_pb = ORDINARY_INPUT_SAMPLES;
   }
	
	// CLOSE Button sampling (input active high)
  if (close_pb == HAL_GPIO_ReadPin(CLOSE_PB_GPIO_Port, CLOSE_PB_Pin) || !--ft_close_pb)
 	 {
    if (!ft_close_pb)
      close_pb = !close_pb;
    ft_close_pb = ORDINARY_INPUT_SAMPLES;
   }
	 
//
  if (mon_ph1 == HAL_GPIO_ReadPin(MON_PH1_3_GPIO_Port, MON_PH1_3_Pin) || !--ft_mon_ph1) 
	 {
    if (!ft_mon_ph1)
      mon_ph1 = !mon_ph1;
    ft_mon_ph1 = ORDINARY_INPUT_SAMPLES;
   }	 
//	 
  if (mon_ph2 == HAL_GPIO_ReadPin(MON_PH2_3_GPIO_Port, MON_PH2_3_Pin) || !--ft_mon_ph2) 
	 {
    if (!ft_mon_ph2)
      mon_ph2 = !mon_ph2;
    ft_mon_ph2 = ORDINARY_INPUT_SAMPLES;
   }	 
// KIP status flags?
  if (open_pb || close_pb ) 
	 {
    kip_stop_ready = 0;
    kip_down_ready = 0;
   }
  //*************************************************************************************
	//******************** DIL switch read **********************************
		// DIL1 Switch sampling (input active low)
	if (run_prog == !HAL_GPIO_ReadPin(DIL_SW1_GPIO_Port, DIL_SW1_Pin) || !--ft_dil_sw1) 
	 {
    if (!ft_dil_sw1) run_prog = !run_prog;
    ft_dil_sw1 = DIP_SW_INPUT_SAMPLES_MAX;
   }
	// DIL2 Switch sampling (input active low)
  if (dil_2 == !HAL_GPIO_ReadPin(DIL_SW2_GPIO_Port, DIL_SW2_Pin) || !--ft_dil_sw2) 
	 {
    if (!ft_dil_sw2) dil_2 = !dil_2;
    ft_dil_sw2 = DIP_SW_INPUT_SAMPLES;
   }
  // DIL3 Switch sampling (input active low)
  if (dil_3 == !HAL_GPIO_ReadPin(DIL_SW3_GPIO_Port, DIL_SW3_Pin) || !--ft_dil_sw3) 
		{
    if (!ft_dil_sw3) dil_3 = !dil_3;
    ft_dil_sw3 = DIP_SW_INPUT_SAMPLES;
    }
 //*************************************************************************************
 //***************** FREE INPUT READ ***************************************************		
 // Free input 1 sampling (input active low) This is digital input but can be analog if wanted
	if (free_1 == HAL_GPIO_ReadPin(FREE_IN1_GPIO_Port, FREE_IN1_Pin) || !--ft_free_1) 
	 {
    if (!ft_free_1) free_1 = !free_1;
    ft_free_1 = EXTENDED_INPUT_SAMPLES;
   }		
 // Free input 2 sampling (input active low) - This is digital input but can be analog if wanted
	if (free_2 ==  HAL_GPIO_ReadPin(FREE_IN2_GPIO_Port, FREE_IN2_Pin) || !--ft_free_2) 
	 {
    if (!ft_free_2) free_2 = !free_2;
    ft_free_2 = EXTENDED_INPUT_SAMPLES;
   }			
 // Free input 3 sampling (input active low) - This is digital input but can be analog if wanted
	if ((safety_test != 1) && (safety_after_time == 0))// only when not safety test itself 29-05-2024 
	{
	 if (free_3 == HAL_GPIO_ReadPin(FREE_IN3_GPIO_Port, FREE_IN3_Pin) || !--ft_free_3) 
	  {
     if (!ft_free_3) free_3 = !free_3;
     ft_free_3 = EXTENDED_INPUT_SAMPLES;
    }
	}
// External auto close switch is not made on this control cause there is free input available for this if needed.
//	
// KIP/GO Button sampling (input active high)
  if (kip_pb == HAL_GPIO_ReadPin(GO_FUNCTION_GPIO_Port, GO_FUNCTION_Pin) || !--ft_kip_pb) 
	 {
    if (!ft_kip_pb)
     kip_pb = !kip_pb;
     ft_kip_pb = ORDINARY_INPUT_SAMPLES;
   }
//	 
  if (kip_pb == 1)  kip_pb = 1; // Radio function use the same input - Go function (kip) 10-01-2022
  if (kip_pb == 0) kip_pb = 0; // Radio function use the same - Go function (kip) 	 
//	EE_read(EE_par_88); // overwrite ex_edge_state, if automatic door lock is selected 10-06-2011
//  if (temp == 3)
//   {
//    if (ref_sw == 1) lock_switch = 1; // switch activated
//    else lock_switch = 0; //
//   } 
//
	if (ex_auto == !HAL_GPIO_ReadPin(EX_AUTO_GPIO_Port, EX_AUTO_Pin) || !--ft_ex_auto) // 22-11-2017 19-03-2019 27-01-2022
   {
	  if (!ft_ex_auto)
     ex_auto = !ex_auto;
     ft_ex_auto = EXTENDED_INPUT_SAMPLES;
	 } 
//
	EE_read(EE_par_16); 
	if (temp == 1) // mechanical 1/2 open limit wanted
   {
	  h_ols = !free_1; // 22-11-2023 free_1 to h_ols - inverted because NC switch 	
   } 
	else if (temp > 1)
   {
    e_h_ols_sw = free_1; // e_h_ols_sw is a NO switch and free in 1 get high when switch is closed 22-11-2023
   }
	else if ((dock_54_active == 1) || (dock_53_active == 1))// 23-06-2021 parameter 16 = 0 29-10-2021 22-11-2023 
   {
    dock_h_input = free_1; // 29-10-2021 er inverteret
    h_ols = 0; // to prevent false in interpreter
   }
  else h_ols = 0;
//	 
if (E_limit == 0) // mechanical limits selected - overrule ols_out with terminal inputs 09-01-2017 10-03-2022 No mechanical lits in this control
 {
	if ((safety_test == 0) && (safety_after_time == 0)) // wait using these values until safety test is ended. 
		                                                  //Else the CLS active will stop the test 03-04-2024
	 {		
    ols_out = !free_2; // 22-11-2023
	  cls_out = !free_3; // 22-11-2023
	 }
 }	
if ((ols == ols_out) || (!--ft_ols)) 
 {
  if (!ft_ols)
  ols = !ols;
	if (E_limit == 0) ft_ols = 1; // with 1 = 1 x 3.33mS = 3.3mS filter (free input for mechanical limits is already filtered) 03-04-2024
	else ft_ols = 6; // with 6 = 6 x 3.33mS = 20mS filter (new positions is comming every 6.5mS)
 }
//
if ((cls == cls_out) || (!--ft_cls)) 
 {
  if (!ft_cls)
  cls = !cls;
	if (E_limit == 0) ft_cls = 1; // with 1 = 1 x 3.33mS = 3.3mS filter (free input for mechanical limits is already filtered) 03-04-2024
  else ft_cls = 6; // with 6 = 6 x 3.33mS = 20mS filter
 }

//
  if (cls == 1) 
  {
   kip_stop_ready = 0; // 
   kip_down_ready = 0; // 
  }	 
	 
 // Photo sensor update
  
    // Photo-1 sampling (input active low)
	temp = EE_read(EE_par_31);
  if ((temp == 1) || (temp == 3) || (temp == 4) || (temp > 5))
	 {
    if (photo1 != HAL_GPIO_ReadPin(SAFETY_PHOTO_GPIO_Port, SAFETY_PHOTO_Pin) || !--ft_photo1)
  	 {
      if (!ft_photo1)
        photo1 = !photo1;
      ft_photo1 = SAFETY_INPUT_SAMPLES;
     }
   }
	else
   photo1 = 0; // Photo-1 not selected 
//	
// Safety edge dw/8k2 update 
 if ((dw8k2 == edge_state) || (!--ft_dw8k2)) 
	{
   if (!ft_dw8k2)
    dw8k2 = !dw8k2;
    ft_dw8k2 = SAFETY_INPUT_SAMPLES;
  }
//
 if (wic_timer != 0) wic_timer--;	// 12-11-2021
 if (wic_slack == 1) // filter timer for wicket door slack rope input 12-11-2018
 {
  if ((WD_activated == 1) | (SC_activated == 1) | (Safety_2_short == 1)) wic_timer = 16; // preset to 50 mS
  else if (wic_timer == 0)
   {
    wic_slack = 0;
    wic_timer = 16;
   }
 }
else
 {
  if ((WD_activated == 0) & (SC_activated == 0) & (Safety_2_short == 0)) wic_timer = 16; // preset to 50 mS
  else if (wic_timer == 0)
   {
    wic_slack = 1;
    wic_timer = 16;
   }
 }	 
 //	 
 if (fraba_timer == 0) fraba_pulse = 0; // if timer = 0, fraba is aktive 
 //
 if (fraba == 1)
  {
   if (fraba_pulse == 0)
    {
     if (HAL_GPIO_ReadPin(OPTICAL_EDGE_GPIO_Port, OPTICAL_EDGE_Pin)== 0) // is input low
      {
       if (ft_fraba == 0)
        {
         EE_read(EE_par_22);
         if (temp > 0)
          {
           // E1_mon = 1; // set monitor fail - not usefull yet
           ft_fraba = 2; // 
          }
         else ft_fraba = 2; //
        }
      }
     else ft_fraba = 2; // reload filtertimer with 6.66 mS  
    }
   else
    {
     if (ft_fraba == 0)
      {
       fraba = 0;
      }
    }
  }
 else
  {
   if (fraba_pulse == 0)
    {
     if (ft_fraba == 0)
        {
         fraba = 1;
         if (HAL_GPIO_ReadPin(OPTICAL_EDGE_GPIO_Port, OPTICAL_EDGE_Pin)== 0) // is input low
          {
           EE_read(EE_par_22);
           if (temp > 0)
            {
             // E1_mon = 1; // set monitor fail set monitor fail - not usefull yet
            }
          }
         else ft_fraba = 2; //  reload filtertimer with 6.66 mS  15-11-2010
        }
    }
   else ft_fraba = 2; //  reload filtertimer with 4 mS  15-11-2010
  }
	if (ft_fraba !=0) ft_fraba--;
	
	
	if (edge_photo_owr == 1) // overrule if push code is activated 10-03-2010 
  {
   photo1 = 0;
   photo2 = 0;
   dw8k2 = 0;
	 ex_dw8k2 = 0;	
   fraba = 0;
  }
	
	
	
 //*********************************************************
                                     // else p_value will be deleted by display_run
 if (run_prog == 0) run_prog_tb = 0; // testbit for just programming select
 //
 // if ((run_prog == 0) && ((parameter == 13) || (parameter == 15))) parameter = 1; // else ecode fail will hold stop
 //
 		
}
