
#include "stm32f0xx_hal.h"
#include "Global_Var.h"
#include <Defines.h>
#include <main.h>
#include <eprom.h>

extern void rev2(void);
extern void stop_open_or_close(void);
extern void r_autoclose_timer(void);
extern uint8_t open_active(void);           // special inverter functions
extern uint8_t close_active(void);          // special inverter functions
extern uint16_t pwr_timer_SER;
uint8_t autoclose_ext;             // time extender for autoclosing
uint16_t photo_active_time;        // photo beam break time for car wash function
uint8_t contactor_off_tim = 7;  	 // preset at startup to prevent uintended activate 05-11-2021
uint8_t up_relay_tim;              // 05-11-2021
uint8_t down_relay_tim;            // 05-11-2021
uint8_t con_on_delay;              // 05-11-2021
uint8_t weldtimer = 60;            //
uint16_t count_timer;              // showing time for upper and lower digits of the count timer used with LED display
uint16_t warn_tim_delay = 0;       // 18-11-2021
uint16_t powerup_tim = 606;        // 01-12-2021 2 sec. powerup timer by 3.3 mS cycle time
uint8_t edge_select_tim = 60;      // 02-12-2021
uint16_t tim_rev;                  // 03-04-2024 3 sec. edge reverse time if selected in parameter 55 
extern uint32_t cls_trafic_tim;    // 17-11-2021
extern uint8_t count_msd; // 
extern uint16_t fail_12v_show;     // timer for 12 og 24 V fail showing
extern uint8_t car_wash_active;    // test bit for car wash function active above half open 02-12-2021
extern uint8_t h_ols;              // 1/2 open limit switch after sw-filter 02-12-2021
extern uint8_t inv_meas_interval;  // 03-12-2021
extern uint16_t fault_delay_1;     // 28-03-2025 delay for relay on by error cause of safety test
extern uint16_t fault_delay_2;     // 28-03-2025 delay for relay on by error cause of safety test
extern uint16_t fault_delay_4;     // 28-03-2025 delay for relay on by error cause of safety test
extern uint16_t fault_delay_5;     // 28-03-2025 delay for relay on by error cause of safety test
//
//**************************************************************************
//
//**************************************************************************
void mtimer() // multitimer funktion der kan bruges når man benytter fast programgennemløbstid
{
	static uint8_t second_time = 0; // testbit for extending time
	
	
	
 if (fraba_timer !=0) fraba_timer--;
 //
 if (count_timer != 0) count_timer--;
 else
  {
   count_timer = 300; // 300 * 3.33mS = 1 sec
   count_msd = ~count_msd; // toggle
  }	
 //
 //	
 if (ch_timer !=0) ch_timer--;
 if (push_timer !=0) push_timer--;
 //
 if (run_timer !=0)
  {   
   if (second_time == 1) run_timer--; // second time testbit is used to double the time
   second_time = !second_time;
  }
 if (rev_time !=0) rev_time--;
 if (rev_time == 0) // only decrement re_open_timer when reverse time = 0
  {
   if (re_open_tim !=0) re_open_tim--; // 18-02-2011
   if (re_open_tim == 0) re_open_tb = 0; // 18-02-2011
  }
 //
 if (rev_time == 0) // special pull_back_time control
  {
   if (pull_back_time == 0)
    {
    }
   else
    {
     pull_back_time--;
     if (pull_back_time == 0)
      {
       if (code_spd_rev == 1)
        {
         if (((close_pb) || (close_disp_pb)) & (cls == 0)) imp_open = 0; //clear impulse open when speed reversebit is set
        }
      }
    }
  }
 //
 if (afterrun_time !=0) afterrun_time--;
 if (edge_fail_timer !=0) edge_fail_timer--;
 if (warn_timer == 0) warn_started = 0;
 if (warn_timer > 0) warn_tim_delay = 400; // 0.8 sec. delay to prevent short flash when warning ended. 28-06-2021
 if (warn_tim_delay !=0) warn_tim_delay--; // 28-06-2021	
 read_int16_eeprom(EE_par_32);
 temp2 = data;
 EE_read(EE_par_16); // 08-12-2025
 temp3 = temp;
 EE_read(EE_par_17);
 if ((temp2 > 0) && (autoclose_timer > 0) && (ols == 1)) // 08-12-2025
  {
  }
 else if ((temp2 > 0) && (autoclose_timer > 0) && (temp3 == 1) && (temp == 1) && (h_ols == 1)) // 08-12-2025
  {
  }
 else if ((temp2 > 0) && (autoclose_timer > 0) && (temp3 > 1) && (temp == 1) && (e_h_ols_sw == 1)) // 08-12-2025
  {
  }
 else	
  {	
   if (warn_timer !=0) warn_timer--;
	}
//
EE_read(EE_par_16);
if ((temp > 3) && (e_h_ols_sw == 1)) // 04-08-2022  halv open push must clear autoclose timer for start warning can work 09-12-2025
 {
  autoclose_timer = 0;
  interlock = 0; //
 }
if ((temp == 3) && (e_h_ols_sw == 1) && (ols == 1)) //  halv open push must clear autoclose timer for start warning can work 09-12-2025
 {
  autoclose_timer = 0;
  interlock = 0; //
 }	
//	
 if ((tighten == 1) & (tighten_ready == 1))
  {
   if (tighten_time == 0)
    {
     //open_gate_off;
		 stop_open_or_close();	
     imp_open = 0;
     tighten = 0;
     rev2(); // preset reverse time
    }
   else tighten_time--;
  }
 EE_read(EE_par_29);
 if (temp == 0) tighten_time = 0; // 28-04-2010 for secure not impulse open by no wire tighten 
//********
if (ex_auto_sw == 1) r_autoclose_timer(); // No Autoclose switch in this control
if ((close_pb == 1) || (close_disp_pb)) // 08-07-2026
 {
  autoclose_timer = 0; // 01-12-2021
  interlock = 0; //01-12-2021
 }	
if (autoclose_ext == 0)
 {
  autoclose_ext = 5; // changed from 9 to 5 after main cycle change from 2 mS to 3.33mS
  EE_read(EE_par_33);
  if (temp > 0) 
   {
    if (photoclose == 1)
     {
      if ((autoclose_timer != 0) && (interlock == 0)) autoclose_timer--; // car wash function selected and photoclose testbit active 01-12-2021
     }
   }
  else
   {
    if ((autoclose_timer != 0) && (interlock == 0)) autoclose_timer--; // car wash function not selected 01-12-2021
   }
 } 
else autoclose_ext--; 
//********
if (force_closing_time_ext == 0) // 23-08-2011
 {
  force_closing_time_ext = 150; // 150 = 0.5 sec at 3.33mS main cycle
  EE_read(EE_par_34);
  if (((temp > 0) && (temp < 5)) || (temp > 5)) // 15-08-2012
   {
    if (force_closing_time != 0) force_closing_time--; // car wash function selected force_closing_time is wanted
   }
  else force_closing_time =240; // force closing is disabled in parameter 34 , preset to value > 0
 } 
else force_closing_time_ext--; 

//********
 if (safety_after_time != 0) safety_after_time--;
if (confirm_learn == 1)
 {
  confirm_timer = 300; // 1 sec. confirming
  confirm_learn = 0;
 }
if (confirm_timer != 0) confirm_timer--; //  
if (flash_timer == 0)
 {
  if ((relearn_ph1 == 1) | (relearn_ph2)) flash_timer = 150; // longer flash here 06-10-2010
  else flash_timer = 90;
  d_flash_tim = !d_flash_tim; // toggle every 300 mS
 } 
else flash_timer--;
if (confirm_timer > 0) d_flash_tim = 1; // overwrite flashing when confirming learned
//
if (spec_edge_tim != 0) spec_edge_tim--; //  spec_edge_timer before error set
if (edge_err_show != 0) edge_err_show--; // edge setup error showing i prog mode
//
if (speed_delay !=0) speed_delay--; //
if (speed_f_timer !=0) speed_f_timer--; //
if (speed_test > 0) speed_test--; // debug
else
 {
 speed_test = 800;
 }
if ((a_speed < 2088) && (speed_test < 600))
 {
  speed_test = 720;
 }
//*******
EE_read(EE_par_21); // 12-04-2018
if ((temp == 6) && (fraba == 1)) // RAYLG selected and connected to optical edge input 12-04-2018
 {
  if ((ols == 1) |  ((open_active() == 1) & (car_wash_active == 1))) // 17-06-2020 02-12-2021
   {
    if (photo_active_time != 0) photo_active_time--; //
    if (photo_active_time == 0) photoclose = 1;
   } 
 } 
else 
 {
  EE_read(EE_par_31); // 26-05-2010
  if (temp > 3)
   {
    if (((photo1 == 1) | (photo2 == 1)) & (photo_pos_observed == 0))// Car wash function downcount only when
                                                                 // photo in frame is released 19-04-2010
     {
      EE_read(EE_par_16); // 01-02-2016
      if ((ols == 1) | ((temp < 2) && (h_ols == 1)) | ((open_active() == 1) & (car_wash_active == 1))) // 02-12-2021
       {
        if ((photo_active_time != 0) && (safety_test != 1) && (safety_after_time == 0)) photo_active_time--; // 29-04-2019 
        if (photo_active_time == 0) photoclose = 1;
       }
     }
    else
     {
      EE_read(EE_par_33);
      temp2 = temp; // prepare for multiply 16 bit
  		if (temp2 == 1) photo_active_time = 6; // reload photo active timer with minimum 20mS 21-09-2018
      else photo_active_time = (temp2 * 30); // reload photo active timer 21-09-2018		 
     }
   }
  else
   {
    if ((photo1 == 1) | (photo2 == 1))// do not take care of photo in frame  26-05-2010
     {
      EE_read(EE_par_16); // 01-02-2016
      if ((ols == 1) | ((temp < 2) && (h_ols == 1))| ((open_active() == 1) & (car_wash_active == 1))) // 02-12-2021
       {
			  if ((photo_active_time != 0) && (safety_test != 1) && (safety_after_time == 0)) photo_active_time--; // 21-09-2018 
        if (photo_active_time == 0) photoclose = 1;
       }
     }
    else
     {
      EE_read(EE_par_33);
      temp2 = temp; // prepare for multiply 16 bit
		  if (temp2 == 1) photo_active_time = 6; // reload photo active timer with minimum 20mS 21-09-2018 
      else photo_active_time = (temp2 * 30); // reload photo active timer 21-09-2018 
     }
   }
 }
//
if (cls == 1) photoclose = 0; // clear photoclose testbit when cls is active
EE_read(EE_par_34); // 15-08-2012
if (temp > 4)
 { 
  if ((ols == 0) | (close_active() == 1)) photoclose = 0;
 }
//*******
if (ver_show != 0) ver_show--; // timer for showing version number at powerup
if (ver_show == 0)
 {
  EE_read(EE_factory);
  if (temp == 0)
   {
    EE_write(EE_factory,255);
    ver_show = 300;
   }
 }
//*******
if (show_clr_tim != 0)
 {
  show_clr_tim--; // timer for showing clear in display
  d_flash_tim = 1;
 }
if ((stop_pb == 1) | (stop_lid_pb == 1) | (stop_disp_pb == 1))
 {
  if (stop_long_tim != 0) stop_long_tim--;
 }
else
 {
  stop_long_tim = 600;
 } 
//***********
if ((open_pb == 1) | (open_disp_pb == 1)) // 07-04-2010  
 {
  if (open_long_tim != 0) open_long_tim--;
 }
else
 {
  if (dil_2 == 1) open_long_tim = 3000; // 15-11-2010
  else open_long_tim = 600;
 } 
//***********
if (rev_time == 0)
 {
  if (speed_opb != 0)
   {
    speed_opb--;
    if (speed_opb == 0)
     {
      if (code_spd_rev == 1)
       {
        if ((open_pb == 1) | (open_disp_pb == 1)) // is open push still activated, possible open if failure on tachopulses
         {
          imp_close = 0; // reset impulse close. 
          //close_gate_off; // stop closing
					stop_open_or_close();
         }
       }
     }
   }
  else rev_spec = 0; // clear special reverse testbit by speed_opb == 0 26-03-2014
 }
 if (push_hold_tim !=0) push_hold_tim--;
 if (hide_timer !=0) hide_timer--;
 if ((run_prog == 0) && (powerup_tim == 0) && ((safety_test != 1) && (safety_after_time == 0))) // only activate interlock in run mode 09-04-2010 01-12-2021 11-04-2023
  {
   if (interlock_tim !=0) interlock_tim--;
  }
 if (st_speed_stop_tim !=0) st_speed_stop_tim--;
 if (small_move !=0) small_move--; // 18-01-2010
 if (push_code_tim !=0) push_code_tim--; // 10-03-2010
 if (photo_learn_tim !=0) photo_learn_tim--; // 23-03-2010
 if (low_bat == 1) // 26-03-2010
  {
   if (bat_timer == 0) bat_timer = 900; // 3 sec.
   bat_timer--;
  }
 if (half_ols_active == 1) half_ols_tim = 90; // 300mS reload timer when electronic half open is active 14-04-2010 
 if (half_ols_tim !=0) half_ols_tim--;
 //if (s_out_tim !=0) s_out_tim--; // 23-08-2010 13-10-2017
 if (safety_tst_time !=0) safety_tst_time--; // 21-10-2010
 if (test_delay !=0) test_delay--; // 21-10-2010
 if (max_w_tst !=0) max_w_tst--; // 02-05-2011
 if (after_w_test !=0) after_w_test--; // 02-05-2011
 if ((w_edge_ok == 1) & (dw8k2 == 0)) after_w_test = 0; // reset time if wireless system have released til edge, to start quicker
 if (e_tst_tim !=0) e_tst_tim--; // 13-01-2012
 if (blinktimer !=0) blinktimer--; // for future use on LCC V3D
 if (pause1 !=0) pause1--; // for future use on LCC V3D
 if (pause2sek !=0) pause2sek--; // for future use on LCC V3D
 if (limits_show !=0) limits_show--; // 30-04-2019
 if (pwr_timer_SER !=0) pwr_timer_SER--; // 14-08-2019
 if (contactor_off_tim !=0) contactor_off_tim--; // 05-11-2021
 if (up_relay_tim !=0) up_relay_tim--; // 05-11-2021
 if (down_relay_tim !=0) down_relay_tim--; // 05-11-2021
 if (con_on_delay !=0) con_on_delay--; //	05-11-2021
 if (weldtimer !=0) weldtimer--; //	05-11-2021
 if (cls_trafic_tim != 0) cls_trafic_tim--; // 01-07-2021
 EE_read(EE_par_80); // 01-07-2021
 if (open_active() == 1) cls_trafic_tim = 0; // clear time when door is running up
 if ((cls == 1) && (temp == 2)) cls_trafic_tim = 200; // preset timer 400 every main cycle 09-08-2021  
 else if (close_active() == 1) // preset time when running down
  {
   switch (temp)
    {
     case 1:
      cls_trafic_tim = 0; // clear time beacuse relay i off on cls
      break;
     case 2:
      // should not be possible 09-08-2021
      break;
     case 3:
      if (close_active() == 1) cls_trafic_tim = 150000; // preset time 5 min when running down
      break;
     case 4:
      if (close_active() == 1) cls_trafic_tim = 600000; // preset time 20 min when running down
      break;
     case 5:
      if (close_active() == 1) cls_trafic_tim = 1800000; // preset time 60 min when running down
      break;
     default:
      break;
    }
  }	
 if (powerup_tim !=0) powerup_tim--; // 24-09-2021 power-up timer added for general purpose use 01-12-2021
 if (inv_meas_interval !=0) inv_meas_interval--; // 03-12-2021
 if (tim_rev != 0) tim_rev--; // 03-04-2024
 if (fault_delay_1 != 0) fault_delay_1--; // 28-03-2025	
 if (fault_delay_2 != 0) fault_delay_2--; // 28-03-2025	
 if (fault_delay_4 != 0) fault_delay_4--; // 28-03-2025	
 if (fault_delay_5 != 0) fault_delay_5--; // 28-03-2025		
}
//*******************************************************************
