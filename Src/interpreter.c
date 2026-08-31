
#include "stm32f0xx_hal.h"
#include "Global_Var.h"
#include "Defines.h"
#include "main.h"
#include "eprom.h"
#include "encoder.h"
//
extern TIM_HandleTypeDef htim17;
//
uint8_t i_open;                       // inactive open testbit
//
static uint8_t block_open;            // block open testbit
static uint8_t test_trial;            // 
uint8_t i_close;                      // inactive close testbit 30-04-2019 Don't need to be static.. it is in the function
uint8_t i_kip;                        // inactive kip testbit 30-04-2019
static uint8_t safety_req_co;         // safety not carried out at powerup. Should be 0 by powerup as static
static uint8_t rev_bit = 0;           // revesing testbit. Should be 0 by powerup as static 
static uint8_t inv_low_speed;         // Inverter low speed testbit 0 by powerup
extern uint8_t inv_fail_active;       // check bit for inverter fail status > 3
extern uint16_t warn_tim_delay;       // 18-11-2021
extern uint16_t powerup_tim;          // 01-12-2021 2 sec. powerup timer by 3.3 mS cycle time
uint8_t start_close;			  	        // 30-04-2019 for learning only 
uint8_t start_open;			  		        // 30-04-2019 for learning only 
uint8_t relay_ovr = 0;                // relay overrule with paramater 80 17-11-2021
uint32_t cls_trafic_tim = 0;          // 17-11-2021
uint8_t dock_relay = 0;               // control for actual output relay for dock leveler control  23-06-2021 parameter 75, 76 or 88
uint8_t dock_53_active = 0;             // controlbit for active dock leveler control function 24-11-2021
uint8_t dock_54_active = 0;           // controlbit for active dock leveler control with half open function 23-06-2021
uint8_t temp5;                        // temp register 25-11-2021
uint8_t m_lock_tim_ready = 0;         // delay for magnetic lock ready before door starting 26-11-2021
uint8_t m_lock_tim_off = 0;           // delay for magnetic lock reactivating 26-11-2021
uint16_t start_cap_tim;               // 1 sec. timer for starting help capacitor 26-11-2021
uint16_t yard_light_tim;              // 120 Yard light timer 26-11-2021
uint8_t i_e_h_ols_sw;                 // 22-11-2023
extern uint8_t ft_close_pb;           // 22-11-2023
uint16_t fault_delay_1;               // 28-03-2025 delay for relay on by error cause of safety test
uint16_t fault_delay_2;               // 28-03-2025 delay for relay on by error cause of safety test
uint16_t fault_delay_4;               // 28-03-2025 delay for relay on by error cause of safety test
uint16_t fault_delay_5;               // 28-03-2025 delay for relay on by error cause of safety test
uint16_t max_unlock_tim = 600;        // 04-03-2026 Maximum unlocking time accepted before error 33
//
//****************** Electronic counter ********************
uint8_t c_pointer;         // pointer for endurance, electronic counter
uint8_t count_bit;         // controlbit for electronic counter
uint8_t count_b1;          // 8 of 24 bit counter
uint8_t count_b2;          // 8 of 24 bit counter
uint8_t count_b3;          // 8 of 24 bit counter
//
//*** functions *********************************************************************************** 
void lamp_code_out(void); 
void rev2(void);
void reload_run_timer(void);
void safety_not_observed(void);
void safety_observed(void);
void safety_req(void);
void chk_encod_finish(void);
void stop(void);
void adap_speed(void);
void speed(void);
void st_speed(void);
void man_speed(void);
void chk_re_open(void);
void open(void);
void chk_for_open(void);
void req_closing(void);
void run_time(void);
void count_inc(void); 
void wire_tighten(void); 
void req_autoclose(void);
void counter_upd(void);
void send_warning(void);
void send_code(void);
void stop_and_limit_req(void);
void speed_adp_norm(void);
void ok_save(void);
void chk_ini_speed(void);
void low_speed(void);
void chk_e4(void);
void chk_for_open_b(void);
void stop_close(void);
void chk_for_close(void);
void test_cls(void);
void r_autoclose_timer(void);
void chk_service(void);
void chk_rev_tighten(void);
void send_ols(void);
void read_lamp_pcb_settings(void);
void send_cls(void);
void clr_pos(void);
void chk_cnt_pos(void);
void chk_cnt_neg(void);
void calc_areas(void);
void tst_w_edge(void);
void chk_photo(void);
void edge_rev_test(void);
void preset_afterrun(void);
void dec_to_service(void);
void chk_omr(void);
void sort_area(void);
void EE_w_areas(void);
void lamp_pcb(void);
void value_upd(void);
void save_speed(void);
void omr_1(void);
void omr_2(void);
void omr_3(void);
void omr_4(void);
//
void relay_aux0(void);
void relay_aux1(void);
void relay_aux2(void);
void relay_aux3(void);
void relay_aux4(void);
void relay_aux5(void);
void test_for_warning(void);
void flash_aux0(void);
void flash_aux1(void);
void flash_aux2(void);
void flash_aux3(void);
void flash_aux4(void);
void flash_aux5(void);
void read_flash_freq(void);
//
void start_door_down(void);       // special inverter functions
void start_open_fast(void);       // special inverter functions
void start_open_low_speed(void);  // special inverter functions
void stop_open_or_close(void);    // special inverter functions
void start_close_fast(void);      // special inverter functions
void start_close_low_speed(void); // special inverter functions
//
void aux2_relay(void);            // 05-01-2017
void dock_ctrl_active(void);      // 24-11-2021
void dock_leveler(void);          // 24-11-2021
void chk_warn_open(void);         // 01-12-2021
extern void delay_us(uint32_t delay_us); // 28-01-2022
extern void spec_traffic_light(void); // 01-02-2022
//
uint8_t open_active(void);           // special inverter functions
uint8_t close_active(void);          // special inverter functions
extern uint8_t e27_modbus;           // error code for modbus timeout fail 16-03-2017
extern uint8_t curtain_sw;           // curtain switch bt ref input 30-04-2019
extern uint8_t curtain_learn;				 // test bit for curtain lerning active 12-02-2020
extern uint8_t sce_bat_state;        // SCE encoder status 05-01-2021
extern uint8_t inverter_use;			   // 1 = inverter use is active - else contactor use 09-11-2021
//
extern uint8_t door_pos_half;        // test bit for half open 24-11-2021
extern uint8_t door_pos_high;        // test bit for door above half open 24-11-2021
extern uint8_t door_pos_low;         // test bit for door below half open 24-11-2021
extern uint8_t encoder_errors;       // summerized encoder error 26-11-2021
extern uint8_t dock_h_input;         // shared h_ols input and  dock half input 01-12-2021
extern uint8_t h_ols;                // 1/2 open limit switch after sw-filter 02-12-2021
//
extern uint8_t bol_on;               // before open limit activated 01-12-2021
extern uint8_t bcl_on;               // before open limit activated 01-12-2021
extern uint8_t inv_low_sp;           // 03-12-2021
extern uint8_t thermo_stop;          // 22-12-2021
extern uint8_t wic_slack;            // filter testbit for wicket door and slack rope 11-01-2022
extern uint16_t tim_rev;             // 03-04-2024 3 sec. edge reverse time if selected in parameter 55 
extern uint8_t learn_h_open_active; // test bit for half open learning setpoint 10-12-2025
//
uint8_t start_command = 0;           // 24-11-2021
uint8_t m_lock_ready = 0;            // 24-11-2021
uint8_t wait_open = 0;               // 24-11-2021
uint8_t door_half_cmd = 0;           // door half open command used with dock leveler 01-12-2021
uint8_t wireless_relay;              // 25-11-2021
uint8_t imp_signal = 0;              // 26-11-2021
uint8_t e24_wick = 0;                // 11-01-2022 test of input is added to safety test
uint16_t open_imp_tim_aux0 = 0;      // open impulse timer for aux0 relay 26-11-2021
uint16_t open_imp_tim_aux1 = 0;      // open impulse timer for aux1 relay 26-11-2021
uint16_t open_imp_tim_aux2 = 0;      // open impulse timer for aux2 relay 26-11-2021
uint16_t open_imp_tim_aux3 = 0;      // open impulse timer for aux3 relay 26-11-2021
uint16_t open_imp_tim_aux4 = 0;      // open impulse timer for aux4 relay 26-11-2021
uint16_t open_imp_tim_aux5 = 0;      // open impulse timer for aux5 relay 26-11-2021
uint16_t flash_aux4_aux5_val = 0;    // adjustable flash freq setting 26-11-2021
uint16_t flash_aux1_tim = 0;         // flash timing for AUX4 trafic light 26-11-2021
uint16_t flash_aux2_tim = 0;         // flash timing for AUX4 trafic light 26-11-2021
uint16_t flash_aux4_tim = 0;         // flash timing for AUX4 trafic light 26-11-2021
uint16_t flash_aux5_tim = 0;         // flash timing for AUX5 trafic light 26-11-2021
uint8_t spec_max_rev;                // special maximum reversetime testbit 03-04-2024
uint8_t lamp_relay_tim = 0;          // 11-12-2025 
//
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
// {
// if (htim->Instance==TIM14)
//	{
//	 a_speed = HAL_TIM_ReadCapturedValue(&htim14, TIM_CHANNEL_1); //read TIM14 channel 1 capture value
//	 if (motor_rpm_1500 == 0) a_speed = a_speed/2;                // when motor 3000 is active, see main
//	 __HAL_TIM_SET_COUNTER(&htim14, 0);	//reset counter after input capture interrupt occurs
//	}
// }

//****************************************************************************************************
//
	void interpreter()
{
 //led_error_on;	  //debug		
 temp = 1; // test af program 15-03-2016	
 EE_read(EE_par_11); // 
 if ((temp != 7)) chk_encod_finish(); // 24-01-2022
 EE_read(EE_par_84);
 if (temp == 1) block_open = 0; // Special fire program, so block_open is always cleared 06-11-2013.
 //
 if ((close_active() == 0) & (open_active() == 0) & (!close_pb) & (!close_disp_pb) & (imp_close == 0))  // is door stopped and close pushbutton released
  {
   max_w_tst = 240; // 800mS 02-05-2011
   w_edge_ok = 0;
   test_trial = 0; 
  }
 EE_read(EE_par_5); // test for low speed activation only if inverter is selected 17-06-2024
 if (temp > 1) 
  {	 
   EE_read(EE_limits_check);	// 14-08-2019
   if (((run_prog == 1) && ((parameter == 12) || (parameter == 13) || (parameter == 14) || (parameter == 15))) || (temp != 0))//  
    {
	   inv_low_speed = 1; // always low speed when programming limits 18-01-2010
		 if (para22_aut == 1) //22-11-2024 is automatic setup of parameter 22 wanted with interter and Kostal enkoder (Condoor)
		  {			 
	     if (inv_low_sp == 1) // measured low speed
        {
         EE_read(EE_par_22);
         if (temp != 50) EE_write(EE_par_22,50); // adjust on fly, if not 50 because low speed is observed by limit adjustment in programming mode
        } 
       else
        {
         EE_read(EE_par_22);
         if (temp != 25) EE_write(EE_par_22,25); // adjust on fly, if not 0 because no low speed is observed by limit adjustment in programming mode
        } 
		  }			
    }
   else
    {
     if (enc_low_sp_p == 1) inv_low_speed = 1; //if low speed decelerate reached
     else inv_low_speed = 0;
    }
  }
	// 
if (learn_h_open_active == 1) inv_low_speed = 1; // always low speed when programming half open point 10-12-2025 	
//*************************  
if (moving_fail == 1) // failure, door started but no position movement
 {
  if ((close_active() == 1) | (open_active() == 1))
   {
    e9_pos_change = 1; // 
    imp_open = 0;
    imp_close = 0;
    if ((open_active() == 1) & (ols == 1)) stop(); //  stop hvis ols er nået 04-06-2013
    if ((close_active() == 1) & (cls == 1)) stop(); //  stop hvis cls er nået 04-06-2013
    if (small_move == 0) // 18-01-2010
     {
      rev2(); // preset reverse time
      //close_gate_off; // if no inverter used
      //open_gate_off;  // if no inverter used
			stop_open_or_close(); 
			EE_read(EE_par_16);
      if ((close_pb) || (close_disp_pb) || ((temp > 2) && (e_h_ols_sw == 1))) i_close = 1; // set inactive close if close pushbutton is active 
			if ((open_pb == 1) | (open_disp_pb == 1)) i_open = 1; // set inactive open if open pushbutton is active changed from block_open 06-11-2013
      if (kip_pb == 1) i_kip = 1; // set i_kip if kip pushbutton is active
      reload_run_timer();
     }
   } 
  else small_move = 300; // preset 1 sek. max movetime  18-01-2010 
 }
else e9_pos_change = 0;
//*************************
if (run_prog == 0) para_test = 0; // switch away from paratest is normal run is selected
EE_read(EE_par_41);
if (temp > 0) // is speed control wanted
 {
  if ((run_prog == 1) && ((parameter == 12) | (parameter == 13) | (parameter == 14) | (parameter == 15) | (parameter == 51)))
   { // no speed control is wanted when programming limits or learning adaptive run time
   }
  else
   {
    if (temp == 3) // is adaptive speed control wanted
     {
      if ((run_prog == 0) && (bit_test(sk2,3) == 0)) e2_speed = 1;
      adap_speed(); //
      speed();
     }
    else if (temp == 4)
     {
      if ((run_prog == 0) && (bit_test(sk2,3) == 0)) e2_speed = 1;
      st_speed(); // singleturn speed is selected
     }
    else 
     {
      man_speed(); 
     }
   }
 }
else // no speed control wanted
 {
  e2_speed = 0; // clear this error - speed control stop
  e7_tacho = 0; // clear this error - tacho failure no pulses or very slow pulses or no position change
  e8_wear = 0; // clear this error - adap speed wear  
 }
//
if (run_prog == 1) safety_close_ok = 1; // no safety req in programming mode 03-05-2022
else if (rev_spec == 0)  safety_req(); // only safety req when not reversing situation 03-05-2022	
//
EE_read(EE_par_84);
if (temp == 2) i_close = 0; // Special fire program, so inactive close is always cleared 21-09-2018. 
// 
//if (e_stop_pb == 1)
// {
//  emergency_out_on; //
// }
//else emergency_out_off; //
// 
if ((e_stop_pb == 1) | (stop_pb == 1) | (e21_fail == 1) | (safe_stop == 1) | (stop_lid_pb == 1) | (stop_disp_pb == 1) |
	  (stop_safety_chain == 1) | (e29_xtal == 1) | (e32_weld == 1 ) | ((e27_modbus) && (inverter_use == 1)) || 
    (inv_fail_active > 0) || (sce_bat_state > 1) || (thermo_stop == 1)) 
     // 15-11-2016 24-11-2016 22-03-2017 24-06-2020 05-01-2021 22-12-2021 08-06-2022
 {
  //kip_stop_ready = 0; // moved because it shall only be cleared when not safety test 04-01-2018
  //kip_down_ready = 0; // moved because it shall only be cleared when not safety test 04-01-2018 
	if ((safety_test != 1) && (safety_after_time == 0))// only when not safety test itself 08-08-2016
	 {
		kip_stop_ready = 0; // 04-01-2018
    kip_down_ready = 0; // 04-01-2018  
    safety_req_co = 0; //  
    test_delay = 60; // 200mS
		rev_bit = 0; // 21-02-2011
    e5_photo = 0; // 21-02-2011
    e6_edge = 0;  // 21-02-2011
    e22_stop = 0; // 16-11-2016
    e23_chain = 0; // 16-11-2016
		warn_timer = 0; // 08-02-2022 reset when stop activated 
		//stop(); // is stop push-button active - moved 04-01-2018 	moved back again 21-09-2018	 
   }
	stop(); // is stop push-button active - moved 04-01-2018 moved back again 21-09-2018 
 }
else if ((wic_slack == 1) && ((safety_test != 1) && (safety_after_time == 0))) // 08-06-2022. Check wic_slack only when not safety test 06-11-2023
 {
  kip_stop_ready = 0; // 19-11-2018
  kip_down_ready = 0; // 19-11-2018
  stop(); // Wicket door safety circuit is activated - stop door. 
 } 
else
 {
	EE_read(EE_par_36);
  if (temp == 0) interlock_tim = 30; // reload interlock timer with 100 mS. // For ConDoor 01-12-2021 
  if (temp == 2) interlock_tim = 1515; // reload interlock timer with 5 sec. 01-12-2021 
	// By Parameter 36:01 the interlock is always off  in gen_ini routine 01-12-2021 
  EE_read(EE_par_16); 
  //if ((half_ols_active == 1) &  (temp > 7)) // 04-11-2010  27-10-2011
  // {
  //  if ((open_pb == 1) | (open_disp_pb == 1) | (kip_pb == 1))
  //   {
  //    stop_pb = 1; // 230 mS simulation stop to clear half_ols_active on uP4
  //    open_pb = 0; // 230 mS simulation disable to prevent i_open
	//    open_disp_pb = 0; // // 230 mS simulation disable to prevent i_open
  //    kip_pb = 0; // 230 mS simulation disable to prevent i_open
  //    ft_stop_pb = 115; //
  //    ft_open_pb = 115; //
	//		ft_open_disp_pb = 115; // 
  //    ft_kip_pb = 115; //
  //    
  //   }
  // }
  chk_re_open(); // 05-02-2014
	if ((open_pb == 1) || (open_disp_pb == 1) || ((temp > 2) && (e_h_ols_sw == 1) && (door_pos_half == 0) && (door_pos_high == 0)
		&& (i_e_h_ols_sw == 0))) // is open push-button or loop1 active 25-05-2021 08-06-2021 22-11-2023
	 {
		SET_BIT(RCC->CSR, RCC_CSR_RMVF); // clear watchdog timeout by oopen P/B if set  29-11-2016 
    if ((ols == 1) || ((temp == 2) && (door_pos_half == 1) && (e_h_ols_sw == 1))) // 08-06-2021
     {
      if ((close_active() == 1) | (cls == 1)) stop(); // 17-01-2011
      if (open_active() == 1) // is door moving up 05-08-2010
       {
        rev2(); // preset reverse time
       }
      if (re_open_tb == 0) i_open = 1; // 05-08-2010 17-02-2014
      //open_gate_off; // stop door up
			stop_open_or_close(); // 
      imp_open = 0;
      warn_started = 0; // reset warning if started 22-03-2013
      imp_close = 0; // clear close command if just waiting for end warning 22-03-2013
     }
    if (speed_opb > 0)
     {
     }
    else
     {
      open();
      chk_for_open();
     }
   }
  else
   {
    i_open = 0;
    block_open = 0;
    // chk_re_open flyttet herfra
    if ((imp_open == 1) & (run_prog == 0))
     {
		 	EE_read(EE_curtain_sw); // 30-04-2019
      temp1 = temp;	
      EE_read(EE_par_1);
      if (((temp == 1) || (temp1 > 0) || curtain_learn == 1) && (pull_back_time == 0) && (tighten_time == 0)) imp_open = 0; //  28-04-2010 30-04-2019 12-02-2020
      EE_read(EE_par_55); // check whether edge reversing max time is selected 03-04-2024
			if ((temp == 1) && (tim_rev > 0) && (spec_max_rev == 1)) imp_open = 1; // set imp_open again. Always reverse in this special function 03-04-2024 
			EE_read(EE_par_35); // 03-11-2010
			if (((temp == 1) || (temp == 3)) && (kip_pb == 1) && (kip_stop_ready == 1)) // 04-01-2018 
       {
				EE_read(EE_par_31);
        if (temp > 0) kip_down_ready = 1; // 10-01-2022
        stop();
       }
      else chk_for_open(); // 
     }
    else
     {
      if (speed_opb > 0) req_closing();
      else
       {
				 if ((kip_pb == 1) && (i_kip == 0)) // is go function activated 04-01-2018
          {
           EE_read(EE_par_35); //   
           temp3 = temp;
           EE_read(EE_par_16);
					 if (((ols == 1) | ((temp < 2) && (h_ols == 1)) | ((temp > 1) && (door_pos_half == 1) && (open_active() == 0) && (close_active() == 0) 
              && (run_prog == 0))) && (temp3 != 2))  // No kip closing by parameter 35:02 16-05-2013_c 11-12-2025 new half open type added	
						{
             EE_read(EE_par_1);
             if (temp >1) 
              {
              }
             else
              {
               if (((edgetype == 3)&&(fraba == 1)) | ((edgetype != 3) && (dw8k2 == 1)) | (photo1 == 1) | (photo2 == 1)) // 30-11-2021
                {
                 i_open = 1; // i_open if edge or photo is active  
                }
              }
             EE_read(EE_par_1); // 
             if ((temp > 2) & (i_kip == 0))
              {
               EE_read(EE_par_31);
               if (temp > 0)
                {
                 if ((photo1 == 0) & (photo2 == 0) & (dw8k2 == 0) & (e1_mon == 0) & (fraba == 0))  // 15-01-2010 
                 {
									imp_close = 1; // set if OK
                  autoclose_timer = 0;  // 17-09-2021 (by kip)
                  interlock = 0; //17-09-2021
                 }
                }
               req_closing();
              }
            }
           else if (open_active() == 1) // // ols not active is door moving up 15-01-2010
            {
             i_kip = 1;
             chk_for_open();
            }
           else if ((close_active() == 1) & (i_kip == 1)) // 15-01-2010
            {
             req_closing(); //
            }
					 else if (((temp3 == 1) || (temp3 == 3))&& (kip_down_ready == 1) && (i_kip == 0)) req_closing(); // 03-11-2010 04-01-2018	
           else
            {
						 EE_read(EE_par_16); // 11-12-2025
             if ((temp == 2) && (door_pos_half == 1) && (e_h_ols_sw == 1) && (run_prog == 0)) i_open = 1; // always prevent kip open when 16:02 and half open is active 	
				     EE_read(EE_par_1);
             if ((temp > 1) & (run_prog == 0) & ( i_kip == 0) & (i_open == 0)) imp_open = 1; // 11-12-2025
             chk_for_open();
            }
          } 
         else
          {
					 if (kip_pb == 0) i_kip = 0; // kip pushbutton released 21-09-2018	
           if (open_active() == 1)
            { 
             //open_gate_off; // if door moving up - stop moving up
						 stop_open_or_close();	
             rev2(); // preset reverse time 15-01-2010
            }
           req_closing(); //
          }
       }
     } 
   }
 }
//
if ((close_active() == 1) | (open_active() == 1)) run_time(); // run_time if door is moving up or down
else
 {
  e_tst_tim = 300; // preset to 1 sec. test time 13-01-2012
  count_inc(); // up counting if count_bit == 1
  wire_tighten(); 
  req_autoclose();
 }
 
//
dock_leveler(); 
counter_upd();
lamp_code_out(); //
}


//
void rev2()
{
 EE_read(EE_par_53);
 temp2 = temp;
 rev_time = (temp2 * 3); // preset reverse time 10mS steps
}	
	
//***************************************************
void reload_run_timer() // rutine flyttet 06-08-2010
 {
 EE_read(EE_par_51);
 if (temp > 0)
 {
  if (temp == 3) // is adaptive runtime selected
   {
    EE_read(E_run);
    if (temp == 1)
     {
      read_int16_eeprom(EE_run_timer);
      temp2 = data;
      if (temp2 < 2250) run_timer = (temp2 + 150); // is learned runtime < 9 sec. add 1 sec to measured time
      else run_timer = (temp2 + (temp2/8)); // 9 - 230 sec. add 12.5%
     }
    else
     {
      if (run_prog == 1)
       {
       }
      else run_timer = 0;
     }
   }
  else 
   {
    if (temp == 1) run_timer = 3000; // 6.666 mS units = 20 sec.
    if (temp == 2) run_timer = 6000; // 6.666 mS units = 40 sec.
    if (temp == 4) run_timer = 9000; // 6.666 mS units = 60 sec. 
   }
 }
 }
//******************************************************
 
 void adap_speed()
 {
	EE_read(EE_overv);
  if (bit_test (temp,0) == 0)
   {
    e20_fail = 1;
    imp_close = 0;
    imp_open = 0;
   }
  else
   {
    e20_fail = 0;
    if (bit_test(sk1,2) == 1) speed_adp_norm();
    else
     {
      if (e8_wear == 1) // values not saved, is wear observed
       {
        bit_set(sk1,2);
        ok_save();
       }
      else speed_adp_norm();
     }
   }
 }	
// 
 void run_time()
 {
	 if ((run_prog == 1) && (parameter == 82) && (s_encoder_tst == 1)) // 13-01-2012
 {
  if (e_tst_tim == 0)
   {
    stop();
   }
 }
else
 {
  e_tst_tim = 300; // preset to 1 sec. test time 13-01-2012
 }
//
if (close_active() == 1) // is door running down
 {
	EE_read(EE_interlock); // 30-11-2021 retttelse så interlock i EE bliver slettet
  if ((interlock == 1) || (temp == 1)) // 09-10-2012 interlock reset moved to here. EEPROM will be reset before the kontaktor activate.
   {
    EE_write(EE_interlock,0);
    interlock = 0;
   } 
	EE_read(EE_par_35); // 04-01-2018
  if (temp == 3) 
   {
    kip_stop_ready = 1; // 04-01-2018
    kip_down_ready = 0; // 04-01-2018
   }
  else
   {
    kip_stop_ready = 0; // 03-11-2010
    kip_down_ready = 0; // 03-11-2010
   }		
 }
bit_clear(sk1,5); // ready for new clear after door run and after 10 sek push
r_autoclose_timer();
EE_read(EE_par_51);
 if (temp > 0)
  {
   if (run_timer == 0)
    {
     stop(); 
     e3_run = 1;
    }
   else e3_run = 0;
  }
 else e3_run = 0; // 29-09-2016	
}
 //
 void speed()
 {
	 if (cls == 0)
  {
   if ((close_active() == 1) | (open_active() == 1)) // is door moving up or down
    {
     if (speed_delay == 0)
      {
       time = __HAL_TIM_GET_COUNTER(&htim17); 
       if (time > 9000)
        {
         if (bit_test(sk2,0) == 1)
          {
           e7_tacho = 1;
           bit_set(sk1,0);
           low_speed();
          }
         else if (open_active() == 1) // is door moving up
          {
          }
         else
          {
           e7_tacho = 1;
           bit_set(sk1,0);
           low_speed();
          } 
        }
       else
        {
         e7_tacho = 0; // clear tacho failure
         if (bit_test(sk2,3) == 1)
          {
           if (bit_test(sk1,1) == 1)
            {
             if (measure_new == 1)
              {
               measure_new = 0;
               if (a_speed >= measure_1)
                {
                }
               else a_speed = measure_1;
               EE_read(EE_par_44);
               switch (temp)
                {
                 case 0:
                  if (a_speed > (speedsetp + (1 * pv05))) low_speed(); // about 0.5% speed reduction
                  else
                   {
                    e2_speed = 0;
                    chk_ini_speed();
                    t1_con_0 = 1; // start timer again
                    bit_clear(sk1,0); // ready for new measures
                    bit_clear(sk1,1); //
                   } 
                  break; 
                 case 1:
                  if (a_speed > (speedsetp + (2 * pv05))) low_speed(); // about 1% speed reduction
                  else
                   {
                    e2_speed = 0;
                    chk_ini_speed();
                    t1_con_0 = 1; // start timer again
                    bit_clear(sk1,0); // ready for new measures
                    bit_clear(sk1,1); //
                   } 
                  break; 
                 case 2:
                  if (a_speed > (speedsetp + (3 * pv05))) low_speed(); // about 1.5% speed reduction
                  else
                   {
                    e2_speed = 0;
                    chk_ini_speed();
                    t1_con_0 = 1; // start timer again
                    bit_clear(sk1,0); // ready for new measures
                    bit_clear(sk1,1); //
                   } 
                  break; 
                 case 3:
                  if (a_speed > (speedsetp + (4 * pv05))) low_speed(); // about 2% speed reduction
                  else
                   {
                    e2_speed = 0;
                    chk_ini_speed();
                    t1_con_0 = 1; // start timer again
                    bit_clear(sk1,0); // ready for new measures
                    bit_clear(sk1,1); //
                   } 
                  break;           
                 case 4:
                  if (a_speed > (speedsetp + (5 * pv05))) low_speed(); // about 2.5% speed reduction
                  else
                   {
                    e2_speed = 0;
                    chk_ini_speed();
                    t1_con_0 = 1; // start timer again
                    bit_clear(sk1,0); // ready for new measures
                    bit_clear(sk1,1); //
                   } 
                  break;
                 case 5:
                  if (a_speed > (speedsetp + (6 * pv05))) low_speed(); // about 3% speed reduction
                  else
                   {
                    e2_speed = 0;
                    chk_ini_speed();
                    t1_con_0 = 1; // start timer again
                    bit_clear(sk1,0); // ready for new measures
                    bit_clear(sk1,1); //
                   } 
                  break; 
                 default:
                 break;
                }
              }
             else
              {
               measure_1 = a_speed;
               measure_new = 1;
               t1_con_0 = 1; // start timer again
               bit_clear(sk1,1); // ready for new measures
              }
            }
          }
        }
      }
     else
      {
       t1_con_0 = 1; // start timer again
       bit_clear(sk1,0); // ready for new measures
       bit_clear(sk1,1); //
      }
      
    }
   else
    {
     speed_delay = sp_delay; // reload speed delay time
     bit_clear(sk1,1); // ready for new measure
    }
   
  }
 else
  {
   speed_delay = sp_delay; // reload speed delay time
   bit_clear(sk1,1); // ready for new measure
  }
 }
 
 void chk_encod_finish()
 {
	if ((run_prog == 1) && (parameter == 82)) // only run this when encoder test 10-12-2025
	{		
	 EE_read(EE_par_16); // 
   if (temp == 1)
	  {
		 EE_write(EE_par_16, 0); // Automatic clear parameter 16 to 16:00 if 16:01 and encoder is selected 23-11-2023
	  }		
	 if (spec_encoder_test > 0) // old r_byte3
    {
     if (e_tst_result == 1)
      {
       if (confirm_timer == 0)
        {
         s_encoder_tst = 0; // stop show
         para_test = 0; // prevent movement by open and close
        }
       else
        {
        // show
        }
      }
     else
      {
       e_tst_result = 1;
       p_value = 0; // back to parameternumber showing after confirming result
       par_val = 0; // back to parameternumber showing after confirming result
       confirm_timer = 600; // 2 sec. showing
      } 
    } 
  }
	else
   {
    spec_encoder_test = 0; // be sure that this is 0 when not encoder test 10-12-2025
   }
 }
 //
 void set_interlock() // 22-11-2023
 {
  if (interlock == 0)
   {
    EE_write(EE_interlock,1); 
    interlock = 1;
   }
 }
 //
 void stop()
 {
	EE_read(EE_par_36); // 16-05-2013
  if ((interlock_tim == 0) && (ols == 1) && (temp != 1) && ((stop_pb == 1) | (stop_lid_pb == 1) | (stop_disp_pb == 1))) 
		                    // activate interlock if interlock timer is 0 in open position and interlock is selected 16-05-2013
   {                    // and only stop_pb for activating this function 17-09-2021 10-02-2022
    set_interlock();
   }
	EE_read(EE_par_16); // 22-11-2023
 if ((((temp == 1) && (h_ols == 1)) || ((temp > 1) && (door_pos_half == 1))) && (((stop_pb == 1) 
	 | (stop_lid_pb == 1) | (stop_disp_pb == 1)) && (interlock_tim == 0))) 
                                        // 05-08-2022 halv open push must clear autoclose timer for start warning can work
  {
   set_interlock();
  } 
	// 
  warn_started = 0; // reset warning if started 22-03-2013
  imp_open = 0;
  imp_close = 0;
	EE_read(EE_par_16); // 08-06-2021
  if ((close_pb == 1) || (close_disp_pb) || ((temp > 2) && (e_h_ols_sw == 1))) i_close = 1; // set inactive close if close pushbutton is active 
  if (((open_pb == 1) |(open_disp_pb == 1)) & (re_open_tb == 0)) block_open = 1; // set block open if open pushbutton is active 17-02-2014
  if (kip_pb == 1) i_kip = 1; // set i_kip if kip pushbutton is active
  reload_run_timer();
	kip_stop_ready = 0; // 04-01-2018  
	if ((close_active() == 1) | (open_active() == 1)) //   
   {
    rev2(); // preset reverse time
    //close_gate_off;
    //open_gate_off;
		stop_open_or_close(); 
	 }
	start_command = 0;  
 }
 //
void st_speed()
{
 if (st_speed_learn_ok == 1)
  {
   bit_set(sk2,3); // set values ok if ok from encoder processor
   p_value = 0;  // go back to parameternumber and prevent changing to previus parameter 
   para_test = 0; // go back to parameternumber and prevent changing to previus parameter 
   prev_para = 0; // go back to parameternumber and prevent changing to previus parameter 
   confirm_learn = 1; // confirm with 2 sec fixed run showing
   s_speed_learn = 0;
   EE_read(EE_s_speed_saved);
  if (temp != 10) // is speed already saved for not saving more than one time
    {
     EE_write(EE_s_speed_saved,10);
    }
  }
 if ((close_active() == 1) | (open_active() == 1)) // is the door moving
  {
   if (st_speed_stop == 1) low_speed(); // door is running and low speed is observed by enkoderprocessor 
   else e2_speed = 0;
  }
 if (st_wear_observed == 1) // is wear observed by enkoderprocessor
  {
   if (speed_f_timer == 0)
    {
     imp_open = 0;
     imp_close = 0;
     e8_wear = 1;
    }
  }
 else
  {
   speed_f_timer = 11; // r_byte2 is already filtered on old V7E
   e8_wear = 0; // cleared on encoder uP by clr_val
  }
}
void man_speed()
{
 if (cls == 0) //is close limit deactivated
  {
   if ((close_active() == 1) | (open_active() == 1))
    {
     if (speed_delay == 0)
      {
       time = __HAL_TIM_GET_COUNTER(&htim17);
       if (time > 9000)
        {
         e7_tacho = 1;
         bit_set(sk1,0);
         low_speed();
        }
       else
        {
         e7_tacho = 0;
         if (bit_test(sk1,1) == 1) //is speed measurement finish
          {
           if (measure_new == 1)
            {
             measure_new = 0;
             if (a_speed >= measure_1)
              {
              }
             else a_speed = measure_1;
             if (close_active() == 1) // is the door running down
              {
               if ((run_prog == 1) && (parameter == 42)) temp = 255; // door is runing down, prevent stop down direction when
                                                                     // adjusting up setpoint
               else if ((run_prog == 1) && (parameter == 43)) temp = m_speed;
               else EE_read(EE_par_43);
              }
             else
              {
               if ((run_prog == 1) && (parameter == 43)) temp = 255; // door is runing up, prevent stop up direction when
                                                                     // adjusting down setpoint
               else if ((run_prog == 1) && (parameter == 42)) temp = m_speed;
               else EE_read(EE_par_42);
              }
             speed_sp = temp; // !!! check om 8 bit flyttes korrect over i 16 bit var
             EE_read(EE_par_41); // 12-12-2012
             if (temp == 5)speed_sp = (8 * (speed_sp + 855)); // adjusment for fiting a_speed measures 850-1050 RPM 12-12-2012
             else speed_sp = (8 * (speed_sp + 510)); // adjusment for fiting a_speed measures 12-12-2012
             if (a_speed > speed_sp) // is measured speed lower than speed setpoint (higher counts means lower speed)
              {
               low_speed();
              }
             else
              {
               t1_con_0 = 1; // start timer again
               bit_clear(sk1,0); // ready for new measures
               bit_clear(sk1,1);
               e2_speed = 0;
              }
            }
           else
            {
             measure_1 = a_speed;
             measure_new = 1;
             t1_con_0 = 1; // start timer again
             bit_clear(sk1,1); // ready for new measures
            }
          }
        }
      }
     else
      {
       t1_con_0 = 1; // start timer again
       bit_clear(sk1,0); // ready for new measures
       bit_clear(sk1,1); //
      }
    }
   else
    {
     speed_delay = sp_delay; // reload speed delay time
     bit_clear(sk1,1); // ready for new measure
    }
  }
}
//

void safety_not_observed()
{
 if (safety_tst_time == 0)
  {
	 safety_req_co = 1;
   safety_close_ok = 0; // safety close is not ok
	 safety_test_off; // 10-11-2021	
   if (((edgetype == 3)&&(fraba == 0)) | ((edgetype != 3) &&(dw8k2 == 0))) e6_edge = 1; // it was edge not working 21-10-2010
   else if (e24_wick == 1) 
	  {
		 //e24_wick is already set	
	  }		 
   else e5_photo = 1; // it photo not working 21-10-2010	
	 safety_after_time = 90; // 15-11-2016	
  }
}

void safety_observed()
{
  safety_req_co = 1; // When stop here all safety test input can be checked here in software 22-11-2016
  safety_close_ok = 1;
  safety_test_off;
  safety_after_time = 90; // preset 150 mS for re-etablishing time. 16-04-09 21-10-2010, 25-02-2011 increse to 300mS
                           // must be > 60 mS is limit of time before the photo has reetablished including input filter
	            					   // 21-12-2010 øget til 200 mS da MFZ fotocell ligger på 148 mS i reaktionstid
}

void safety_req()
{
	 if ((close_active() == 0) & (open_active() == 0) && (powerup_tim == 0)) // is door stopped 21-02-2011
    {
     if (test_delay == 0) // 21-02-2011
      {
       if (ols == 1) rev_bit = 0; // 
       if ((cls == 1) | (rev_bit == 1))
        {
				 e5_photo = 0;  // clear all fails on cls, always OK on CLS 16-02-2017
				 e6_edge = 0;   //
				 e22_stop = 0;  //
				 e23_chain = 0; //
         safety_req_co = 1; // 
         safety_close_ok = 1; //	
				 safety_test_off; // 07-03-2011
				}
       else
        {
         if (edge_photo_owr == 1) safety_close_ok = 1; // is push code for safety cancel activated 21-02-2011
         if (safety_req_co == 0) // is safety not carried out 21-02-2011
          {
           if (safety_close_ok == 1) // is safety ok for this closing cycle 
            {
             safety_test_off; // 07-03-2011
            }
           else
            {
             if (safety_test == 1) // is safety test already startet
              {
              }
             else
              {
							 safety_test_on;         // Start safety test
							 safety_tst_time = 90; // preset safety test time to 300 mS 21-10-2010, 300mS 25-02-2011
              }
						 //
             e5_photo = 1;  // set all fails to be ready for reset 16-11-2016
						 e6_edge = 1;   //
						 e22_stop = 1;  //
						 e23_chain = 1; //
						 e24_wick = 1;	// 11-01-2022
             if ((e_stop_pb == 1)	&& (stop_pb == 1) && (stop_lid_pb == 1)) e22_stop = 0;        // all is OK - clear this fail		
             if (stop_safety_chain == 1) e23_chain = 0;                                         // OK clear this fail
						 if (((edgetype == 3)&&(fraba == 1)) | ((edgetype != 3) &&(dw8k2 == 1)))	e6_edge = 0; //OK edge primary edge
						 if (wic_slack == 1) e24_wick = 0;	// OK input is working 11-01-2022 							
						 EE_read(EE_par_31);	
						 if (temp == 0) e5_photo = 0;                                        // OK no Photo is selected
						 if (((temp == 1) || (temp == 4)) && (photo1 == 1)) e5_photo = 0;    // OK Photo1 
						 if (((temp == 2) || (temp == 5)) && (photo2 == 1)) e5_photo = 0;    // OK Photo2 	
						 if (((temp == 3) || (temp >5)) && (photo1 == 1) && (photo2 == 1)) e5_photo = 0;    // OK Photo2 		
             //
						 if ((run_prog == 1) && ((parameter == 14) || (parameter == 12)))	e6_edge = 0; // no edge test when programming limits 23-12-09 + 27-01-2010
						 //	
						 if ((e5_photo == 1) || (e6_edge == 1) || (e22_stop == 1) || (e23_chain == 1) || (e24_wick == 1)) 
						  {
							 safety_not_observed();	
						  }							 
						 else
							{
							 safety_observed();	
							}
						}
	        } 
				 else safety_test_off; // safety test is already OK so stop test
        } 
      }
     else
      {
       safety_close_ok = 0; // 21-02-2011 
      }
    }
   else 
    {
     test_delay = 60; // 200mS from stop to safety test 21-02-2011
     safety_tst_time = 90; // 300mS 25-02-2011
     rev_bit = 0; //
     safety_req_co = 0; //
    }
}

void chk_re_open()
{
 if ((safety_test == 0) && (safety_after_time == 0))// because safety test can make STOP PB to reverse 05-02-2014
  {
   if ((re_open_tim > 0) && ((photo1 == 1) | (photo2 == 1) | (open_pb == 1) | (open_disp_pb == 1))) re_open_tb = 1; // 18-02-2011 31-01-2014
   // photo1 or photo 2 is always = 0 if they are not selected in parameter 31
   if ((ols == 0) & (re_open_tb == 1)) imp_open = 1;
   // re_open_tb is usefull because photo can disapear before door roll off the OLS
  }	
}
void open()
{
 EE_read(EE_par_16); // 08-06-2021
 if ((close_pb == 1) || (close_disp_pb) || ((temp > 2) && (e_h_ols_sw == 1)))
  {
   i_close = 1; // if close pushbutton also on, set inactive close testbit
   EE_read(EE_par_84); // 12-11-2024
	 if (temp == 3) stop(); // if this special function active the door must stop. 
  }		
 EE_read(EE_par_1);
 if ((temp > 1) & (run_prog == 0) & (block_open == 0) & (i_open == 0) & ( ols == 0))
  {
   EE_read(EE_par_16);
   if (((temp < 2) && (h_ols == 0)) || (temp > 1)) imp_open = 1;
   EE_read(EE_par_41); // is adap speed control selected
   if ((temp > 2)&& (temp < 5)) // 12-12-2012
    {
     if ((bit_test(sk2,3) == 0) | (e8_wear == 1)) // is speed not learned or wear observed
      {
       imp_open = 0; // clear again when speed not learned not ready or wear observed only deadman is possible
      }
    }
  }
}
//
void chk_for_open()
{
 chk_e4(); // check service
 if (close_active() == 1) i_open = 0; // is the door moving down 07-01-2011
 if (i_open == 0)
  {
	 if (speed_opb > 0)
    {
    }
   else
    {
		 EE_read(EE_par_55); // check whether edge reversing max time is selected 03-04-2024
     if ((temp == 1) && (tim_rev == 0) && (spec_max_rev == 1))
		  {
		   imp_open = 0; 
       spec_max_rev = 0;
      }			 
     if (imp_open == 1) // rettet 2/2 2009
      {
			 edge_photo_owr = 0; // 10-03-2010
       EE_read(EE_par_16);
       if ((ols == 1) | ((temp < 2) && (h_ols == 1))) // is OLS active or mechanical 1/2 open active
        {
         imp_open = 0; // always clear if ols or half ols is active 27-04-2010
         kip_stop_ready = 0; // 03-11-2010
         kip_down_ready = 0; // 03-11-2010
         if (open_active() == 1)
          {
           //open_gate_off;
		  	   stop_open_or_close();	
           // imp_open = 0; // 27-04-2010
           rev2(); // preset reverse time
           if (kip_pb == 1) i_kip = 1; // set i_kip if kip pushbutton is active
          }
        }
       else chk_for_open_b(); 
      }
     else
      {
			 EE_read(EE_par_16); 
       if ((open_pb == 1) || (open_disp_pb == 1) || ((temp > 1) && (e_h_ols_sw == 1) && (door_pos_half == 0) && (door_pos_high == 0)
				 && (i_e_h_ols_sw == 0)))  // 27-10-2011 25-06-2019 25-05-2021 08-06-2021 22-11-2023
        {
         if ((ols == 1) || ((temp < 2) && (h_ols == 1)) || ((temp == 2) && (door_pos_half == 1) && (e_h_ols_sw == 1))) // is OLS active
          {
           if (open_active() == 1)
            {
						 stop_open_or_close();	
             rev2(); // preset reverse time
             i_open = 1;
             if (kip_pb == 1) i_kip = 1; // set i_kip if kip pushbutton is active
            }
          } 
         else chk_for_open_b(); // deadman open
        }
      }
    }
  } 
}
void chk_ini_speed()
{
 EE_read(EE_par_44);
switch (temp)
 {
  case 0:
   wear_value = (10 * pv05); // 5% wear value   
  break; 
  //
  case 1:
   wear_value = (10 * pv05); // 5% wear value   
  break; 
  //
  case 2:
   wear_value = (10 * pv05); // 5% wear value   
  break; 
  //
  case 3:
    wear_value = (10 * pv05); // 5% wear value   
  break; 
  // 
  case 4: 
   wear_value = (12 * pv05); // 6% wear value   
  break; 
  //
  case 5: 
   wear_value = (14 * pv05); // 7% wear value   
  break; 
  //
  default:
  break;
 }
 if (bit_test(sk2,7) == 1)
  {
   if (a_speed > (i_speedsetp + wear_value)) // is measured speed lower than initial speed + wear_value 21-12-2010
    {
     if (bit_test(sk1,4) == 1) // is it 2. time wear is observed
      {
       if (speed_f_timer == 0)
        {
         imp_open = 0;
         imp_close = 0;
         e8_wear = 1;
        }
       else bit_set(sk1,4);
      }
     else
      {
       bit_set(sk1,4);
       speed_f_timer = 11; // 37mS filter
      }
    }
  }	
}
//
void low_speed()
{
 t1_con_0 = 1;
 if (bit_test(sk1,0) == 1) // is it 2. time low speed is observed
  {
   bit_clear(sk1,0); //low speed is observed, prepare for new measure
   bit_clear(sk1,1);
   e2_speed = 1;
   if (close_active() == 1) //is door running down
    {
     if (speed_opb == 0) // 12-04-2010
      {
       imp_close = 0;
       pull_back_time = 30; // 100 mS at 3.33mS main cycle time
       i_close = 1;
       imp_open = 1;
       code_spd_rev = 1;
       //close_gate_off;
			 stop_open_or_close();	
       rev2(); // preset reverse time
       reload_run_timer();
       st_speed_stop_tim = 240; // 800mS nescessary for singleturn reverse by closing cause of delay filters 08-12-09
       rev_bit = 1; // 21-02-2011
       if (autoclose_dis < 10) autoclose_dis++; // 23-08-2011
      }
    }
   else if (st_speed_stop_tim == 0) //door running up, check about singleturn reverse
    {
     imp_open = 0;
     kip_stop_ready = 0; // 03-11-2010
     reload_run_timer();
     //open_gate_off; // stop door open
		 stop_open_or_close();	
     i_open = 1;
     rev2(); // preset reverse time
     EE_read(EE_par_1);
     if (temp == 4) speed_opb = 150; // 500mS speed reversing by stop in open direction is selected 12-04-2010
     else speed_opb = 0; // no reversing
     code_spd_rev = 1; // set testbit for open speed reverse
     if (autoclose_dis < 10) autoclose_dis++; // 23-08-2011
    }
   else
    {
    }
  }
 else
  {
   bit_set(sk1,0);
   bit_clear(sk1,1); //Ready for new measure
  }	
}
//
void req_closing()
{
	chk_e4(); // check service
EE_read(EE_par_35); // 03-11-2010
if (temp == 2) kip_down_ready = 0 ; // just to be sure that no kip closing if parameter 35:02 16-05-2013_c
//
EE_read(EE_par_16); // 08-06-2021
if ((temp == 4) && (e_h_ols_sw == 1) && (door_pos_half == 1))
 {
  close_pb = 1; // make a short close command 200 mS in this special case 22-11-2023
  ft_close_pb = 66;
  i_e_h_ols_sw = 1;
 }
else
 {
  if (e_h_ols_sw == 0) i_e_h_ols_sw = 0; // only clear inactive bit id button is released 22-11-2022
 }
//	

if (((close_pb) || (close_disp_pb)) || ((kip_pb == 1) && (kip_down_ready) && (!i_kip)) || ((temp > 2) && (e_h_ols_sw == 1) && (door_pos_half == 0) && (door_pos_low == 0)))	
  {
   autoclose_dis = 0; // 23-08-2011
 // interlock reset removed from here 09-10-2012  
 //******** edge_photo_fail bit controlling 01-02-2012 ****
	 if (((edgetype == 3)&&(fraba == 1)) | ((edgetype != 3) && ((dw8k2 == 1) ))) // is edge active when close_pb is activated
    {
     edge_photo_fail = 1;
    }
   
   else edge_photo_fail = 0;
   if ((photo1 == 1) | (photo2 == 1)) // is photo active when close_pb is activated
    {
     edge_photo_fail = 1;
    } 
   else
    {
    }
 //********************************************************
   if (i_close == 1) stop_close(); // 23-12-09
   else
    {
     if (((safety_close_ok == 1) && (safety_after_time == 0)) || (cls == 1)) // 13-04-2011
      {
       EE_read(EE_par_1);
       if ((temp > 2) && (edge_photo_owr == 0)) imp_close = 1; // set impulse close if selected by parameter 1 10-03-2010
       chk_for_close();
      }
     else 
      {
       EE_read(EE_par_2);
			 if ((temp == 1) || (edge_photo_owr == 1)) chk_for_close(); // 21-02-2011 deadman selected by fail on safety test
				                                                          // 29-03-2017 edge_photo_owr added	
      }
    }
  }
else
 {
  edge_photo_fail = 0; // No showing when close pushbutton is released 01-02-2012
  i_close = 0; // push-button released
	EE_read(EE_curtain_sw); // 30-04-2019
  temp1 = temp;	 
  EE_read(EE_par_1);
  if ((temp > 2) && (temp1 == 0) && (edge_photo_owr == 0)) // is impulse close selected 10-03-2010 30-04-2019 only deadman if curtain switch has been activated 
	 {
   }
  else
   {
    imp_close = 0; // 
   }
  if (((imp_close == 1) || (speed_opb > 0)) & (run_prog == 0)) // is impulse close active or reversing startet
   {
    if ((e5_photo == 0) && (e6_edge == 0) && (e22_stop == 0) && (e23_chain == 0)) // 21-10-2010 18-11-2016
     {
      if (((safety_close_ok == 1) && (safety_after_time == 0)) || (cls == 1) || (rev_spec == 1)) // 13-04-2011 26-03-2014
       {
        EE_read(EE_par_1); // is edge monitoring wanted and e1_on = 1 (=edge fail)
        if ((temp > 2) & (e1_mon == 1)) imp_close = 0; //  -  -
        else
         {
          EE_read(EE_par_41); // is adap speed control selected
          if ((temp > 2) && (temp < 5))
           {
            if ((bit_test(sk2,3) == 0) | (e8_wear == 1)) // is speed not learned or wear observed
             {
              imp_close = 0; // when speed not learned not ready og wear observed only deadman is possible
             }
            else chk_for_close();
           }
          else chk_for_close(); 
         }
       }
      else imp_close = 0; // 21-02-2011
     }
    else // 21-10-2010
     {
      imp_close = 0; // 21-02-2011
     }
   }
  else
   {
    temp = temp3; //debug
    test_cls(); // 08-05-2012
    stop_close();
   }
 }
}
//
void count_inc()
{
 if (count_bit == 1)
 {
  if (count_b1 %10 == 0) ser_count_d = 1; // service counter 
  else ser_count_d = 0; // service counter
  chk_service(); // service counter
  EE_read(EE_c_pointer);
  c_pointer = temp;
  EE_read(EE_count_b1 + c_pointer); //get 1. byte LS
  count_b1 = temp;
  if (count_b1 == 255)
   {
    count_b1 = 0; // there was carry
    EE_write((EE_count_b1 + c_pointer), count_b1); // save new value in EEPROM
    EE_read(EE_count_b2);
    count_b2 = temp;
    EE_read(EE_count_b3);
    count_b3 = temp;
    if (count_b2 == 255)
     {
      count_b2 = 0; // there was carry
      count_b3++; //
      EE_write(EE_count_b2, count_b2); // save new value in EEPROM
      EE_write(EE_count_b3, count_b3); // save new value in EEPROM
     }
    else
     {
      count_b2++;
      EE_write(EE_count_b2, count_b2); // save new value in EEPROM
     }
   }
  else
   {
    count_b1++;
    EE_write((EE_count_b1 + c_pointer), count_b1); // save new value in EEPROM
   }
  count_bit = 0;
  EE_read(EE_count_b1 + c_pointer); //
  while((temp != count_b1) && (e20_fail == 0))
   {
    if (c_pointer > 9)
     {
      e20_fail = 1; 
     }
    else
     {
      c_pointer++;
      EE_write(EE_c_pointer, c_pointer);
      EE_write((EE_count_b1 + c_pointer), count_b1); // save new value in EEPROM
     }
   }
 }
}
//
void wire_tighten()
{
 EE_read(EE_par_29);
  if (temp > 0) // is wire tighten selected
   {
    if (cls == 1)
     {
      if (e1_mon == 1) // is fail active
       {
       }
      else
       {
        if (open_active() == 1) // is door moving up
         {
         }
        else
         {
          if (tighten_ready == 1) // is tighten already selected
           {
           }
          else
           {
            EE_read(EE_par_29);
            if (temp == 1) tighten_time = 4;// 4 x 3.33 mS = 13 mS should be 5 mS on contactor output at 25 VDC
            if (temp == 2) tighten_time = 6;// 6 x 3.33 mS = 20 mS should be 10 mS on contactor output at 25 VDC
            if (temp == 3) tighten_time = 8;// 8 x 3.33 mS = 27 mS should be 22 mS on contactor output at 25 VDC
            if (temp == 4) tighten_time = 10;// 10 x 3.33mS = 33mS should be 28 mS on contactor output at 25 VDC
            if ((open_active() == 0) & (tighten == 0)) // if closing ended 15-11-2010
             {
              tighten = 1;
              chk_rev_tighten();
              rev_time = 60; // preadjust for fixed 200 mS revtime at 3.33mS main cycle
             }
            else
             {
              if (tighten == 1) chk_rev_tighten(); 
             }
           }
         }
       }
     }
    else
     {
      tighten_ready = 0;
      tighten = 0;
     }
   }
  else
   {
   }
}
//
void req_autoclose()
{
 reload_run_timer(); // reload run_timer when door not moving
 EE_read(EE_par_36); // 16-05-2013_b
 if ((e9_pos_change == 1) && (ols == 1) && (temp != 1)) // activate interlock if E:09 is aktive in open position 16-05-2013_b 01-12-2021
  {
   if (interlock == 0)
    {
     EE_write(EE_interlock,1); 
     interlock = 1;
    }
  }
 EE_read(EE_par_17); // 12-04-2010
 if (temp == 0)
  {
   EE_read(EE_par_16);
   if ((temp < 2) && (h_ols == 1) && (ols == 0)) r_autoclose_timer(); 
   else if (door_pos_half == 1) r_autoclose_timer();
  }
//
 EE_read(EE_par_41);
 if ((temp > 2) && (temp < 5) && (bit_test(sk2,3) == 0)) // is adap. speed control selected and learned not finish
  {
   r_autoclose_timer(); 
  }
 else
  {
   read_int16_eeprom(EE_par_32);
   temp2 = data;
   if (temp2 > 0)
    {
     if ((interlock == 1) && (close_pb == 0) && (close_disp_pb == 0)) // 01-12-2021 08-07-2026
      {
       r_autoclose_timer(); // 25-02-2011
      }
     else if (autoclose_dis > 2)
      {
       r_autoclose_timer(); // 23-08-2011
      }
     else
      {
       if (e1_mon == 1) r_autoclose_timer();
       if ((photo1 == 1) | (photo2 == 1) | (stop_pb == 1) | (stop_lid_pb == 1) | (stop_disp_pb == 1)) r_autoclose_timer();
			 if (((edgetype == 3)&&(fraba == 1)) | ((edgetype != 3) && (dw8k2 == 1))) // 30-11-2021
        {
         EE_read(EE_par_21); // 02-05-2011
         if (temp == 5)
          {
           if (after_w_test == 0) r_autoclose_timer(); // only reload if after wireless test time i s ended
          }
         else r_autoclose_timer(); // if edge active 27-05-2010
        }
       EE_read(EE_par_1);
       if (temp > 2)
        {
         EE_read(EE_par_16);
				 if ((ols == 1) || ((temp > 1) && (door_pos_half == 1)) ||((temp < 2) && (h_ols == 1))) // is open limt active og mechanical 1/2 open active 10-08-2021	
          {
           EE_read(EE_par_35); // also reload autoclose by kip if p35 = 2. 04-06-2013
           if ((open_pb == 1) | (open_disp_pb == 1) | (imp_open == 1) | ((kip_pb == 1) && (temp == 2)))
            {
             if (warn_timer == 0) imp_open = 0; // only clear if warntimer is not running 09-12-2025
             r_autoclose_timer();
            }
           else
            {
             if ((autoclose_timer == 0) || (force_closing_time == 0)) // 23-08-2011
              {
               imp_close = 1;
               re_open_tim = 300; // 18-02-2011 - øget til 1 sek. 31-01-2014
              }
            }
          }
         else r_autoclose_timer();
        }
       else
        {
        }
      }
    }
  }
}
//
void counter_upd()
{
 if (open_active() == 1) // is door moving up 
  {
   if (cls == 1) // is close limit switch active
    {
     // it can be wire tighten so wait counting
    }
   else
    {
     count_bit = 1; 
    }
  }
}
//
void chk_e4()
{
 EE_read(EE_par_59);
 if ((temp == 1) && (e4_ser == 1))
  {
   imp_open = 0;
   imp_close = 0;
  }	
}
//
void chk_for_open_b()
{
 if (close_active() == 1) // is the door moving down
  {
	 rev_bit = 1; // 21-02-2011
   EE_read(EE_par_35);
   if ((close_active()==1) && (temp == 3) && (kip_pb == 1) && (open_pb == 0) && (open_disp_pb == 0)) // 04-01-2018 21-09-2018
    {
     imp_open = 0; // no reversing in this special kip mode but 03-07-2017
     kip_down_ready = 0; // prepare for open 04-01-2018
     kip_stop_ready = 0; // 04-01-2018
     rev_bit = 0; // no reversing in this special kip 04-01-2018
     i_kip = 1;
    }
   stop_open_or_close(); // close gate off  
   imp_close = 0;
   rev2(); // preset reverse time
  }
 if (open_active() == 1) // is the door moving up 23-12-09
  {
	 imp_close = 0;
   edge_photo_owr = 0; // 10-03-2010
   if (kip_pb == 0) kip_stop_ready = 1; // 03-11-2010
  }
 if (rev_time == 0)
  {
   if (block_open == 0)
    {
		 temp4 = 0; // temp 4 is 0 if not set by the following	01-12-2021
		 EE_read(EE_par_71); // Is magnetic lock selec on one of the relays
     if ((temp == 50) || (temp == 56)) temp4 = 1; // 04-03-2026
     EE_read(EE_par_72); // Is magnetic lock selec on one of the relays
     if ((temp == 50) || (temp == 56)) temp4 = 1; // 04-03-2026
     EE_read(EE_par_74); // Is magnetic lock selec on one of the relays
     if ((temp == 50) || (temp == 56)) temp4 = 1; // 04-03-2026
     EE_read(EE_par_75); // Is magnetic lock selec on one of the relays
		 if ((temp == 50) || (temp == 56)) temp4 = 1; // 04-03-2026	
			if (temp4 == 0)  m_lock_ready = 1; // 16-06-2020 28-06-2021. m_lock_ready always 1 when lock not selected
     if (m_lock_ready == 1) // 16-09-2019
      {
       chk_warn_open(); // 01-12-2021
       if (wait_open == 1) // 09-10-2019
        {
        }
       else
        {
				 if (inv_low_speed == 1) start_open_low_speed();
			   else start_open_fast();	
         EE_read(EE_par_16); // 08-06-2021
         if ((close_pb == 1) || (close_disp_pb) || ((temp > 2) && (e_h_ols_sw == 1))) i_close = 1; // 08-06-2021
         if (cls == 0)
          {
           dw_ok = 0;
           ok_down = 1;
          }
        }
      } 
     else
      {
       start_command = 1;
      } 	
    }
  }
}
//
void stop_close()
{
 imp_close = 0;
 EE_read(EE_par_16); 
 if ((close_pb == 1) || (close_disp_pb) || ((temp > 2) && (e_h_ols_sw == 1))) i_close = 1; // set inactive close if close pushbutton is active 07-06-2021	
 if (kip_pb == 1) i_kip = 1; // set inactive kip if kip pushbutton is active
 if (close_active() == 1) 
  {
   rev2(); // preset reverse time
   //close_gate_off;
	 stop_open_or_close();	
  } 
 //	
 EE_read(EE_par_21); // 13-10-2017
 if ((temp == 5) && (after_w_test == 0)) aux1_off; // output_low ready for new test 13-10-2017 	
}
//
void chk_for_close()
{
 EE_read(EE_par_21); // 02-05-2011
 if (temp == 5) 
  {
	 tst_w_edge(); // if wireless optical is selected
  }
else
 {
  w_edge_ok = 1;
  after_w_test = 0;
 }
if ((w_edge_ok == 1) && (after_w_test == 0)) //02-05-2011
 {
  EE_read(EE_par_31);
  if (((temp == 4) || (temp == 6)) && (photo_pos_observed == 1) && (photo_learn == 0)) photo1_dis = 1; //23-03-2010
  else photo1_dis = 0;
  if (((temp == 5) || (temp == 7)) && (photo_pos_observed == 1) && (photo_learn == 0)) photo2_dis = 1;
  else photo2_dis = 0;
  if ((((photo1 == 1) & (photo1_dis == 0)) | ((photo2 == 1) & (photo2_dis == 0))) & (photo_learn == 0)) // 11-03-2010 06-10-2010
   {
    chk_photo();
    EE_read(EE_par_2); // 21-12-2010
    if ((((close_pb) || (close_disp_pb)) && (temp == 1)) || (edge_photo_owr == 1)) // 29-03-2017
     {
      imp_close = 0; // only possible deadman close with fail on loop
      test_cls();
      edge_rev_test();
     }
    else stop_close(); // 21-12-2010
   }
  else
   {
    if (open_active() == 1) // if door moving up
     {
     }
    else
     {
      EE_read(EE_relearn_ph1);
      if (temp == 1) imp_close = 0; // only possible deadman close because relearn of photo1 in frame is needed 06-10-2010
      EE_read(EE_relearn_ph2);
      if (temp == 1) imp_close = 0; // only possible deadman close because relearn of photo2 in frame is needed 06-10-2010
      test_cls();
      edge_rev_test(); 
     }
   }
 } 	
}
//
void test_cls()
{
 EE_read(EE_par_22);
if (temp > 0) // is monitoring of edge selected  
 {
  if (close_active() ==1 ) // is door moving down
   {
    if (cls == 1) // is close limit active
     {
      if (afterrun_time == 0)
       {
        //close_gate_off; // stop door down
				stop_open_or_close(); 
        warn_started = 0; // clear warning when close limit is aktive 08-11-2013
        rev2(); // preset reverse time
        ok_down = 0;
        i_close = 1; // 05-08-2010
       }
      else
       {
				if (((edgetype == 1) || (edgetype == 4)) && (dw8k2 == 1)) e1_mon = 0; //primary pne edge active clear fail 12-01-2022
			 }
     }
    else preset_afterrun();
     
   }
 if (close_active() == 1) edge_fail_timer = 150; // is door moving down, preset edge_fail_timer with 500 mS
 else
  {
  if ((cls == 1) && (ver_show < 250)) // wait 0.5 sec.for CLS to be correct after powerup 20-02-2014
   {
    if (ver_show > 0) dw_ok = 1; // set dw_ok if we in a powerup situation with cls active 20-02-2014
    if (dw_ok == 1) imp_close = 0;
    else
     {
			if (((edgetype == 3)&&(fraba == 1)) | ((edgetype != 3) && (dw8k2 == 1))) // if adge active 12-01-2022
       {
        dw_ok = 1;
        imp_close = 0;
       }
      else if (edge_fail_timer == 0)
       {
        if ((edgetype == 1) || (edgetype == 4)) e1_mon = 1; // only by pne edge 10-03-2010
        if ((run_prog == 1) && (parameter == 41)) e1_mon = 0; // clear e1_mon again if programming of speed is active
        imp_close = 0;
       }
     }
   }
  }
 } 
else
 {
  e1_mon = 0;
  if (cls == 1)
   {
    if (close_active() == 1) // is door moving down 05-08-2010
     {
      rev2(); // preset reverse time
     }
    i_close = 1; // 05-08-2010
    //close_gate_off; // stop door down
		stop_open_or_close(); 
    warn_started = 0; // clear warning when close limit is aktive 08-11-2013
    imp_close = 0;
   }
 }	
}
//
void r_autoclose_timer()
{
 read_int16_eeprom(EE_par_32);
 temp2 = data;
 if (temp2 != 0) 
  { 
   autoclose_timer = (temp2 * 50); //
	 ex_auto_sw = 0; // autoclose shall be active. There is no switch in this control 02-02-2022 
	 warn_started = 0; // clear warning if active when autoclose is reloaded 08-07-2026	
	 warning = 0; // 08-07-2026	
	 warn_timer = 0; // 08-07-2026	
  }
 //
 EE_read(EE_par_34); // 15-08-2011
 if ((temp == 0) || (temp == 5)) force_closing_time = 240; // force closing time disabled, but preset to min time to prevent closing
 if ((temp == 1) || (temp == 6)) force_closing_time = 240; // 2 min
 if ((temp == 2) || (temp == 7)) force_closing_time = 600; // 5 min
 if ((temp == 3) || (temp == 8)) force_closing_time = 1200; // 10 min
 if ((temp == 4) || (temp == 9)) force_closing_time = 2400; // 20 min	
}
//
void chk_service()
{
 EE_read(EE_par_58);
 if (temp > 0)
  {
   if (ser_count_d == 1) dec_to_service();
  }
 else
  {
   e4_ser = 0;
  }	
}
//
void chk_rev_tighten()
{
 if (rev_time == 0)
  {
   tighten_ready = 1;
   imp_open = 1;
  }	
}
//
void lamp_code_out() // changes 24-11-2021 in this function 
{
 temp5 = 0; // temp5 is 0 if not set by the following of one of the relays setup	01-12-2021
            // Lamp-PCB can only be used if no other relays is used with warning setup and parameter 74 and
	          // parameter 75 must be 0 setup.	29-12-2021
 EE_read(EE_par_71); // 16-09-2019 
 if (((temp < 1) || (temp > 6)) && (temp != 9))	temp5 = 1; // now 1 04-03-2026
 EE_read(EE_par_72); // 16-09-2019 
 if (((temp < 1) || (temp > 6)) && (temp != 9))	temp5++; // now 2	
 EE_read(EE_par_74); // 16-09-2019 
 if (temp == 0)	temp5++;                 // now 3
 EE_read(EE_par_75); // 16-09-2019 
 if (temp == 0)	temp5++;                 // now 4	
 //
 if (temp5 > 3) // 28-06-2021 08-09-2021 01-12-2021 all not using warning - Lamp PCB can be used 04-03-2026
  {
   lp_req_doub = !lp_req_doub; // toggle for double time making
   if (lp_req_doub == 1)
    {
     EE_read(EE_par_41);
     if ((temp > 2) && (temp < 5)) // is adaptive speed control selected 12-12-2012
      {
       if (bit_test(sk2,3) == 1)  // is values ok
        {
         if (warn_started == 1)
          {
           send_warning();
           send_code();
          }
         else stop_and_limit_req();
        }
       else
        {
        }
      }
     else
      {
       if (warn_started == 1)
        {
         send_warning();
         send_code();
        }
       else stop_and_limit_req();
      }
    }
  }
 else 
  {
	 if ((temp > 3) && (warning == 1)) // 08-06-2022 - clear warning if not wanted from relay setup or lamp-PCB
                                     // from relay setup or lamp-pcb 04-03-2026
    {
     warning = 0;
     EE_read(EE_warning); // 
     if (temp == 1)
      {
       EE_write(EE_warning, 0); // only write to EEPROM is warning is changed on LAMP PCB only
      }
    }	
   read_flash_freq(); // 26-11-2021
	 relay_aux4();
   relay_aux5();
  }
 //start_command = 0; // This test bit for magnetic lock function is updated each program cycle. 16-09-2019	
 relay_aux1(); // 01-12-2021 these relays is still in use when Lamp-PCB is in use
 relay_aux2();
 start_command = 0; // This test bit for magnetic lock function is updated each program cycle. Line moved 04-03-2026		
}

//
void send_warning() // this light code is 11
{
 bit_set(light_code,0);
 bit_set(light_code,1);
}
//
void send_ols() // this light code is 01
{
 bit_set(light_code,0);
 bit_clear(light_code,1); 	
}
//
void send_cls() // this light code is 10
{
 bit_clear(light_code,0);
 bit_set(light_code,1); 	
}
//
void send_code()
{
 // bit 0
 lamp_dat_off;
 if (bit_test(light_code,0) == 1) lamp_dat_on; // setup the databit 0
 lamp_clk_on; // set clock high
	
 for(uint8_t i=0;i<96;i++) // 48MHz clock medfører 4 uS eller måske 6 uS
  {
   __NOP();
  }
 lamp_clk_off; // set clock low
 // bit 1
 for(uint8_t i=0;i<192;i++) // 48MHz clock medfører 4 uS eller måske 6 uS
  {
   __NOP();
  }
 lamp_dat_off;
 if (bit_test(light_code,1) == 1) lamp_dat_on; // setup the databit 1
 lamp_clk_on; // set clock high
 for(uint8_t i=0;i<96;i++) // 48MHz clock medfører 4 uS eller måske 8 uS
  {
   __NOP();
  }	
 lamp_clk_off; // set clock low
}
//
void read_lamp_pcb_settings()
{
 GPIO_InitTypeDef GPIO_InitStruct; // Need to be here in function for changing I/O on fly	
 for(uint8_t i=0;i<96;i++) // 48MHz clock medfører 4 uS eller måske 6 uS - 2 uS delay is minimum for lamp pcb communication
  {
   __NOP();
  }	
  /*Configure GPIO pin : LAMP_DAT_Pin */ // Change to input
  GPIO_InitStruct.Pin = LAMP_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN; // not made like this on LCCV3D but should be I think
  HAL_GPIO_Init(LAMP_DATA_GPIO_Port, &GPIO_InitStruct);	
 lamp_clk_on; // set clock high to give signal about there is ready for reading
 for(uint16_t i=0;i<720;i++) // 48MHz clock result in 30 uS or maybe 45 uS - sholud be 30 uS and not 25 uS for reliable com.
  {
   __NOP();
  }		
warning = 1; // set warning testbit to 1 if its not cleared of the following lines
if (lamp_dat == 0) warning = 0; //
lamp_clk_off;
/*Configure GPIO pins : LAMP_DAT_Pin as output */ // Change back to output again
  GPIO_InitStruct.Pin = LAMP_DATA_Pin|LAMP_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LAMP_DATA_GPIO_Port, &GPIO_InitStruct);
//disable_interrupts(GLOBAL); // all interrupts OFF - Anders mener ikke den behøves
temp = 0;
if (warning == 1) temp = 1; // set bit 0 to 1 if true
EE_write(EE_warning, temp);
// enable_interrupts(GLOBAL); // enable interrupts again after EEPROM writing - Anders mener ikke den behøves	
}
//
void clr_pos()
{
 counterpos = 0; // clear counterpos
 if (open_active() == 1) // is door moving up
  {
   bit_set(sk2,2); //set for position counter active
  }
 else if (close_active() == 1) // is door moving down - can be afterrun 04-11-2011
  {
  }
 else
  {
   if (bit_test(sk1,3) == 0) // is values adjusted for this door cycle
    {
    }
   else
    {
     if (bit_test(sk2,3) == 1)
      {
       if (open_cnt > 10) chk_omr();  // 10 
      }
     else chk_omr();
    }
  }	
}
//
void chk_cnt_pos()
{
 bit_set(sk1,3); // values adjusted this door cycle
 if (bit_test(sk2,1) == 1)
  {
   bit_clear(sk2,5);
   counterpos++;
   bit_clear(sk2,1);
  }
 sort_area();	
}
//
void chk_cnt_neg()
{
 if (bit_test(sk2,1) == 1)
  {
   if (counterpos > 0)
    {
     bit_clear(sk2,5);
     counterpos--;
    }
   else
    {
    }
   bit_clear(sk2,1);
  }
 sort_area();	
}
//
void calc_areas()
{
 bit_clear(sk2,2);
 open_cnt++;
 calc_counterpos = counterpos;
 div_2 =0;
 while (calc_counterpos > 230) // not 255 because  it must be possible to overrun 10% without register overrun
  {
   calc_counterpos = calc_counterpos/2;
   div_2++;
  }
 calc_counterpos = calc_counterpos/4;
 if (bit_test(sk2,0) == 1)
  {
   if ((calc_counterpos > (omsk12 +3)) || (calc_counterpos < (omsk12 -3))) // is old values more that 3 bit from new
    {
     EE_w_areas();
    }
  }
 else EE_w_areas(); 	
}
//
void ok_save()
{
	// disable_interrupts(GLOBAL); // disable interupt when writing to EEPROM - Anders mener ikke denne behøves
 stop_close(); // fast stop to prevent overrun by learning 19-04-2010
 data = ce1;
 adr = EEce1;
 write_int16_eeprom();
 data = ce2;
 adr = EEce2;
 write_int16_eeprom();
 data = ce3;
 adr = EEce3;
 write_int16_eeprom();
 data = ce4;
 adr = EEce4;
 write_int16_eeprom();
 data = oe1;
 adr = EEoe1;
 write_int16_eeprom();
 data = oe2;
 adr = EEoe2;
 write_int16_eeprom();
 data = oe3;
 adr = EEoe3;
 write_int16_eeprom();
 data = oe4;
 adr = EEoe4;
 write_int16_eeprom();
 pv05 = ce1/200;
 data = pv05;
 adr = EEpv05;
 write_int16_eeprom();
 sp_adj = pv05/2; // 21-12-2010
 data = sp_adj;
 adr = EE_sp_adj;
 write_int16_eeprom();
 p_value = 0;  // go back to parameternumber and prevent changing to previus parameter 
 para_test = 0; // go back to parameternumber and prevent changing to previus parameter 
 prev_para = 0; // go back to parameternumber and prevent changing to previus parameter 
 confirm_learn = 1; // confirm with 2 sec fixed run showing
 if (bit_test(sk2,7) == 1)
  {
  }
 else
  {
   ice1 = ce1;
   data = ice1;
   adr = EEice1;
   write_int16_eeprom();
   ice2 = ce2;
   data = ice2;
   adr = EEice2;
   write_int16_eeprom();
   ice3 = ce3;
   data = ice3;
   adr = EEice3;
   write_int16_eeprom();
   ice4 = ce4;
   data = ice4;
   adr = EEice4;
   write_int16_eeprom();
   ioe1 = oe1;
   data = ioe1;
   adr = EEioe1;
   write_int16_eeprom();
   ioe2 = oe2;
   data = ioe2;
   adr = EEioe2;
   write_int16_eeprom();
   ioe3 = oe3;
   data = ioe3;
   adr = EEioe3;
   write_int16_eeprom();
   ioe4 = oe4;
   data = ioe4;
   adr = EEioe4;
   write_int16_eeprom();
   //
   bit_set(sk2,7);
  }
 // enable_interrupts(GLOBAL); // enable after saving in EEPROM - Anders mener ikke denne behøves
}
void speed_adp_norm()
{
 if (bit_test(sk2,5) == 1) // is position counter saved
  {
  }
 else if((close_active() == 1) | (open_active() == 1)) // is door moving up or down
  {
  }
 else if(bit_test(sk2,0) == 1) // areas must have been saved one time before EEprom test
  {
   while(bit_test(sk2,5) == 0)
    {
     calc_counterpos = counterpos;
     temp4 = div_2;
     while (temp4 > 0)
      {
       calc_counterpos = calc_counterpos/2;
       temp4--;
      }
     EE_read(EE_c_point_pos);
     temp3 = temp;
		 //__disable_irq();	// 17-06-2024
		 //void HAL_NVIC_DisableIRQ(TIM15_IRQn); // Anders mener ikke det behøves
     // disable_interrupts(GLOBAL); // disable interupt mens der skrives til EEPROM
     EE_write((EEcntpos + temp3), calc_counterpos);
     EE_read(EEcntpos + temp3);
     if (temp == calc_counterpos)
      {
       bit_set(sk2,5); // for not saving again
      }
     else
      {
       temp3++;
       if (temp3 > 20)
        {
         EE_write(EE_overv,0);
         bit_set(sk2,5);
        }
       else EE_write(EE_c_point_pos,temp3);
      }
    }
	 // enable_interrupts(GLOBAL); // Anders mener ikke det behøves 
  }
 if (cls == 1)
  {
   clr_pos();
  } 
 else if (open_active() == 1) chk_cnt_pos(); // if door moving up 
 else if (close_active() == 1) chk_cnt_neg(); // if door moving down
 else if ((ols == 1) && (half_ols_active == 0) && (half_ols_tim == 0)) 
  {
   if (bit_test(sk2,2) == 1) calc_areas(); 
  }
 else
  {
   speed_o = 0;
   speed_c = 0;
   bit_clear(sk2,2);
  }
}
//
void stop_and_limit_req()
{
 if ((ols == 0) && (cls == 0)) lamp_relay_tim = 100; // reload time with 200mS 11-12-2025
 else
  {
   if (lamp_relay_tim != 0) lamp_relay_tim--;
  }	
 if ((ols == 1) && (lamp_relay_tim == 0)) // 11-12-2025
  {
   if (light_code == 0)
    {
     send_ols();
     send_code();
     read_lamp_pcb_settings();
    }
   else
    {
     send_ols();
     send_code(); 
    }
  }
 else
  {
   if ((cls == 1) && (lamp_relay_tim == 0)) send_cls(); // 11-12-2025
   else light_code = 0;
   send_code();
  }
}
//
void tst_w_edge()
{
 if (w_edge_ok == 1)
  {
  }
 else
  {
   if (test_trial == 1)
    {
     e6_edge = 1;
    }
   else
    {
		 switch (wireless_relay) // 25-11-2021 changes due to all relay can be used for this
		  {
       case 0:
				//aux0_on; do not exist
        break;
			 case 1:
				aux1_on;
        break;
       case 2:
				aux2_on;
        break; 
       case 3:
				//aux3_on; do not exist
        break; 
       case 4:
				aux4_on;
        break;
       case 5:
				aux5_on;
        break;  			 
		  }			 
     after_w_test = 300; // preset after test time to 1000 mS - 900mS delay have been seen
     if (dw8k2 == 1)
      {
       test_trial = 1;
       w_edge_ok = 1;
			 switch (wireless_relay)
		    {
         case 0:
				  //aux0_off; do not exist
          break;
			   case 1:
				  aux1_off;
          break;
         case 2:
				  aux2_off;
          break; 
         case 3:
				  // aux3_off; do not exist
          break; 
         case 4:
				  aux4_off;
          break;
         case 5:
				  aux5_off;
          break;  			 
		    }			 	
      }
     else
      {
       if (max_w_tst == 0)
        {
         test_trial = 1;
         w_edge_ok = 0;
				 switch (wireless_relay)
		      {
           case 0:
				    // aux0_off;
            break;
			     case 1:
				    aux1_off;
            break;
           case 2:
				    aux2_off;
            break; 
           case 3:
				    ///aux3_off;
            break; 
           case 4:
				    aux4_off;
            break;
           case 5:
				    aux5_off;
            break;  			 
		      }			 		
			  }
      }
    }
  }	
}
//
void chk_photo()
{
 EE_read(EE_par_2); // 21-12-2010
 if ((cls == 1) || (((temp == 1) || (edge_photo_owr == 1)) && ((close_pb) || (close_disp_pb)))) // 21-12-2010 29-03-2017
  {
  }
 else
  {
   if (((photo1 == 1) & (photo1_dis == 0)) | ((photo2 == 1) & (photo2_dis == 0))) // 11-03-2010
    {
     warn_started = 0; // clear warning if started 08-11-2013
     if (close_active() == 1) // is door moving down
      {
       pull_back_time = 30; // preset to 100 mS at 3.33mS main cycle
       imp_open = 1;
       rev_bit = 1; //21-02-2011
      }
    }
  }	
}
//
void edge_rev_test()
{
  if (cls == 1)
  {
   if (((edgetype == 3)&&(fraba == 1)) | ((edgetype != 3) && (dw8k2 == 1))) // if edge active 27-05-2010 30-11-2021
    {
     if (close_active() == 1) // is the door moving down 
      {
       dw_ok = 1;
       //close_gate_off; // stop door down
			 stop_open_or_close();	
       rev2(); // preset reverse time
       imp_close = 0;
      }
     else
      {
      }
    }
   else
    {
     EE_read(EE_par_22);
     if (temp > 0) // is monitoring of edge selected
      {
       if (rev_time == 0)
        {
         if (ok_down == 1) lamp_pcb();
        }
      }
     else
      {
      }
    }
  }
 else // cls = 0
  {
   if ((run_prog == 1) && ((parameter == 14) || (parameter == 12))) // is programming electronic limit active, 
                                                                    // don´t test edge 23-12-09 + 27-01-2010
    {
     if (rev_time == 0)
      {
       if (i_close == 1)
        {
        }
       else lamp_pcb(); // ready for close
      }
    }
   else
    {
     if (((edgetype == 3)&&(fraba == 1)) | ((edgetype != 3) && (dw8k2 == 1))) // if edge active 27-05-2010 30-11-2021
      {
       EE_read(EE_par_2); // 21-12-2010
       if (((close_pb) || (close_disp_pb)) && ((temp == 1) || (edge_photo_owr == 1))) // deadman by these cond. 29-03-2017
        {
         if (i_close == 1)
          {
          }
         else lamp_pcb(); // ready for close as deadman
        }
       else
        {
         i_close = 1;
         if (kip_pb == 1) i_kip = 1; // set i_kip if kip pushbutton is active
         if (imp_open == 1) 
          {
          }
         else
          {
           imp_close = 0; //
           if (close_active() == 1) // is the door moving down
            {
             pull_back_time = 30; // preset speed pullback time to 100 mS (3.33 mS units)
             imp_open = 1; // set impulse open
             //close_gate_off; // stop door down
						 stop_open_or_close();	
						 EE_read(EE_par_21); // 12-04-2018
             if (temp == 6) rev2(); // use photo rev. when RAYLG is selected
             else	
							{
               EE_read(EE_par_52);
               temp2 = temp;
               if (temp2 == 0) rev_time = 2; // revtime = 4 mS if 0 is selected
               else rev_time = (temp2 * 3);
              }
						 reload_run_timer();
             rev_bit = 1; // 21-02-2011
             tim_rev = 900; // max rev open time 3 sec. if selected in parameter 55	03-04-2024
             spec_max_rev = 1;	// 03-04-2024						
            }
           else
            {
            }
          }
        }  
       }
      else // edge not active
       {
        if (rev_time == 0)
         {
          if (i_close == 1)
           {
           }
          else lamp_pcb(); // ready for close
         }
       }
    }
 }	
}
//
void preset_afterrun()
{
 EE_read(EE_par_16); // 08-06-2021
 if ((temp > 1) && ((door_pos_high == 1) || (door_pos_half == 1))) // 10-08-2021
  {
   afterrun_time = 10;
  }
 else if ((dock_54_active == 1) && ((door_pos_high == 1) || (door_pos_half == 1))) // 23-06-2021
  {
   afterrun_time = 10;
  }  
 else
  {	
	 EE_read(EE_par_22);
   temp2 = temp;
   afterrun_time = (temp2 * 3);
	}		
}
//
void dec_to_service()
{
 read_int16_eeprom(EE_ser_count);
 temp2 = data;
 if (temp2 >= 10)
  {
   data = temp2 - 10; //
   adr = EE_ser_count;
   write_int16_eeprom();
   e4_ser = 0;
  }
 else 
  {
   e4_ser = 1;
  }	
}
//
void chk_omr()
{
 bit_clear(sk1,3); // clear control bit for values OK
 if (speed_o == 15)
  {
   if (speed_c == 15)
    {
     if (bit_test(sk2,0) == 1)
      {
       value_upd();
       save_speed();
      }
     else 
      {
       mso = 0; // keep values
       msc = 0; // keep values
      } 
     
    }
   else
    {
     mso = 0; // keep values
     msc = 0; // keep values
    } 
  }
 else
  {
   mso = 0; // keep values
   msc = 0; // keep values
  }	
}
//
void sort_area()
{
 calc_a_speed = a_speed;
 calc_counterpos = counterpos;
 temp3 = div_2;
 while (temp3 > 0)
  {
   calc_counterpos = calc_counterpos/2;
   temp3--;
  }
 if (calc_counterpos > 255) calc_counterpos = 255;
 if (speed_delay == 0)
  {
   if (a_speed > 7000)
    {
     temp2 = a_speed; // debug test
    }
   if (bit_test(sk2,0) == 1)
    {
     if (calc_counterpos < omsk12) omr_1();
     else if (calc_counterpos < omsk23) omr_2();
     else if (calc_counterpos < omsk34) omr_3();
     else omr_4();
    }
  }	
}
//
void EE_w_areas()
{
 //disable_interrupts(GLOBAL); // disable interupt when saving a_speed - Anders mener ikke det behøves
 omsk12 = calc_counterpos;
 temp3 = omsk12; // 8 bit
 EE_write(EEomsk12,omsk12);
 omsk23 = (temp3 + omsk12);
 EE_write(EEomsk23,omsk23);
 omsk34 = (temp3 + omsk23);
 EE_write(EEomsk34,omsk34);
 bit_set(sk2,0);
 EE_write(EE_div_2,div_2);
 //enable_interrupts(GLOBAL); //	- Anders mener ikke det behøves
}
//
void lamp_pcb()
{
 EE_read(EE_par_1);
 temp3 = temp;
 EE_read(EE_par_41);
 if ((temp3 < 3) || (run_prog == 1)) start_door_down(); // no warning when deadman closing is selected 09-10-2019
 else if ((temp > 2) && (temp < 5) && (bit_test(sk2,3) == 0)) start_door_down(); // if adaptive speed selected and learn not finished 12-12-2012
 else
  {
   test_for_warning();
   if (warning == 0) start_door_down();
   // else if ((ols == 0) & (h_ols == 0)) start_door_down(); // 09-10-2019
   else if ((warn_started == 0) && ((close_active() == 0) && (open_active() == 0))) // warning not started and door not running
    {
     EE_read(EE_par_77);
	   temp3 = temp;
     EE_read(EE_par_78); 		
     warn_timer = (temp + temp3); // 2 forwarning times is added by autoclosing
     warn_timer = (warn_timer * 300); // adjusted to 1 sek. = 300 main cycles
     warn_started = 1;
    }
   else 
    {
     if (warn_timer == 0)
      {
       start_door_down();
      }
     else 
      {
       reload_run_timer(); // reload runtimer when warning is running 09-10-2019
      }
    }
  }  
}
//
void value_upd()
{
 speed_c =0;
speed_o =0;
if (bit_test(msc,0) == 1)
 {
  if (bit_test(msc,1) == 1) ce1 = ce1 + sp_adj; // 21-12-2010
 }
else ce1 = ce1 - sp_adj; // 21-12-2010
if (bit_test(msc,2) == 1)
 {
  if (bit_test(msc,3) == 1) ce2 = ce2 + sp_adj; // 21-12-2010
 }
else ce2 = ce2 - sp_adj; // 21-12-2010
if (bit_test(msc,4) == 1)
 {
  if (bit_test(msc,5) == 1) ce3 = ce3 + sp_adj; // 21-12-2010
 }
else ce3 = ce3 - sp_adj; // 21-12-2010
if (bit_test(msc,6) == 1)
 {
  if (bit_test(msc,7) == 1) ce4 = ce4 + sp_adj; // 21-12-2010
 }
else ce4 = ce4 - sp_adj; // 21-12-2010
if (bit_test(mso,0) == 1)
 {
  if (bit_test(mso,1) == 1) oe1 = oe1 + sp_adj; // 21-12-2010
 }
else oe1 = oe1 - sp_adj; // 21-12-2010
if (bit_test(mso,2) == 1)
 {
  if (bit_test(mso,3) == 1) oe2 = oe2 + sp_adj; // 21-12-2010
 }
else oe2 = oe2 - sp_adj; // 21-12-2010
if (bit_test(mso,4) == 1)
 {
  if (bit_test(mso,5) == 1) oe3 = oe3 + sp_adj; // 21-12-2010
 }
else oe3 = oe3 - sp_adj; // 21-12-2010
if (bit_test(mso,6) == 1)
 {
  if (bit_test(mso,7) == 1) oe4 = oe4 + sp_adj; // 21-12-2010
 }
else oe4 = oe4 - sp_adj; // 21-12-2010
mso =0;
msc =0;
bit_set(sk2,3); // value ok 	
}
//
void save_speed()
{
 if (bit_test(sk2,7) == 0) ok_save();
 else
  {
   EE_read(EEce1);
   if (temp == 0) ok_save();
   else
    {
     if (open_cnt > 9) // 
      {
       open_cnt = 0;
       ok_save();
      }
    }
  }	
}
//
void omr_1()
{
 if (open_active() == 1) // is door moving up
  {
   if (a_speed <= oe1)
    {
    }
   else if (bit_test(sk2,3) == 0) oe1 = a_speed;
   else if (bit_test(mso,0) == 1) bit_set(mso,0);
   else if (open_cnt == opd_value_1)
    {
     bit_set(mso,1);
     bit_set(mso,0);
    }
   i_speedsetp = ioe1;
   bit_set(speed_o,0);
   speedsetp = oe1;
  }
 else // door moving down
  {
  if (a_speed <= ce1)
    {
    }
   else if (bit_test(sk2,3) == 0) ce1 = a_speed;
   else if (bit_test(msc,0) == 1) bit_set(msc,0);
   else if (open_cnt == opd_value_1)
    {
     bit_set(msc,1);
     bit_set(msc,0);
    }
   i_speedsetp = ice1;
   bit_set(speed_c,0);
   speedsetp = ce1;
  }	
	
}
//**************************************************************************
void omr_2()
{
 if (open_active() == 1) // is door moving up
  {
   if (a_speed <= oe2)
    {
    }
   else if (bit_test(sk2,3) == 0) oe2 = a_speed;
   else if (bit_test(mso,2) == 1) bit_set(mso,2);
   else if (open_cnt == opd_value_1)
    {
     bit_set(mso,3);
     bit_set(mso,2);
    }
   i_speedsetp = ioe2;
   bit_set(speed_o,1);
   speedsetp = oe2;
  }
 else // door moving down
  {
  if (a_speed <= ce2)
    {
    }
   else if (bit_test(sk2,3) == 0) ce2 = a_speed;
   else if (bit_test(msc,2) == 1) bit_set(msc,2);
   else if (open_cnt == opd_value_1)
    {
     bit_set(msc,3);
     bit_set(msc,2);
    }
   i_speedsetp = ice2;
   bit_set(speed_c,1);
   speedsetp = ce2;
  }
}
//**************************************************************************
void omr_3()
{
 if (open_active() == 1) // is door moving up
  {
   if (a_speed <= oe3)
    {
    }
   else if (bit_test(sk2,3) == 0) oe3 = a_speed;
   else if (bit_test(mso,4) == 1) bit_set(mso,4);
   else if (open_cnt == opd_value_1)
    {
     bit_set(mso,5);
     bit_set(mso,4);
    }
   i_speedsetp = ioe3;
   bit_set(speed_o,2);
   speedsetp = oe3;
  }
 else // door moving down
  {
  if (a_speed <= ce3)
    {
    }
   else if (bit_test(sk2,3) == 0) ce3 = a_speed;
   else if (bit_test(msc,4) == 1) bit_set(msc,4);
   else if (open_cnt == opd_value_1)
    {
     bit_set(msc,5);
     bit_set(msc,4);
    }
   i_speedsetp = ice3;
   bit_set(speed_c,2);
   speedsetp = ce3;
  }
}
//**************************************************************************
void omr_4()
{
 if (open_active() == 1) // is door moving up
  {
   if (a_speed <= oe4)
    {
    }
   else if (bit_test(sk2,3) == 0) oe4 = a_speed;
   else if (bit_test(mso,6) == 1) bit_set(mso,6);
   else if (open_cnt == opd_value_1)
    {
     bit_set(mso,7);
     bit_set(mso,6);
    }
   i_speedsetp = ioe4;
   bit_set(speed_o,3);
   speedsetp = oe4;
  }
 else // door moving down
  {
  if (a_speed <= ce4)
    {
    }
   else if (bit_test(sk2,3) == 0) ce4 = a_speed;
   else if (bit_test(msc,6) == 1) bit_set(msc,6);
   else if (open_cnt == opd_value_1)
    {
     bit_set(msc,7);
     bit_set(msc,6);
    }
   i_speedsetp = ice4;
   bit_set(speed_c,3);
   speedsetp = ce4;
  }
}
void start_door_down()
{
 if (kip_pb == 1) i_kip = 1; // set i_kip to prevent reverse when runing down by constant go function push
 temp4 = 0; // temp 4 is 0 if not set by the following	01-12-2021
 EE_read(EE_par_71); // Is magnetic lock selec on one of the relays
 if ((temp == 50) || (temp == 56)) temp4 = 1; // 04-03-2026
 EE_read(EE_par_72); // Is magnetic lock selec on one of the relays
 if ((temp == 50) || (temp == 56)) temp4 = 1; // 04-03-2026
 EE_read(EE_par_74); // Is magnetic lock selec on one of the relays
 if ((temp == 50) || (temp == 56)) temp4 = 1; // 04-03-2026
 EE_read(EE_par_75); // Is magnetic lock selec on one of the relays
 if ((temp == 50) || (temp == 56)) temp4 = 1; // 04-03-2026		
 if (temp4 == 0)  m_lock_ready = 1; // 16-06-2020 28-06-2021	m_lock_ready always 1 when lock not selected
 if (m_lock_ready == 1) // 16-09-2019
  {
   if (inv_low_speed == 1) start_close_low_speed();
   else start_close_fast();	
  }
 else
  {
   start_command = 1;
  }	
 code_spd_rev = 0; // clear code speed reverse 
}
//
void start_open_fast()
 { 
  if (inverter_use == 1)	
   {
    speed_stage_0_off;
    speed_stage_1_off;
    door_run_on;	
   }
  else
   {
    start_open = 1; // 09-11-2021
   }
 }	
//
void start_open_low_speed()
 {	
  if (inverter_use == 1)
   {
    start_open = 1;	// 30-04-2019 for learning only	
    door_run_off;	// prevent change to fast before low speed
    speed_stage_0_on;
    speed_stage_1_off;
    door_run_on;	
   }
  else
   {
    start_open = 1; // 09-11-2021 same as fast in contactor version
   } 
 }
//
void stop_open_or_close()
{
 if (inverter_use == 1)	
  {	
   start_open = 0;	// 30-04-2019 for learning only	
   start_close = 0; // 30-04-2019 for learning only		
   door_run_off;	// 
   speed_stage_0_off;
   speed_stage_1_off;
  }
 else
  {
	 start_open = 0;	// 09-11-2021	
   start_close = 0; // 09-11-2021	
  }
}
//
void start_close_fast()
{
 if (inverter_use == 1)	
  {
   speed_stage_0_off;
   speed_stage_1_on;
   door_run_on;	
  }
 else
  {
   start_close = 1; // 09-11-2021
  }	 
}
//
void start_close_low_speed()
{
 if (inverter_use == 1)	
  {	
   start_close = 1; // 30-04-2019 for learning only		
   door_run_off;	// prevent change to fast before low speed
   speed_stage_0_on;
   speed_stage_1_on;
   door_run_on;	
  }
 else
	{
	 start_close = 1; // 09-11-2021	same as fast in contactor version
	}
}	
//
uint8_t open_active(void)           // special inverter functions for test open active
{
 if (inverter_use == 1)	
  {		
   if ((speed_stage_1 == 0) && (door_run == 1))
    {
     return 1; // test for both high and low speed
    }
   else
    {
     return 0;
    }
  }
 else
	{
	 if (start_open == 1) return 1;
   else return 0;		
	}
}
//
uint8_t close_active(void)           // special inverter functions for test close active
{
 if	(inverter_use == 1)	 
  {
   if ((speed_stage_1 == 1) && (door_run == 1)) return 1; // test for both high and low speed
   else return 0;
  }
 else
	{
	 if (start_close == 1) return 1;
   else return 0;		
	}
}
//
//***********************************************************************************************
//***************************************************************************************************
void relay_aux1() // 16-09-2019 big change 16-06-2020
 {
  EE_read(EE_par_80); // 01-07-2021
  temp3 = temp; 
  if ((temp3 > 1) && (temp3 < 6) && (cls_trafic_tim > 0)) relay_ovr = 1; // relay ON on CLS
  else relay_ovr = 0; 
  EE_read(EE_par_71);
  switch (temp)
   {
    case 0:
     aux1_off;  // Relay off
     break;
    case 1:
     if ((relay_ovr == 1) && (cls == 1)) aux1_on; // relay ON on CLS 01-07-2021
     else if (warn_timer > 0) flash_aux1(); // 09-10-2019
     else if ((close_active() == 1) | (open_active() == 1)) aux1_on; // Relay ON when door running 
     else if ((ols == 0) && (cls == 0)) aux1_on; // Relay ON when stopped between limits 01-07-2021
     else aux1_off;
     break;
    case 2:
     if ((relay_ovr == 1) && (cls == 1)) aux1_on; // relay ON on CLS
     else if (warn_timer > 0) flash_aux1(); // not only on ols and h_ols 29-05-2020
     else if((close_active() == 1) | (open_active() == 1)) flash_aux1(); // Relay ON when door running 
     else if ((ols == 0) && (cls == 0)) flash_aux1(); // Relay flash when stopped between limits 01-07-2021
     else aux1_off;
     break;
    case 3:
     if ((relay_ovr == 1) && (cls == 1)) aux1_on; // relay ON on CLS
     else if (warn_timer > 0) aux1_on; // not only on ols and h_ols 18-06-2020 
     else if((close_active() == 1) | (open_active() == 1)) aux1_on; // Relay ON when door running
     else if ((ols == 0) && (cls == 0)) aux1_on; // Relay ON when stopped between limits 01-07-2021 
     else aux1_off; 
     break;
     case 4:
     if ((relay_ovr == 1) && (cls == 1)) aux1_on; // relay ON on CLS
     else if (warn_timer > 0) flash_aux1(); // warning before each running 29-05-2020
     else aux1_off;
     break;
    case 5:
     if (warn_timer > 0) flash_aux1(); // 29-05-2020
     else if (ols == 1) aux1_on; // 29-05-2020
     else aux1_off;
     break;
    case 6:
     if ((warn_timer > 0) || (warn_tim_delay > 0)) aux1_off; // 28-06-2021
     else if (ols == 1) aux1_on; // 
     else aux1_off;
     break;
		case 9:
     // do nothing here - controlled by AUX0
     break;
//
    case 10:
     if (ols == 1) aux1_on;
     else aux1_off;
     break;
    case 11:
     if (cls == 1) aux1_on;
     else aux1_off;
     break;
    case 12:
		 if (ols == 0) aux1_on;
     else aux1_off;
     break;
    case 13:
     if (cls == 0) aux1_on;
     else aux1_off;
     break;
    case 14:
     EE_read(EE_par_16);
     if ((temp < 2) && (h_ols == 1) && (ols == 0)) aux1_on; // 29-05-2020 18-06-2020 
     else if ((temp > 1) && (door_pos_half == 1) && (close_active()==0) && (open_active()==0)) aux1_on; // 29-05-2020 18-06-2020 08-06-2021 05-12-2025
     else aux1_off;
     break;
//
    case 20:
     if ((close_active()==0) && (open_active()==0)) imp_signal = 0; // ready for new impulse when stopped
     if (open_imp_tim_aux1 > 0) open_imp_tim_aux1--; //
     else 
      {
       EE_read(EE_par_16); 
       if (((open_pb == 1) | (open_disp_pb == 1) || ((temp > 2) && (e_h_ols_sw == 1) && (door_pos_low == 1) && (close_active()==0)) || (imp_open == 1)) 
				 && (imp_signal == 0)) // 08-07-2026
				 // 25-05-2021 08-12-2025 09-12-2025
        { 
         open_imp_tim_aux1 = 500;
         imp_signal = 1;
        }
      }
     if (open_imp_tim_aux1 > 0) aux1_on; 
     else aux1_off;
     break;
    case 21:
      if (ols == 0)
			 {
			  open_imp_tim_aux1 = 1000; // 2 sec. reload
				aux1_off; // Turn off if set 08-12-2025
			 }
      else if (door_pos_half == 1) // false OLS, do not turn on yet 08-12-2025
			 {
			 }
	    else		
			 {
        if (open_imp_tim_aux1 > 0) open_imp_tim_aux1--; // OLS == 1
        if (open_imp_tim_aux1 > 0) aux1_on; 
        else aux1_off;
       }
     break;
 //    
    case 25:
     if ((close_active()==1) | (open_active()==1)) aux1_on; // Brake, relay is activated when motor is running.
     else aux1_off;
     break;
    case 26:
     if ((close_active()==1) | (open_active()==1)) aux1_off; // Brake negated, relay is activated when motor is not running.
     else aux1_on;
     break;
 //   
    case 30:
     if (bcl_on == 1) aux1_on; // is before close limit active 26-11-2021
     else aux1_off;
     break;
    case 31:
     if (bol_on == 1) aux1_on; // is before open limit active 26-11-2021
     else aux1_off;
     break;
    case 32:
     if (open_active()==1) aux1_on;
     else  aux1_off;
     break;
    case 33:
     if (close_active()==1) aux1_on;
     else  aux1_off;
     break;
    case 34:
      if ((close_active()==1) | (open_active()==1)) aux1_on;
      else  aux1_off;
     break;
//
    case 40:
     if ((e1_mon == 1) | (e7_tacho == 1) | (e2_speed == 1) | (e3_run == 1) | (e4_ser == 1) | (e5_photo == 1) | (e6_edge == 1) | (e10_edge == 1) | (e8_wear == 1) | 
     (e9_pos_change == 1) | (e20_fail == 1) | (e21_fail == 1) | (encoder_errors == 1) | (e_stop_pb == 1) | (stop_safety_chain == 1) |
		 (safe_stop == 1) | (e29_xtal == 1) | ((err_edge == 1) && (run_prog == 0)) | ((ols == 1) && (cls == 1))) // 08-05-2025
      {
			 if (fault_delay_1 == 0) 	aux1_on; // Relay ON when error when safety test ended 28-03-2025
			}
     else
		  {
		   aux1_off;
       fault_delay_1 = 150; // 28-03-2025
		  }			 
     break;
    case 41:
      if ((((edgetype == 3)&&(fraba == 1)) | ((edgetype != 3) && (dw8k2 == 1))) && (hide_timer == 0))
			 {
			 aux1_on; // Relay ON
			 }				
      else if (((e6_edge == 1) || (e1_mon == 1)) && ((safety_test != 1) && (safety_after_time == 0))) // 11-12-2025_e
			 {
			aux1_on; // if adge active or if error on edge
			 }
      else aux1_off;
     break;
    case 42:
      if (e4_ser == 1) aux1_on;
      else  aux1_off;
     break;
    case 43:
			EE_read(EE_par_31);
      if ((temp == 4) && (photo_pos_observed == 1) && (photo_learn == 0)) photo1_dis = 1; // 11-12-2025_c
      else photo1_dis = 0;
		  if ((((photo1 == 1) & (photo1_dis == 0)) | (e5_photo == 1)) && (hide_timer == 0) && ((safety_test != 1) && (safety_after_time == 0)))
			 {
			  aux1_on; // 11-12-2025_c 11-12-2025_e
			 }	
      else  aux1_off;
     break;
    case 44: // aux1_relay is controlled ecode1 error on encoder processor 07-04-2021
      if ((ecode1 == 1) | (e9_pos_change == 1)) aux1_on; // AUX1 ON by ecode1 fail - no answer on encoder and no position change
      else aux1_off;  // AUX1 OFF 
     break;
		case 45:
     if ((e1_mon == 1) | (e7_tacho == 1) | (e2_speed == 1) | (e3_run == 1) | (e4_ser == 1) | (e5_photo == 1) | (e6_edge == 1) | (e10_edge == 1) | (e8_wear == 1) | 
     (e9_pos_change == 1) | (e20_fail == 1) | (e21_fail == 1) | (encoder_errors == 1) | (e_stop_pb == 1) | (stop_safety_chain == 1) |
		 (safe_stop == 1) | (e29_xtal == 1) | ((err_edge == 1) && (run_prog == 0)) | ((ols == 1) && (cls == 1))) // 08-05-2025
      {
			 if (fault_delay_1 == 0) 	aux1_off; // Relay ON when error when safety test ended 28-03-2025
			}
     else
		  {
		   aux1_on;
       fault_delay_1 = 150; // 28-03-2025
		  }			 
     break;
//
    case 50:
		 if ((warn_started == 1) && (warn_timer > 0)) // 11-12-2025_b
      {
       m_lock_ready = 1; // m_lock_ready is always ready when warning is used, because warning time i longer than the m_lock_tim _ready
      }
     else
      {	
	     if (start_command == 1)
        {
         aux1_on;
         m_lock_tim_off = 200; // preset for turn off time
         if (m_lock_tim_ready == 0) m_lock_ready = 1;
         else
          {
           m_lock_tim_ready--;
          }
        }
       else
        {
         m_lock_tim_ready = 200; // preset for turn on time
         if ((close_active()==0) && (open_active() == 0) && (m_lock_tim_off > 0) && (cls == 1)) // 10-12-2025
          {
           aux1_on; // keep relay on until off time ended
           m_lock_tim_off--; // down count for turn off relay
           m_lock_ready = 1; // 
          } 
         if ((m_lock_tim_off == 0) && (cls == 1)) // keeped relay on until CLS is active 29-05-2020
          {
           aux1_off;
           m_lock_ready = 0; // no start_command and m_lock_tim_off ended.
          }
			   else aux1_on; // 10-12-2025	
        }
		  }
     break;
    case 51:
     if ((close_active()==0) & (open_active()==0)) 
      {
       aux1_off;
       start_cap_tim = 500;
      }
     else
      {
       if (start_cap_tim > 0) // door is running up or down test timer status
        {
         start_cap_tim--;
         aux1_on; // starting capacitor active
        }
       else aux1_off; // starting capacitor deactivated 
      }
     break;
    case 52:
     if ((open_pb == 1) | (open_disp_pb == 1) | (imp_open == 1)) // 08-07-2026
      {
       yard_light_tim = 36036; // preset to 120 sec. by 3.33 mS main cycle time 11-12-2025_f
       aux1_on;
      }
     else if (yard_light_tim == 0) aux1_off;
		 else yard_light_tim--;
     break;
    case 53:
     if (dock_relay == 1) aux1_on;
     else aux1_off;
     break;
    case 54:
     if (dock_relay == 1) aux1_on;
     else aux1_off;
     break;
		case 55:
		 EE_read(EE_par_21); // change parameter 21 if not already done
     if (temp != 5)
     EE_write(EE_par_21,5);	
     wireless_relay = 1;	// 23-11-2023	 
		 break;
		case 56: // 04-03-2026
		 if ((warn_started == 1) && (warn_timer > 0) && (free_2 == 1)) // 
      {
       m_lock_ready = 1; // m_lock_ready is always ready when warning is used, because warning time i longer than the m_lock_tim _ready
      }
     else
      {	
	     if (start_command == 1)
        {
         aux1_on;
         m_lock_tim_off = 225; // preset for turn off time
         if (free_2 == 1)
				  {
					 e33_unlock_fail = 0;	
				   m_lock_ready = 1;
					 max_unlock_tim = 600;
				  }
         else
          {
           if (max_unlock_tim > 0) max_unlock_tim--;
					 else e33_unlock_fail = 1;	 
          }
        }
       else
        {
				 if ((close_active()==1) || (open_active() == 1))	// is door running
				  {
					 m_lock_tim_off = 225; // preset for turn off time
				  }
				 else	if (m_lock_tim_off == 0)
				  {
					 aux1_off;
					 m_lock_ready = 0; //
           max_unlock_tim = 600;						
				  }
				 else
				  {
           aux1_on; // keep relay on until off time ended
           m_lock_tim_off--; // down count for turn off relay
           m_lock_ready = 1; // 
				  }					 
			  }
		  }
     break; 
//
    default:
     break;
   }
  return;
 }
//***************************************************************************************************
void relay_aux2() // 16-09-2019 big change 16-06-2020
 {
  EE_read(EE_par_80); // 01-07-2021
  temp3 = temp; 
  if ((temp3 > 1) && (temp3 < 6) && (cls_trafic_tim > 0)) relay_ovr = 1; // relay ON on CLS
  else relay_ovr = 0; 
  EE_read(EE_par_72);
  switch (temp)
   {
    case 0:
     aux2_off;  // Relay off
     break;
    case 1:
     if ((relay_ovr == 1) && (cls == 1)) aux2_on; // relay ON on CLS 01-07-2021
     else if (warn_timer > 0) flash_aux2(); // 09-10-2019
     else if ((close_active() == 1) | (open_active() == 1)) aux2_on; // Relay ON when door running 
     else if ((ols == 0) && (cls == 0)) aux2_on; // Relay ON when stopped between limits 01-07-2021
     else aux2_off;
     break;
    case 2:
     if ((relay_ovr == 1) && (cls == 1)) aux2_on; // relay ON on CLS
     else if (warn_timer > 0) flash_aux2(); // not only on ols and h_ols 29-05-2020
     else if((close_active() == 1) | (open_active() == 1)) flash_aux2(); // Relay ON when door running 
     else if ((ols == 0) && (cls == 0)) flash_aux2(); // Relay flash when stopped between limits 01-07-2021
     else aux2_off;
     break;
    case 3:
     if ((relay_ovr == 1) && (cls == 1)) aux2_on; // relay ON on CLS
     else if (warn_timer > 0) aux2_on; // not only on ols and h_ols 18-06-2020 
     else if((close_active() == 1) | (open_active() == 1)) aux2_on; // Relay ON when door running
     else if ((ols == 0) && (cls == 0)) aux2_on; // Relay ON when stopped between limits 01-07-2021 
     else aux2_off; 
     break;
     case 4:
     if ((relay_ovr == 1) && (cls == 1)) aux2_on; // relay ON on CLS
     else if (warn_timer > 0) flash_aux2(); // warning before each running 29-05-2020
     else aux2_off;
     break;
    case 5:
     if (warn_timer > 0) flash_aux2(); // 29-05-2020
     else if (ols == 1) aux2_on; // 29-05-2020
     else aux2_off;
     break;
    case 6:
     if ((warn_timer > 0) || (warn_tim_delay > 0)) aux2_off; // 28-06-2021
     else if (ols == 1) aux2_on; // 
     else aux2_off;
     break;
		case 9:
     // do nothing here - controlled by AUX0
     break;
//
    case 10:
     if (ols == 1) aux2_on;
     else aux2_off;
     break;
    case 11:
     if (cls == 1) aux2_on;
     else aux2_off;
     break;
    case 12:
     if (ols == 0) aux2_on;
     else aux2_off;
     break;
    case 13:
     if (cls == 0) aux2_on;
     else aux2_off;
     break;
    case 14:
     EE_read(EE_par_16);
		 if ((temp < 2) && (h_ols == 1) && (ols == 0)) aux2_on; // 29-05-2020 18-06-2020 
     else if ((temp > 1) && (door_pos_half == 1) && (close_active()==0) && (open_active()==0)) aux2_on; // 29-05-2020 18-06-2020 08-06-2021 05-12-2025
     else aux2_off;
     break;
//
    case 20:
     if ((close_active()==0) && (open_active()==0)) imp_signal = 0; // ready for new impulse when stopped
     if (open_imp_tim_aux2 > 0) open_imp_tim_aux2--; //
     else 
      {
       EE_read(EE_par_16);
			 if (((open_pb == 1) | (open_disp_pb == 1) || ((temp > 2) && (e_h_ols_sw == 1) && (door_pos_low == 1) && (close_active()==0))
				 || (imp_open == 1)) && (imp_signal == 0)) // 08-07-2026
				 // 25-05-2021 08-12-2025 09-12-2025	
        {
         open_imp_tim_aux2 = 500;
         imp_signal = 1;
        }
      }
     if (open_imp_tim_aux2 > 0) aux2_on; 
     else aux2_off;
     break;
    case 21:
		 if (ols == 0)
			 {
			  open_imp_tim_aux2 = 1000; // 2 sec. reload
				aux2_off; // Turn off if set 08-12-2025
			 }
      else if (door_pos_half == 1) // false OLS, do not turn on yet 08-12-2025
			 {
			 }
	    else		
			 {
        if (open_imp_tim_aux2 > 0) open_imp_tim_aux2--; // OLS == 1
        if (open_imp_tim_aux2 > 0) aux2_on; 
        else aux2_off;
       }	
		 break;
 //    
    case 25:
     if ((close_active()==1) | (open_active()==1)) aux2_on; // Brake, relay is activated when motor is running.
     else aux2_off;
     break;
    case 26:
     if ((close_active()==1) | (open_active()==1)) aux2_off; // Brake negated, relay is activated when motor is not running.
     else aux2_on;
     break;
 //   
    case 30:
     if (bcl_on == 1) aux2_on; // is before close limit active 26-11-2021
     else aux2_off;
     break;
    case 31:
     if (bol_on == 1) aux2_on; // is before open limit active 26-11-2021
     else aux2_off;
     break;
    case 32:
     if (open_active()==1) aux2_on;
     else  aux2_off;
     break;
    case 33:
     if (close_active()==1) aux2_on;
     else  aux2_off;
     break;
    case 34:
      if ((close_active()==1) | (open_active()==1)) aux2_on;
      else  aux2_off;
     break;
//
    case 40:
		 if ((e1_mon == 1) | (e7_tacho == 1) | (e2_speed == 1) | (e3_run == 1) | (e4_ser == 1) | (e5_photo == 1) | (e6_edge == 1) | (e10_edge == 1) | (e8_wear == 1) | 
     (e9_pos_change == 1) | (e20_fail == 1) | (e21_fail == 1) | (encoder_errors == 1) | (e_stop_pb == 1) | (stop_safety_chain == 1) |
		 (safe_stop == 1) | (e29_xtal == 1) | ((err_edge == 1) && (run_prog == 0)) | ((ols == 1) && (cls == 1))) // 08-05-2025
      {
			 if (fault_delay_1 == 0) 	aux2_on; // Relay ON when error when safety test ended 28-03-2025
			}
     else
		  {
		   aux2_off;
       fault_delay_1 = 150; // 28-03-2025
		  }			 
     break;	
    case 41:
      if ((((edgetype == 3)&&(fraba == 1)) | ((edgetype != 3) && ((dw8k2 == 1) ))) && (hide_timer == 0)) aux2_on; // Relay ON 
      else if (((e6_edge == 1) || (e1_mon == 1)) && ((safety_test != 1) && (safety_after_time == 0))) // 11-12-2025_e
			 {
			aux2_on; // if adge active or if error on edge
			 }
      else aux2_off;
     break;
    case 42:
      if (e4_ser == 1) aux2_on;
      else  aux2_off;
     break;
    case 43:
		  EE_read(EE_par_31);
      if ((temp == 4) && (photo_pos_observed == 1) && (photo_learn == 0)) photo1_dis = 1; // 11-12-2025_c
      else photo1_dis = 0;
		  if ((((photo1 == 1) & (photo1_dis == 0)) | (e5_photo == 1)) && (hide_timer == 0) && ((safety_test != 1) && (safety_after_time == 0)))
			 {
			  aux2_on; // 11-12-2025_c 11-12-2025_e
			 }	
	    else  aux2_off;
     break;
    case 44: // aux2_relay is controlled ecode1 error on encoder processor 07-04-2021
      if ((ecode1 == 1) | (e9_pos_change == 1)) aux2_on; // AUX2 ON by ecode1 fail - no answer on encoder and no position change
      else aux2_off;  // AUX2 OFF 
     break;
		case 45:
     if ((e1_mon == 1) | (e7_tacho == 1) | (e2_speed == 1) | (e3_run == 1) | (e4_ser == 1) | (e5_photo == 1) | (e6_edge == 1) | (e10_edge == 1) | (e8_wear == 1) | 
     (e9_pos_change == 1) | (e20_fail == 1) | (e21_fail == 1) | (encoder_errors == 1) | (e_stop_pb == 1) | (stop_safety_chain == 1) |
		 (safe_stop == 1) | (e29_xtal == 1) | ((err_edge == 1) && (run_prog == 0)) | ((ols == 1) && (cls == 1))) // 08-05-2025
      {
			 if (fault_delay_1 == 0) 	aux2_off; // Relay ON when error when safety test ended 28-03-2025
			}
     else
		  {
		   aux2_on;
       fault_delay_1 = 150; // 28-03-2025
		  }			 
     break;
//
    case 50:
     if ((warn_started == 1) && (warn_timer > 0)) // 11-12-2025_b
      {
       m_lock_ready = 1; // m_lock_ready is always ready when warning is used, because warning time i longer than the m_lock_tim _ready
      }
     else
      {	 
	 		 if (start_command == 1)
        {
         aux2_on;
         m_lock_tim_off = 200; // preset for turn off time
         if (m_lock_tim_ready == 0) m_lock_ready = 1;
         else
          {
           m_lock_tim_ready--;
          }
        }
       else
        {
         m_lock_tim_ready = 200; // preset for turn on time
			   if ((close_active()==0) && (open_active() == 0) && (m_lock_tim_off > 0) && (cls == 1)) // 10-12-2025	
          {
           aux2_on; // keep relay on until off time ended
           m_lock_tim_off--; // down count for turn off relay
           m_lock_ready = 1; // 
          } 
         if ((m_lock_tim_off == 0) && (cls == 1)) // keeped relay on until CLS is active 29-05-2020
          {
           aux2_off;
           m_lock_ready = 0; // no start_command and m_lock_tim_off ended.
          }
			   else aux2_on; // 10-12-2025		
        }
		  }
     break;
    case 51:
     if ((close_active()==0) & (open_active()==0)) 
      {
       aux2_off;
       start_cap_tim = 500;
      }
     else
      {
       if (start_cap_tim > 0) // door is running up or down test timer status
        {
         start_cap_tim--;
         aux2_on; // starting capacitor active
        }
       else aux2_off; // starting capacitor deactivated 
      }
     break;
    case 52:
     if ((open_pb == 1) | (open_disp_pb == 1) | (imp_open == 1)) // 08-07-2026
      {
       yard_light_tim = 36036; // preset to 120 sec. by 3.33 mS main cycle time 11-12-2025_f
       aux2_on;
      }
     else if (yard_light_tim == 0) aux2_off;
     else 
		  {			 
			 EE_read(EE_par_71); // check for already down count 11-12-2025
       if (temp != 52) yard_light_tim--;
		  } 
     break;
    case 53:
     if (dock_relay == 1) aux2_on;
     else aux2_off;
     break;
    case 54:
     if (dock_relay == 1) aux2_on;
     else aux2_off;
     break;
		case 55:
		 EE_read(EE_par_21); // change parameter 21 if not already done
     if (temp != 5)
     EE_write(EE_par_21,5);	
     wireless_relay = 2;		// 23-11-2023 
		 break;
		case 56: // 04-03-2026
		 if ((warn_started == 1) && (warn_timer > 0) && (free_2 == 1)) // 
      {
       m_lock_ready = 1; // m_lock_ready is always ready when warning is used, because warning time i longer than the m_lock_tim _ready
      }
     else
      {	
	     if (start_command == 1)
        {
         aux2_on;
         m_lock_tim_off = 225; // preset for turn off time
         if (free_2 == 1)
				  {
					 e33_unlock_fail = 0;	
				   m_lock_ready = 1;
					 max_unlock_tim = 600;
				  }
         else
          {
           if (max_unlock_tim > 0) max_unlock_tim--;
					 else e33_unlock_fail = 1;	 
          }
        }
       else
        {
				 if ((close_active()==1) || (open_active() == 1))	// is door running
				  {
					 m_lock_tim_off = 225; // preset for turn off time
				  }
				 else	if (m_lock_tim_off == 0)
				  {
					 aux2_off;
					 m_lock_ready = 0; //
           max_unlock_tim = 600;						
				  }
				 else
				  {
           aux2_on; // keep relay on until off time ended
           m_lock_tim_off--; // down count for turn off relay
           m_lock_ready = 1; // 
				  }					 
			  }
		  }
     break; 
//
    default:
     break;
   }
  return;
 }
//***************************************************************************************************
//***************************************************************************************************
//
void relay_aux4() // 16-09-2019 big change 16-06-2020
 {
  EE_read(EE_par_80); // 01-07-2021
  temp3 = temp; 
  if ((temp3 > 1) && (temp3 < 6) && (cls_trafic_tim > 0)) relay_ovr = 1; // relay ON on CLS
  else relay_ovr = 0; 
  EE_read(EE_par_74);
  switch (temp)
   {
    case 0:
     aux4_off;  // Relay off
     break;
    case 1:
     if ((relay_ovr == 1) && (cls == 1)) aux4_on; // relay ON on CLS 01-07-2021
     else if (warn_timer > 0) flash_aux4(); // 09-10-2019
     else if ((close_active() == 1) | (open_active() == 1)) aux4_on; // Relay ON when door running 
     else if ((ols == 0) && (cls == 0)) aux4_on; // Relay ON when stopped between limits 01-07-2021
     else aux4_off;
     break;
    case 2:
     if ((relay_ovr == 1) && (cls == 1)) aux4_on; // relay ON on CLS
     else if (warn_timer > 0) flash_aux4(); // not only on ols and h_ols 29-05-2020
     else if((close_active() == 1) | (open_active() == 1)) flash_aux4(); // Relay ON when door running 
     else if ((ols == 0) && (cls == 0)) flash_aux4(); // Relay flash when stopped between limits 01-07-2021
     else aux4_off;
     break;
    case 3:
     if ((relay_ovr == 1) && (cls == 1)) aux4_on; // relay ON on CLS
     else if (warn_timer > 0) aux4_on; // not only on ols and h_ols 18-06-2020 
     else if((close_active() == 1) | (open_active() == 1)) aux4_on; // Relay ON when door running
     else if ((ols == 0) && (cls == 0)) aux4_on; // Relay ON when stopped between limits 01-07-2021 
     else aux4_off; 
     break;
     case 4:
     if ((relay_ovr == 1) && (cls == 1)) aux4_on; // relay ON on CLS
     else if (warn_timer > 0) flash_aux4(); // warning before each running 29-05-2020
     else aux4_off;
     break;
    case 5:
     if (warn_timer > 0) flash_aux4(); // 29-05-2020
     else if (ols == 1) aux4_on; // 29-05-2020
     else aux4_off;
     break;
    case 6:
     if ((warn_timer > 0) || (warn_tim_delay > 0)) aux4_off; // 28-06-2021
     else if (ols == 1) aux4_on; // 
     else aux4_off;
     break;
//
    case 10:
     if (ols == 1) aux4_on;
     else aux4_off;
     break;
    case 11:
     if (cls == 1) aux4_on;
     else aux4_off;
     break;
    case 12:
     if (ols == 0) aux4_on;
     else aux4_off;
     break;
    case 13:
     if (cls == 0) aux4_on;
     else aux4_off;
     break;
    case 14:
     EE_read(EE_par_16);
     if ((temp < 2) && (h_ols == 1) && (ols == 0)) aux4_on; // 29-05-2020 18-06-2020 
     else if ((temp > 1) && (door_pos_half == 1) && (close_active()==0) && (open_active()==0)) aux4_on; // 29-05-2020 18-06-2020 08-06-2021 05-12-2025
     else aux4_off;
     break;
//
    case 20:
     if ((close_active()==0) && (open_active()==0)) imp_signal = 0; // ready for new impulse when stopped
     if (open_imp_tim_aux4 > 0) open_imp_tim_aux4--; //
     else 
      {
       EE_read(EE_par_16);
			 if (((open_pb == 1) | (open_disp_pb == 1) || ((temp > 2) && (e_h_ols_sw == 1) && (door_pos_low == 1) && (close_active()==0)) 
				 || (imp_open == 1)) && (imp_signal == 0)) // 08-07-2026
				 // 25-05-2021 08-12-2025 09-12-2025	
        {
         open_imp_tim_aux4 = 500;
         imp_signal = 1;
        }
      }
     if (open_imp_tim_aux4 > 0) aux4_on; 
     else aux4_off;
     break;
    case 21:
			if (ols == 0)
			 {
			  open_imp_tim_aux4 = 1000; // 2 sec. reload
				aux4_off; // Turn off if set 08-12-2025
			 }
      else if (door_pos_half == 1) // false OLS, do not turn on yet 08-12-2025
			 {
			 }
	    else		
			 {
        if (open_imp_tim_aux4 > 0) open_imp_tim_aux4--; // OLS == 1
        if (open_imp_tim_aux4 > 0) aux4_on; 
        else aux4_off;
       }
		 break;
 //    
    case 25:
     if ((close_active()==1) | (open_active()==1)) aux4_on; // Brake, relay is activated when motor is running.
     else aux4_off;
     break;
    case 26:
     if ((close_active()==1) | (open_active()==1)) aux4_off; // Brake negated, relay is activated when motor is not running.
     else aux4_on;
     break;
 //   
    case 30:
     if (bcl_on == 1) aux4_on; // is before close limit active 26-11-2021
     else aux4_off;
     break;
    case 31:
     if (bol_on == 1) aux4_on; // is before open limit active 26-11-2021
     else aux4_off;
     break;
    case 32:
     if (open_active()==1) aux4_on;
     else  aux4_off;
     break;
    case 33:
     if (close_active()==1) aux4_on;
     else  aux4_off;
     break;
    case 34:
      if ((close_active()==1) | (open_active()==1)) aux4_on;
      else  aux4_off;
     break;
//
    case 40:
		 if ((e1_mon == 1) | (e7_tacho == 1) | (e2_speed == 1) | (e3_run == 1) | (e4_ser == 1) | (e5_photo == 1) | (e6_edge == 1) | (e10_edge == 1) | (e8_wear == 1) | 
     (e9_pos_change == 1) | (e20_fail == 1) | (e21_fail == 1) | (encoder_errors == 1) | (e_stop_pb == 1) | (stop_safety_chain == 1) |
		 (safe_stop == 1) | (e29_xtal == 1) | ((err_edge == 1) && (run_prog == 0))| ((ols == 1) && (cls == 1))) // 08-05-2025
      {
			 if (fault_delay_1 == 0) 	aux4_on; // Relay ON when error when safety test ended 28-03-2025
			}
     else
		  {
		   aux4_off;
       fault_delay_1 = 150; // 28-03-2025
		  }			 
     break;	
	  case 41:
      if ((((edgetype == 3)&&(fraba == 1)) | ((edgetype != 3) && (dw8k2 == 1))) && (hide_timer == 0)) aux4_on; // Relay ON
      else if (((e6_edge == 1) || (e1_mon == 1)) && ((safety_test != 1) && (safety_after_time == 0))) // 11-12-2025_e
			 {
			aux4_on; // if adge active or if error on edge
			 }
      else aux4_off;
     break;
    case 42:
      if (e4_ser == 1) aux4_on;
      else  aux4_off;
     break;
    case 43:
			EE_read(EE_par_31);
      if ((temp == 4) && (photo_pos_observed == 1) && (photo_learn == 0)) photo1_dis = 1; // 11-12-2025_c
      else photo1_dis = 0;
		  if ((((photo1 == 1) & (photo1_dis == 0)) | (e5_photo == 1)) && (hide_timer == 0) && ((safety_test != 1) && (safety_after_time == 0)))
			 {
			  aux4_on; // 11-12-2025_c 11-12-2025_e
			 }	
	    else  aux4_off;
     break;
    case 44: // aux4_relay is controlled ecode1 error on encoder processor 07-04-2021
      if ((ecode1 == 1) | (e9_pos_change == 1)) aux4_on; // AUX4 ON by ecode1 fail - no answer on encoder and no position change
      else aux4_off;  // AUX4 OFF 
     break;
		case 45:
     if ((e1_mon == 1) | (e7_tacho == 1) | (e2_speed == 1) | (e3_run == 1) | (e4_ser == 1) | (e5_photo == 1) | (e6_edge == 1) | (e10_edge == 1) | (e8_wear == 1) | 
     (e9_pos_change == 1) | (e20_fail == 1) | (e21_fail == 1) | (encoder_errors == 1) | (e_stop_pb == 1) | (stop_safety_chain == 1) |
		 (safe_stop == 1) | (e29_xtal == 1) | ((err_edge == 1) && (run_prog == 0))| ((ols == 1) && (cls == 1))) // 08-05-2025
      {
			 if (fault_delay_1 == 0) 	aux4_off; // Relay ON when error when safety test ended 28-03-2025
			}
     else
		  {
		   aux4_on;
       fault_delay_1 = 150; // 28-03-2025
		  }			 
     break;
//
    case 50:
		 if ((warn_started == 1) && (warn_timer > 0)) // 11-12-2025_b
      {
       m_lock_ready = 1; // m_lock_ready is always ready when warning is used, because warning time i longer than the m_lock_tim _ready
      }
     else
      {		
	     if (start_command == 1)
        {
         aux4_on;
         m_lock_tim_off = 200; // preset for turn off time
         if (m_lock_tim_ready == 0) m_lock_ready = 1;
         else
          {
           m_lock_tim_ready--;
          }
        }
       else
        {
         m_lock_tim_ready = 200; // preset for turn on time
			   if ((close_active()==0) && (open_active() == 0) && (m_lock_tim_off > 0) && (cls == 1)) // 10-12-2025	
          {
           aux4_on; // keep relay on until off time ended
           m_lock_tim_off--; // down count for turn off relay
           m_lock_ready = 1; // 
          } 
         if ((m_lock_tim_off == 0) && (cls == 1)) // keeped relay on until CLS is active 29-05-2020
          {
           aux4_off;
           m_lock_ready = 0; // no start_command and m_lock_tim_off ended.
          }
			   else aux4_on; // 10-12-2025		
        }
		  }
     break;
    case 51:
     if ((close_active()==0) & (open_active()==0)) 
      {
       aux4_off;
       start_cap_tim = 500;
      }
     else
      {
       if (start_cap_tim > 0) // door is running up or down test timer status
        {
         start_cap_tim--;
         aux4_on; // starting capacitor active
        }
       else aux4_off; // starting capacitor deactivated 
      }
     break;
    case 52:
     if ((open_pb == 1) | (open_disp_pb == 1) | (imp_open == 1)) // 08-07-2026
      {
       yard_light_tim = 36036; // preset to 120 sec. by 3.33 mS main cycle time 11-12-2025_f
       aux4_on;
      }
     else if (yard_light_tim == 0) aux4_off;
     else 
			{
       EE_read(EE_par_71); // check for already down count 11-12-2025
       temp3 = temp;
       EE_read(EE_par_72);
       if ((temp != 52) && (temp3 != 52)) yard_light_tim--;
      }
     break;
    case 53:
     if (dock_relay == 1) aux4_on;
     else aux4_off;
     break;
    case 54:
     if (dock_relay == 1) aux4_on;
     else aux4_off;
     break;
		case 55:
		 EE_read(EE_par_21); // change parameter 21 if not already done
     if (temp != 5)
     EE_write(EE_par_21,5);	
     wireless_relay = 4;		 
		 break;
		case 56: // 04-03-2026
		 if ((warn_started == 1) && (warn_timer > 0) && (free_2 == 1)) // 
      {
       m_lock_ready = 1; // m_lock_ready is always ready when warning is used, because warning time i longer than the m_lock_tim _ready
      }
     else
      {	
	     if (start_command == 1)
        {
         aux4_on;
         m_lock_tim_off = 225; // preset for turn off time
         if (free_2 == 1)
				  {
					 e33_unlock_fail = 0;	
				   m_lock_ready = 1;
					 max_unlock_tim = 600;
				  }
         else
          {
           if (max_unlock_tim > 0) max_unlock_tim--;
					 else e33_unlock_fail = 1;	 
          }
        }
       else
        {
				 if ((close_active()==1) || (open_active() == 1))	// is door running
				  {
					 m_lock_tim_off = 225; // preset for turn off time
				  }
				 else	if (m_lock_tim_off == 0)
				  {
					 aux4_off;
					 m_lock_ready = 0; //
           max_unlock_tim = 600;						
				  }
				 else
				  {
           aux4_on; // keep relay on until off time ended
           m_lock_tim_off--; // down count for turn off relay
           m_lock_ready = 1; // 
				  }					 
			  }
		  }
     break; 
//
    default:
     break;
   }
  return;
 }
//***************************************************************************************************
//**************************************************************************
void relay_aux5() // 16-09-2019 big change 16-06-2020
 {
  EE_read(EE_par_80); // 01-07-2021
  temp3 = temp; 
  if ((temp3 > 1) && (temp3 < 6) && (cls_trafic_tim > 0)) relay_ovr = 1; // relay ON on CLS
  else relay_ovr = 0; 
  EE_read(EE_par_75);
  switch (temp)
   {
    case 0:
     aux5_off;  // Relay off
     break;
    case 1:
     if ((relay_ovr == 1) && (cls == 1)) aux5_on; // relay ON on CLS 01-07-2021
     else if (warn_timer > 0) flash_aux5(); // 09-10-2019
     else if ((close_active() == 1) | (open_active() == 1)) aux5_on; // Relay ON when door running 
     else if ((ols == 0) && (cls == 0)) aux5_on; // Relay ON when stopped between limits 01-07-2021
     else aux5_off;
     break;
    case 2:
     if ((relay_ovr == 1) && (cls == 1)) aux5_on; // relay ON on CLS
     else if (warn_timer > 0) flash_aux5(); // not only on ols and h_ols 29-05-2020
     else if((close_active() == 1) | (open_active() == 1)) flash_aux5(); // Relay ON when door running 
     else if ((ols == 0) && (cls == 0)) flash_aux5(); // Relay flash when stopped between limits 01-07-2021
     else aux5_off;
     break;
    case 3:
     if ((relay_ovr == 1) && (cls == 1)) aux5_on; // relay ON on CLS
     else if (warn_timer > 0) aux5_on; // not only on ols and h_ols 18-06-2020 
     else if((close_active() == 1) | (open_active() == 1)) aux5_on; // Relay ON when door running
     else if ((ols == 0) && (cls == 0)) aux5_on; // Relay ON when stopped between limits 01-07-2021 
     else aux5_off; 
     break;
     case 4:
     if ((relay_ovr == 1) && (cls == 1)) aux5_on; // relay ON on CLS
     else if (warn_timer > 0) flash_aux5(); // warning before each running 29-05-2020
     else aux5_off;
     break;
    case 5:
     if (warn_timer > 0) flash_aux5(); // 29-05-2020
     else if (ols == 1) aux5_on; // 29-05-2020
     else aux5_off;
     break;
    case 6:
     if ((warn_timer > 0) || (warn_tim_delay > 0)) aux5_off; // 28-06-2021
     else if (ols == 1) aux5_on; // 
     else aux5_off;
     break;
//
    case 10:
     if (ols == 1) aux5_on;
     else aux5_off;
     break;
    case 11:
     if (cls == 1) aux5_on;
     else aux5_off;
     break;
    case 12:
     if (ols == 0) aux5_on;
     else aux5_off;
     break;
    case 13:
     if (cls == 0) aux5_on;
     else aux5_off;
     break;
    case 14:
     EE_read(EE_par_16);
     if ((temp < 2) && (h_ols == 1) && (ols == 0)) aux5_on; // 29-05-2020 18-06-2020 
     else if ((temp > 1) && (door_pos_half == 1) && (close_active()==0) && (open_active()==0)) aux5_on; // 29-05-2020 18-06-2020 08-06-2021 05-12-2025
     else aux5_off;
     break;
//
    case 20:
     if ((close_active()==0) && (open_active()==0)) imp_signal = 0; // ready for new impulse when stopped
     if (open_imp_tim_aux5 > 0) open_imp_tim_aux5--; //
     else 
      {
       EE_read(EE_par_16); 
			 if (((open_pb == 1) | (open_disp_pb == 1) || ((temp > 2) && (e_h_ols_sw == 1) && (door_pos_low == 1) && (close_active()==0)) 
				 || (imp_open == 1)) && (imp_signal == 0)) // 08-07-2026
				 // 25-05-2021 08-12-2025 09-12-2025
			  {
         open_imp_tim_aux5 = 500;
         imp_signal = 1;
        }
      }
     if (open_imp_tim_aux5 > 0) aux5_on; 
     else aux5_off;
     break;
 //			
    case 21:
			if (ols == 0)
			 {
			  open_imp_tim_aux5 = 1000; // 2 sec. reload
				aux5_off; // Turn off if set 08-12-2025
			 }
      else if (door_pos_half == 1) // false OLS, do not turn on yet 08-12-2025
			 {
			 }
	    else		
			 {
        if (open_imp_tim_aux5 > 0) open_imp_tim_aux5--; // OLS == 1
        if (open_imp_tim_aux5 > 0) aux5_on; 
        else aux5_off;
       }
	   break;
 //    
    case 25:
     if ((close_active()==1) | (open_active()==1)) aux5_on; // Brake, relay is activated when motor is running.
     else aux5_off;
     break;
    case 26:
     if ((close_active()==1) | (open_active()==1)) aux5_off; // Brake negated, relay is activated when motor is not running.
     else aux5_on;
     break;
 //   
    case 30:
     if (bcl_on == 1) aux5_on; // is before close limit active 26-11-2021 
     else aux5_off;
     break;
    case 31:
     if (bol_on == 1) aux5_on; // is before open limit active 26-11-2021
     else aux5_off;
     break;
    case 32:
     if (open_active()==1) aux5_on;
     else  aux5_off;
     break;
    case 33:
     if (close_active()==1) aux5_on;
     else  aux5_off;
     break;
    case 34:
      if ((close_active()==1) | (open_active()==1)) aux5_on;
      else  aux5_off;
     break;
//
    case 40:
		 if ((e1_mon == 1) | (e7_tacho == 1) | (e2_speed == 1) | (e3_run == 1) | (e4_ser == 1) | (e5_photo == 1) | (e6_edge == 1) | (e10_edge == 1) | (e8_wear == 1) | 
     (e9_pos_change == 1) | (e20_fail == 1) | (e21_fail == 1) | (encoder_errors == 1) | (e_stop_pb == 1) | (stop_safety_chain == 1) |
		 (safe_stop == 1) | (e29_xtal == 1) | ((err_edge == 1) && (run_prog == 0))| ((ols == 1) && (cls == 1))) // 08-05-2025
      {
			 if (fault_delay_1 == 0) 	aux5_on; // Relay ON when error when safety test ended 28-03-2025
			}
     else
		  {
		   aux5_off;
       fault_delay_1 = 150; // 28-03-2025
		  }			 
     break;	
    case 41:
      if ((((edgetype == 3)&&(fraba == 1)) | ((edgetype != 3) && (dw8k2 == 1))) && (hide_timer == 0)) aux5_on; // Relay ON
      else if (((e6_edge == 1) || (e1_mon == 1)) && ((safety_test != 1) && (safety_after_time == 0))) // 11-12-2025_e
			 {
			aux5_on; // if adge active or if error on edge
			 }
      else aux5_off;
     break;
    case 42:
      if (e4_ser == 1) aux5_on;
      else  aux5_off;
     break;
    case 43:
			EE_read(EE_par_31);
      if ((temp == 4) && (photo_pos_observed == 1) && (photo_learn == 0)) photo1_dis = 1; // 11-12-2025_c
      else photo1_dis = 0;
		  if ((((photo1 == 1) & (photo1_dis == 0)) | (e5_photo == 1)) && (hide_timer == 0) && ((safety_test != 1) && (safety_after_time == 0)))
			 {
			  aux5_on; // 11-12-2025_c 11-12-2025_e
			 }	
	    else  aux5_off;
     break;
    case 44: // aux5_relay is controlled ecode1 error on encoder processor 07-04-2021
      if ((ecode1 == 1) | (e9_pos_change == 1)) aux5_on; // AUX5 ON by ecode1 fail - no answer on encoder and no position change
      else aux5_off;  // AUX5 OFF 
     break;
		case 45:
     if ((e1_mon == 1) | (e7_tacho == 1) | (e2_speed == 1) | (e3_run == 1) | (e4_ser == 1) | (e5_photo == 1) | (e6_edge == 1) | (e10_edge == 1) | (e8_wear == 1) | 
     (e9_pos_change == 1) | (e20_fail == 1) | (e21_fail == 1) | (encoder_errors == 1) | (e_stop_pb == 1) | (stop_safety_chain == 1) |
		 (safe_stop == 1) | (e29_xtal == 1) | ((err_edge == 1) && (run_prog == 0))| ((ols == 1) && (cls == 1))) // 08-05-2025
      {
			 if (fault_delay_1 == 0) 	aux5_off; // Relay ON when error when safety test ended 28-03-2025
			}
     else
		  {
		   aux5_on;
       fault_delay_1 = 150; // 28-03-2025
		  }			 
     break;
//
    case 50:
		 if ((warn_started == 1) && (warn_timer > 0)) // 11-12-2025_b
      {
       m_lock_ready = 1; // m_lock_ready is always ready when warning is used, because warning time i longer than the m_lock_tim _ready
      }
     else
      {		
       if (start_command == 1)
        {
         aux5_on;
         m_lock_tim_off = 200; // preset for turn off time
         if (m_lock_tim_ready == 0) m_lock_ready = 1;
         else
          {
           m_lock_tim_ready--;
          }
        }
       else
        {
         m_lock_tim_ready = 200; // preset for turn on time
			   if ((close_active()==0) && (open_active() == 0) && (m_lock_tim_off > 0) && (cls == 1)) // 10-12-2025	
          {
           aux5_on; // keep relay on until off time ended
           m_lock_tim_off--; // down count for turn off relay
           m_lock_ready = 1; // 
          } 
         if ((m_lock_tim_off == 0) && (cls == 1)) // keeped relay on until CLS is active 29-05-2020
          {
           aux5_off;
           m_lock_ready = 0; // no start_command and m_lock_tim_off ended.
          }
			   else aux5_on; // 10-12-2025		
        }
		  }
     break;
    case 51:
     if ((close_active()==0) & (open_active()==0)) 
      {
       aux5_off;
       start_cap_tim = 500;
      }
     else
      {
       if (start_cap_tim > 0) // door is running up or down test timer status
        {
         start_cap_tim--;
         aux5_on; // starting capacitor active
        }
       else aux5_off; // starting capacitor deactivated 
      }
     break;
    case 52:
     if ((open_pb == 1) | (open_disp_pb == 1) | (imp_open == 1)) // 08-07-2026
      {
       yard_light_tim = 36036; // preset to 120 sec. by 3.33 mS main cycle time 11-12-2025_f
       aux5_on;
      }
     else if (yard_light_tim == 0) aux5_off;
     else 
			 {
       EE_read(EE_par_71); // check for already down count 11-12-2025
       temp3 = temp;
       EE_read(EE_par_72);
			 temp1 = temp;	 
			 EE_read(EE_par_74);	 
       if ((temp != 52) && (temp1 != 52) && (temp3 != 52)) yard_light_tim--;
      }
     break;
    case 53:
     if (dock_relay == 1) aux5_on;
     else aux5_off;
     break;
    case 54:
     if (dock_relay == 1) aux5_on;
     else aux5_off;
     break;
		case 55:
		 EE_read(EE_par_21); // change parameter 21 if not already done
     if (temp != 5)
     EE_write(EE_par_21,5);	
     wireless_relay = 5;		 
		 break;
		case 56: // 04-03-2026
		 if ((warn_started == 1) && (warn_timer > 0) && (free_2 == 1)) // 
      {
       m_lock_ready = 1; // m_lock_ready is always ready when warning is used, because warning time i longer than the m_lock_tim _ready
      }
     else
      {	
	     if (start_command == 1)
        {
         aux5_on;
         m_lock_tim_off = 225; // preset for turn off time
         if (free_2 == 1)
				  {
					 e33_unlock_fail = 0;	
				   m_lock_ready = 1;
					 max_unlock_tim = 600;
				  }
         else
          {
           if (max_unlock_tim > 0) max_unlock_tim--;
					 else e33_unlock_fail = 1;	 
          }
        }
       else
        {
				 if ((close_active()==1) || (open_active() == 1))	// is door running
				  {
					 m_lock_tim_off = 225; // preset for turn off time
				  }
				 else	if (m_lock_tim_off == 0)
				  {
					 aux5_off;
					 m_lock_ready = 0; //
           max_unlock_tim = 600;						
				  }
				 else
				  {
           aux5_on; // keep relay on until off time ended
           m_lock_tim_off--; // down count for turn off relay
           m_lock_ready = 1; // 
				  }					 
			  }
		  }
     break; 
//
    default:
     break;
   }
  return;
 }
//**************************************************************************
void flash_aux1()
{
 if (flash_aux1_tim > (flash_aux4_aux5_val/2)) aux1_on; // 27-05-2020
 else aux1_off;
 if (flash_aux1_tim > 0) flash_aux1_tim--;
 else flash_aux1_tim = flash_aux4_aux5_val; // 27-05-2020
}
//************************************************************************** 
void flash_aux2()
{
 if (flash_aux2_tim > (flash_aux4_aux5_val/2)) aux2_on; // 27-05-2020
 else aux2_off;
 if (flash_aux2_tim > 0) flash_aux2_tim--;
 else flash_aux2_tim = flash_aux4_aux5_val; // 27-05-2020
}
//************************************************************************** 
void flash_aux4()
{
 if (flash_aux4_tim > (flash_aux4_aux5_val/2)) aux4_on; // 27-05-2020
 else aux4_off;
 if (flash_aux4_tim > 0) flash_aux4_tim--;
 else flash_aux4_tim = flash_aux4_aux5_val; // 27-05-2020
}
//**************************************************************************
void flash_aux5()
{
 if (flash_aux5_tim > (flash_aux4_aux5_val/2)) aux5_on; // 27-05-2020
 else aux5_off;
 if (flash_aux5_tim > 0) flash_aux5_tim--;
 else flash_aux5_tim = flash_aux4_aux5_val; // 27-05-2020
}
//************************************************************************** 
//**************************************************************************
void read_flash_freq() // 27-05-2020
{
 EE_read(EE_par_79);
 switch (temp)
  {
   case 1:
   flash_aux4_aux5_val = 720; // reload value for 25 flashes/min
   break;
   case 2:
   flash_aux4_aux5_val = 450; // reload value for 40 flashes/min
   break;
   case 3:
   flash_aux4_aux5_val = 300; // reload value for 60 flashes/min
   break;
   case 4:
   flash_aux4_aux5_val = 200; // reload value for 90 flashes/min
   break;
   case 5:
   flash_aux4_aux5_val = 150; // reload value for 120 flashes/min
   break;
   default:
   break;   
  }
}
//************************************************************************** 
 
//**************************************************************************************************
//**************************************************************************
void test_for_warning()
{
 temp5 = 0; // temp5 is 0 if not set by the following of one of the relays setup	01-12-2021 
 EE_read(EE_par_71); // 01-12-2021  
 if ((temp > 0) && (temp < 7))	temp5 = 1; // 05-12-2025
 EE_read(EE_par_72); // 01-12-2021 
 if ((temp > 0) && (temp < 7))	temp5 = 1; // 05-12-2025	
 EE_read(EE_par_74); // 01-12-2021  
 if ((temp > 0) && (temp < 7))	temp5 = 1; // 05-12-2025
 EE_read(EE_par_75); // 01-12-2021  
 if ((temp > 0) && (temp < 7))	temp5 = 1; // 05-12-2025	
 //
 if (temp5 == 1) // 28-06-2021 08-09-2021 01-12-2021	
  {
   warning = 1; // 
   EE_read(EE_par_77);
   temp3 = temp;
   EE_read(EE_par_78);
   if ((temp == 0) && (temp3 == 0)) warning = 0; // warning = 0 but run down without warning 28-06-2021
  }
 else
  {
   EE_read(EE_warning);
   if (temp == 1) warning = 1;
   else warning = 0;
  }
} 
 
//************************************************************************** 
void chk_warn_open() // 09-10-2019
{
 if (((imp_close == 1) || (e1_mon == 1)) && (close_active() == 0) && (open_active() == 0)) // clear warning for close if this is running 30-09-2020
  {
   imp_close = 0;
   warn_timer = 0;
   warn_started = 0;
  }
 EE_read(EE_par_1);
 temp3 = temp;
 EE_read(EE_par_41);
 if (((temp > 2) && (temp < 5) && (bit_test(sk2,3) == 0)) || (temp3 < 2) || (run_prog == 1)) // if adaptive speed selected and learn not finished or deadman open selected
  {
   wait_open = 0;
  }
 else 
  {
   test_for_warning();
   if (warning == 0)
    {
     wait_open = 0;
    }
   else if ((warn_started == 0) && (close_active() == 0) && (open_active() == 0) && (rev_bit == 0))
    {
     EE_read(EE_par_77);
	 warn_timer = temp; // forwarning time for open
     warn_timer = (warn_timer * 300); // in seconds
     if (warn_timer > 0)
      {
       warn_started = 1;
       wait_open = 1;
      }
     else
      {
       wait_open = 0;
      }
    }
   else if (warn_timer == 0)
    {
     wait_open = 0;
    } 
   else
    {
     reload_run_timer();
     wait_open = 1;
    } 
  }
}
//**************************************************************************
void dock_ctrl_active()
{
	EE_read(EE_par_25);	//
  if (temp != 2)
	 {
    EE_write(EE_par_25, 2);  // Adjust to use h_ols input on free_1 if not already set for this.
   }		
	//
	if (temp3 == 1)     // 16-06-2020 28-06-2021
   {
		if ((door_pos_half == 1) && (dock_h_input == 0) && (i_kip == 0) && (open_active() == 0) && (close_active() == 0)) kip_down_ready = 1; // 09-12-2025
    if ((door_pos_half == 1) && (dock_h_input == 1) && (i_kip == 0) && (open_active() == 0) && (close_active() == 0)) kip_down_ready = 0; // 09-12-2025 
    dock_53_active = 0; // 29-10-2021
    dock_54_active = 1;
    door_half_cmd = 0; // no half open command if not set by the following
    if ((open_pb == 1) | (open_disp_pb == 1) || (imp_open == 1)) // 08-07-2026
     {
      if ((dock_h_input == 0) && (door_pos_low == 1)) 
       {
        door_half_cmd = 1;
       }
      else if ((dock_h_input == 1) && ((door_pos_half == 1) || (door_pos_high == 1)))
       {
       }
      else stop();
     }
    else if ((close_pb == 1) || (close_disp_pb) || (imp_close == 1)) // 08-07-2026
     {
      if ((dock_h_input == 1) && (door_pos_high == 1)) 
       {
        door_half_cmd = 1;
       }
      else if ((dock_h_input == 0) && ((door_pos_half == 1) || (door_pos_low == 1)))
       {
       }
      else
       {
        stop();
       }
     }
    else
     {
     }
    if (door_pos_half == 1) dock_relay = 1;
    else dock_relay = 0;
   }
  else if (temp4 == 1)
   {
    dock_54_active = 0;
    dock_53_active = 1; // 29-10-2021
    if (ols == 1) dock_relay = 1;
    else dock_relay = 0;
    if (dock_h_input == 1) // 29-10-2021
     {
      stop();
     }
   }
	else
	 {
    delay_us(1); // for debug only, should not be possible to ge here 
	 }		
}
//**************************************************************************************************
void dock_leveler()
{
 if (run_prog == 1)
  {
  }
 else
  {
	 temp3 = 0;	// temp 3 is 0 if not set by the following	01-12-2021
	 temp4 = 0; // temp 4 is 0 if not set by the following	01-12-2021
	 EE_read(EE_par_71); // Is dock leveler fully open selected on one of the relays
	 if (temp == 54) temp3 = 1;	
   if (temp == 53) temp4 = 1;
   EE_read(EE_par_72); // Is dock leveler fully open selected on one of the relays
	 if (temp == 54) temp3 = 1;
   if (temp == 53) temp4 = 1;
   EE_read(EE_par_74); // Is dock leveler fully open selected on one of the relays
	 if (temp == 54) temp3 = 1;	
   if (temp == 53) temp4 = 1;
   EE_read(EE_par_75); // Is dock leveler fully open selected on one of the relays
	 if (temp == 54) temp3 = 1;	
	 if (temp == 53) temp4 = 1;	
   if ((temp3 == 1) || (temp4 == 1)) dock_ctrl_active();    // 16-06-2020 28-06-2021	
	 else 
	 {
		delay_us(1); // for debug only, no dock leveler wanted 
	 }
	}
}
//**************************************************************************************************

