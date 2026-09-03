#include "stm32f0xx_hal.h"
#include "Global_Var.h"
#include <Defines.h>
#include <main.h>
#include <eprom.h>
#include <modbus.h>
static HAL_StatusTypeDef status = HAL_OK; // definering af status og start initialisering er HAL_OK
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart7;
//extern void getModbusParam(uint32_t modbusParam, uint16_t* value);
extern void r_autoclose_timer(void);
extern void reload_run_timer(void);

extern void clr_val(void);
extern uint8_t open_active(void);           // special inverter functions
extern uint8_t close_active(void);          // special inverter functions
extern uint8_t new_calc;                    // Controlbit for new calculation required.
extern uint8_t inverter_use;			          // 1 = inverter use is active - else contactor use 09-11-2021

void send_to_display(void);
void receive_from_display(void);
void display_prog(void);
void display_run(void);
void parameter_adjust(void);
void chk_para_tst(void);
void req_val_change(uint8_t min, uint8_t max);
void req_val_change_b16(uint16_t min, uint16_t max);
void show_run(void);
void reload_ser_cnt(void);
void display_auto(void);
void show_status(void);
void count_status(void);
void show_L_errors(void);
void e1(void);
void e2(void);
void e3(void);
void e4(void);
void e5(void);
void e6(void);
void e7(void);
void e8(void);
void e9(void);
void e10(void);
void e20(void);
void e21(void);
void e22(void);
void e23(void);
void e24(void);
void e27(void);
void e28(void);
void e29(void);
void e32 (void);
void e33 (void);
void enc_ecode9(void);
void enc_ecode8(void);
void enc_ecode7(void);
void enc_ecode4(void);
//void enc_ecode3(void);
void enc_ecode2(void);
void enc_ecode1(void);
void no_error(void);
void select_error(uint8_t temp);
void chk_inverter_status(void);
void calc_a_secment(void);
void calc_a_secment_main(void);
void IC7_load(void);
void secment_value(void);
void calc_a_secment_p(void);
void show_led_p_num(void);
//
uint8_t a_digit;                       // LED display counter for digit according to the multiplexing proces
uint8_t a_secment;                     // LED display active secments
//
uint8_t ds_byte[8]; 			             // array with 8 bytes
uint8_t value_changed;                 // test bit for fine adjustment
uint8_t stop_pb_tb;                    // testbit for stop release used in servicecounter setup and clear
uint8_t next_status;                   // control bit for selecting next parameter
uint32_t E_counter;                    // electronic counter
uint8_t TX_now;	                       // togle bit
uint8_t push_buttons;                  //
static uint8_t rxmissing = 0;          // timeout counter in case of failure in com 30-10-2023
uint8_t curtain_learn;				         // test bit for curtain lerning active 12-02-2020
uint8_t inv_com_state = 0;             // test reg for inverter parameter adjustment - 21-05-2017 static removed because it's not declared in the function
uint16_t inverter_param;               //
uint16_t slip_calc;                    //
uint16_t temp_103;                     //
uint16_t temp_104;                     //
uint16_t pole_count;                   //
uint16_t temp_poles;                   //
uint16_t inv_run_status_1005;          // request of inverter running status parameter 1005
uint8_t inv_fail_active;               // check bit for inverter fail status > 3
uint16_t rated_motor_freq;             // temp register for modbus parameter
uint8_t direction_setup_tst_bit;       // 30-04-2019
uint8_t gdv1;                          // testbit for graphic display version 1 use. If ! grafic display is selected. If 0 LED display is selected.
uint8_t parameter_max;                 // parameter_max depend on inverter use or not 16-12-2021
uint8_t count_1; // for electronic counter
uint8_t count_10; // for electronic counter
uint8_t count_100; // for electronic counter
uint8_t count_1000; // for electronic counter
uint8_t count_10000; // for electronic counter
uint8_t count_100000; // for electronic counter
uint8_t count_1000000; // for electronic counter
uint8_t count_msd; // 
uint16_t fail_volt_show; // timer for 12 og 24 V fail showing 19-11-2021
uint16_t timer_p;       // timer for parameter show above 99 16-12-2021
uint16_t hundreds_tim;  // flash timer for LED display showing above 99 20-01-2022
uint8_t hundreds;       // flash timer testbit for LED display showing above 99 20-01-2022
uint16_t LED_temp;       // temp register for LED showing in every program cycle 20-01-2022
uint8_t h;              // calculation hundreds for LED purpose 20-01-2022
uint8_t temp_old;       // used for automatic trafic light adjusting 01-02-2022
uint8_t EE_par_5_old;   // used for setting e27_modbus fail when changing to inverter running 08-06-2022
//
extern uint8_t info_by_learn;          // special testbit for showing missing encoderanswer by learning
extern uint8_t direction_setup_ok;     // 30-04-2019
extern uint8_t e_type;								 // 30-04-2019
extern uint8_t show_photo_dis_point;   // testbit for showing in display 19-03-2019 30-04-2019
uint16_t INV_status_temp = 0;          // temp inverter status 24-06-2020
extern uint8_t sce_bat_state;			     // SCE enkoder battery state. 0 = OK, 1 = below alarm level, 2 = to low level 05-01-2021
extern uint8_t encod_safe;             // sce power fail observing 05-01-2021
//
extern uint8_t WD_activated;   // 1 = Wicket Door activated. 10-11-2021
extern uint8_t SC_activated;   // 1 = slack cable activated. 10-11-2021
extern uint8_t Safety_2_short; // 1 = safety_2 terminals are short-circuit  10-11-2021 
extern uint8_t wic_slack;      // filter testbit for wicket door and slack rope 10-11-2021
extern uint16_t ad_wicket;     // measured AD value for wicket 11-11-2021
extern uint8_t wic_slack_range; // testbit for showing missing circuit in terminals 12-11-2021
extern uint8_t c_pointer;         // pointer for endurance, electronic counter
extern uint8_t count_bit;         // controlbit for electronic counter
extern uint8_t count_b1;          // 8 of 24 bit counter
extern uint8_t count_b2;          // 8 of 24 bit counter
extern uint8_t count_b3;          // 8 of 24 bit counter
//
extern uint8_t door_pos_half;     // test bit for half open 24-11-2021
extern uint8_t h_ols;             // 1/2 open limit switch after sw-filter 02-12-2021
extern uint8_t thermo_stop;       // 22-12-2021
extern uint8_t e24_wick;          // 11-01-2022 test of input is added to safety test
extern uint8_t fail_5v;           // no fail by powerup
extern uint8_t fail_12v;          // no fail by powerup
extern uint8_t fail_24v;          // no fail by powerup
extern uint8_t encoder_errors;    // summerized encoder error 08-06-2022
extern uint32_t cls_trafic_tim;   // 11-12-2025
//
uint8_t disp_svar;								// checkbit for disply responds
uint8_t rx_timer;                 // filter for no display responds
extern uint8_t uart3_buf[1];      // uart 3 buffer
uint8_t DL_active;                // Testbit for Dock Leveler activated 17-06-2024 
extern uint8_t learn_h_open_active; // test bit for half open learning setpoint 09-12-2025

// We always send 8 bytes to display unit, but all bytes are not always in use. 
//
void pole_adjust() // 26-09-2018 pole adjust and slip calculation
{
read_int16_eeprom(EE_par_103); // get value 26-09-2018
temp_103 = data;
read_int16_eeprom(EE_par_104); // get value 26-09-2018
temp_104 = data;
if (temp_104 < 10000)
 {
	if ((temp_103 < 3590) && (temp_103 > 1990)) pole_count = 2;
	else if ((temp_103 < 1810) && (temp_103 > 1190)) pole_count = 4; 
	else if ((temp_103 < 1200) && (temp_103 > 820)) pole_count = 6;
 }
else // temp_104 = 50, 60 or 100Hz
 {
	if ((temp_103 < 7160) && (temp_103 > 3980)) pole_count = 2;
	else if ((temp_103 < 3600) && (temp_103 > 2380)) pole_count = 4; 
	else if ((temp_103 < 2400) && (temp_103 > 1640)) pole_count = 6;
 }	
//
 	if (temp_104 == 5000) temp_104 = 50;
  else if (temp_104 == 6000) temp_104 = 60;
	else if (temp_104 == 10000) temp_104 = 100; 
  slip_calc = ((temp_104 * 60/(pole_count-2)) - temp_103);
	if (slip_calc > 3000) slip_calc = 3000; // 3000 rpm is max
	if (slip_calc < 1) slip_calc = 1;   // 1 is min- 0 is also ok for S100 but not possible in real life 
}	
//
//************************************************************************
void learn_wd_circuit() // 11-11-2021
{
 EE_read(EE_learn_wd_ready);
 if (temp == 1)
  {
  }
 else
  {
   data = ad_wicket; //
   if (ad_wicket > 4050) // is it a NC contact in the wicket terminals
    {
     EE_write(EE_par_24,5);
     serviceModbusHeartbeat(inverter_use);
    }
   else
    {
     EE_write(EE_par_24,4);
     serviceModbusHeartbeat(inverter_use);
    }
   adr = EE_WD_value;
   write_int16_eeprom();
   serviceModbusHeartbeat(inverter_use);
   EE_write(EE_learn_wd_ready,1);
   serviceModbusHeartbeat(inverter_use);
  }
}
//************************************************************************
void display_prog()
{
	if (prog_mode_tb == 0) // is programming mode just selected after powerup
 {
 parameter = 1; // yes, chose parameter 01
 p_value = 0; // always start with parameter number showing
 prog_mode_tb = 1; // set controlbit
 }
 else
 {
  if (run_prog_tb == 0) // 09-04-2010
   {
    p_value = 0;
    par_val = 0; // simulate stop depressed 
    run_prog_tb = 1;
		push_buttons = 0; // always clear when programming just selected if now display is connected 09-06-2022 
   }
  // just keep selected parameter
 }
parameter_adjust(); 
}
// 
//
void display_keypad()
{
// *** UPS parameters which must be 1 or 2 in this special program 09-05-2017 21-09-2018	
//EE_read(EE_par_84);	// 29-04-2019
//if (temp == 0)  EE_write(EE_par_84, 1);	// switch to 1 if parameter 84 should be 0 by a fail 21-09-2018 	
//	
//EE_read(EE_par_89);	
//if (temp != 1) EE_write(EE_par_89, 1);	// be sure this is 1	14-08-2019
if (inverter_use == 1) parameter_max = 125; // depend on inverter or contactor use 16-12-2021
else parameter_max = 86;	// 03-04-2024
// ***************************************************************	
EE_read(EE_par_5); // 15-11-2021
if ((temp == 0) || (temp == 2)) gdv1 = 0;
else gdv1 = 1;
if (temp > 1)  inverter_use = 1; 
else inverter_use = 0; // change 08-06-2022
// ***************************************************************
a_digit = a_digit + 1;
if (a_digit > 5) a_digit = 1; // 4 digit + colon, multiplexing proces	
if ((run_prog == 0) | (limit_indi > 1) | (show_photo_dis_point == 1)) display_run(); // 27-06-2024
else display_prog();
//	
if (confirm_timer == 0) show_photo_dis_point = 0; //27-06-2024 moved from encoder.c			
// 
if ((gdv1 == 0) | (run_prog == 1))
 {
	switch (a_digit)
   {
    case 1:
    //digit_4_on; // T5 OFF V7E display colon digit 3 and 4
		digit_2_on; // T8 OFF LCCV4D display colon is digit 2 and 3 06-05-2022
    digit_3_on; // T7 OFF
    IC7_load();
    digit_1_off; // T9 ON
    break;
    case 2:
    digit_1_on; // T9 OFF
    IC7_load();
    digit_2_off; // T8 ON
    break;
    case 3:
    digit_2_on; // T8 OFF
    IC7_load();
    digit_3_off; // T7 ON
    break;
    case 4:
    digit_3_on; // T7 OFF
    IC7_load();
    digit_4_off; // T5 ON
    break;
    case 5:
		digit_4_on; // T5 OFF V7E display colon digit 3 and 4
    IC7_load();
		digit_2_off; // T8 ON LCCV4D display colon is digit 2 and 3 06-05-2022
    digit_3_off; // T7 ON
    //digit_4_off; // T5 ON V7E display colon digit 3 and 4
    break;
    default:
    break;
   }   
 }
else // graphic display used turn off LED
 {
	digit_1_on; // T9 OFF
	digit_2_on; // T8 OFF 
	digit_3_on; // T7 OFF
	digit_4_on; // T5 OFF 
 }
//
// 
if ((gdv1 == 1) || (parameter == 5))
 {
  //option_on; // debug	 
  if (HAL_UART_GetState(&huart7) == HAL_UART_STATE_BUSY) // 30-10-2023
   {
    // error, the uart1 should not be busy unless by EEPROM writes 
   }
  else
   {
		// Using the HAL_UART_Receive function (in blocking mode) to receive
    // keypad input. However, a timeout of 0 ms is specified, that is,
    // if a byte is already received it is immediately handled and the
    // return status is HAL_OK. Otherwise, the HAL_UART_Receive function
    // still returns immediately, but with a return status of HAL_TIMEOUT.
    // Thus, using it this way (with a timeout-value of 0) the function
    // will never actually block. Futher more, receive faults are being
    // counted in order to be able to act upon excessive communication
    // failures.
    if (HAL_UART_Receive(&huart7, &push_buttons, 1, 0) == HAL_OK)
     rxmissing = 0;
    else if (++rxmissing > 250) 
     {
	    rxmissing = 250;  // prevent roll over
      if (run_prog == 0) push_buttons = 4; // Simulate stop if no signal is comming from keypad in 1 sek.ome error action! 24-05-2022
     }
     // Transmission is initiated only after receiving is fully completed and
     // is always itself completed before the next periodic initialization.
     // No interference are ever possible between rx and tx interrupts.
		//option_off; // debug 
    //if (ds_byte[4] == 0)
	  // {
	  //  HAL_Delay(0); // 06-01-2022 debug skal ikke være der. Det laver 1 mS forsinkelse OBS
	  // }
		//option_on; // debug 
    HAL_UART_Transmit_IT(&huart7, ds_byte, 8);
		//option_off; // debug 
   }
 }  
 
// 
}
//
//***************************************************
void back_to_parameter_num()
{
 para_test = 0; // 02-01-2012
 s_encoder_tst = 0; // 02-01-2012
 e_tst_result = 0; // 02-01-2012
 p_value = 0; // switch to parameter number
 par_val = 0;
}
//***************************************************
void parameter_adjust() // 11-10-2021
{
	EE_read(EE_factory);
if ((temp == 0) && (ver_show > 0))
 {
	 ds_byte[0] = 1; //prog mode display
	 ds_byte[1] = 2;
	 ds_byte[2] = main_version;
	 ds_byte[3] = sub_version;
	 if (temp == 0) ds_byte[4] = 1;	 // show FACTORY RESET by bytes 
	 else ds_byte[4] = 0; // show software version only 
 }
else
 {
 r_autoclose_timer(); // 06-08-2010
 if (parameter != 51) reload_run_timer(); // reload runtimer 06-08-2010
//*****	
EE_read(EE_par_71);
if (((temp == 54) || (temp == 53)) && (para_test == 1)) // if dock leveler half open input is in use for this parameter 16 is then disabled 29-10-2021 10-12-2025
 {
  EE_read(EE_par_16);
  if (temp > 0) EE_write(EE_par_16, 0);  // clear if half open function when dock leveler with half open is used
 }
EE_read(EE_par_72);
if (((temp == 54) || (temp == 53)) && (para_test == 1)) // if dock leveler half open input is in use for this parameter 16 is then disabled 29-10-2021 10-12-2025
 {
  EE_read(EE_par_16);
  if (temp > 0) EE_write(EE_par_16, 0);  // clear if half open function when dock leveler with half open is used
 }
EE_read(EE_par_74);
if (((temp == 54) || (temp == 53)) && (para_test == 1)) // if dock leveler half open input is in use for this parameter 16 is then disabled 29-10-2021 10-12-2025
 {
  EE_read(EE_par_16);
  if (temp > 0) EE_write(EE_par_16, 0);  // clear if half open function when dock leveler with half open is used
 }
EE_read(EE_par_75);
if (((temp == 54) || (temp == 53)) && (para_test == 1)) // if dock leveler half open input is in use for this parameter 16 is then disabled 29-10-2021 10-12-2025
 {
  EE_read(EE_par_16);
  if (temp > 0) EE_write(EE_par_16, 0);  // clear if half open function when dock leveler with half open is used
 }
//*****	
 	 DL_active = 0; // 17-06-2024	
	 EE_read(EE_par_71);	
	 if (temp == 54) DL_active = 1;	
	 EE_read(EE_par_72);	
	 if (temp == 54) DL_active = 1;
   EE_read(EE_par_74);	
	 if (temp == 54) DL_active = 1;
   EE_read(EE_par_75);	
	 if (temp == 54) DL_active = 1;					
	 EE_read(EE_par_16); 
	 temp3 = temp;
   EE_read(EE_par_25); // 17-06-2024
	 if ((temp != 2) && ((temp3 > 0) || (DL_active == 1))) // Is half open wanted at parameter 16 or by Dock level xx:54 11-12-2025_g
	  {
     EE_write(EE_par_25, 2); // Parameter 25 automatic set to 2
	  }	
   else	if ((temp != 0) && (DL_active != 1) && (temp3 == 0)) EE_write(EE_par_25, 0); // Parameter 25 cleared back to 0 17-06-2024 11-12-2025_g	
//*****	
	 EE_read(EE_par_60); // 17-06-2024
	 temp3 = temp;
	 EE_read(EE_par_27);	
   if (temp3 == 1)
	  {
		 if (temp != 2) EE_write(EE_par_27, 2); // Parameter 27 automatic set to 2
	  }	
   else if (E_limit == 0)
	  {
		 if (temp != 1) EE_write(EE_par_27, 1); // Parameter 27 automatic set to 1 17-06-2024
	  }
   else if (temp != 0) EE_write(EE_par_27, 0); // Parameter 27 automatic set to 0 17-06-2024
//*****
	 EE_read(EE_par_26);	// 04-03-2026
	 if (E_limit == 0)
	  {		 
     if (temp != 1) EE_write(EE_par_26, 1); // Parameter 26 automatic set to 1 17-06-2024
		}
	 else
	  {		 
	   temp4 = 0; // temp 4 is 0 if not set by the following	04-03-2026
     EE_read(EE_par_71); // Is magnetic lock with feed back selected on one of the relays
     if (temp == 56) temp4 = 1;
     EE_read(EE_par_72); // Is magnetic lock with feed back selected on one of the relays
     if (temp == 56) temp4 = 1;
     EE_read(EE_par_74); // Is magnetic lock with feed back selected on one of the relays
     if (temp == 56) temp4 = 1;
     EE_read(EE_par_75); // Is magnetic lock with feed back selected on one of the relays
     if (temp == 56) temp4 = 1;	
		 EE_read(EE_par_26);	// 04-03-2026	
		 if ((temp4 == 0) && (temp != 0)) EE_write(EE_par_26, 0); // Parameter 26 automatic set to 0 if no lock 04-03-2026
		}
//*****	
 if (p_value == 1) // is parameter value selected
  {
   if ((stop_pb == 1) | (stop_lid_pb == 1) | (stop_disp_pb == 1))
	  {
     if (par_val == 1)
			{
       switch (parameter) // 22-06-2021 if else structure replaced by switch structure to add 75:54 76:54 and 88:54 11-10-2021
        {
         case 12:
				 if (para_test == 0) EE_write(EE_learn_wd_ready, 0); // 08-06-2022
				 EE_read(EE_par_16); // 11-12-2025 clear parameter 16 if new limits is learned
         if (temp > 0) EE_write(EE_par_16, 0);  // clear if half open function when dock leveler with half open is used
         chk_para_tst();
         break;
         //
         case 13:
         if (value_changed == 1) chk_para_tst();
         else
          {
           p_value = 0;
           par_val = 0;
          }
         break;
         //
         case 14:
				 if (para_test == 0) EE_write(EE_learn_wd_ready, 0); // 08-06-2022
				 EE_read(EE_par_16); // 11-12-2025 clear parameter 16 if new limits is learned
         if (temp > 0) EE_write(EE_par_16, 0);  // clear if half open function when dock leveler with half open is used
         chk_para_tst();
         break;
         //
         case 15:
         if (value_changed == 1) chk_para_tst();
         else
          {
           p_value = 0;
           par_val = 0;
          }
         break;
         //
         case 16:
         EE_read(EE_par_16);
         if (temp > 1) 
          {
           chk_para_tst();
          } 
         else
          {
           p_value = 0;
           par_val = 0;
          }
         break;
         //
         case 24: // 12-11-2021
          {
           EE_read(EE_par_24);
           if (((temp == 4) || (temp == 5))&& (p_value == 1))
            {
             EE_write(EE_learn_wd_ready, 0); // 16-06-2021
             learn_wd_circuit(); // 16-06-2021
             calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 10-08-2021
             EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 10-08-2021 
            }
           para_test = 0; // 
           p_value = 0; // switch to parameter number
           par_val = 0; // controlling stop release
          }
         break;
         //
         case 31:
         EE_read(EE_par_31); // 11-03-2010
         if ((parameter == 31) && ((temp == 4) || (temp == 5) || (temp == 6) || (temp == 7) || (temp == 9))) // 07-04-2021
          {
           if (para_test == 1)
            {
            }
           else
            {
             photo_learn = 1;
             photo_learn_tim = 500; // 23-03-2010 27-06-2024 incresed from 250 to 500 for long push not saving setpoint
             stop_pb_tb = 1;
             para_test = 1;
             par_val = 0;
             if ((temp == 4) || (temp == 6)) EE_write(EE_relearn_ph1, 0); // no photo1 relearn not needed 06-10-2010
             if ((temp == 5) || (temp == 7)) EE_write(EE_relearn_ph2, 0); // no photo2 relearn not needed 06-10-2010
            }
          }
         else
          {
           back_to_parameter_num();
          }
         break;
         //
         case 41:
         EE_read(EE_par_41);
         if ((parameter == 41) && (temp > 2) && (temp < 5)) // 12-12-2012
          {
           if (para_test == 1)
            {
             para_test = 0;
             p_value = 0;
             par_val = 0;
            }
           else
            {
             stop_pb_tb = 1;
             para_test = 1;
             par_val = 0;
             if (temp == 4)
              {
							 s_speed_learn = 1; //Start learning
							 st_speed_learn_ok = 0;	
               EE_write(EE_s_speed_saved,0); // clear powerup test for singlespeed learned
               bit_clear(sk2,3); // clear testbit for value learned	
							}
             else clr_val(); // clear adaptive speed values 
            } 
          } 
         else
          {
           back_to_parameter_num();
          }
         break;
         //
         case 51:
           EE_read(EE_par_51);
           if ((parameter == 51) && (temp == 3))
            {
             if (para_test == 1)
              {
              }
             else
              {
               //disable_interrupts(GLOBAL); // disable interupt mens der skrives til EEPROM - not needed said Anders 
               EE_write(E_run,0); // new runtimer learning wanted
               //enable_interrupts(GLOBAL); // - not needed said Anders
               para_test = 1;
               par_val = 0;
              }
            }
           else
            {
             p_value = 0; // switch to parameter number 03-06-2010
             par_val = 0;
            }
	       break;
         //
         case 58:
         break;
         //
				 case 60:
				 EE_read(EE_par_60);
         if (temp == 1)	
				  {
				   if (para_test == 1)
            {
             confirm_learn = 1; // set of confirm timer
        		 p_value = 0;
             para_test = 0;
             par_val = 0;
						}	
				   else
					  {
             stop_pb_tb = 1;
             para_test = 1;
             par_val = 0;	
						 // confirm_learn = 1;	Skal hvis ikke være der 07-02-2022
						 EE_write(EE_curtain_rep_saved, 0); // ready for new curtain position	
            } 
				  }
				 else
				  {
           para_test = 0;
           p_value = 0;
           par_val = 0;
          }			 	 
				 break;
				 //
         case 71:
         EE_read(EE_par_71);
         if (temp == 54)  //  dock leveler half open learn 29-10-2021
          {
           chk_para_tst();
          } 
         else
          {
           p_value = 0;
           par_val = 0;
          }
         break;
         //
         case 72:
         EE_read(EE_par_72);
         if (temp == 54)  //  dock leveler half open learn 29-10-2021
          {
           chk_para_tst();
          } 
         else
          {
           p_value = 0;
           par_val = 0;
          }
         break;
         //
         case 74:
         EE_read(EE_par_74);
         if (temp == 54)  //  dock leveler half open learn 29-10-2021
          {
           chk_para_tst();
          } 
         else
          {
           p_value = 0;
           par_val = 0;
          }
         break;
         //
         case 75:
         EE_read(EE_par_75);
         if (temp == 54)  //  dock leveler half open learn 29-10-2021
          {
           chk_para_tst();
          } 
         else
          {
           p_value = 0;
           par_val = 0;
          }
         break;
         //
         default:
         back_to_parameter_num();
         break;
        }
       } 
    }
   else par_val = 1;  
  }
 else // parameter number aktive
  {
   if ((open_pb == 1) | (open_disp_pb == 1))
    {
     if (next_para == 1)
      {
       parameter++; // select next parameter
			 inv_com_state = 0; // always start with 0 when new parameter 26-09-2018
       if (parameter == 3) parameter = 5; // 4 don´t exist 21-12-2010 30-04-2019 09-11-2021
       EE_read(EE_par_11);				
			 if (parameter == 6)
			  {				 
				 if (((e_type == 0) && (ecode1 == 1) && (limits_show == 0)) || (temp == 7)) parameter = 11;
				 else parameter = 12;	// 25-01-2022
				}
			 if (parameter == 13) parameter = 14; // 13 don't wanted in this program	30-04-2019
			 if (parameter == 15) parameter = 16; // 15 don't wanted in this program	30-04-2019					
       if (parameter == 18) parameter = 21; // 18-20 don´t exist 12-04-2010
			 if (parameter == 23) parameter = 24; // 23 don´t exist 17-03-2022	
	     if (parameter == 28) parameter = 29; // 24-28 don´t exist 11-05-2010 12-11-2021
       if (parameter == 30) parameter = 31; // 30 don´t exist
	     if (parameter == 37) parameter = 41; // 37-40 don´t exist 03-11-2010 23-08-2011 16-05-2013
			 if (parameter == 42) parameter = 44; // 42-43 don´t exist
			 if (parameter == 45) parameter = 51; // 45-50 don´t exist	
       if (parameter == 54) parameter = 55; // 54 don´t exist 03-04-2024
			 if (parameter == 56) parameter = 58; // 56-57 don´t exist	03-04-2024
			 if (parameter == 61) parameter = 71; // 60-70 don´t exist 30-04-2019 03-05-2022
			 if (parameter == 73) parameter = 74; // 03-05-2022	
			 if (parameter == 76) parameter = 77; // 76 don´t exist 16-12-2021	
       if (parameter == 83) parameter = 84; // 83 don´t exist 11-03-2011 23-12-2011 06-11-2013
	     if (parameter == 87) parameter = 100; // 87-99 don´t exist 11-03-2011 23-12-2011 06-11-2013 21-11-2016 03-04-2024
			 if (parameter == 106) parameter = 109; // 106-108 don´t exist 11-03-2011 16-09-2016		
			 if (parameter == 116) parameter = 120; // 116-119 don´t exist 11-03-2011 16-09-2016	
       next_para = 0;
			 if ((inverter_use ==0) && (parameter > 94))	parameter = 1;
       if (parameter > parameter_max) parameter = 1;
       if (parameter == 12) // Electronic limit learning open
        {
         EE_read(EE_par_11);
         if (temp == 7) parameter = 16; // jump to 16 if electronic limits is not selected 24-01-2022
        }
			 if (parameter == 32)
        {
         EE_read(EE_par_1);
         if ((temp == 1) || (temp == 2)) parameter = 33; // auto close not possible if deadman is selected
         // EE_read(EE_par_31); disabled 28-08-09
         // if (temp == 0) parameter = 33; // auto close not possible if no photo safety is wanted
        }
       if (parameter == 33)
        {
         read_int16_eeprom(EE_par_32);
         temp2 = data;
         if (temp2 == 0) parameter = 35; // no car wash is autoclose is disabled
				 EE_read(EE_par_21); // 02-12-2021
         temp3 = temp;	
         EE_read(EE_par_31); // 02-09-2011
         if ((temp == 0) && (temp3 != 6)) parameter = 35; // no car wash if no photo is selected and no RAYLG selected 12-04-2018
        }
       if (parameter == 34) // 23-08-2011
        {
         EE_read(EE_par_33);
         if (temp == 0) parameter = 35; // no forced closing if no wash funktion is selected
        }
       if (parameter == 35) // 03-11-2010
        {
         EE_read(EE_par_31); // 02-09-2011
         if (temp == 0) parameter = 36; // no special Go function if no photo is selected
        }
       if (parameter == 36) // 16-05-2013
        {
         //EE_read(EE_par_32);  
         read_int16_eeprom(EE_par_32); // check 2 bytes 14-08-2013
         temp2 = data;
         if (temp2 == 0) parameter = 41; // no meaning if autoclose is not selected. 16-05-2013
        }
       if (parameter == 42)
        {
         EE_read(EE_par_41);
         if (temp == 0) parameter = 51; // no speed control is wanted
         if ((temp > 2) && (temp < 5)) parameter = 44; // adaptive speed is selected
        }
       else if ((p_value == 0) & (new_m_speed == 1)) // returned to parameter number but new value wanted
        {
         temp = m_speed;
         EE_write(EE_par_42, temp);  // save new value
         new_m_speed =0; // clear controlbit - value saved
        }
       if (parameter == 43)
        {
         if ((p_value == 0) & (new_m_speed == 1)) // returned to parameter number but new value wanted
          {
           temp = m_speed;
           EE_write(EE_par_43, temp);  // save new value
           new_m_speed =0; // clear controlbit - value saved
          }
        }
       if (parameter == 44)
        {
         EE_read(EE_par_41); // check about adaptive speed is selected
         if ((temp < 3) || (temp == 5)) parameter = 51; // No adaptiv speed, skip parameter 44 12-12-2012
        }
			 if (parameter == 60) // 01-04-2019
        {
         EE_read(EE_par_60); // 
					if (temp == 1)
					{
           EE_read(EE_par_84); //
					 if (temp == 3) EE_write(EE_par_84, 0); // reset parameter 84 if 84:03 is selected. Only one kan use option input 30-04-2019 26-11-2021
          }						
				}			
			 if (parameter == 81) // 09-03-2010
        {
         EE_read(EE_par_11); // check about electronic limits is selected
         if (temp == 7) parameter = 84; // mechanical limits is selected, skip parameter 81 24-01-2022 03-04-2024
        }
       if (parameter == 82) // 04-01-2012
        {
         EE_read(EE_par_11); // check about Dal encoder is selected
         if (temp > 2) parameter = 84; // No Dal encoder, skip parameter 82 06-11-2013
        }
			 if (parameter == 85) // 04-01-2012	
			  { 
		     EE_read(EE_par_11); // check about electronic limits is selected
         if (temp == 7)
				 {
					 if (inverter_use ==0) parameter = 1; // 03-04-2024
					 else if (inverter_use ==1)	parameter = 100; // 03-04-2024
				 }
			  }
	     if (parameter == 115)
        {
         EE_read(EE_par_11); // check about electronic limits is selected
         if (temp == 7) parameter = 120; // electroni limits is selected, skip parameter 115 24-01-2022
        }
			if (parameter == 125)
        {
         EE_read(EE_par_11); // check about electronic limits is selected
         if (temp == 7) parameter = 1; // electroni limits is selected, skip parameter 125 24-01-2022
        }	
      }
    }
   else next_para = 1;
   if ((close_pb) || (close_disp_pb))
    {
     if (prev_para == 1)
      {
       parameter--;
			 inv_com_state = 0; // always start with 0 when new parameter 26-09-2018	
       prev_para = 0;
       if (parameter == 0) parameter = parameter_max;
			 if ((inverter_use ==0) && (parameter > 94))	parameter = 86;	// 16-09-2016 03-04-2024
			 if (parameter == 119) parameter = 115; // 116-119 don´t exist	16-09-2016
			 if (parameter == 108) parameter = 105; // 106-108 don´t exist	16-09-2016	
			 if (parameter == 99) parameter = 86; // 87-99 don´t exist	21-11-2016 03-04-2024
			 if (parameter == 83) parameter = 82; // 86-85 don´t exist 11-03-2011 23-12-2011 06-11-2013 21-11-2016
       if (parameter == 76) parameter = 75; // 76 don´t exist 16-12-2021
			 if (parameter == 73) parameter = 72; // 73 don´t exist 03-05-2022	
       if (parameter == 70) parameter = 60; // 70-61 don´t exist 30-04-2019	03-05-2022 
			 if (parameter == 57) parameter = 55; // 57-56 don´t exist 03-04-2024
			 if (parameter == 54) parameter = 53; // 54 don´t exist	03-04-2024
	     if (parameter == 50) parameter = 44; // 50-45 don´t exist
		   if (parameter == 43) parameter = 41; // 42-43 don´t exist		
	     if (parameter == 40) parameter = 36; // 40-37 don´t exist 03-11-2010 16-05-2013
	     if (parameter == 30) parameter = 29; // 30 don´t exist 23-08-2011
       if (parameter == 28) parameter = 27; // 28 don´t exist 11-05-2010 12-11-2021
			 if (parameter == 23) parameter = 22; // 23 don´t exist 17-03-2022	
       if (parameter == 20) parameter = 17; // 20-18 don´t exist 12-04-2010
			 if (parameter == 15) parameter = 14; // 	15 not wanted in this program 30-04-2019
			 if (parameter == 13) parameter = 12; // 	15 not wanted in this program	30-04-2019
			 EE_read(EE_par_11);
       if (parameter == 11)
			  {				 
				 if (((e_type == 0) && (ecode1 == 1) && (limits_show == 0)) || (temp == 7)) parameter = 11;
				 else parameter = 10;	// 25-01-2022
				} 
			 if (parameter == 10) parameter = 5; // 23-28 don´t exist 21-12-2010 and 11 reenabled in this program 17-01-2022 
			 if (parameter == 4) parameter = 2; // 23-28 don´t exist 21-12-2010 and 11 not needed in this program 30-04-2019 09-11-2021	
       
       if (parameter == 125) // 09-03-2010 16-09-2016
        {
         EE_read(EE_par_11); // check about electronic limits is selected
         if (temp == 7) parameter = 124; // electronic limits is selected, skip parameter 125 24-01-2022
        }
		 if (parameter == 115) // 09-03-2010 16-09-2016
        {
         EE_read(EE_par_11); // check about electronic limits is selected
         if (temp == 7) parameter = 114; // electronic limits is selected, skip parameter 115 24-01-2022
        }	
     if (parameter == 86) // 04-01-2012	
			  { 
		     EE_read(EE_par_11); // check about electronic limits is selected
         if (temp == 7) parameter = 84; // mechanical limits is selected, skip parameter 81 24-01-2022 03-04-2024 
			  } 
	   if (parameter == 82) // 09-03-2010 23-12-2011
        {
         EE_read(EE_par_11); // check about electronic limits is selected
         if (temp == 7) parameter = 80; // electronic limits is selected, skip parameter 82 24-01-2022 03-04-2024
         else if ((temp > 2) && (temp != 7)) parameter = 81; // only Dal enkoder have battery feature 04-01-2012 24-01-2022
        }
			 if (parameter == 60) // 30-04-2019
        {
         EE_read(EE_par_60); // 
					if (temp == 1)
					{
           EE_read(EE_par_84); //
					 if (temp == 3) EE_write(EE_par_84, 0); // reset parameter 84 if 84:03 is selected. Only one kan use ref sw. input 30-04-2019 26-11-2021
          }						
				}			
			 if (parameter == 44)
        {
         EE_read(EE_par_41); // check about adaptive speed is selected
         if ((temp < 3) || (temp == 5)) parameter = 43; // 12-12-2012
        }
       if (parameter == 43)
        {
         EE_read(EE_par_41); // check about manuel speed is selected
         if ((temp == 3) || (temp == 4) || (temp < 1)) parameter = 41; // if adaptive speed or no speed is selected
        }
       if (parameter == 36) // 16-05-2013
        {
         // EE_read(EE_par_32); 
         read_int16_eeprom(EE_par_32); // check 2 bytes 14-08-2013
         temp2 = data; 
         if (temp2 == 0) parameter = 35; // no meaning if autoclose is not selected. 16-05-2013
        }
       if (parameter == 35) // 03-11-2010
        {
         EE_read(EE_par_31); // 02-09-2011
         if (temp == 0) parameter = 32; // no special Go function and no car wash if no photo is selected
        }
			 if (parameter == 34) // 02-12-2021
        {
         EE_read(EE_par_21); // 
         temp3 = temp; // 
         EE_read(EE_par_31); // 
         if ((temp == 0) && (temp3 != 6)) parameter = 32; // no special Go function and no car wash if no photo is selected and no RAYLG selected 12-04-2018
        }	
       if (parameter == 34) // 23-08-2011
        {
         EE_read(EE_par_33);
         if (temp == 0) parameter = 33; // no forced closing if no wash funktion is selected
        }
       if (parameter == 33)
        {
         read_int16_eeprom(EE_par_32);
         temp2 = data;
         if (temp2 == 0) parameter = 32; // no car wash is autoclose is disabled 10-12-09
        }
       if (parameter == 32)
        {
         EE_read(EE_par_1);
         if ((temp == 1) || (temp == 2)) parameter = 31; // auto close not possible if deadman is selected
         // EE_read(EE_par_31); disabled 28-08-09
         //if (temp == 0) parameter = 31; // auto close not possible if no photo safety is wanted
        }
			 if (parameter == 14)
        {
         EE_read(EE_par_11); // check about electronic limits is selected
         if (temp == 7) parameter = 2; // 30-04-2019 should not be possible but change anyway 24-01-2022 
        }	
			}
    }
   else prev_para = 1;
   //
   if ((stop_pb == 1) || (stop_lid_pb == 1) || (stop_disp_pb == 1))
    {
     if (par_val == 1)
      {
       if (parameter == 82)
        {
         s_encoder_tst = 1; // 23-12-2011
         e_tst_result = 0; // 23-12-2011
         para_test = 1; // 02-01-2012
        }
       p_value = 1; // parameter value showing
       par_val = 0;
      }
    }
   else par_val = 1;
  }
//
//
EE_read(EE_par_31); // 23-03-2010
if (temp > 3)
 {
  EE_read(EE_par_11);
  if (temp == 7) // 24-01-2022
   {
    EE_write(EE_par_31, 1); // reset to normal both photo if selected back to mekanical switches to tell that 
                            // something is maybe wrong
   }
 }
 //
//EE_read(EE_par_31); // 03-11-2010
//if (temp == 0)
// {
//  EE_read(EE_par_35);
//  if (temp > 0)
//   {
//    EE_write(EE_par_35, 0); // reset to normal kip is no photo is selected. Not wanted from ConDoor
//   }
// }
//
//EE_read(EE_par_32);  15-08-2012
read_int16_eeprom(EE_par_32); // check 2 bytes 14-08-2013
temp2 = data;
if (temp2 == 0)
 {
  EE_read(EE_par_33);
  if (temp > 0)
   {
    EE_write(EE_par_33, 0); // No washfunction if no autoclose is selected
   }
 }
//
EE_read(EE_par_33); // 23-08-2011
if (temp == 0)
 {
  EE_read(EE_par_34);
  if (temp > 0)
   {
    EE_write(EE_par_34, 0); // No forced closing if no washfunction is selected
   }
 }
//
if (a_digit == 5 && para_test == 0)
 {
  a_secment = 0x7F; // (0b01111111) // colon should almays be on in prog_mode unless in RUN state
 }
if (a_digit == 5 && ((para_test == 1) || (confirm_timer > 0)))
 {
  a_secment = 0xFF; // (0b11111111) // parameter test (run state)
 }
else if (a_digit == 5)
 {
  a_secment = 0x7F; // (0b01111111); // colon should almays be on in prog_mode unless in RUN state
 }
// 
//EE_read(EE_par_21); // 13-10-2017 Change to relay parameter value 55
//if (temp == 5)
// {
//  EE_read(EE_par_87);
//  if (temp < 4)
//   {
//    EE_write(EE_par_87, 4); // Wireless edge is selected AUX 1 is used for test function
//   }
// }  
// 
 ds_byte[5] = 0; // not in use in prog mode yet
 ds_byte[6] = 0; // not in use in prog mode yet
 ds_byte[7] = 0; // not in use in prog mode yet
 //*********************************************************************************************************************
 //*********************************************************************************************************************
 switch (parameter) 
  {
    case 1: // parameter 01 selected
    EE_read(EE_par_1);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(1,4); // request value change with min max value for this parameter 12-04-2010
     }
    if (new_EE == 1)
     {
      EE_write(EE_par_1, temp);  // save new value
      calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 26-10-2011
      EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 26-10-2011
     }
		ds_byte[0] = 1; // Prog mode active
		ds_byte[1] = 1; // show parameter 01 
		ds_byte[2] = 0; // -
		ds_byte[3] = temp; // parameter value
		ds_byte[4] = 0;	
		if (a_digit == 1) a_secment = 0xC0; // 0b11000000 digit1 = 0
    if (a_digit == 2) a_secment = 0xF9; // 0b11111001 digit2 = 1
    if (a_digit == 3) a_secment = 0xC0; // 0b11000000 digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
		break;
//  
    case 2: // parameter 02 selected
    EE_read(EE_par_2);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(0,1); // request value change with min max value for this parameter 12-04-2010
     }
    if (new_EE == 1)
     {
      EE_write(EE_par_2, temp);  // save new value
      calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 26-10-2011
      EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 26-10-2011
     } 
    ds_byte[0] = 1; // Prog mode active
		ds_byte[1] = 2; // show parameter 02 
		ds_byte[2] = 0; // -
		ds_byte[3] = temp;
		ds_byte[4] = 0;
		if (a_digit == 1) a_secment = 0xC0; // (0b11000000) digit1 = 0
    if (a_digit == 2) a_secment = 0xA4; // (0b10100100) digit2 = 2
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
    break;
//		 
    case 5: // parameter 05 selected 09-11-2021
    EE_read(EE_par_5);
		EE_par_5_old = temp; // 08-06-2022
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
      req_val_change(0,3); // request value change with min max value for this parameter 12-04-2010
     }
    if (new_EE == 1)
     {
      EE_write(EE_par_5, temp);  // save new value
			if ((temp > 1) && (EE_par_5_old < 2)) e27_modbus = 1; // New powerup required with connected inverter 08-06-2022
                                                            // inverter must be initialized	  
      calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 26-10-2011
      EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 26-10-2011
     } 
		ds_byte[0] = 1; // Prog mode active
		ds_byte[1] = 5; // show parameter 05 
		ds_byte[2] = 0; // -
		ds_byte[3] = temp;
		ds_byte[4] = 0;
		if (a_digit == 1) a_secment = 0xC0; // (0b11000000) digit1 = 0
    if (a_digit == 2) a_secment = 0x92; // (0b10010010) digit2 = 5
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); // 
		break;		 
//  
    case 11: // parameter 11 selected
    EE_read(EE_par_11);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(1,9); // request value change with min max value for this parameter 
     }
    if (new_EE == 1)
		 {
      //if ((open_pb == 1) && (temp == 7)) temp = 8; // 09-06-2022 7 is not available in this control 22-11-2023
			//if ((close_pb == 1) && (temp == 7)) temp = 6; // 09-06-2022 7 is not available in this control 22-11-2023 
		  EE_write(EE_par_11, temp);  // save new value
		 }
		ds_byte[0] = 1; // Prog mode active
		ds_byte[1] = 11; // show parameter 11 
		ds_byte[2] = 0; // -
		ds_byte[3] = temp; 
		ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0xF9; // (0b11111001) digit1 = 1
    if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
   	break;
//    
    case 12: // parameter 12 selected, no value here, it's E. limit learning proces
    if (para_test ==1) 
     {
			EE_read(EE_encod_safe);
      if (temp != encod_safe) EE_write(EE_encod_safe,encod_safe); // must be cleared when new learning
			sce_bat_state = 0; 
      show_run();
			learn_wd_circuit(); // 08-06-2022  
      if (new_EE == 1)
       {
        new_EE = 0; // this new EE is saved on uP2
        confirm_learn = 1;
				EE_write(EE_learn_wd_ready, 0); // 11-10-2021
        serviceModbusHeartbeat(inverter_use);
        learn_wd_circuit(); // 10-11-2021 
				calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 10-11-2021
        EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 10-11-2021 
        serviceModbusHeartbeat(inverter_use);
        EE_write(EE_par_13, 5); // preset to centervalue when new limit learned
        serviceModbusHeartbeat(inverter_use);
        EE_write(EE_par_41, 0); // preset to no speedcontrol when changing limits 10-12-09
        serviceModbusHeartbeat(inverter_use);
        EE_read(EE_par_31);
        if ((temp == 4) || (temp == 6)) EE_write(EE_relearn_ph1, 1); // photo1 in frame is active - relearn needed 06-10-2010
        if ((temp == 5) || (temp == 7)) EE_write(EE_relearn_ph2, 1); // photo1 in frame is active - relearn needed 06-10-2010
        serviceModbusHeartbeat(inverter_use);
				EE_write(EE_curtain_rep_saved, 0); // 12-02-2020 
        serviceModbusHeartbeat(inverter_use);
				p_value = 0;
        para_test = 0;
        par_val = 0;
       }
      break;
     }
    else if ((limit_indi > 0) || (confirm_timer > 0))
     {
      break; // learned limit is indicated in display_run
     }
    else
     {
			if (info_by_learn == 1)
			 {
				enc_ecode1(); // 13-03-2017 show if encoder is not answering
        d_flash_tim = 1; // stop flash when ERR shown				 
				break; 
			 }
      else 
			 {				
			  ds_byte[0] = 1;  // Prog mode active
		    ds_byte[1] = 12; // show parameter 12 
		    ds_byte[2] = 0;  // open limit symbol skal be shown in parameter 12 handled by display unit
  	    ds_byte[3] = 0;  //
        if (a_digit == 1) a_secment = 0xF9; // (0b11111001) digit1 = 1
        if (a_digit == 2) a_secment = 0xA4; // (0b10100100) digit2 = 2
        if (a_digit == 3) a_secment = 0xDE; // (0b11011110) digit3 = symbol
        if (a_digit == 4) a_secment = 0xFC; // (0b11111100) digit4 = symbol
			 }
		 }			 
		break; 
//
    case 13: // parameter 13 selected
    if (para_test ==1) 
     {
      if (new_EE == 1) // is stop pushbutton pressed again in function parameter_adjust rountine
       {
        p_value = 1; // return to parameter value
        para_test = 0;
        par_val = 0;
       }
      else show_run();
      break;
     }
    else
     {
      EE_read(EE_par_13);
      new_EE = 0;
      if ((p_value == 1) & (para_test == 0))
       {
        req_val_change(1,9); // request value change with min max value for this parameter
       }
      if (new_EE == 1) EE_write(EE_par_13, temp);  // save new value
			ds_byte[0] = 1;    // Prog mode active
		  ds_byte[1] = 13;   // show parameter 13 
		  ds_byte[2] = 0;    //  not in use yet
		  ds_byte[3] = temp; //
			ds_byte[4] = 0;
      if (a_digit == 1) a_secment = 0xF9; // (0b11111001) digit1 = 1
      if (a_digit == 2) a_secment = 0xCF; // (0b11001111) digit2 = 3
      if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
      if (a_digit == 4) calc_a_secment(); //
      break;
     }
//    
    case 14: // parameter 14 selected
    if (para_test ==1) 
     {
			if (direction_setup_tst_bit == 0) // 30-04-2019
			 {				
			  direction_setup_ok = 0; // reset for new direction one time only by start learning 14
        direction_setup_tst_bit = 1;
				EE_write(EE_learning_ok,0); // 27-03-2019  
       } 
			show_run();
			learn_wd_circuit(); // 08-06-2022  
      if (new_EE == 1)
       {
				direction_setup_ok = 1; // set if not set because limits are only moved a little 30-04-2019
				direction_setup_tst_bit = 0; 
				EE_write(EE_learning_ok,1); // 27-03-2019 
        serviceModbusHeartbeat(inverter_use);
        new_EE = 0; // this new EE is saved on uP2
        confirm_learn = 1;
				EE_write(EE_learn_wd_ready, 0); // 11-10-2021
        serviceModbusHeartbeat(inverter_use);
        learn_wd_circuit(); // 10-11-2021 
				calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 10-11-2021
        EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 10-11-2021  
        serviceModbusHeartbeat(inverter_use);
        EE_write(EE_par_15, 5); // preset to centervalue when new limit learned
        serviceModbusHeartbeat(inverter_use);
        EE_write(EE_par_41, 0); // preset to no speedcontrol when changing limits 10-12-09
        serviceModbusHeartbeat(inverter_use);
        EE_read(EE_par_31);
        if ((temp == 4) || (temp == 6)) EE_write(EE_relearn_ph1, 1); // photo1 in frame is active - relearn needed 06-10-2010
        if ((temp == 5) || (temp == 7)) EE_write(EE_relearn_ph2, 1); // photo1 in frame is active - relearn needed 06-10-201
        serviceModbusHeartbeat(inverter_use);
				EE_write(EE_curtain_rep_saved, 0);  
        serviceModbusHeartbeat(inverter_use);
        p_value = 0;
        para_test = 0;
        par_val = 0;
       }
      break;
     }
    else if ((limit_indi > 0) || (confirm_timer > 0))
     {
      break; // learned limit is indicated in display_run
     }
    else
     {
			if (info_by_learn == 1)
			 {
				enc_ecode1(); // 13-03-2017 show if encoder is not answering
        d_flash_tim = 1; // stop flash when ERR shown				 
			 }
      else 
			 {	 
			  ds_byte[0] = 1;  // Prog mode active
		    ds_byte[1] = 14; // show parameter 14 
		    ds_byte[2] = 0;  // close limit symbol skal be shown in parameter 14 handled by display unit
  	    ds_byte[3] = 0;  //
        if (a_digit == 1) a_secment = 0xF9; // (0b11111001) digit1 = 1
        if (a_digit == 2) a_secment = 0x99; // (0b10011001) digit1 = 4 
        if (a_digit == 3) a_secment = 0xE7; // (0b11011110) digit3 = symbol 30-10-2023
        if (a_digit == 4) a_secment = 0xF3; // (0b11111100) digit4 = symbol 30-10-2023
       }
		 }			 
      break; 
//
    case 15: // parameter 15 selected
    if (para_test ==1) 
     {
      if (new_EE == 1) // is stop pushbutton pressed again in function parameter_adjust rountine
       {
        p_value = 1; // return to parameter value
        para_test = 0;
        par_val = 0;
       }
      else show_run();
      break;
     }
    else
     {
      EE_read(EE_par_15);
      new_EE = 0;
      if ((p_value == 1) & (para_test == 0))
       {
        req_val_change(1,9); // request value change with min max value for this parameter 
       }
      if (new_EE == 1) EE_write(EE_par_15, temp);  // save new value
			ds_byte[0] = 1;    // Prog mode active
		  ds_byte[1] = 15;   // show parameter 15 
		  ds_byte[2] = 0;    //  not in use yet
		  ds_byte[3] = temp; //
			ds_byte[4] = 0; 
			if (a_digit == 1) a_secment = 0xF9; // (0b11111001) digit1 = 1
      if (a_digit == 2) a_secment = 0xB2; // (0b10110010) digit2 = 5
      if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
      if (a_digit == 4) calc_a_secment(); // 
			break;
     }
//
    case 16: // parameter 16 selected
    EE_read(EE_par_16);
		learn_h_open_active = 0; // clear if not set by the following 09-12-2025
    if (para_test ==1) 
     {
      show_run();
			learn_h_open_active = 1; // 09-12-2025 
      if (new_EE == 1)
       {
        new_EE = 0; // this new EE is saved on uP2
        confirm_learn = 1;
        p_value = 0;
        para_test = 0;
        par_val = 0;
       }
      break;
     }
    else if ((limit_indi > 0) || (confirm_timer > 0))
     {
      break; // learned limit is indicated in display_run
     }
    else
     {
      EE_read(EE_par_16); // 18-05-2021
      new_EE = 0;
      if ((p_value == 1) & (para_test == 0))
       {
        req_val_change(0,4); // request value change with min max value for this parameter 25-06-2019 18-05-2021 22-11-2023
       }
      
      //if ((open_pb == 1) && (temp == 1)) temp = 2; // 20-12-2018 30-09-2020
      //if ((close_pb == 1) && (temp == 1)) temp = 0; // 20-12-2018 30-09-2020
      if (temp > 0) // 23-06-2021
       {
        temp4 = temp; // save for later re-insert
				EE_read(EE_par_71); // prevent half open if XX:54 dock leveler is selected
        if (temp == 54) temp4 = 0;
        EE_read(EE_par_72); // prevent half open if XX:54 dock leveler is selected
        if (temp == 54) temp4 = 0;
        EE_read(EE_par_74); // prevent half open if XX:54 dock leveler is selected
        if (temp == 54) temp4 = 0;
        EE_read(EE_par_75); // prevent half open if XX:54 dock leveler is selected
        if (temp == 54) temp4 = 0;
        temp = temp4; // reinsert
       }
	    if (new_EE == 1) 
			 {
        //if ((open_pb == 1) && (temp == 1)) temp = 2; // 09-06-2022 1 is not available in this control - no mech. limits 22-11-2023
			  //if ((close_pb == 1) && (temp == 1)) temp = 0; // 09-06-2022 7 is not available in this control 
			  EE_write(EE_par_16, temp);  // save new value
			 }
		  ds_byte[0] = 1;    // Prog mode active
		  ds_byte[1] = 16;   // show parameter 16 
		  ds_byte[2] = 0;    //  not in use yet
		  ds_byte[3] = temp; // 
		  ds_byte[4] = 0;
      if (a_digit == 1) a_secment = 0xF9; // (0b11111001) digit1 = 1
      if (a_digit == 2) a_secment = 0x82; // (0b10000010) digit2 = 6 
      if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
      if (a_digit == 4) calc_a_secment(); // 
      break;
	   }
//
    case 17: // parameter 17 selected
    EE_read(EE_par_17);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(0,1); // request value change with min max value for this parameter 
     }
    if (new_EE == 1) EE_write(EE_par_17, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 17;   // show parameter 17 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0xF9; // (0b11111001) digit1 = 1
    if (a_digit == 2) a_secment = 0xF8; // (0b11111000) digit2 = 7
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); // 
    break;
//
    case 21: // parameter 21 selected
    EE_read(EE_par_21);
		new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(1,6); // request value change with min max value for this parameter 28-04-2011 
     }
		if ((open_pb == 1) && (temp == 4)) temp = 5; // 02-12-2021 4 is not available in this control 
    if ((close_pb == 1) && (temp == 4)) temp = 3; // 02-12-2021 
    if (new_EE == 1) EE_write(EE_par_21, temp);  // save new value
    if ((err_edge == 1) && (edge_err_show > 450))
     {
			d_flash_tim = 1; // stop flash when ERR shown
			//
      ds_byte[0] = 1;    // Prog mode active
			temp2 = 1021;
			ds_byte[1] = temp2;    // show err in display unit 1021
      ds_byte[2] = temp2>>8;
      ds_byte[4] = 255;  //  show no value
      if (a_digit == 1) a_secment = 0x86; // (0b10000110) digit1 = E
      if (a_digit == 2) a_secment = 0xCE; // (0b11001110) digit2 = r
      if (a_digit == 3) a_secment = 0xCE; // (0b11001110) digit3 = r
      if (a_digit == 4) a_secment = 0xFF; // (0b11111111) digit4 = off
      if (a_digit == 5) a_secment = 0xFF; // (0b11111111) colon = off
     }
    else
     {
      if (edge_err_show == 0) edge_err_show = 900;
			ds_byte[0] = 1;    // Prog mode active
		  ds_byte[1] = 21;   // show parameter 21 
		  ds_byte[2] = 0;    //  not in use yet
		  ds_byte[3] = temp; //
      ds_byte[4] = 0;
      if (a_digit == 1) a_secment = 0xA4; // (0b10100100) digit1 = 2
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1 
      if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
      if (a_digit == 4) calc_a_secment(); //
     } 
    break;
//    
    case 22: // parameter 22 selected
    EE_read(EE_par_22);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(0,50); // request value change with min max value for this parameter 06-10-2010 
     }
    if (new_EE == 1)
	   {
      EE_write(EE_par_22, temp);  // save new value
      dw_ok = 1; // prevent fail when programming is deselected 08-05-2012
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 22;   // show parameter 22 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0xA4; // (0b10100100) digit1 = 2
    if (a_digit == 2) a_secment = 0xA4; // (0b10100100) digit2 = 2
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
    break;
//
		case 24: // parameter 24 selected
    EE_read(EE_par_24); // 11-05-2010
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
      req_val_change(4,5); // request value change with min max value for this parameter 30-11-2021
     }
    if (new_EE == 1)
     {
      EE_write(EE_par_24, temp);  // save new value
      calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 26-10-2011
      EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 26-10-2011
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 24;   // show parameter 24 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;		
		if (a_digit == 1) a_secment = 0xA4; // (0b10100100) digit1 = 2
		if (a_digit == 2) a_secment = 0x99; // (0b10011001) digit1 = 4 
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
		break; 
// 
    case 25: // parameter 23 selected - Free input 1 30-11-2021
    EE_read(EE_par_25); // 
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
      req_val_change(0,2); // request value change with min max value for this parameter 12-11-2021
     }
    if (new_EE == 1)
     {
      EE_write(EE_par_25, temp);  // save new value
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 25;   // show parameter 25 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;		
		if (a_digit == 1) a_secment = 0xA4; // (0b10100100) digit1 = 2
		if (a_digit == 2) a_secment = 0x92; // (0b10010010) digit2 = 5 
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
		break; 
// 
    case 26: // parameter 23 selected - Free input 2 30-11-2021
    EE_read(EE_par_26); // 11-05-2010
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
      req_val_change(0,3); // request value change with min max value for this parameter 12-11-2021 04-03-2026
     }
    if (new_EE == 1)
     {
      EE_write(EE_par_26, temp);  // save new value
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 26;   // show parameter 26 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;		
		if (a_digit == 1) a_secment = 0xA4; // (0b10100100) digit1 = 2
		if (a_digit == 2) a_secment = 0x82; // (0b10000010) digit2 = 6 
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
		break; 
// 		 
		case 27: // parameter 23 selected - Free input 3 30-11-2021
    EE_read(EE_par_27); // 11-05-2010
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
      req_val_change(0,2); // request value change with min max value for this parameter 12-11-2021
     }
    if (new_EE == 1)
     {
      EE_write(EE_par_27, temp);  // save new value
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 27;   // show parameter 27 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;		
		if (a_digit == 1) a_secment = 0xA4; // (0b10100100) digit1 = 2
    if (a_digit == 2) a_secment = 0xF8; // (0b11111000) digit2 = 7
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
		break;  
//		 
    case 29: // parameter 29 selected
    EE_read(EE_par_29);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
      req_val_change(0,4); // request value change with min max value for this parameter 
     }
    if (new_EE == 1) EE_write(EE_par_29, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 29;   // show parameter 29 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0xA4; // (0b10100100) digit1 = 2
    if (a_digit == 2) a_secment = 0x90; // (0b10010000) digit2 = 9
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
    break; 
//
    case 31: // parameter 31 selected
    if (para_test ==1) // 11-03-2010
     {
      show_run();
      break;
     }
    EE_read(EE_par_31);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(0,4); // request value change with min max value for this parameter 14-03-2010 
     }
    if (new_EE == 1)
     {
      if ((open_pb == 1) && (temp == 2)) temp = 4; // 08-06-2022 2 and 3 is not available in this control 
			if ((close_pb == 1) && (temp == 3)) temp = 1; // 08-06-2022 2 and 3 is not available in this control  
      EE_write(EE_par_31, temp);  // save new value
      if (temp < 4)
       {
        EE_write(EE_relearn_ph1, 0); // no photo1 relearn not needed 06-10-2010
        EE_write(EE_relearn_ph2, 0); // no photo2 relearn not needed 06-10-2010
       }
      calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 26-10-2011
      EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 26-10-2011
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 31;   // show parameter 31 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0xB0; // (0b10110000) digit1 = 3
    if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
    break;
//
    case 32: // parameter 32 selected
    read_int16_eeprom(EE_par_32);
    temp2 = data;
    if ((p_value == 1) & (para_test == 0))
     {
      if ((open_pb == 1) | (open_disp_pb == 1))// is open push button active
       {
        if (temp2 < autoclose_max)
         {
          if (ch_timer == 0)
           {
						if (temp2 >99) temp2 = temp2 + 10;
            else temp2++; 
            data = (temp2); //
            adr = EE_par_32;
            write_int16_eeprom();
            calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 26-10-2011
            EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 26-10-2011
            ch_timer = 60; // changed from 100 at new 3.33mS main cycle
           }
         }               
       }
      if ((close_pb) || (close_disp_pb)) // is close push button active
       {
        if (temp2 > 0) 
         {
          if (ch_timer == 0)
           {
						if (temp2 >109) temp2 = temp2 - 10;
            else temp2--; 
            data = (temp2); //
            adr = EE_par_32;
            write_int16_eeprom();
            calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 26-10-2011
            EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 26-10-2011
            ch_timer = 60; // changed from 100 at new 3.33mS main cycle
           }
         }
       }
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 32;   // show parameter 32
		ds_byte[2] = 0;    //  not in use yet 
		ds_byte[3] = temp2;    //  
		ds_byte[4] = temp2>>8; //
		if (temp2 > 99) temp = temp2/10; // adjust for LED
    else temp = temp2; 
		if (a_digit == 1) a_secment = 0xB0; // (0b10110000) digit1 = 3
    if (a_digit == 2) a_secment = 0xA4; // (0b10100100) digit2 = 2
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); // 
		break;
//
    case 33: // parameter 33 selected
    EE_read(EE_par_33);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(0,30); // request value change with min max value for this parameter 
     }
    if (new_EE == 1) EE_write(EE_par_33, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 33;   // show parameter 33 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0xB0; // (0b10110000) digit1 = 3
    if (a_digit == 2) a_secment = 0xB0; // (0b10110000) digit1 = 3
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
    break;
//
    case 34: // parameter 34 selected 23-08-2011
    EE_read(EE_par_34);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(0,9); // request value change with min max value for this parameter 15-08-2012 
     }
    if (new_EE == 1) EE_write(EE_par_34, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 34;   // show parameter 34 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0xB0; // (0b10110000) digit1 = 3
    if (a_digit == 2) a_secment = 0x99; // (0b10011001) digit1 = 4
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
    break;
//
    case 35: // parameter 35 selected
    EE_read(EE_par_35);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(0,3); // request value change with min max value for this parameter 16-05-2013_c 04-01-2018
     }
    if (new_EE == 1) EE_write(EE_par_35, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 35;   // show parameter 35 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; // 
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0xB0; // (0b10110000) digit1 = 3
    if (a_digit == 2) a_secment = 0x92; // (0b10010010) digit2 = 5
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
    break; 
//
    case 36: // parameter 36 selected
    EE_read(EE_par_36);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
      req_val_change(0,2); // request value change with min max value for this parameter 01-12-2021
     }
    if (new_EE == 1) EE_write(EE_par_36, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 36;   // show parameter 36 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0xB0; // (0b10110000) digit1 = 3
    if (a_digit == 2) a_secment = 0x82; // (0b10000010) digit2 = 6
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
    break; 
//
    case 41: // parameter 41 selected
    if ((para_test ==1) || (confirm_timer > 0)) 
     {
      show_run();
      break;
     }
    else
     {
      s_speed_learn = 0; // deactivate singleturn speed learn command when not run
      EE_read(EE_par_41);
      new_EE = 0;
      if ((p_value == 1) & (para_test == 0))
       {
        req_val_change(0,4); // request value change with min max value for this parameter 21-11-2023
       }
			if ((e_type == 0x01) && (temp == 1)) temp = 4; // 21-11-2023
			if ((e_type == 0x01) && (temp == 3)) temp = 0; // 21-11-2023 
			if ((e_type != 0x01) && (temp == 4))  temp = 3; // 21-11-2023
			if ((e_type != 0x01) && (temp == 1))  temp = 3; // 21-11-2023 
			if ((e_type != 0x01) && (temp == 2))  temp = 0; // 21-11-2023
      if (new_EE == 1) EE_write(EE_par_41, temp);  // save new value
			ds_byte[0] = 1;    // Prog mode active
		  ds_byte[1] = 41;   // show parameter 41 
		  ds_byte[2] = 0;    //  not in use yet
		  ds_byte[3] = temp; //
      ds_byte[4] = 0;
      if (a_digit == 1) a_secment = 0x99; // (0b10011001) digit1 = 4
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
      if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
      if (a_digit == 4) calc_a_secment(); //
      break;
     } 
//
    case 44: // parameter 44 selected
    EE_read(EE_par_44);
    new_EE = 0;
    temp3 = temp;
    EE_read(EE_par_41);
    if ((temp == 4) && (temp3!= 2) && (temp3!= 5)) new_EE = 1; // if wrong value is selected, change this
    temp = temp3;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(0,5); // request value change with min max value for this parameter 
     }
    if (new_EE == 1)   // save new value
     {
      temp3 = temp;
      EE_read(EE_par_41);
      if ((temp == 4) && (temp3 <= 2)) temp3 = 2;
      if ((temp == 4) && (temp3 == 3)) temp3 = 5;
      if ((temp == 4) && (temp3 == 4)) temp3 = 2;
      EE_write(EE_par_44, temp3);
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 44;   // show parameter 44 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0x99; // (0b10011001) digit1 = 4
    if (a_digit == 2) a_secment = 0x99; // (0b10011001) digit2 = 4
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
    break;   
//
    case 51: // parameter 51 selected
    if ((para_test ==1) || (confirm_timer > 0)) 
     {
      show_run();
      break;
     }
    else
     {
      EE_read(EE_par_51);
      new_EE = 0;
      if ((p_value == 1) & (para_test == 0))
       {
        req_val_change(0,4); // request value change with min max value for this parameter 20-02-2014
       }
      if (new_EE == 1) EE_write(EE_par_51, temp);  // save new value
			ds_byte[0] = 1;    // Prog mode active
		  ds_byte[1] = 51;   // show parameter 51 
		  ds_byte[2] = 0;    //  not in use yet
		  ds_byte[3] = temp; // 
      ds_byte[4] = 0;
      if (a_digit == 1) a_secment = 0x92; // (0b10010010) digit1 = 5
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
      if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
      if (a_digit == 4) calc_a_secment(); //
      break;
     } 
//
    case 52: // parameter 52 selected
    EE_read(EE_par_52);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(0,99); // request value change with min max value for this parameter 
     }
    if (new_EE == 1)
     {
      EE_write(EE_par_52, temp);  // save new value
      calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 26-10-2011
      EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 26-10-2011
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 52;   // show parameter 52 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; // 
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0x92; // (0b10010010) digit1 = 5
    if (a_digit == 2) a_secment = 0xA4; // (0b10100100) digit2 = 2
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
		break;   
//
    case 53: // parameter 53 selected
    EE_read(EE_par_53);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(5,99); // request value change with min max value for this parameter 
     }
    if (new_EE == 1) EE_write(EE_par_53, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 53;   // show parameter 53 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; // 
    ds_byte[4] = 0;
		if (a_digit == 1) a_secment = 0x92; // (0b10010010) digit1 = 5
    if (a_digit == 2) a_secment = 0xB0; // (0b10110000) digit2 = 3
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
    break;     
//
		case 55: // parameter 55 selected 03-04-2024
    EE_read(EE_par_55);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(0,1); // request value change with min max value for this parameter 03-04-2024
     }
    if (new_EE == 1) EE_write(EE_par_55, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 55;   // show parameter 55 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; // 
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0x92; // (0b10010010) digit1 = 5
    if (a_digit == 2) a_secment = 0x92; // (0b10010010) digit2 = 5
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
    break;  
//		 
    case 58: // parameter 58 selected
    EE_read(EE_par_58);
    if ((p_value == 0) & (para_test == 0))
     {
      new_EE = 0;
      stop_pb_tb = 0;
     }
    else
     {
      if ((p_value == 1) & (para_test == 0)) 
       {
        EE_read(EE_par_58_new);
        req_val_change(0,4); // request value change with min max value for this parameter 
       }
      if (new_EE == 1)
       {
        EE_write(EE_par_58_new,temp);
        new_EE = 0;
        if (stop_long_tim == 0) reload_ser_cnt();
       }
      else
       {
        if (stop_long_tim == 0) new_EE = 1;
        else
         {
          if (stop_pb_tb == 1)
           {
            if ((stop_pb == 0) && (stop_lid_pb == 0) && (stop_disp_pb == 0))
             {
              p_value = 0;
              par_val = 0;
             }
           }
          else
           {
            if (((stop_pb == 1) || (stop_lid_pb == 1) || (stop_disp_pb == 1)) && (par_val == 1)) stop_pb_tb = 1;
           }
         }
       }
     } 
    if (show_clr_tim > 0)
     {
			ds_byte[0] = 1;        // Prog mode active
			temp2 = 1058;
			ds_byte[1] = temp2;    // show CLEAR
      ds_byte[2] = temp2>>8;
      ds_byte[4] = 255;      //  show no value
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xC6; // (0b11000110) digit2 = C
      if (a_digit == 3) a_secment = 0xC7; // (0b11000111) digit3 = L
      if (a_digit == 4) a_secment = 0xCE; // (0b11001110) digit3 = r
      if (a_digit == 5) a_secment = 0xFF; // (0b11111111) colon off
     }
    else
     {
			ds_byte[0] = 1;    // Prog mode active
		  ds_byte[1] = 58;   // show parameter 58 
		  ds_byte[2] = 0;    //  not in use yet
		  ds_byte[3] = temp; //
      ds_byte[4] = 0;
      if (a_digit == 1) a_secment = 0x92; // (0b10010010) digit1 = 5
      if (a_digit == 2) a_secment = 0x80; // (0b10000000) digit2 = 8
      if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
      if (a_digit == 4) calc_a_secment(); //
     }
    break;     
//
    case 59: // parameter 59 selected
    EE_read(EE_par_59);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(0,1); // request value change with min max value for this parameter 
     }
    if (new_EE == 1) EE_write(EE_par_59, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 59;   // show parameter 59 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0x92; // (0b10010010) digit1 = 5
    if (a_digit == 2) a_secment = 0x90; // (0b10010000) digit2 = 9
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
    break; 
//
		case 60: // parameter 60 selected 30-04-2019 12-02-2020
    if ((para_test ==1) || (confirm_timer > 0)) 
     {
			curtain_learn = 1; 
      show_run();
			break;
     }
    else
     {
			curtain_learn = 0; 
			EE_read(EE_par_60);
			if (temp == 0)
			 {
        EE_read(EE_curtain_rep_saved);
				if (temp == 1) EE_write(EE_curtain_rep_saved, 0); // 12-02-2020   
       }
      new_EE = 0;
      if ((p_value == 1) & (para_test == 0))
       {
        req_val_change(0,1); // request value change with min max value for this parameter
       }
			if (E_limit == 0) temp = 0; // no curtain function with mechanical switches 17-06-2024 
      if (new_EE == 1) EE_write(EE_par_60, temp);  // save new value
			ds_byte[0] = 1;    // Prog mode active
		  ds_byte[1] = 60;   // show parameter 60 
		  ds_byte[2] = 0;    //  not in use yet
		  ds_byte[3] = temp; //
      ds_byte[4] = 0;
      if (a_digit == 1) a_secment = 0x82; // (0b10000010) digit1 = 6
      if (a_digit == 2) a_secment = 0xC0; // (0b11000000) digit3 = 0
      if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
      if (a_digit == 4) calc_a_secment(); //
      break; 
     } 
//******************************************************************************************************
    case 71: // parameter 71 as in V7E SR
    EE_read(EE_par_71); // 18-05-2021
    if (para_test ==1) 
     {
      show_run();
      if (new_EE == 1)
       {
        new_EE = 0; // this new EE is saved on uP2
        confirm_learn = 1;
        p_value = 0;
        para_test = 0;
        par_val = 0;
       }
      break;
     }
    else if ((limit_indi > 0) || (confirm_timer > 0))
     {
      break; // learned limit is indicated in display_run
     }
    else
     {
			EE_read(EE_par_11); // ready for test later 04-03-2026 
      temp1 = temp;			 
      EE_read(EE_par_71); // 18-05-2021
      new_EE = 0;
      if ((p_value == 1) & (para_test == 0)) // 10-06-2022
       {
        req_val_change(0,56); // request value change with min max value for this parameter 22-06-2021 04-03-2026
        if ((open_pb == 1) && (temp == 7))  temp = 10; // 7 does not exist 29-05-2020 16-06-2020 28-06-2021 10-06-2022
        if ((open_pb == 1) && (temp == 15)) temp = 20; // 15 does not exist 29-05-2020 16-06-2020
        if ((open_pb == 1) && (temp == 22)) temp = 25; // 22 does not exist 29-05-2020 16-06-2020
        if ((open_pb == 1) && (temp == 27)) temp = 30; // 27 does not exist 16-06-2020 16-06-2020
        if ((open_pb == 1) && (temp == 35)) temp = 40; // 35 does not exist 29-05-2020 16-06-2020
        if ((open_pb == 1) && (temp == 46)) temp = 50; // 46 does not exist 16-06-2020 08-05-2025
				if ((temp == 56) && (temp1 == 7)) temp = 55; // 56 only available with electronic limits 04-03-2026
        //
        if ((close_pb == 1) && (temp == 49)) temp = 45; // 49 does not exist 16-06-2020 08-05-2025
        if ((close_pb == 1) && (temp == 39)) temp = 34; // 39 does not exist 16-06-2020
        if ((close_pb == 1) && (temp == 29)) temp = 26; // 29 does not exist 29-05-2020 16-06-2020
        if ((close_pb == 1) && (temp == 24)) temp = 21; // 24 does not exist 16-06-2020
        if ((close_pb == 1) && (temp == 19)) temp = 14; // 19 does not exist 29-05-2020 16-06-2020
        if ((close_pb == 1) && (temp == 9)) temp = 6; // 9 does not exist 29-05-2020 16-06-2020 28-06-2021 10-06-2022
				//
       }
			temp1 = temp; // move to temp to temp1 04-03-2026
			EE_read(EE_par_26); // adjust automatic to 26:02 when this relay door lock is selected 04-03-2026
      if ((temp1 == 56) && (temp != 2) && (E_limit == 1))	EE_write(EE_par_26, 2);	
      temp = temp1; // move back again to fit code below 04-03-2026		 
      //
        if (new_EE == 1) EE_write(EE_par_71, temp);  // save new value 
			  ds_byte[0] = 1;    // Prog mode active
		    ds_byte[1] = 71;   // show parameter 71 
		    ds_byte[2] = 0;    //  not in use yet
		    ds_byte[3] = temp; // 
        ds_byte[4] = 0;
        if (a_digit == 1) a_secment = 0xF8; // (0b11111000) digit1 = 7
        if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
        if (a_digit == 3) calc_a_secment(); //
        if (a_digit == 4) calc_a_secment(); //
			break;
     }
//*******************************************************************************************************
    case 72: // parameter 72 as in V7E SR
    //EE_read(EE_par_70); // 01-02-2022 check for special trafic light selected in parameter 70 10-06-2022
    //temp1 = temp; //			
    EE_read(EE_par_72); // 18-05-2021
    if (para_test ==1) 
     {
      show_run();
      if (new_EE == 1)
       {
        new_EE = 0; // this new EE is saved on uP2
        confirm_learn = 1;
        p_value = 0;
        para_test = 0;
        par_val = 0;
       }
      break;
     }
    else if ((limit_indi > 0) || (confirm_timer > 0))
     {
      break; // learned limit is indicated in display_run
     }
    else
     {
      EE_read(EE_par_72); // 18-05-2021
      new_EE = 0;
      if ((p_value == 1) & (para_test == 0) && (temp1 != 9)) // 01-02-2022 check for special trafic light selected in parameter 70
       {
        req_val_change(0,56); // request value change with min max value for this parameter 22-06-2021 04-03-2026
        if ((open_pb == 1) && (temp == 7))  temp = 10;  // 7 does not exist 29-05-2020 16-06-2020 28-06-2021 10-06-2022
        if ((open_pb == 1) && (temp == 15)) temp = 20; // 15 does not exist 29-05-2020 16-06-2020
        if ((open_pb == 1) && (temp == 22)) temp = 25; // 22 does not exist 29-05-2020 16-06-2020
        if ((open_pb == 1) && (temp == 27)) temp = 30; // 27 does not exist 16-06-2020 16-06-2020
        if ((open_pb == 1) && (temp == 35)) temp = 40; // 35 does not exist 29-05-2020 16-06-2020
        if ((open_pb == 1) && (temp == 46)) temp = 50; // 46 does not exist 16-06-2020 08-05-2025
				if ((temp == 56) && (temp1 == 7)) temp = 55; // 56 only available with electronic limits 04-03-2026 
        //
        if ((close_pb == 1) && (temp == 49)) temp = 45; // 49 does not exist 16-06-2020 08-05-2025
        if ((close_pb == 1) && (temp == 39)) temp = 34; // 39 does not exist 16-06-2020
        if ((close_pb == 1) && (temp == 29)) temp = 26; // 29 does not exist 29-05-2020 16-06-2020
        if ((close_pb == 1) && (temp == 24)) temp = 21; // 24 does not exist 16-06-2020
        if ((close_pb == 1) && (temp == 19)) temp = 14; // 19 does not exist 29-05-2020 16-06-2020
        if ((close_pb == 1) && (temp == 9)) temp = 6; // 9 does not exist 29-05-2020 16-06-2020 28-06-2021 10-06-2022
       }
			temp1 = temp; // move to temp to temp1 04-03-2026
			EE_read(EE_par_26); // adjust automatic to 26:02 when this relay door lock is selected 04-03-2026
      if ((temp1 == 56) && (temp != 2))	EE_write(EE_par_26, 2);	
      temp = temp1; // move back again to fit code below 04-03-2026	 
      //
        if (new_EE == 1) EE_write(EE_par_72, temp);  // save new value
			  ds_byte[0] = 1;    // Prog mode active
		    ds_byte[1] = 72;   // show parameter 72 
		    ds_byte[2] = 0;    //  not in use yet
		    ds_byte[3] = temp; //
        ds_byte[4] = 0;
        if (a_digit == 1) a_secment = 0xF8; // (0b11111000) digit1 = 7
        if (a_digit == 2) a_secment = 0xA4; // (0b10100100) digit2 = 2
        if (a_digit == 3) calc_a_secment(); //
        if (a_digit == 4) calc_a_secment(); //
			break;
     }
//*******************************************************************************************************
    case 74: // parameter 45 selected 16-11-2021 copy from V7E SR
    EE_read(EE_par_74); // 18-05-2021
    if (para_test ==1) 
     {
      show_run();
      if (new_EE == 1)
       {
        new_EE = 0; // this new EE is saved on uP2
        confirm_learn = 1;
        p_value = 0;
        para_test = 0;
        par_val = 0;
       }
      break;
     }
    else if ((limit_indi > 0) || (confirm_timer > 0))
     {
      break; // learned limit is indicated in display_run
     }
    else
     {
      EE_read(EE_par_74); // 18-05-2021
      new_EE = 0;
      if ((p_value == 1) & (para_test == 0))
       {
        req_val_change(0,56); // request value change with min max value for this parameter 22-06-2021 04-03-2026
        if ((open_pb == 1) && (temp == 7))  temp = 10; // 7 does not exist 29-05-2020 16-06-2020 28-06-2021
        if ((open_pb == 1) && (temp == 15)) temp = 20; // 15 does not exist 29-05-2020 16-06-2020
        if ((open_pb == 1) && (temp == 22)) temp = 25; // 22 does not exist 29-05-2020 16-06-2020
        if ((open_pb == 1) && (temp == 27)) temp = 30; // 27 does not exist 16-06-2020 16-06-2020
        if ((open_pb == 1) && (temp == 35)) temp = 40; // 35 does not exist 29-05-2020 16-06-2020
        if ((open_pb == 1) && (temp == 46)) temp = 50; // 46 does not exist 16-06-2020 08-05-2025
				if ((temp == 56) && (temp1 == 7)) temp = 55; // 56 only available with electronic limits 04-03-2026 
        //
        if ((close_pb == 1) && (temp == 49)) temp = 45; // 49 does not exist 16-06-2020 08-05-2025
        if ((close_pb == 1) && (temp == 39)) temp = 34; // 39 does not exist 16-06-2020
        if ((close_pb == 1) && (temp == 29)) temp = 26; // 29 does not exist 29-05-2020 16-06-2020
        if ((close_pb == 1) && (temp == 24)) temp = 21; // 24 does not exist 16-06-2020
        if ((close_pb == 1) && (temp == 19)) temp = 14; // 19 does not exist 29-05-2020 16-06-2020
        if ((close_pb == 1) && (temp == 9)) temp = 6; // 9 does not exist 29-05-2020 16-06-2020 28-06-2021
       }
			temp1 = temp; // move to temp to temp1 04-03-2026
			EE_read(EE_par_26); // adjust automatic to 26:02 when this relay door lock is selected 04-03-2026
      if ((temp1 == 56) && (temp != 2))	EE_write(EE_par_26, 2);	
      temp = temp1; // move back again to fit code below 04-03-2026	 
      //
        if (new_EE == 1) EE_write(EE_par_74, temp);  // save new value
			  ds_byte[0] = 1;    // Prog mode active
		    ds_byte[1] = 74;   // show parameter 74 
		    ds_byte[2] = 0;    //  not in use yet
		    ds_byte[3] = temp; //
        ds_byte[4] = 0;
        if (a_digit == 1) a_secment = 0xF8; // (0b11111000) digit1 = 7
        if (a_digit == 2) a_secment = 0x99; // (0b10010010) digit2 = 4
        if (a_digit == 3) calc_a_secment(); //
        if (a_digit == 4) calc_a_secment(); //
			break;
     }
//*******************************************************************************************************
    case 75: // parameter 75 selected 16-11-2021 copy from V7E SR
    EE_read(EE_par_75); // 18-05-2021
    if (para_test ==1) 
     {
      show_run();
      if (new_EE == 1)
       {
        new_EE = 0; // this new EE is saved on uP2
        confirm_learn = 1;
        p_value = 0;
        para_test = 0;
        par_val = 0;
       }
      break;
     }
    else if ((limit_indi > 0) || (confirm_timer > 0))
     {
      break; // learned limit is indicated in display_run
     }
    else
     {
      EE_read(EE_par_75); // 18-05-2021
      new_EE = 0;
      if ((p_value == 1) & (para_test == 0))
       {
        req_val_change(0,56); // request value change with min max value for this parameter 22-06-2021 04-03-2026
        if ((open_pb == 1) && (temp == 7))  temp = 10; // 7 does not exist 29-05-2020 16-06-2020 28-06-2021
        if ((open_pb == 1) && (temp == 15)) temp = 20; // 15 does not exist 29-05-2020 16-06-2020
        if ((open_pb == 1) && (temp == 22)) temp = 25; // 22 does not exist 29-05-2020 16-06-2020
        if ((open_pb == 1) && (temp == 27)) temp = 30; // 27 does not exist 16-06-2020 16-06-2020
        if ((open_pb == 1) && (temp == 35)) temp = 40; // 35 does not exist 29-05-2020 16-06-2020
        if ((open_pb == 1) && (temp == 46)) temp = 50; // 46 does not exist 16-06-2020 08-05-2025
				if ((temp == 56) && (temp1 == 7)) temp = 55; // 56 only available with electronic limits 04-03-2026 
        //
        if ((close_pb == 1) && (temp == 49)) temp = 45; // 49 does not exist 16-06-2020 08-05-2025
        if ((close_pb == 1) && (temp == 39)) temp = 34; // 39 does not exist 16-06-2020
        if ((close_pb == 1) && (temp == 29)) temp = 26; // 29 does not exist 29-05-2020 16-06-2020
        if ((close_pb == 1) && (temp == 24)) temp = 21; // 24 does not exist 16-06-2020
        if ((close_pb == 1) && (temp == 19)) temp = 14; // 19 does not exist 29-05-2020 16-06-2020
        if ((close_pb == 1) && (temp == 9)) temp = 6; // 9 does not exist 29-05-2020 16-06-2020 28-06-2021
       }
			temp1 = temp; // move to temp to temp1 04-03-2026
			EE_read(EE_par_26); // adjust automatic to 26:02 when this relay door lock is selected 04-03-2026
      if ((temp1 == 56) && (temp != 2))	EE_write(EE_par_26, 2);	
      temp = temp1; // move back again to fit code below 04-03-2026	 
      //
        if (new_EE == 1) EE_write(EE_par_75, temp);  // save new value
			  ds_byte[0] = 1;    // Prog mode active
		    ds_byte[1] = 75;   // show parameter 75 
		    ds_byte[2] = 0;    //  not in use yet
		    ds_byte[3] = temp; //
        ds_byte[4] = 0;
        if (a_digit == 1) a_secment = 0xF8; // (0b11111000) digit1 = 7
        if (a_digit == 2) a_secment = 0x92; // (0b10010010) digit2 = 5
        if (a_digit == 3) calc_a_secment(); //
        if (a_digit == 4) calc_a_secment(); //
			break;
     }
//*******************************************************************************************************		 
    case 77: // parameter 77 selected 16-09-2019
    EE_read(EE_par_77);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
      req_val_change(0,10); // request value change with min max value for this parameter
     }
    if (new_EE == 1) EE_write(EE_par_77, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 77;   // show parameter 77 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0xF8; // (0b11111000) digit1 = 7
    if (a_digit == 2) a_secment = 0xF8; // (0b11111000) digit2 = 7
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
		break; 
//****************************************************************************************************
    case 78: // parameter 78 selected 16-09-2019
    EE_read(EE_par_78);
    if ((p_value == 1) & (para_test == 0))
     {
      if (open_pb == 1) // is open push button active
       {
        if (temp < 120)
         {
          if (ch_timer == 0)
           {
            if (temp >99) temp = temp + 10;
            else temp++;
            EE_write(EE_par_78, temp);
            ch_timer = 100;
           }
         }               
       }
      if (close_pb == 1) // is close push button active
       {
        if (temp > 0) 
         {
          if (ch_timer == 0)
           {
            if (temp >109) temp = temp - 10;
            else temp--;
            EE_write(EE_par_78, temp);
            ch_timer = 100;
           }
         }
       }
     }
    ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 78;   // show parameter 78 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;
    if (temp > 99) temp = temp/10;			 
    if (a_digit == 1) a_secment = 0xF8; // (0b11111000) digit1 = 7
    if (a_digit == 2) a_secment = 0x80; // (0b10000000) digit2 = 8
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
    break;
//****************************************************************************************************		 
    case 79: // parameter 79 selected 27-05-2020
    EE_read(EE_par_79);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
      req_val_change(1,5); // request value change with min max value for this parameter
     }
    if (new_EE == 1) EE_write(EE_par_79, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 79;   // show parameter 79 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0xF8; // (0b11111000) digit1 = 7
    if (a_digit == 2) a_secment = 0x98; // (0b10011000) digit2 = 9
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
		break; 
//****************************************************************************************************		 
   case 80: // parameter 80 selected 01-07-2021
   EE_read(EE_par_80);
   new_EE = 0;
   if ((p_value == 1) & (para_test == 0))
    {
     req_val_change(1,5); // request value change with min max value for this parameter
    }
   if (new_EE == 1) EE_write(EE_par_80, temp);  // save new value
	 if (temp == 3)  cls_trafic_tim = 150000; // preset time 5 min start with time if just selected 11-12-2025
   if (temp == 4)  cls_trafic_tim = 600000; // preset time 20 min
   if (temp == 5)  cls_trafic_tim = 1800000; // preset time 60 min
	 ds_byte[0] = 1;    // Prog mode active
	 ds_byte[1] = 80;   // show parameter 80 
	 ds_byte[2] = 0;    //  not in use yet
	 ds_byte[3] = temp; //
   ds_byte[4] = 0;
   if (a_digit == 1) a_secment = 0x80; // (0b10000000) digit1 = 8
   if (a_digit == 2) a_secment = 0xC0; // (0b11000000) digit2 = 0
   if (a_digit == 3) calc_a_secment(); //
   if (a_digit == 4) calc_a_secment(); //
	 break; 
//****************************************************************************************************		 
    case 81: // parameter 81 selected 09-03-2010
    EE_read(EE_par_81);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(0,3); // request value change with min max value for this parameter 
     }
    if (new_EE == 1) EE_write(EE_par_81, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 81;   // show parameter 81 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
		ds_byte[4] = 0;
		if (a_digit == 1) a_secment = 0x80; // (0b10000000) digit1 = 8
    if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
    if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
		break; 
//
    case 82: // parameter 82 selected 23-12-2011
    if ((s_encoder_tst == 1) & (e_tst_result == 0)) 
     {
      show_run();
      break;
     }
    else if (confirm_timer > 0)
     {
      temp = spec_encoder_test;
      if (temp == 1) // ERRA skal vises 13-01-2012
       {
				ds_byte[0] = 1;        // Prog mode active
			  temp2 = 1082;
			  ds_byte[1] = temp2;    // show ERROR NO ANSWER
        ds_byte[2] = temp2>>8;
        ds_byte[4] = 255;      //  show no value 
				if (a_digit == 1) a_secment = 0x86; //(0b10000110); // digit1 symbol E
        if (a_digit == 2) a_secment = 0xCE; //(0b11001110); // digit2 symbol R
        if (a_digit == 3) a_secment = 0xCE; //(0b11001110); // digit2 symbol R
        if (a_digit == 4) a_secment = 0x88; //(0b10001000); // digit4 symbol A
        if (a_digit == 5) a_secment = 0xFF; //(0b11111111); // colon symbol	 
				break;
       }
      if (temp == 2)
       {
				ds_byte[0] = 1;        // Prog mode active
			  temp2 = 1182;
			  ds_byte[1] = temp2;    // show ERROR BATTERY LOW
        ds_byte[2] = temp2>>8;
        ds_byte[4] = 255;      //  show no value 
        if (a_digit == 1) a_secment = 0x86; //(0b10000110); // digit1 symbol E
        if (a_digit == 2) a_secment = 0xFF; //(0b11111111); // digit2 symbol -
        if (a_digit == 3) a_secment = 0x80; //(0b10000000); // digit3 symbol B
        if (a_digit == 4) a_secment = 0x88; //(0b10001000); // digit4 symbol A
        if (a_digit == 5) a_secment = 0xFF; //(0b11111111); // colon symbol
        break;
       }
      if (temp == 3) // ERRP skal vises 13-01-2012
       {
				ds_byte[0] = 1;        // Prog mode active
			  temp2 = 1282;
			  ds_byte[1] = temp2;    // show ERROR POSITION
        ds_byte[2] = temp2>>8;
        ds_byte[4] = 255;      //  show no value
				if (a_digit == 1) a_secment = 0x86; //(0b10000110); // digit1 = E
        if (a_digit == 2) a_secment = 0xCE; //(0b11001110); // digit2 = r
        if (a_digit == 3) a_secment = 0xCE; //(0b11001110); // digit3 = r
        if (a_digit == 4) a_secment = 0x8C; //(0b10001100); // digit3 symbol P
        if (a_digit == 5) a_secment = 0xFF; //(0b11111111); // colon = off
				break;         
       } 
      else
       {
				ds_byte[0] = 1;        // Prog mode activ
				temp2 = 1181;
			  ds_byte[1] = temp2;    // show ENCODER BATT. VOLTAGE
        ds_byte[2] = temp2>>8; 
				temp2 = 300 + temp;
			  ds_byte[3] = temp2;    // battery voltage
        ds_byte[4] = temp2>>8;
				if (a_digit == 1) a_secment = 0xFF; //(0b11111111); // digit1 = OFF
        if (a_digit == 2) a_secment = 0xB0; //(0b10110000); // digit2 = 3
        if (a_digit == 3) calc_a_secment(); //
        if (a_digit == 4) calc_a_secment(); //
        if (a_digit == 5) a_secment = 0x7F; //(0b01111111); // colon symbol on
				break; 
       }
     }
    else
     {
			ds_byte[0] = 1;    // Prog mode active
		  ds_byte[1] = 82;   // show parameter 82 
		  ds_byte[2] = 0;    //  not in use yet
		  ds_byte[3] = 0;    // 
      ds_byte[4] = 255;  //  show no value	29-03-2017
			if (a_digit == 1) a_secment = 0x80; //(0b10000000); // digit1 = 8
      if (a_digit == 2) a_secment = 0xA4; //(0b10100100); // digit2 = 2
      if (a_digit == 3) a_secment = 0xC0; //(0b11000000); // digit3 = 0
      if (a_digit == 4) a_secment = 0xC0; //(0b11000000); // digit4 = 0
			break; 
     }
//
    case 84: // parameter 84 selected 06-11-2013
    EE_read(EE_par_84);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(0,3); // request value change with min max value for this parameter 21-09-2018	 29-04-2019 12-11-2024
     }
    if (new_EE == 1) EE_write(EE_par_84, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 84;   // show parameter 84 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;
		if (a_digit == 1) a_secment = 0x80; //(0b10000000); // digit1 = 8
    if (a_digit == 2) a_secment = 0x99; //(0b10011001); // digit2 = 4
    if (a_digit == 3) a_secment = 0xC0; //(0b11000000); // digit3 = 0
    if (a_digit == 4) calc_a_secment(); //
		break; 
//
		case 85: // parameter 85 selected - relay on before close limit 07-01-2022
    EE_read(EE_par_85); // 
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
      req_val_change(5,95); // request value change with min max value for this parameter 12-11-2021 04-02-2022
     }
    if (new_EE == 1)
     {
      EE_write(EE_par_85, temp);  // save new value
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 85;   // show parameter 85 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;		
		if (a_digit == 1) a_secment = 0x80; // (0b10000000) digit1 = 8
		if (a_digit == 2) a_secment = 0x92; // (0b10010010) digit2 = 5 
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
		break; 
//  
		case 86: // parameter 86 selected - relay on before open limit 07-01-2022
    EE_read(EE_par_86); // 
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
      req_val_change(5,95); // request value change with min max value for this parameter 12-11-2021
     }
    if (new_EE == 1)
     {
      EE_write(EE_par_86, temp);  // save new value
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 86;   // show parameter 86 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;		
		if (a_digit == 1) a_secment = 0x80; // (0b10000000) digit1 = 8 
		if (a_digit == 2) a_secment = 0x82; // (0b10000010) digit2 = 6 
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
		break; 
//
    case 91: // parameter 91 selected
    EE_read(EE_par_91);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(1,90); // request value change with min max value for this parameter 
     }
    if (new_EE == 1)
     {
		  EE_write(EE_par_91, temp);  // save new value
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 91;   // show parameter 91 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; // 
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0x98; // (0b10011000) digit2 = 9
    if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
		break;   
//		 
	  case 92: // parameter 92 selected
    EE_read(EE_par_92);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(1,90); // request value change with min max value for this parameter 
     }
    if (new_EE == 1)
     {
      EE_write(EE_par_92, temp);  // save new value
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 92;   // show parameter 92 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; // 
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0x98; // (0b10011000) digit2 = 9
    if (a_digit == 2) a_secment = 0xA4; //(0b10100100); digit2 = 2
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
		break;   
//		 	 
		case 93: // parameter 93 selected
    EE_read(EE_par_93);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(1,90); // request value change with min max value for this parameter 
     }
    if (new_EE == 1)
     {
      EE_write(EE_par_93, temp);  // save new value
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 93;   // show parameter 93 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; // 
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0x98; // (0b10011000) digit2 = 9
    if (a_digit == 2) a_secment = 0xB0; // (0b10110000) digit2 = 3
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
		break;   
//		 	  
		case 94: // parameter 94 selected
    EE_read(EE_par_94);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(1,90); // request value change with min max value for this parameter 
     }
    if (new_EE == 1)
     {
      EE_write(EE_par_94, temp);  // save new value
     }
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 94;   // show parameter 93 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; // 
    ds_byte[4] = 0;
    if (a_digit == 1) a_secment = 0x98; // (0b10011000) digit2 = 9
    if (a_digit == 2) a_secment = 0x99; // (0b10011001) digit2 = 4
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
		break;   
//		 	 
    case 100: // Inverter motor setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
		 }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F110E_RATED_POWER, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp = inverter_param; // 16 bit var is put in a 8 bit var!	
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 100;   // show parameter 100 
		     ds_byte[2] = 0;    //  not in use yet
		     ds_byte[3] = temp; //
				 ds_byte[4] = 0;
				 //
				 LED_temp = temp;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 //	
				 if ((p_value == 1) & (para_test == 0))
          {
					 //req_val_change(1,4); // request value change with min max value for this parameter 16-09-2016
				                          // 4 shall be shown as 1.5kW on display unit with LSis 14-09-2018
					 req_val_change(2,6); // request value change with min max value for this parameter 16-09-2016
				                        // 2 means 0.4 kW in M100 inverter and 6 means 2.2 kW 27-10-2021		
          } 
         if (new_EE == 1)
		      { 
					 data = temp; // Save in LCC V3 Inverter EEPROM also 21-05-2017
           adr = EE_par_100;
           write_int16_eeprom(); 	
		       status = setModbusParam(F110E_RATED_POWER, temp);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;	
        default:
        break;				
			 }
		 } 
		temp = (LED_temp & 0xFF); // move to temp 20-01-2022
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xC0; // (0b11000000) digit2 = 0
      if (a_digit == 2) a_secment = 0xC0; // (0b11000000) digit2 = 0
		 }
		switch (temp)
		 {
		  case 2: // 0.4kW
       if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit2 = 0
       if (a_digit == 4) a_secment = 0x99; // (0b10010010) digit2 = 4
       break;
      case 3: // 0.75kW 0.8 shown
       if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit2 = 0
       if (a_digit == 4) a_secment = 0x80; // (0b10000000) digit1 = 8
       break;
      case 4: // 1.1kW 
       if (a_digit == 3) a_secment = 0xF9; // (0b11111001) digit2 = 1
       if (a_digit == 4) a_secment = 0xF9; // (0b11111001) digit2 = 1
       break;
      case 5: // 1.5kW 
       if (a_digit == 3) a_secment = 0xF9; // (0b11111001) digit2 = 1
       if (a_digit == 4) a_secment = 0x92; // (0b10010010) digit2 = 5
       break;
		  case 6: // 2.2kW 
       if (a_digit == 3) a_secment = 0xA4; // (0b10100100) digit1 = 2
       if (a_digit == 4) a_secment = 0xA4; // (0b10100100) digit1 = 2
       break;
		  default:
       break;			 
		 }	
 	 break;
//
		 
//		 
    case 101: // Inverter motor setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F120F_RATED_VOLT, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp2 = inverter_param; // 16 bit var 
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 101;   // show parameter 101 
		     ds_byte[2] = 0;    //  not in use 
		     ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
				 //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(170,250); // request value change with min max value for this parameter 16-09-2016 14-09-2018
		      } 
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_101;
           write_int16_eeprom(); 	
		       status = setModbusParam(F120F_RATED_VOLT, temp2);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;
        default:
        break;					
			 }
		 }
    temp = (LED_temp & 0xFF); // move to temp 20-01-2022
		h = temp/100; 
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
			if (h > 0)
			 {
				temp = h; 
        if (a_digit == 3) a_secment = 0xFF; // (0b11111111) digit1 = off
				if (a_digit == 4) calc_a_secment(); // 
			 }
      else
			 {
        if (a_digit == 3) calc_a_secment(); //
        if (a_digit == 4) calc_a_secment(); //
			 }				
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xC0; // (0b11000000) digit2 = 0
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
			if (h > 0)
			 {
				temp = temp - (100 * h); // subtract hundreds , 1 or 2 only
       }
			if (a_digit == 3) calc_a_secment(); //
      if (a_digit == 4) calc_a_secment(); // 
		 }			 
	 break;		 
//	
    case 102: // Inverter motor setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F120D_RATED_CURR, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp2 = inverter_param; // 16 bit var 
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 102;   // show parameter 102 
		     ds_byte[2] = 0;    //  not in use 
		     ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
				 //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(10,100); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_102;
           write_int16_eeprom(); 		
		       status = setModbusParam(F120D_RATED_CURR, temp2);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;	
        default:
        break;					
			 }
		 } 
		temp = (LED_temp & 0xFF); // move to temp 20-01-2022
		h = temp/100; 
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
			if (h > 0)
			 {
				temp = h; 
        if (a_digit == 3) a_secment = 0xFF; // (0b11111111) digit1 = off
				if (a_digit == 4) calc_a_secment(); // 
			 }
      else
			 {
        if (a_digit == 3) calc_a_secment(); //
        if (a_digit == 4) calc_a_secment(); //
			 }				
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xC0; // (0b11000000) digit2 = 0
      if (a_digit == 2) a_secment = 0xA4; // (0b10100100) digit1 = 2 
			if (h > 0)
			 {
				temp = temp - (100 * h); // subtract hundreds , 1 or 2 only
       }
			if (a_digit == 3) calc_a_secment(); //
      if (a_digit == 4) calc_a_secment(); // 
		 }			 
	 break;		 
//			 
   case 103: // Inverter motor setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F120C_RATED_SPEED, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)                                        // rated slip freq M100 26-09-2018
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 //inv_com_state = 0; // must be a fail 26-09-2018
			   //temp2 = inverter_param; // 16 bit var 	
				 read_int16_eeprom(EE_par_103); // check 2 bytes 26-09-2018
         temp2 = data;
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 103;   // show parameter 103
		     ds_byte[2] = 0;    //  not in use 
				 ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
				 //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(830,7150); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_103;
           write_int16_eeprom(); 
 	        }
				 pole_adjust();	// 26-09-2018	
         if (slip_calc != inverter_param)
				  {					 
           status = setModbusParam(F120C_RATED_SPEED, slip_calc);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
					}	
				 inv_com_state = 2;
        break;
        case 2:
				 status = getModbusParam(F120B_MOTOR_POLES, &temp_poles); // motor pole counts in non blocking mode
		     if (status != HAL_OK)                                        // 26-09-2018
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 3;
				  }	
         break;	
				case 3:
				 if (pole_count != temp_poles)	
				  {
					 status = setModbusParam(F120B_MOTOR_POLES, pole_count);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				  }	
				 inv_com_state = 0;
         break;
	      default:
        break;					
			 }
		 } 
		temp = (LED_temp & 0xFF); // move to temp 20-01-2022
		if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
			temp = (LED_temp/100);  // most signifikant byte 
			if ((a_digit == 3) && (temp > 9)) calc_a_secment(); //
			else if (a_digit == 3) a_secment = 0xFF; // (0b11111111) digit3 = off 
      if (a_digit == 4) calc_a_secment(); // 
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xC0; // (0b11000000) digit2 = 0
      if (a_digit == 2) a_secment = 0xB0; // (0b10110000) digit2 = 3 
			temp = (LED_temp%100);  // show below hundreds 
			if (a_digit == 3) calc_a_secment(); //
      if (a_digit == 4) calc_a_secment(); // 
		 }			 
	 break;		 
//			 		 
    case 104: // Inverter motor setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F1112_RATED_FREQ, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     inv_com_state = 1;
				 break;	
        case 1:
				 temp2 = inverter_param; // 16 bit var 	
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 104;   // show parameter 104
		     ds_byte[2] = 0;    //  not in use 
				 ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
		     //
				 if (temp2 == 5000) temp2 = 100;
				 else if (temp2 == 6000) temp2 = 200; // 26-09-2018 only 50, 60and 100 Hz must be selected
				 else if (temp2 == 10000) temp2 = 300;
				 else temp2 = 100; // if out of range push back
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(100,300); // request value change with min max value for this parameter LSis Base freq.
		      } 
				 //	
				 LED_temp = temp2;   // save for later use for LED 21-01-2022	
				 //	
				 if (temp2 == 100) temp2 = 5000; // 26-09-2018
				 else if (temp2 == 200) temp2 = 6000;
				 else if (temp2 == 300) temp2 = 10000;	
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_104;
           write_int16_eeprom(); 			
					 rated_motor_freq = temp2;
           status = setModbusParam(F1112_RATED_FREQ, temp2);  // save new value
					 if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }	
			    }	
				 inv_com_state = 2;	
				 break;
        case 2:					
				 pole_adjust();	// 26-09-2018	
				 status = getModbusParam(F120C_RATED_SPEED, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)                                        // rated slip freq M100 26-09-2018
		      {
           // fejlmedelelse 
		       break; 
          }			
		     inv_com_state = 3;	// must wait to next cycle to get the value
				 break;
				case 3:	 
				 if (slip_calc != inverter_param)
				  {					 
           status = setModbusParam(F120C_RATED_SPEED, slip_calc);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
					}
		     inv_com_state = 0;					
				 break;
        default:
         break;					
			 }
		 } 
		if (LED_temp == 100) temp = 50; // only 50, 60 and 100 is possible
    if (LED_temp == 200) temp = 60; // only 50, 60 and 100 is possible
		if (LED_temp == 300) temp = 100; // only 50, 60 and 100 is possible  
		h = temp/100; 
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
			if (h > 0)
			 {
				temp = h; 
        if (a_digit == 3) a_secment = 0xFF; // (0b11111111) digit1 = off
				if (a_digit == 4) calc_a_secment(); // 
			 }
      else
			 {
        if (a_digit == 3) calc_a_secment(); //
        if (a_digit == 4) calc_a_secment(); //
			 }				
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xC0; // (0b11000000) digit2 = 0
      if (a_digit == 2) a_secment = 0x99; // (0b10011000) digit2 = 4 19-11-2024
			if (h > 0)
			 {
				temp = temp - (100 * h); // subtract hundreds , 1 or 2 only
       }
			if (a_digit == 3) calc_a_secment(); //
      if (a_digit == 4) calc_a_secment(); // 
		 } 
	 break;		 
//			 		  
    case 105: // Inverter motor setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 //status = getModbusParam(F800_AUTOTUNE, &inverter_param); // get inverter parameter in non blocking mode  - Not available in LSis
		     //if (status != HAL_OK)
		     // {
           // fejlmedelelse 
		     //  break; 
         // }			
		     //else
		     // {
		       inv_com_state = 1;
				 // }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp = 0;    //inverter_param; // 16 bit var is put in a 8 bit var! - Not available in LSis
         ds_byte[0] = 1;    // Prog mode active
	    	 ds_byte[1] = 105;   // show parameter 105 
		     ds_byte[2] = 0;    //  not in use yet
		     ds_byte[3] = temp; // 
         ds_byte[4] = 0;
         //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 //				
		     if ((p_value == 1) & (para_test == 0))
          {
           req_val_change(0,0); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      { 
					 data = temp; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_105;
           write_int16_eeprom(); 			
		       // status = setModbusParam(F800_AUTOTUNE, temp);  // save new value - Not available in LSis
			     //if (status != HAL_OK)
		       // {
             // fejlmedelelse 
		 	     //  break; 
            //}
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;
        default:
        break;					
			 }
		 } 
		temp = (LED_temp & 0xFF); // move to temp 20-01-2022
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xC0; // (0b11000000) digit2 = 0
      if (a_digit == 2) a_secment = 0x92; // (0b10010010) digit2 = 5
		 }
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); // 
	 break;		 
//*****************************************************
   case 109: // Inverter open setup 16-03-2017
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F1114_FREQ_UP_LIMIT, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp2 = inverter_param; // 16 bit var 	
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 109;   // show parameter 109
		     ds_byte[2] = 0;    //  not in use 
		     ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
				 //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 LED_temp = LED_temp/100; // divide because 10HZ is temp2 value 1000
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(5000,20000); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_109;
           write_int16_eeprom(); 			
		       status = setModbusParam(F1114_FREQ_UP_LIMIT, temp2);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
				 break;
        default:
        break;					
			 }
		}
		temp = (LED_temp & 0xFF); // move to temp 20-01-2022
		h = temp/100; 
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
			if (h > 0)
			 {
				temp = h; 
        if (a_digit == 3) a_secment = 0xFF; // (0b11111111) digit1 = off
				if (a_digit == 4) calc_a_secment(); // 
			 }
      else
			 {
        if (a_digit == 3) calc_a_secment(); //
        if (a_digit == 4) calc_a_secment(); //
			 }				
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xC0; // (0b11000000) digit2 = 0
      if (a_digit == 2) a_secment = 0x98; // (0b10011000) digit2 = 9
			if (h > 0)
			 {
				temp = temp - (100 * h); // subtract hundreds , 1 or 2 only
       }
			if (a_digit == 3) calc_a_secment(); //
      if (a_digit == 4) calc_a_secment(); // 
		 }
   break;		 
//******************************************************************************				 
   case 110: // Inverter open setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F1101_TARGET_FREQ, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp2 = inverter_param; // 16 bit var 	
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 110;   // show parameter 110
		     ds_byte[2] = 0;    //  not in use 
		     ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
				 //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 LED_temp = LED_temp/100; // divide because 10HZ is temp2 value 1000
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(1000,20000); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_110;
           write_int16_eeprom(); 			
		       status = setModbusParam(F1101_TARGET_FREQ, temp2);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;
        default:
        break;					
			 }
		 }
		temp = (LED_temp & 0xFF); // move to temp 20-01-2022
		h = temp/100; 
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
			if (h > 0)
			 {
				temp = h; 
        if (a_digit == 3) a_secment = 0xFF; // (0b11111111) digit1 = off
				if (a_digit == 4) calc_a_secment(); // 
			 }
      else
			 {
        if (a_digit == 3) calc_a_secment(); //
        if (a_digit == 4) calc_a_secment(); //
			 }				
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xF9; // (0b11111001) digit2 = 1
      if (a_digit == 2) a_secment = 0xC0; // (0b11000000) digit2 = 0
			if (h > 0)
			 {
				temp = temp - (100 * h); // subtract hundreds , 1 or 2 only
       }
			if (a_digit == 3) calc_a_secment(); //
      if (a_digit == 4) calc_a_secment(); // 
		 } 
	 break;		 
//				 
  case 111: // Inverter open setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F1232_STP_1_FREQ, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp2 = inverter_param; // 16 bit var 	
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 111;   // show parameter 111
		     ds_byte[2] = 0;    //  not in use 
		     ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
				 //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 LED_temp = LED_temp/100; // divide because 10HZ is temp2 value 1000
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(500,20000); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_111;
           write_int16_eeprom(); 			
		       status = setModbusParam(F1232_STP_1_FREQ, temp2);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;
        default:
        break;					
			 }
		 } 
		temp = (LED_temp & 0xFF); // move to temp 20-01-2022
		h = temp/100; 
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
			if (h > 0)
			 {
				temp = h; 
        if (a_digit == 3) a_secment = 0xFF; // (0b11111111) digit1 = off
				if (a_digit == 4) calc_a_secment(); // 
			 }
      else
			 {
        if (a_digit == 3) calc_a_secment(); //
        if (a_digit == 4) calc_a_secment(); //
			 }				
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xF9; // (0b11111001) digit2 = 1
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
			if (h > 0)
			 {
				temp = temp - (100 * h); // subtract hundreds , 1 or 2 only
       }
			if (a_digit == 3) calc_a_secment(); //
      if (a_digit == 4) calc_a_secment(); // 
		 }  
 	 break;		 
//					 
  case 112: // Inverter open setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F1103_ACC_TIME, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp2 = inverter_param; // 16 bit var 
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 112;   // show parameter 112
		     ds_byte[2] = 0;    //  not in use 
		     ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
				 //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(1,100); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_112;
           write_int16_eeprom(); 			
		       status = setModbusParam(F1103_ACC_TIME, temp2);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;
        default:
        break;					
			 }
		 } 
		temp = (LED_temp & 0xFF); // move to temp 20-01-2022
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xF9; // (0b11111001) digit2 = 1
      if (a_digit == 2) a_secment = 0xA4; // (0b10100100) digit1 = 2
		 }
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //	 
	 break;	
//
   case 113: // Inverter open setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F1247_STEP_1_DEC, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp2 = inverter_param; // 16 bit var 	
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 113;   // show parameter 113
		     ds_byte[2] = 0;    //  not in use 
		     ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
				 //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(1,100); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_113;
           write_int16_eeprom(); 			
		       status = setModbusParam(F1247_STEP_1_DEC, temp2);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;
        default:
        break;					
			 }
		 }
    temp = (LED_temp & 0xFF); // move to temp 20-01-2022
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xF9; // (0b11111001) digit2 = 1
      if (a_digit == 2) a_secment = 0xB0; // (0b10110000) digit2 = 3
		 }
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //			 
	 break;	
//
  case 114: // Inverter open setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F1104_DEC_TIME, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp2 = inverter_param; // 16 bit var 	
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 114;   // show parameter 114
		     ds_byte[2] = 0;    //  not in use 
		     ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
				 //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(1,100); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_114;
           write_int16_eeprom(); 			
		       status = setModbusParam(F1104_DEC_TIME, temp2);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;
        default:
        break;					
			 }
		 } 
		temp = (LED_temp & 0xFF); // move to temp 20-01-2022
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xF9; // (0b11111001) digit2 = 1
      if (a_digit == 2) a_secment = 0x99; // (0b10010010) digit2 = 4
		 }
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //	 
	 break;	
//
	 case 115: // parameter 115 selected
    EE_read(EE_par_115);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(5,50); // request value change with min max value for this parameter 
     }
    if (new_EE == 1) EE_write(EE_par_115, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 115;   // show parameter 115 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //  
		ds_byte[4] = 0; 
		//
    if (hundreds == 1)
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xF9; // (0b11111001) digit2 = 1
      if (a_digit == 2) a_secment = 0x92; // (0b10010010) digit2 = 5
		 }
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //		 
		// 
    break; 	 

//***********************************************
  case 120: // Inverter close setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F1233_STP_2_FREQ, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp2 = inverter_param; // 16 bit var 
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 120;   // show parameter 120
		     ds_byte[2] = 0;    //  not in use 
		     ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
				 //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 LED_temp = LED_temp/100; // divide because 10HZ is temp2 value 1000
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(1000,20000); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_120;
           write_int16_eeprom(); 			
		       status = setModbusParam(F1233_STP_2_FREQ, temp2);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;
        default:
        break;					
			 }
		 }
		temp = (LED_temp & 0xFF); // move to temp 20-01-2022
		h = temp/100; 
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
			if (h > 0)
			 {
				temp = h; 
        if (a_digit == 3) a_secment = 0xFF; // (0b11111111) digit1 = off
				if (a_digit == 4) calc_a_secment(); // 
			 }
      else
			 {
        if (a_digit == 3) calc_a_secment(); //
        if (a_digit == 4) calc_a_secment(); //
			 }				
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xA4; // (0b10100100) digit1 = 2
      if (a_digit == 2) a_secment = 0xC0; // (0b11000000) digit2 = 0
			if (h > 0)
			 {
				temp = temp - (100 * h); // subtract hundreds , 1 or 2 only
       }
			if (a_digit == 3) calc_a_secment(); //
      if (a_digit == 4) calc_a_secment(); // 
		 }   
	 break;	
//		 
 case 121: // Inverter close setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F1234_STP_3_FREQ, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp2 = inverter_param; // 16 bit var 	
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 121;   // show parameter 121
		     ds_byte[2] = 0;    //  not in use 
		     ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
				 //
				 LED_temp = temp2;        // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 LED_temp = LED_temp/100; // divide because 10HZ is temp2 value 1000
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(500,20000); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_121;
           write_int16_eeprom(); 			
		       status = setModbusParam(F1234_STP_3_FREQ, temp2);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;
        default:
        break;					
			 }
		 }
		temp = (LED_temp & 0xFF); // move to temp 20-01-2022
		h = temp/100; 
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
			if (h > 0)
			 {
				temp = h; 
        if (a_digit == 3) a_secment = 0xFF; // (0b11111111) digit1 = off
				if (a_digit == 4) calc_a_secment(); // 
			 }
      else
			 {
        if (a_digit == 3) calc_a_secment(); //
        if (a_digit == 4) calc_a_secment(); //
			 }				
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xA4; // (0b10100100) digit1 = 2
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
			if (h > 0)
			 {
				temp = temp - (100 * h); // subtract hundreds , 1 or 2 only 
       }
			if (a_digit == 3) calc_a_secment(); //
      if (a_digit == 4) calc_a_secment(); // 
		 }    
	 break;	
//		 
case 122: // Inverter close setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F1248_STEP_2_ACC, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp2 = inverter_param; // 16 bit var 	
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 122;   // show parameter 122
		     ds_byte[2] = 0;    //  not in use 
		     ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
				 //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(1,100); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_122;
           write_int16_eeprom(); 			
		       status = setModbusParam(F1248_STEP_2_ACC, temp2);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;
        default:
        break;					
			 }
		 } 
		temp = (LED_temp & 0xFF); // move to temp 20-01-2022
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xA4; // (0b10100100) digit1 = 2
      if (a_digit == 2) a_secment = 0xA4; // (0b10100100) digit1 = 2
		 }
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //	 
	 break;
//
 case 123: // Inverter close setup
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F124B_STEP_3_DEC, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp2 = inverter_param; // 16 bit var 
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 123;   // show parameter 123
		     ds_byte[2] = 0;    //  not in use 
		     ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
				 //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022
				 //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(1,100); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      { 
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_123;
           write_int16_eeprom(); 			
		       status = setModbusParam(F124B_STEP_3_DEC, temp2);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;
        default:
        break;					
			 }
		 } 
		temp = (LED_temp & 0xFF); // move to temp 20-01-2022
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xA4; // (0b10100100) digit1 = 2
      if (a_digit == 2) a_secment = 0xB0; // (0b10110000) digit2 = 3
		 }
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //	 
	 break; 
//
  case 124: // Inverter close setup - Parameter 114 and 124 use the same parameter because we turn off all - P1, P2 and P3
		new_EE = 0;	
		status = getModbusState();
		if (status == HAL_BUSY)
		 {
     }			 
		else
		 {
			switch (inv_com_state)
			 {
				case 0:
				 status = getModbusParam(F1104_DEC_TIME, &inverter_param); // get inverter parameter in non blocking mode
		     if (status != HAL_OK)
		      {
           // fejlmedelelse 
		       break; 
          }			
		     else
		      {
		       inv_com_state = 1;
				  }	
         break;	
        case 1:
				 inv_com_state = 0;
			   temp2 = inverter_param; // 16 bit var 	
				 ds_byte[0] = 1;    // Prog mode active
		     ds_byte[1] = 124;   // show parameter 124
		     ds_byte[2] = 0;    //  not in use 
		     ds_byte[3] = temp2;    // value
         ds_byte[4] = temp2>>8;	// value
         //
				 LED_temp = temp2;   // save for later use with 8 bit only so mask most signifikant bits 20-01-2022 
         //
				 if ((p_value == 1) & (para_test == 0))
          {
           req_val_change_b16(1,100); // request value change with min max value for this parameter 16-09-2016
		      } 
         if (new_EE == 1)
		      {
					 data = temp2; // Save in LCC V3 Inverter EEPROM also
           adr = EE_par_124;
           write_int16_eeprom(); 			
		       status = setModbusParam(F1104_DEC_TIME, temp2);  // save new value
			     if (status != HAL_OK)
		        {
             // fejlmedelelse 
		 	       break; 
            }
				   inv_com_state = 2;
		      }	
				 break;
        case 2:
         inv_com_state = 0;
         break;
        default:
        break;					
			 }
		 } 
	  temp = (LED_temp & 0xFF); // move to temp 20-01-2022
    if (hundreds == 1) // 20-01-2022
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xA4; // (0b10100100) digit1 = 2
      if (a_digit == 2) a_secment = 0x99; // (0b10010010) digit2 = 4
		 }
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //	 
	 break; 
//
    case 125: // parameter 125 selected
    EE_read(EE_par_125);
    new_EE = 0;
    if ((p_value == 1) & (para_test == 0))
     {
    req_val_change(5,50); // request value change with min max value for this parameter 
     }
    if (new_EE == 1) EE_write(EE_par_125, temp);  // save new value
		ds_byte[0] = 1;    // Prog mode active
		ds_byte[1] = 125;   // show parameter 125 
		ds_byte[2] = 0;    //  not in use yet
		ds_byte[3] = temp; //
    ds_byte[4] = 0;    //
    //
		if (hundreds == 1)
		 {
      if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = off
      if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1
		 }
    else
		 {			
      if (a_digit == 1) a_secment = 0xA4; // (0b10100100) digit1 = 2
      if (a_digit == 2) a_secment = 0x92; // (0b10010010) digit2 = 5
		 }
    if (a_digit == 3) calc_a_secment(); //
    if (a_digit == 4) calc_a_secment(); //
    break; 
//
   default:
    break;
  }
//	
  hundreds_tim++; // 20-01-2022 used for LED for values above 99
  if ((hundreds_tim > 0) && (hundreds_tim < 750)) hundreds = 1;	 
	else hundreds = 0;	
  if (hundreds_tim > 1500) hundreds_tim = 0;	
//	
 if (p_value == 0)
  {
   if (d_flash_tim == 0) // flash both needed for parameter 5 setup
    {
		 ds_byte[0] = 1;    // Prog mode active
		 ds_byte[1] = 255;  // parameternumber OFF by 255 
		 ds_byte[2] = 0;    //  not in use yet
		 if (a_digit == 1) a_secment = 0xFF; //(0b11111111); // digit1 = off
     if (a_digit == 2) a_secment = 0xFF; //(0b11111111); // digit2 = off 
		}
  }
 else if (para_test == 1)
  {
   if (d_flash_tim == 0)
    {
		 if (gdv1 == 1)
		  {	
		   ds_byte[0] = 1;    // Prog mode active
		   ds_byte[1] = 255;  // parameternumber OFF by 255 
		   ds_byte[2] = 0;    //  not in use yet
		   ds_byte[3] = 0;    // 	
		   ds_byte[4] = 255;  //	value off
			}
		 else
			{
			 if (a_digit == 1) a_secment = 0xFF; //(0b11111111); // digit1 = off
       if (a_digit == 2) a_secment = 0xFF; //(0b11111111); // digit2 = off 
       if (a_digit == 3) a_secment = 0xFF; //(0b11111111); // digit3 = off
       if (a_digit == 4) a_secment = 0xFF; //(0b11111111); // digit4 = off 
			}
    }
  } 
 else if (d_flash_tim == 0) // flash both needed for parameter 5 setup
  {
	 ds_byte[4] = 255;  // 	value off	
   if (a_digit == 3) a_secment = 0xFF; //(0b11111111); // digit3 = off
   if (a_digit == 4) a_secment = 0xFF; //(0b11111111); // digit4 = off	
  }
 }
}
//
//
void display_run()
{
 ds_byte[2] = 0; // preset values if not in use below
 ds_byte[3] = 0;	
 ds_byte[4] = 0;
 ds_byte[5] = 0;
 ds_byte[6] = 0;
 ds_byte[7] = 0;	
 EE_read(EE_factory);
if ((temp == 0) && (ver_show > 0))
 {
	if (temp == 0) ds_byte[4] = 1;	 // show FACTORY RESET by bytes 
	else ds_byte[4] = 0; // show software version only  
	temp = sub_version; // showing program number shortly by powerup or reset, adjust this when new main program is made
  if (spec_factory == 1) temp = temp + 10; // 11-03-2011
  if (spec_factory == 2) temp = temp + 20; // 11-03-2011
  if (spec_factory == 3) temp = temp + 30; // 11-03-2011 
	if (gdv1 == 1)
	 {		
	  ds_byte[0] = 0; //Run mode display
	  ds_byte[1] = 2; //software version select
	  ds_byte[2] = main_version;
	  ds_byte[3] = temp;
	 }
  else
   {
    if (a_digit == 1) a_secment = 0x8E; // (0b10001110) digit1 symbol
    if (a_digit == 2) a_secment = 0x88; // (0b10001000) digit2 symbol 
    if (a_digit == 3) a_secment = 0xC6; // (0b11000110) digit3 symbol
    if (a_digit == 4) a_secment = 0xFF; // (0b11111111) digit4 symbol
    if (a_digit == 5) a_secment = 0xFF; // (0b11111111) colon symbol
   }		 
 }
else if (ver_show > 0)
 {
	ds_byte[4] = 0; // show software version only  
	temp = sub_version; // showing program number shortly by powerup or reset, adjust this when new main program is made
  if (spec_factory == 1) temp = temp + 10; // 11-03-2011
  if (spec_factory == 2) temp = temp + 20; // 11-03-2011
  if (spec_factory == 3) temp = temp + 30; // 11-03-2011
  if (gdv1 == 1)
	 {		
	  ds_byte[0] = 0; //Run mode display
	  ds_byte[1] = 2; //software version select
	  ds_byte[2] = main_version;
	  ds_byte[3] = temp;
	 }
	else
	 {
		calc_a_secment(); // digit 3 and 4 adjusted
		temp = main_version; // get main version number
    calc_a_secment_main(); 		 
		if (a_digit == 5) a_secment = 0xFF; // (0b11111111) no colon symbol  
	 }
 }
//
else if (limits_show > 0) // 30-04-2019 for use with dalmatic display
 {
	if (gdv1 == 1)
	 { 
	  ds_byte[0] = 0; //Run mode display
	  ds_byte[1] = 4; //Limits info: Run mode display	 
    if (E_limit == 0)
	   {
		  ds_byte[2] = 0; //Run mode display
	   }
	  else
	   {
      if (e_type == 0) ds_byte[2] = 2; //Show Dalmatic Encoder 
		  if (e_type == 1) ds_byte[2] = 1; //Show Kostal Encoder 
      if (e_type == 2) ds_byte[2] = 3; //Show Feig Encoder
      if (e_type == 4) ds_byte[2] = 4; //Show SCE RS485 encoder 18-11-2020		 
	   }	
   }
  else
	 {
    if (E_limit == 0)
	   {
		  if (a_digit == 1) a_secment = 0x86; // (0b10000110) E digit1 symbol
      if (a_digit == 2) a_secment = 0xC6; // (0b11000110) C digit2 symbol 
      if (a_digit == 3) a_secment = 0xBF; // (0b10111111) - digit3 symbol
      if (a_digit == 4) a_secment = 0xC0; // (0b11000000) 0 digit4 symbol
      if (a_digit == 5) a_secment = 0xFF; // (0b11111111) no colon symbol
		 }
		else
		 {
      temp = e_type;
			if (a_digit == 1) a_secment = 0x86; // (0b10000110) E digit1 symbol
      if (a_digit == 2) a_secment = 0xC6; // (0b11000110) C digit2 symbol 
      if (a_digit == 3) a_secment = 0xBF; // (0b10111111) - digit3 symbol
      if (a_digit == 4) calc_a_secment(); //
      if (a_digit == 5) a_secment = 0xFF; // (0b11111111) no colon symbol
     }			
   }		
 }	
else if (show_photo_dis_point == 1) 
 {
	if (gdv1 == 1)
	 { 
	  ds_byte[0] = 0; //Run mode display
	  ds_byte[1] = 6; //show_photo_dis_point Run mode display
   }
  else
	 {
    if (a_digit == 1) a_secment = 0x8C; // (0b10001100) P digit1 symbol Photo set
    if (a_digit == 2) a_secment = 0x89; // (0b10001001) H digit2 symbol 
    if (a_digit == 3) a_secment = 0xBF; // (0b10111111) - digit3 symbol
    if (a_digit == 4) a_secment = 0x92; // (0b10010010) S digit4 symbol 17-06-2024
    if (a_digit == 5) a_secment = 0xFF; // (0b11111111) no colon symbol
	 }		
 } 
else
 {
 p_value = 0; // always start with parameter number next time prog is selected 14-04-09
 para_test = 0; // 17-01-2012
 s_encoder_tst = 0; // 17-01-2012
 if (disp_status == 1) 
  {
   show_status(); //
   // show status is selected function called from main
  }
 else if (ecode3 == 1) // SER shall be shown 21-09-2011
  {
	 if (gdv1 == 1)
	  {		 
	   ds_byte[0] = 0;  //Run mode display
	   ds_byte[1] = 50; //Inverter fail - Try new power-up - between door limits	
		}
	 else
		{
		 if (a_digit == 1) a_secment = 0x92; // (0b10010010); // digit1 symbol S 21-09-2011
     if (a_digit == 2) a_secment = 0x86; // (0b10000110); // digit2 symbol E
     if (a_digit == 3) a_secment = 0xCE; // (0b11001110); // digit3 symbol R
     if (a_digit == 4) a_secment = 0xFF; // (0b11111111); // digit4 symbol no aktive LED
     if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol	
		}
  }
 else     // normal display showing included priority
  {
   EE_read(EE_par_11);
   if ((low_bat == 1) && (temp > 0) && (temp != 7) && (bat_timer > 600)) // 2 sec. is low battery observed 26-03-2010
    {
		 if (gdv1 == 1)
		  {			 
		   ds_byte[0] = 0;  //Run mode display
	     ds_byte[1] = 51; //ENCODER BATT. LOW
      }
     else
		  {
       if (a_digit == 1) a_secment = 0x86; // (0b10000110); // digit1 symbol E
       if (a_digit == 2) a_secment = 0xFF; // (0b11111111); // digit2 symbol O
       if (a_digit == 3) a_secment = 0x80; // (0b10000000); // digit3 symbol B
       if (a_digit == 4) a_secment = 0x88; // (0b10001000); // digit4 symbol A
       if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
      }			 
	  }
   else
    {
     if (limit_indi > (indi_time - 1)) hide_timer = 60; // 200mS hiding some of the symbols after limit learning process
			                                       // 
     status_para = 1; //preparing for showing counts when status is selected
     error_counter = 1;
     if ((safety_test == 1) || (safety_after_time > 0)) hide_timer = 30; // hide symbols under safety test 21-02-2011    
     //
		 EE_read(EE_par_71); // 30-11-2021
     if (temp == 54) temp4 = 54;		
		 EE_read(EE_par_72); // 30-11-2021
     if (temp == 54) temp4 = 54;		
		 EE_read(EE_par_74); // 23-06-2021
     if (temp == 54) temp4 = 54;
     EE_read(EE_par_75); // 23-06-2021
     if (temp == 54) temp4 = 54;
		 //	
		 EE_read(EE_par_5); // 24-01-2022
		 if (temp > 1) // is inverter selected
		  {		 
		   chk_inverter_status();	// 13-03-2017
		  }
		 EE_read(EE_par_16);	
		 if (inv_fail_active > 3)
		  {
				// do noting, it is handled by chk_inverter_status
		  }	
		 else if ((e32_weld == 1) || (ecode3 == 1)) e32(); // 12-01-2022	26-01-2022
		 else if (e33_unlock_fail == 1) e33(); // 04-03-2026	
		 else if (e29_xtal == 1) e29(); // 24-11-2016
		 else if (e28_wdt == 1) e28(); // 29-11-2016
		 else if ((e27_modbus == 1) && (inverter_use == 1))  e27(); // 16-03-2017 08-12-2021 08-06-2022
		 else if ((fail_24v == 1) && (hide_timer == 0)) // 
      {
			 fail_volt_show++; // 
       if (fail_volt_show > 606) fail_volt_show = 0; // 13-01-2022	
			 switch (gdv1)
			  {
         case 0:
          if (fail_volt_show < 303)
           {
            if (a_digit == 1) a_secment = 0x8E; // (0b10001110); // digit1 = F
            if (a_digit == 2) a_secment = 0x88; // (0b10001000); // digit2 = A
            if (a_digit == 3) a_secment = 0xF9; // (0b11111001); // digit3 = I
            if (a_digit == 4) a_secment = 0xC7; // (0b11000111); // digit4 = L
            if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon = off
           }
          if ((fail_volt_show >= 303) && (fail_volt_show < 606) && (hide_timer == 0)) // 02-03-2015
           {
            if (a_digit == 1) a_secment = 0xFF; // (0b11111111); // digit1 = off
            if (a_digit == 2) a_secment = 0xA4; // (0b10100100); // digit2 = 2
            if (a_digit == 3) a_secment = 0x99; // (0b10011001); // digit3 = 4
            if (a_digit == 4) a_secment = 0xC1; // (0b11000001); // digit4 = V
            if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon = off
           }
          break;
         case 1:				 
			    ds_byte[0] = 0;  //Run mode display
	        ds_byte[1] = 55; //Error 24V
          break;
				 default:
          break;				 
			  }				 
			}
		 else if ((fail_12v == 1) && (hide_timer == 0)) // 
      {
			 fail_volt_show++; // 
       if (fail_volt_show > 606) fail_volt_show = 0; // 13-01-2022	
			 switch (gdv1)
			  {
         case 0:
          if (fail_volt_show < 303)
           {
            if (a_digit == 1) a_secment = 0x8E; // (0b10001110); // digit1 = F
            if (a_digit == 2) a_secment = 0x88; // (0b10001000); // digit2 = A
            if (a_digit == 3) a_secment = 0xF9; // (0b11111001); // digit3 = I
            if (a_digit == 4) a_secment = 0xC7; // (0b11000111); // digit4 = L
            if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon = off
           }
          if ((fail_volt_show >= 303) && (fail_volt_show < 606) && (hide_timer == 0)) // 02-03-2015
           {
            if (a_digit == 1) a_secment = 0xFF; // (0b11111111); // digit1 = off
            if (a_digit == 2) a_secment = 0xF9; // (0b11111001); // digit2 = 1
            if (a_digit == 2) a_secment = 0xA4; // (0b10100100); // digit2 = 2
            if (a_digit == 4) a_secment = 0xC1; // (0b11000001); // digit4 = V
            if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon = off
           }
          break;
         case 1:				 
			    ds_byte[0] = 0;  //Run mode display
	        ds_byte[1] = 56; //Error 12V
          break;
				 default:
          break;				 
			  }				 
			}
		 else if ((wic_slack_range == 0) && (hide_timer == 0) && (e_stop_pb == 1))  // 21-11-2023
      {
			 if (gdv1 == 1)
			  {				 
			   ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 14; //STOP symbol OBS skal ændres til 25 når display software er revideret 21-11-2023
        }
       else
			  {
         if (a_digit == 1) a_secment = 0xA7; // (0b10100111); // digit1 symbol E stop symbol 21-11-2023
         if (a_digit == 2) a_secment = 0xB7; // (0b10110111); // digit2 symbol 
         if (a_digit == 3) a_secment = 0xB7; // (0b10110111); // digit3 symbol
         if (a_digit == 4) a_secment = 0xB3; // (0b10110011); // digit4 symbol
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
        }				 
			}	
		 else if ((wic_slack_range == 0) && (hide_timer == 0) && (((stop_pb == 1) || (stop_lid_pb == 1) || (stop_disp_pb == 1)) ||
			 ((cls == 1) && (ols == 1)))) // 05-08-2010 08-06-2022
      {
			 if (gdv1 == 1)
			  {				 
			   ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 14; //STOP symbol
        }
       else
			  {
         if (a_digit == 1) a_secment = 0xC6; // (0b11000110); // digit1 symbol
         if (a_digit == 2) a_secment = 0xF6; // (0b11110110); // digit2 symbol 
         if (a_digit == 3) a_secment = 0xF6; // (0b11110110); // digit3 symbol
         if (a_digit == 4) a_secment = 0xF0; // (0b11110000); // digit4 symbol
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
        }				 
			}
		 else if ((wic_slack_range == 0) && (thermo_stop == 1)) // 22-12-2021
      {
			 if (gdv1 == 1)
			  {				 
			   ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 25; //Thermo/Spec door STOP symbol 
        }
       else
			  {
         if (a_digit == 1) a_secment = 0xF8; // (0b11111000); // digit1 symbol T
         if (a_digit == 2) a_secment = 0x89; // (0b10001001); // digit2 symbol H
         if (a_digit == 3) a_secment = 0x86; // (0b10000110); // digit3 symbol E
         if (a_digit == 4) a_secment = 0xCE; // (0b11001110); // digit4 symbol R
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
        }				 
			}	
		 else if ((Safety_2_short == 1) & (wic_slack == 1)) // 19-11-2018 11-12-2018
      {
			 if (gdv1 == 1)
			  {
				 ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 200; //Shorted text
			  }
			 else
			  {	
         if (a_digit == 1) a_secment = 0x92; // (0b10010010); // digit1 symbol S
         if (a_digit == 2) a_secment = 0x89; // (0b10001001); // digit2 symbol H 
         if (a_digit == 3) a_secment = 0xC0; // (0b11000000); // digit3 symbol 0
         if (a_digit == 4) a_secment = 0xCE; // (0b11001110); // digit4 symbol R
         if (a_digit == 5) a_secment = 0XFF; // (0b11111111); // colon symbol
				}
	    }
     else if ((WD_activated == 1) & (SC_activated == 1) & (wic_slack == 1) & 
             (((encoder_errors == 0) & (run_prog == 0) && (hide_timer == 0)) | (run_prog == 1))) // 20-12-2018 08-06-2022 
      {
			 if (gdv1 == 1)
			  {
				 ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 203; //slack rope and wicket door activated text
			  }
			 else	
			  {
         if (a_digit == 1) a_secment = 0x9B; // (0b10011011); // digit2 symbol Symbol cable 
         if (a_digit == 2) a_secment = 0xAD; // (0b10101101); // digit3 symbol Symbol cable 
         if (a_digit == 3) a_secment = 0xC6; // (0b11000110); // digit3 symbol Symbol Wicket door
         if (a_digit == 4) a_secment = 0xF0; // (0b11110000); // digit4 symbol Symbol Wicket door
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
			  }
      }
		 else if ((wic_slack_range == 1) && (hide_timer == 0)) //  23-05-2022 always show if out of range
      {
			 if (gdv1 == 1)
			  {
				 ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 203; //slack rope and wicket door activated text
			  }
			 else	
			  {
         if (a_digit == 1) a_secment = 0x9B; // (0b10011011); // digit2 symbol Symbol cable 
         if (a_digit == 2) a_secment = 0xAD; // (0b10101101); // digit3 symbol Symbol cable 
         if (a_digit == 3) a_secment = 0xC6; // (0b11000110); // digit3 symbol Symbol Wicket door
         if (a_digit == 4) a_secment = 0xF0; // (0b11110000); // digit4 symbol Symbol Wicket door
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
			  }
      }	
		 else if ((wic_slack_range == 1) && (hide_timer == 0)) //  08-06-2022 always show if out of range
      {
			 if (gdv1 == 1)
			  {
				 ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 203; //slack rope and wicket door activated text
			  }
			 else	
			  {
         if (a_digit == 1) a_secment = 0x9B; // (0b10011011); // digit2 symbol Symbol cable 
         if (a_digit == 2) a_secment = 0xAD; // (0b10101101); // digit3 symbol Symbol cable 
         if (a_digit == 3) a_secment = 0xC6; // (0b11000110); // digit3 symbol Symbol Wicket door
         if (a_digit == 4) a_secment = 0xF0; // (0b11110000); // digit4 symbol Symbol Wicket door
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
			  }
      }	
		 else if ((SC_activated == 1) & (wic_slack == 1) &
             (((encoder_errors == 0) & (run_prog == 0) && (hide_timer == 0)) | (run_prog == 1))) // 08-06-2022
      {
			 if (gdv1 == 1)
			  {
				 ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 201; //slack rope activated text
			  }
			 else	
			  {	
         if (a_digit == 1) a_secment = 0xFF; // (0b11111111); // digit1 symbol turned off
         if (a_digit == 2) a_secment = 0x9B; // (0b10011011); // digit2 symbol Symbol  
         if (a_digit == 3) a_secment = 0xAB; //(0b10101101); // digit3 symbol Symbol
         if (a_digit == 4) a_secment = 0xFF; // (0b11111111); // digit4 symbol turned off
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
				}
      }
     else if ((WD_activated == 1) & (wic_slack == 1) &
             (((encoder_errors == 0) & (run_prog == 0) && (hide_timer == 0)) | (run_prog == 1))) // 19-11-2018 11-12-2018 98-06-2022
      {
			 if (gdv1 == 1)
			  {
				 ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 202; // wicket door activated text
			  }
			 else	
			  {	
         if (a_digit == 1) a_secment = 0xFF; // (0b11111111); // digit1 symbol turned off
         if (a_digit == 2) a_secment = 0xFF; // (0b11111111); // digit2 symbol turned off  
         if (a_digit == 3) a_secment = 0xC6; // (0b11000110); // digit3 symbol Symbol
         if (a_digit == 4) a_secment = 0xF0; // (0b11110000); // digit4 symbol Symbol
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
				}
      }	
		 else if ((stop_safety_chain == 1) && ((hide_timer == 0) | (run_prog == 1))) // 15-11-2016 08-06-2022
      {
			 if (gdv1 == 1)
			  {	
				 ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 46; //STOPPED Safety chain
        }
       else
			  {
         if (a_digit == 1) a_secment = 0x9E; // (0b10011110); // digit1 symbol safety chain 21-11-2023
         if (a_digit == 2) a_secment = 0xBE; // (0b10111110); // digit2 symbol   
         if (a_digit == 3) a_secment = 0xBE; // (0b10111110); // digit3 symbol 
         if (a_digit == 4) a_secment = 0xBC; // (0b10111100); // digit4 symbol 
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
        }				 
			}
		 else if (sce_bat_state == 2) // 05-01-2021
		  {
			 if (gdv1 == 1)
			  {	
				 ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 104; //SCE encoder battery fail - Replace battery - Check limits
				}
       else
		    {
         if (a_digit == 1) a_secment = 0x86; // (0b10000110); // digit1 symbol E
         if (a_digit == 2) a_secment = 0xFF; // (0b11111111); // digit2 symbol O
         if (a_digit == 3) a_secment = 0x80; // (0b10000000); // digit3 symbol B
         if (a_digit == 4) a_secment = 0x88; // (0b10001000); // digit4 symbol A
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
        }		
		  }
		 else if (sce_bat_state == 3) // 05-01-2021
			{
			 if (gdv1 == 1)
			  {	
			   ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 105; //SCE Encoder reset - Replace battery - Relearn limits 	
				}
			 else
        {
         if (a_digit == 1) a_secment = 0x86; // (0b10000110); // digit1 symbol E
         if (a_digit == 2) a_secment = 0xFF; // (0b11111111); // digit2 symbol O
         if (a_digit == 3) a_secment = 0x80; // (0b10000000); // digit3 symbol B
         if (a_digit == 4) a_secment = 0x88; // (0b10001000); // digit4 symbol A
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
        }
		  }	
	   else if ((err_edge == 1) & (run_prog == 0) & (hide_timer == 0)) // 11-02-2011 + 25-02-2011
      {
       if (edge_err_show > 450)
        {
         d_flash_tim = 1; // stop flash when ERR shown
				 if (gdv1 == 1)
			    {	
				   ds_byte[0] = 0;  //Run mode display
	         ds_byte[1] = 53; //SETUP ERROR
          }
         else
				  {
           if (a_digit == 1) a_secment = 0x86; // (0b10000110); // digit1 = E
           if (a_digit == 2) a_secment = 0xCE; // (0b11001110); // digit2 = r
           if (a_digit == 3) a_secment = 0xCE; // (0b11001110); // digit3 = r
           if (a_digit == 4) a_secment = 0xFF; // (0b11111111); // digit4 = off
           if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon = off
          }					 
        }
       else
        {
         if (edge_err_show == 0) edge_err_show = 900; // 3 sec. period
				 d_flash_tim = 1; // stop flash when ERR shown
				 if (gdv1 == 1)
			    {	
				   ds_byte[0] = 0;  //Run mode display
	         ds_byte[1] = 54; // Edge symbol + Safete Edge		
					}
				 else
				  {
           if (a_digit == 1) a_secment = 0xF7; // (0b11110111); // digit1 symbol edge symbol
           if (a_digit == 2) a_secment = 0xF7; // (0b11110111); // digit2 symbol 
           if (a_digit == 3) a_secment = 0xF7; // (0b11110111); // digit3 symbol
           if (a_digit == 4) a_secment = 0xF7; // (0b11110111); // digit4 symbol
           if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
				  }					 
			  } 
      }
		 else if (ecode1 == 1) // ecodex priority corrected
		  {
       if (new_calc == 0) enc_ecode1(); // //hvis der er ved at gemmes i EEPROM skal ikke vises fejl
			}	
		 else if ((ecode2 == 1) && (limit_indi == 0)) enc_ecode2(); // 13-03-2017		
		 else if (ecode4 == 1) enc_ecode4(); //
		 else if (ecode7 == 1) enc_ecode7(); //
		 else if (ecode8 == 1) enc_ecode8(); //	
		 else if (ecode9 == 1) enc_ecode9(); // Encoder error code shown on display. EE_par_11 can not be 0 in this prog.11-11-2016		
		 //	
		 else if ((open_pb == 1) | (open_disp_pb == 1)) // 15-11-2010  
      {
			 if (gdv1 == 1)
		    {	
	  		 ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 15; // Open push symbol
				}
       else
			  {
         if (a_digit == 1) a_secment = 0xFF; // (0b11111111); // digit1 symbol
         if (a_digit == 2) a_secment = 0xDC; // (0b11011100); // digit2 symbol 
         if (a_digit == 3) a_secment = 0xDC; // (0b11011100); // digit3 symbol
         if (a_digit == 4) a_secment = 0xFF; // (0b11111111); // digit4 symbol
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
        }				 
      }
     else if ((photo1 == 1) && (hide_timer == 0) && (cls == 0) && (photo1_dis == 0)) // 15-11-2016
      {
			 if (gdv1 == 1)
		    {	
			   ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 18; // Photo1 symbol
        }
       else
			  {
				 if (a_digit == 1) a_secment = 0x8F; // (0b10001111); // digit1 symbol
         if (a_digit == 2) a_secment = 0xB9; // (0b10111001); // digit2 symbol 
         if (a_digit == 3) a_secment = 0xFF; // (0b11111111); // digit3 symbol
         if (a_digit == 4) a_secment = 0xF9; // (0b11111001); // digit4 = 1
         if (a_digit == 5) a_secment = 0x7F; // (0b01111111); // colon symbol
			  }
			 
			} 
     else if ((relearn_ph1 == 1) & (d_flash_tim == 0) & (cls == 0) & (limit_indi == 0)) // 06-10-2010 
      {
			 if (gdv1 == 1)
		    {		
				 ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 18; // Photo1 symbol	- can flash here
        }
       else
			  {
         if (a_digit == 1) a_secment = 0x8F; // (0b10001111); // digit1 symbol
         if (a_digit == 2) a_secment = 0xB9; // (0b10111001); // digit2 symbol 
         if (a_digit == 3) a_secment = 0xFF; // (0b11111111); // digit3 symbol
         if (a_digit == 4) a_secment = 0xF9; // (0b11111001); // digit4 = 1
         if (a_digit == 5) a_secment = 0x7F; // (0b01111111); // colon symbol
			  }				 
      } 
     else if ((photo2 == 1) && (hide_timer == 0) && (cls == 0) && (photo2_dis == 0)) // 15-11-2016
      {
			 if (gdv1 == 1)
		    {	
			   ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 19; // Photo2 symbol
				}	
       else
			  {
         if (a_digit == 1) a_secment = 0x8F; // (0b10001111); // digit1 symbol
         if (a_digit == 2) a_secment = 0xB9; // (0b10111001); // digit2 symbol 
         if (a_digit == 3) a_secment = 0xFF; // (0b11111111); // digit3 symbol
         if (a_digit == 4) a_secment = 0xA4; // (0b10100100); // digit4 = 2
         if (a_digit == 5) a_secment = 0x7F; // (0b01111111); // colon symbol
        }				 
      } 
     else if ((relearn_ph2 == 1) & (d_flash_tim == 0) & (cls == 0) & (limit_indi == 0)) // 06-10-2010
      {
			 if (gdv1 == 1)
		    {	
			   ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 19; // Photo2 symbol
        }
       else
			  {
         if (a_digit == 1) a_secment = 0x8F; // (0b10001111); // digit1 symbol
         if (a_digit == 2) a_secment = 0xB9; // (0b10111001); // digit2 symbol 
         if (a_digit == 3) a_secment = 0xFF; // (0b11111111); // digit3 symbol
         if (a_digit == 4) a_secment = 0xAF; // (0b10100100); // digit4 = 2
         if (a_digit == 5) a_secment = 0x7F; // (0b01111111); // colon symbol
        }				 
      } 
		 else if (e1_mon == 1) e1(); // Error code showing moved to higher priority 09-12-2025	
     else if (e9_pos_change == 1) e9(); // Error code showing 22-03-2017 moved in priority so it can be shown on limits
     else if ((cls == 1) && (hide_timer == 0)) // 03-04-2024
      {
			 if ((sce_bat_state == 1) && (run_prog == 0) & (hide_timer == 0))	//05-01-2021
			  {
			   if (edge_err_show > 450) // using this timer again must be ok
          {
           d_flash_tim = 1; // stop flash when ERR shown
					 if (gdv1 == 1)
		        {	
				     ds_byte[0] = 0;  //Run mode display
	           ds_byte[1] = 103; // Encoder battery low - Replace battery - Check limits
            }
           else
					  {
             if (a_digit == 1) a_secment = 0x86; // 0x86; // (0b10000110); // digit1 symbol E
             if (a_digit == 2) a_secment = 0xFF; // 0xFF; // (0b11111111); // digit2 symbol O
             if (a_digit == 3) a_secment = 0x80; // 0x80; // (0b10000000); // digit3 symbol B
             if (a_digit == 4) a_secment = 0x88; // 0x88; // (0b10001000); // digit4 symbol A
             if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
            }						 
          }
         else
          {
           if (edge_err_show == 0) edge_err_show = 900; // 3 sec. period
				   d_flash_tim = 1; // stop flash when ERR shown
					 if (gdv1 == 1)
		        {	
				     ds_byte[0] = 0;  //Run mode display
	           ds_byte[1] = 12; // normal close limit symbol
						}
           else
					  {
             if (a_digit == 1) a_secment = 0xC7; // (0b11000111); // digit1 symbol
             if (a_digit == 2) a_secment = 0xF7; // (0b11110111); // digit2 symbol 
             if (a_digit == 3) a_secment = 0xF7; // (0b11110111); // digit3 symbol
             if (a_digit == 4) a_secment = 0xF1; // (0b11110001); // digit4 symbol
             if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
            }						 
			    } 
			  }	
       else	
			  {
         if (gdv1 == 1)
		      {
		       ds_byte[0] = 0;  //Run mode display
	         ds_byte[1] = 12; // Close limit symbol	
          }
				 else
					{
           if (a_digit == 1) a_secment = 0xC7; // (0b11000111); // digit1 symbol
           if (a_digit == 2) a_secment = 0xF7; // (0b11110111); // digit2 symbol 
           if (a_digit == 3) a_secment = 0xF7; // (0b11110111); // digit3 symbol
           if (a_digit == 4) a_secment = 0xF1; // (0b11110001); // digit4 symbol
           if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
          }						 
				}				
			}
     else if (((edgetype == 3)&&(fraba == 1)) | ((edgetype != 3) && (dw8k2 == 1)) 
		 && ((hide_timer == 0) && (cls == 0))) 
      {
			 if (gdv1 == 1)
		    {	
				 ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 20; // Safety edge symbol
        }
       else
			  {
         if (a_digit == 1) a_secment = 0xF7; // (0b11110111); // digit1 symbol
         if (a_digit == 2) a_secment = 0xF7; // (0b11110111); // digit2 symbol 
         if (a_digit == 3) a_secment = 0xF7; // (0b11110111); // digit3 symbol
         if (a_digit == 4) a_secment = 0xF7; // (0b11110111); // digit4 symbol
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
        }				 
			}  
     else if (((close_pb) || (close_disp_pb)) && (e5_photo == 0) && (e6_edge == 0) && (e22_stop == 0) && (e23_chain == 0) && (e10_edge == 0)) // 21-10-2010 18-11-2016 10-11-2021
      {
			 if (gdv1 == 1)
		    {	
				 ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 16; // Close push button symbol
        }
       else
			  {
         if (a_digit == 1) a_secment = 0xFF; // (0b11111111); // digit1 symbol
         if (a_digit == 2) a_secment = 0xE3; // (0b11100011); // digit2 symbol 
         if (a_digit == 3) a_secment = 0xE3; // (0b11100011); // digit3 symbol
         if (a_digit == 4) a_secment = 0xFF; // (0b11111111); // digit4 symbol
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
        }				 
      }
     else if ((kip_pb == 1) && (e5_photo == 0) && (e6_edge == 0) && (e22_stop == 0) && (e23_chain == 0) && (e10_edge == 0)) // 21-10-2010 18-11-2016 10-11-2021
      {
       if (e9_pos_change == 1)
        {
         e9(); // Error code showing 04-06-2013
        }
       else
        {
				 if (gdv1 == 1)
		      {		
				   ds_byte[0] = 0;  //Run mode display
	         ds_byte[1] = 17; // Go funktion Push symbol
          }	
         else
				  {
           if (a_digit == 1) a_secment = 0xE3; // (0b11100011); // digit1 symbol
           if (a_digit == 2) a_secment = 0xDC; // (0b11011100); // digit2 symbol 
           if (a_digit == 3) a_secment = 0xE3; // (0b11100011); // digit3 symbol
           if (a_digit == 4) a_secment = 0xDC; // (0b11011100); // digit4 symbol
           if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
          }					 
        } 
      }
     else if (((h_ols == 1) && (temp < 2)) && (e5_photo == 0) && (e6_edge == 0) && (e22_stop == 0) && (e23_chain == 0) && (e10_edge == 0)) // 21-10-2010 18-11-2016 10-11-2021
      {
       read_int16_eeprom(EE_par_32); // 11-02-2011
       temp2 = data;
       EE_read(EE_par_17);
       if ((temp2 > 0) && (temp > 0) && (hide_timer == 0)) display_auto(); //
       else
        {
				 if (gdv1 == 1)
		      {	
				   ds_byte[0] = 0;  //Run mode display
	         ds_byte[1] = 13; // Half OLS active symbol
					}
				 else
				  {
           if (a_digit == 1) a_secment = 0xAF; // (0b10101111); // digit1 symbol
           if (a_digit == 2) a_secment = 0xBF; // (0b10111111); // digit2 symbol 
           if (a_digit == 3) a_secment = 0xBF; // (0b10111111); // digit3 symbol
           if (a_digit == 4) a_secment = 0xBB; // (0b10111011); // digit4 symbol
           if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
          }					 
				}
      }
     else if (((temp > 1) || (temp4 == 54)) && (door_pos_half == 1)) // 08-06-2021
      {
       read_int16_eeprom(EE_par_32); // 
       temp2 = data;
       EE_read(EE_par_17);
       temp3 = temp; // move to another reg. for test purpose later
       if ((temp2 > 0) && (temp3 > 0) && (hide_timer == 0)) display_auto(); //
       else
        {
				 if (gdv1 == 1)
          {	
           ds_byte[0] = 0;  //Run mode display
	         ds_byte[1] = 13; // Half OLS active symbol	
          }           
				 else
				  {					 
           if (a_digit == 1) a_secment = 0xAF; // (0b10101111); // digit1 symbol
           if (a_digit == 2) a_secment = 0xBF; // (0b10111111); // digit2 symbol 
           if (a_digit == 3) a_secment = 0xBF; // (0b10111111); // digit3 symbol
           if (a_digit == 4) a_secment = 0xBB; // (0b10111011); // digit4 symbol
           if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
				  }
        }
      }
     else if (((ols == 1) && (e5_photo == 0) && (e6_edge == 0) && (e22_stop == 0) && (e23_chain == 0) && (e10_edge == 0)) 
			 || ((ols == 1) && ((safety_test == 1) || (safety_after_time > 0))))// 21-10-2010 18-11-2016 10-11-2021 30-10-2023
      {
       read_int16_eeprom(EE_par_32); // 11-02-2011
       temp2 = data;
       EE_read(EE_par_17);
       temp3 = temp; // move to another reg. for test purpose later
       if ((temp2 > 0) && (hide_timer == 0)) display_auto(); //
       else
        {
				 if (gdv1 == 1)
				 {					 
				  ds_byte[0] = 0;  //Run mode display
	        ds_byte[1] = 11; // OLS active symbol
				 }
        else
				 {
          if (a_digit == 1) a_secment = 0xCE; // (0b11001110); // digit1 symbol
          if (a_digit == 2) a_secment = 0xFE; // (0b11111110); // digit2 symbol 
          if (a_digit == 3) a_secment = 0xFE; // (0b11111110); // digit3 symbol
          if (a_digit == 4) a_secment = 0xF8; // (0b11111000); // digit4 symbol
          if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
         }					
        }
      }
		 // else if (e1_mon == 1) e1(); // Error code showing moved to higher priority
     else if (e7_tacho == 1) e7(); // Error code showing
     else if (e2_speed == 1) e2(); // Error code showing
     else if (e3_run == 1) e3(); // Error code showing
     else if (e4_ser == 1) e4(); // Error code showing
     else if ((e5_photo == 1) && (hide_timer == 0)) e5(); // Error code showing 08-06-2022
		 else if ((e6_edge == 1) && (hide_timer == 0)) e6(); // Error code showing 08-06-2022
		 else if ((e10_edge == 1) && (hide_timer == 0)) e10(); // Error code showing 08-06-2022
     else if (e8_wear == 1) e8(); // Error code showing
     else if (e20_fail == 1) e20(); // Error code showing
     else if (e21_fail == 1) e21(); // Error code showing
		 else if ((e22_stop == 1) && (hide_timer == 0)) e22(); // Error code showing 08-06-2022
     else if ((e23_chain == 1) && (hide_timer == 0)) e23(); // Error code showing	08-06-2022
		 else if ((e24_wick == 1) && (hide_timer == 0)) e24(); // Error code showing	08-06-2022	
		 //
     else if (open_active() == 1) 
      {
			 if (gdv1 == 1)
			  {				 
			   ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 22; // Door running up symbol
			  }
       else
			  {
         if (a_digit == 1) a_secment = 0xDC; // (0b11011100); // digit1 symbol
         if (a_digit == 2) a_secment = 0xFF; // (0b11111111); // digit2 symbol 
         if (a_digit == 3) a_secment = 0xFF; // (0b11111111); // digit3 symbol
         if (a_digit == 4) a_secment = 0xDC; // (0b11011100); // digit4 symbol
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
        }				 
      }
     else if (close_active() == 1) 
      {
			 if (gdv1 == 1)
			  {		
			   ds_byte[0] = 0;  //Run mode display
	       ds_byte[1] = 23; // Door running down symbol
        }
       else
			  {
         if (a_digit == 1) a_secment = 0xE3; // (0b11100011); // digit1 symbol
         if (a_digit == 2) a_secment = 0xFF; // (0b11111111); // digit2 symbol 
         if (a_digit == 3) a_secment = 0xFF; // (0b11111111); // digit3 symbol
         if (a_digit == 4) a_secment = 0xE3; // (0b11100011); // digit4 symbol
         if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
			  }				 
      }
     else
      {
			 EE_read(EE_curtain_sw);
			 if (temp > 0) // shall special curtain text be shown
			  {
				 if (gdv1 == 1)
			    {	
           ds_byte[0] = 0;  //Run mode display
	         ds_byte[1] = 102; // special curtain text
          }
         else
				  {
           if (a_digit == 1) a_secment = 0xBF; // (0b10111111); // digit1 symbol horisontal line in middle
           if (a_digit == 2) a_secment = 0xBF; // (0b10111111); // digit2 symbol 
           if (a_digit == 3) a_secment = 0xBF; // (0b10111111); // digit3 symbol
           if (a_digit == 4) a_secment = 0xBF; // (0b10111111); // digit4 symbol
           if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
          }					 
			  } 				 
			 else if (hide_timer == 0)
        {
				 if (gdv1 == 1)
			    {	
				   ds_byte[0] = 0;  //Run mode display
	         ds_byte[1] = 10; // 4 chairs standby symbol
          }	
         else
				  {
           if (a_digit == 1) a_secment = 0xA9; // (0b10101001); // digit1 symbol chair shown
           if (a_digit == 2) a_secment = 0xA9; // (0b10101001); // digit2 symbol chair shown
           if (a_digit == 3) a_secment = 0xA9; // (0b10101001); // digit3 symbol chair shown
           if (a_digit == 4) a_secment = 0xA9; // (0b10101001); // digit4 symbol chair shown
           if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol
				  }					 
        }
       else // switch off symbols
        {
				 if (gdv1 == 1)
			    {	
				   ds_byte[0] = 0;  //Run mode display
	         ds_byte[1] = 40; // On watch if available
          }
         else
				  {
           if (a_digit == 1) a_secment = 0xFF; // (0b11111111); // digit1 symbol No watch is made yet - turn off digits
           if (a_digit == 2) a_secment = 0xFF; // (0b11111111); // digit2 symbol No watch is made yet - turn off digits
           if (a_digit == 3) a_secment = 0xFF; // (0b11111111); // digit3 symbol No watch is made yet - turn off digits
           if (a_digit == 4) a_secment = 0xFF; // (0b11111111); // digit4 symbol No watch is made yet - turn off digits
           if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // colon symbol No watch is made yet - turn off digits
          }					 
        }
      }
    }
  }
 }	
}
//
void send_to_display(void)
{
	
}
//
void receive_from_display()
{
	
}
//
void chk_para_tst()
{
 if (para_test == 1)
  {
   new_EE = 1; // ready for saving i parameter switch routine
   value_changed = 0;
  }
 else
  {
   para_test = 1;
   par_val = 0;
  } 	
}
//
void req_val_change(uint8_t min, uint8_t max) // 
{
 static uint8_t open_pb_tb; // testbit for open push button to control incrementing parametervalue
 static uint8_t close_pb_tb; // testbit for open push button to control decrementing parametervalue
 if (temp < min) temp = min;
 if (temp > max) temp = max;
 if ((open_pb == 1) | (open_disp_pb == 1)) // is open push button active
      {
       value_changed = 1; 
       if (push_hold_tim == 0)
        {
         if (open_pb_tb == 0)
          {
          }
         else
          {
           open_pb_tb = 0;
           push_hold_tim = 30; // 100mS
          }
        }
       else
        {
         if (open_pb_tb == 0)
          {
           if (temp < max) temp++; // is the value < max parameter value, if yes increment, else no inc.
           new_EE = 1; // ready for saving new value
           open_pb_tb = 1; // release before next increment
          } 
        }
      } 
     else
      {
       open_pb_tb = 0; // push button released 
      }
     if ((close_pb) || (close_disp_pb)) // is close push button active
      {
       value_changed = 1;
       if (push_hold_tim == 0)
        {
         if (close_pb_tb == 0)
          {
          }
         else
          {
           close_pb_tb = 0;
           push_hold_tim = 30;
          }
        }
       else
        {
         if (close_pb_tb == 0)
          {
           if (temp > min ) temp--; // is the value > min parameter value, if yes decrement, else no dec.
            new_EE = 1; // ready for saving new value
            close_pb_tb = 1; // release before next increment
          }
        }
      }
     else
      {
       close_pb_tb = 0; // push button released 
      }
     if ((!close_pb) && (!close_disp_pb) && (!open_pb) && (!open_disp_pb)) push_hold_tim = 150; // preset to 0.5 sec 
     //
     return;	
}
//
//
void req_val_change_b16(uint16_t min, uint16_t max) // 16-09-2016
{
 static uint8_t open_pb_tb; // testbit for open push button to control incrementing parametervalue
 static uint8_t close_pb_tb; // testbit for open push button to control decrementing parametervalue
 if (temp2 < min) temp2 = min;
 if (temp2 > max) temp2 = max;	
 if ((open_pb == 1) | (open_disp_pb == 1)) // is open push button active
      {
       value_changed = 1; 
       if (push_hold_tim == 0)
        {
         if (open_pb_tb == 0)
          {
          }
         else
          {
           open_pb_tb = 0;
           push_hold_tim = 30; // 100mS
          }
        }
       else
        {
         if (open_pb_tb == 0)
          {
					 if ((ds_byte[1] == 101) || (ds_byte[1] == 103)) 
					  {
             if (temp2 < max) temp2 = temp2 + 10;
						 if (ds_byte[1] == 103)	
						  {
							 if ((temp2 > 1800) && (temp2 < 2000)) temp2 = 2000; // values not exist
               if ((temp2 > 3590) && (temp2 < 3990)) temp2 = 3990; // values not exist								
							}
						}	
           else if ((ds_byte[1] == 104) || (ds_byte[1] == 109) || (ds_byte[1] == 110) || (ds_byte[1] == 111) || 
						 (ds_byte[1] == 120) || (ds_byte[1] == 121))
					  {
             if (temp2 < max) temp2 = temp2 + 100;
						}
           else if (temp2 < max) temp2++; // is the value < max parameter value, if yes increment, else no inc.
           new_EE = 1; // ready for saving new value
           open_pb_tb = 1; // release before next increment
          } 
        }
      } 
     else
      {
       open_pb_tb = 0; // push button released 
      }
     if ((close_pb) || (close_disp_pb)) // is close push button active
      {
       value_changed = 1;
       if (push_hold_tim == 0)
        {
         if (close_pb_tb == 0)
          {
          }
         else
          {
           close_pb_tb = 0;
           push_hold_tim = 30;
          }
        }
       else
        {
         if (close_pb_tb == 0)
          {
					 if ((ds_byte[1] == 101) || (ds_byte[1] == 103))  
					  {
             if (temp2 > min) temp2 = temp2 - 10;
						 if (ds_byte[1] == 103)	
						  {
							 if ((temp2 < 2000) && (temp2 > 1800)) temp2 = 1800; // values not exist
               if ((temp2 < 3990) && (temp2 > 3590)) temp2 = 3590; // values not exist								
							}	
						}	
					 else if ((ds_byte[1] == 104) || (ds_byte[1] == 109) || (ds_byte[1] == 110) || (ds_byte[1] == 111) || 
						 (ds_byte[1] == 120) ||(ds_byte[1] == 121))
					  {
             if (temp2 > min) temp2 = temp2 - 100;
						}	
           else if (temp2 > min ) temp2--; // is the value > min parameter value, if yes decrement, else no dec.
            new_EE = 1; // ready for saving new value
            close_pb_tb = 1; // release before next increment
          }
        }
      }
     else
      {
       close_pb_tb = 0; // push button released 
      }
     if ((!close_pb) && (!close_disp_pb) && (!open_pb) && (!open_disp_pb)) push_hold_tim = 150; // preset to 0.5 sec 
     //
     return;	
}
//
//
void show_run()
{
 if (gdv1 == 1)
  {
   ds_byte[0] = 1;    // Prog mode active
   temp2 = 1012;
   ds_byte[1] = temp2;    // show RUN in display unit by 1012
   ds_byte[2] = temp2>>8;
   ds_byte[4] = 255;  //  show no value
  }
 else
  {
   if (a_digit == 1) a_secment = 0xCE; // (0b11001110) digit1 = R
   if (a_digit == 2) a_secment = 0xC1; // (0b11000001) digit2 = U
   if (a_digit == 3) a_secment = 0xC8; // (0b11001000) digit3 = N
   if (a_digit == 4) a_secment = 0xFF; // (0b11111111) digit4 = OFF
  }	 
}
//
void reload_ser_cnt()
{
 p_value = 0; // switch to parameter number
 par_val = 0;
 show_clr_tim = 600; // reload show clear timer with 2 sec.
 EE_read(EE_par_58_new);
 EE_write(EE_par_58, temp);
 e4_ser = 0; 
 adr = EE_ser_count; // 19-05-2010
 switch (temp)
  {
   case 1:
   data = 985; // 15 by debug test, else 985 19-05-2010
   write_int16_eeprom();
   break;
   case 2:
   data = 4923; // subtract 1.54% because only count_b1 is used for downcount 19-05-2010
   write_int16_eeprom();
   break;
   case 3:
   data = 9846; // subtract 1.54% because only count_b1 is used for downcount 19-05-2010
   write_int16_eeprom();
   break;
   case 4:
   data = 19692; // subtract 1.54% because only count_b1 is used for downcount 19-05-2010
   write_int16_eeprom();
   break;
   default:
   break;
  }
}//
//************************************************************************
void e1()
{
 if (gdv1 == 1)
	{
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 60; // Edge fail code
  }
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
   if (a_digit == 4) a_secment = 0xF9; // (0b11111001) digit4  = 1
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
 return;
}
//************************************************************************
void e2()
{
 if (gdv1 == 1)
  {	 
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 61; // Force control code
  }
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
   if (a_digit == 4) a_secment = 0xA4; // (0b10100100) digit4 = 2
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
 return;	
}
//************************************************************************
void e3()
{
 if (gdv1 == 1)
  {	 
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 62; // run time code	
  }
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
   if (a_digit == 4) a_secment = 0xB0; // (0b10110000) digit4 = 3
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
 return;	
}
//************************************************************************
void e4()
{
 if (gdv1 == 1)
  {	 
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 63; // Service counter expired code
  }
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
   if (a_digit == 4) a_secment = 0x99; // (0b10011001) digit4 = 4
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
return;
}
//************************************************************************
void e5()
{
 if (gdv1 == 1)
  {	 
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 64; // Photo circuit fail code
  }
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
   if (a_digit == 4) a_secment = 0x92; // (0b10010010) digit4 = 5
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
 return;
}
//************************************************************************
void e6()
{
 if (gdv1 == 1)
  {	 
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 65; // Safety edge code
  }
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
   if (a_digit == 4) a_secment = 0x82; // (0b10000010) digit4 = 6
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
 return;
}
//************************************************************************
void e7()
{
 if (gdv1 == 1)
  {	 
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 66; // Tacho failure code
  }
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
   if (a_digit == 4) a_secment = 0xF8; // (0b11111000) digit4 = 7
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
 return;
}
//************************************************************************
void e8()
{
 if (gdv1 == 1)
  {	 
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 67; // Wear, force control code
  }
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
   if (a_digit == 4) a_secment = 0x80; // (0b10000000) digit4 = 8
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
 return;
}
//************************************************************************
void e9()
{
 if (gdv1 == 1)
  {	 
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 68; // No position change code
  }
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
   if (a_digit == 4) a_secment = 0x90; // (0b10010000) digit4 = 9
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) // colon symbol
  }	 
 return;
}
//************************************************************************
void e10()
{
 if (gdv1 == 1)
  {	 
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 74; // edge 2/wicket door fail
  }
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xF9; // (0b11111001) digit3 = 1
   if (a_digit == 4) a_secment = 0xC0; // (0b11000000) digit4 = 0
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
 return;
}
//************************************************************************
void e20()
{
 if (gdv1 == 1)
  {	 
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 69; // EEPROM fail code
  }
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xA4; // (0b10100100) digit3 = 2
   if (a_digit == 4) a_secment = 0xC0; // (0b11000000) digit4 = 0
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
 return;
}
//************************************************************************
void e21()
{
 if (gdv1 == 1)
  {		
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 70; // EEPROM powerup fail code
  }
 else	
  {
	 if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xA4; // (0b10100100) digit3 = 2
   if (a_digit == 3) a_secment = 0xF9; // (0b11111001) digit3 = 1
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol	
  }
}
//************************************************************************
void e22()
{
 if (gdv1 == 1)
  {	
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 72; // Stop circuit fail
	}
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xA4; // (0b10100100) digit3 = 2
   if (a_digit == 3) a_secment = 0xA4; // (0b10100100) digit3 = 2
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol		
  }	 
}
//************************************************************************
void e23()
{
 if (gdv1 == 1)
  {	
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 73; // Safety chain circuit fail (2 linier)		
	}
 else
	{
	 if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xA4; // (0b10100100) digit3 = 2
   if (a_digit == 4) a_secment = 0xB0; // (0b10110000) digit4 = 3
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol		
	}
}
//************************************************************************
void e24()
{
 if (gdv1 == 1)
  {	
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 74; // E:24 – Wicket/slack c. fail (2 linier)		
	}
 else
	{
	 if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xA4; // (0b10100100) digit3 = 2
   if (a_digit == 4) a_secment = 0x99; // (0b10011001) digit4 = 4
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol		
	}
}

//************************************************************************
void e27()
{
 if (gdv1 == 1)
  {	
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 77; //Modbus timeout check connection and make a new powerup
	}
 else
	{
	 if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xA4; // (0b10100100) digit3 = 2
   if (a_digit == 4) a_secment = 0xF8; // (0b11111000) digit4 = 7
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol	
	}
}
//************************************************************************
void e28()
{	
 if (gdv1 == 1)
  {	
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 78; //program hard fault interrupt occured	 	
	}
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xA4; // (0b10100100) digit3 = 2
   if (a_digit == 4) a_secment = 0x80; // (0b10000000) digit4 = 8
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
}
//************************************************************************
void e29()
{	
 if (gdv1 == 1)
  {	
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 79; //Processor main clock failed
  }
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 
   if (a_digit == 3) a_secment = 0xA4; // (0b10100100) digit3 = 2
   if (a_digit == 4) a_secment = 0x90; // (0b10010000) digit4 = 9
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
}
//************************************************************************
void e30() // not in use anymore I think 19-11-2024 replaced by e31 for inverter failure in 2021
{
 if (gdv1 == 1)
  {	
   ds_byte[0] = 0;     // Run mode active
   ds_byte[1] = 130;   // show parameter 104
   ds_byte[2] = 60;    // hit the default for no number showing 
   ds_byte[3] = 0;     // drive ratio not in use yetvalue
   ds_byte[4] = 0;	    // not in use		
  }
 else
  {
	 if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 	
   if (a_digit == 3) a_secment = 0xB0; // (0b10110000) digit4 = 3
   if (a_digit == 4) a_secment = 0xC0; // (0b11000000) digit4 = 0
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
}
//************************************************************************
void e32() // Phase monitoring fail = Contactor fail shall be shown 12-01-2022
 {
	if (gdv1 == 1)
	 {		 
	  ds_byte[0] = 0;  //Run mode display
	 ds_byte[1] = 95; //E:32 Contactor Fail - Try new power-up - between door limits	
	 }
	else
	 {
		if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
    if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 	
    if (a_digit == 3) a_secment = 0xB0; // (0b10110000) digit4 = 3
    if (a_digit == 4) a_secment = 0xA4; // (0b10100100) digit4 = 2
    if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
	 }
 }
//************************************************************************
void e33() // Phase monitoring fail = Contactor fail shall be shown 04-03-2026
 {
	if (gdv1 == 1)
	 {		 
	  ds_byte[0] = 0;  //Run mode display
	 ds_byte[1] = 96; //E:33 unlock fail after activating relay output for this	
	 }
	else
	 {
		if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
    if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 	
    if (a_digit == 3) a_secment = 0xB0; // (0b10110000) digit3 = 3
    if (a_digit == 4) a_secment = 0xB0; // (0b10110000) digit4 = 3
    if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
	 }
 } 
//************************************************************************ 
void e50() // used for a little turn off time 30-10-2023
{
	if (gdv1 == 1)
  {	
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 40; //turn off display	 	
	}
 else
  {
   if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol off
   if (a_digit == 2) a_secment = 0xFF; // (0b11111111) digit2 symbol off
   if (a_digit == 3) a_secment = 0xFF; // (0b11111111) digit3 symbol off
   if (a_digit == 4) a_secment = 0xFF; // (0b11111111) digit4 symbol off
   if (a_digit == 5) a_secment = 0xFF; // (0b11111111) colon symbol off
  }	 
} 
 
//**************************************************************************
void enc_ecode9()
{
 if (gdv1 == 1)
  {	
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 49; // EEPROM fail code	
  }
 else
  {
   if (a_digit == 1) a_secment = 0x86; // (0b10000110) digit1 symbol E
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol E
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
	 if (a_digit == 4) a_secment = 0x90; // (0b10010000) digit4 = 9
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
}
void enc_ecode8()
{
 if (gdv1 == 1)
  {	
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 48; // Kostal encoder power fail
  }
 else
  {
   if (a_digit == 1) a_secment = 0x86; // (0b10000110) digit1 symbol E
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol E
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
	 if (a_digit == 4) a_secment = 0x80; // (0b10000000) digit4 = 8
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol	
  }	 
}
void enc_ecode7()
{
 if (gdv1 == 1)
  {	
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 47; // Encoder position out of learned area		
	}
 else
  {
   if (a_digit == 1) a_secment = 0x86; // (0b10000110) digit1 symbol E
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol E
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
	 if (a_digit == 4) a_secment = 0xF8; // (0b11111000) digit4 = 7
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol	
  }	 
}
void enc_ecode4()
{
 if (gdv1 == 1)
  {	
	 ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 44; // limit calculating error. Check encoder direction		
	}
 else
  {
   if (a_digit == 1) a_secment = 0x86; // (0b10000110) digit1 symbol E
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol E
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
	 if (a_digit == 4) a_secment = 0x99; // (0b10011001) digit4 = 4
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol	
  }	 
}
//void enc_ecode3() // not in use
//{
// ds_byte[0] = 0;  //Run mode display
// ds_byte[1] = 43; // Motor running fail		
//}
void enc_ecode2()
{
 if (gdv1 == 1)
  {	
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 42; // Limits not learned
  }
 else
  {
   if (a_digit == 1) a_secment = 0x86; // (0b10000110) digit1 symbol E
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol E
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
	 if (a_digit == 4) a_secment = 0xA4; // (0b10100100) digit4 = 2
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol	
  }	 
}
void enc_ecode1()
{
 if (gdv1 == 1)
  {	 
   ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 41; // No answer from encoder	
   ds_byte[2] = 0; // other values shall be 0 if info_by_learn is active 13-03-2017
   ds_byte[3] = 0;	
   ds_byte[4] = 0;
   ds_byte[5] = 0;
   ds_byte[6] = 0;
   ds_byte[7] = 0;
  }
 else
  {
   if (a_digit == 1) a_secment = 0x86; // (0b10000110) digit1 symbol E
   if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol E
   if (a_digit == 3) a_secment = 0xC0; // (0b11000000) digit3 symbol
   if (a_digit == 4) a_secment = 0xF9; // (0b11111001) digit4  = 1
   if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
  }	 
}
//************************************************************************
void no_error()
{
if (gdv1 == 1) // 21-11-2023
 {	
  ds_byte[0] = 0;  //Run mode display
  ds_byte[1] = 71; // 			
  return;
 }
else
 {
  if (a_digit == 1) a_secment = 0xBF; // (0b10111111) digit1 symbol midt line // 21-11-2023
  if (a_digit == 2) a_secment = 0xBF; // (0b10111111) digit1 symbol midt line
  if (a_digit == 3) a_secment = 0xBF; // (0b10111111) digit1 symbol midt line
  if (a_digit == 4) a_secment = 0xBF; // (0b10111111) digit1 symbol midt line
  if (a_digit == 5) a_secment = 0xFF; // (0b11111111) colon symbol - No symbol
 }
} 
//************************************************************************
void display_auto()
{
 if (e9_pos_change == 1)
  {
   e9(); // Error code showing 04-06-2013
  }
 else
  {
   temp2 = autoclose_timer;
   temp2 = temp2/50; // time for this is about 50uS - Temp2 is now seconds
	 if (gdv1 == 1)
	  {		 
	   ds_byte[0] = 0;  //Run mode display
     ds_byte[1] = 100; // Auto close time		
	   ds_byte[2] = temp2>>8;    // count down value
     ds_byte[3] = temp2;
    }
   else
	  {
     if (a_digit == 4)  //
      {
       if (temp2 > 9)
        {
         while (temp2 > 9)
         temp2 = temp2 - 10;
        }
       if (temp2== 0) a_secment = 0xC0; // (0b11000000); //
       if (temp2== 1) a_secment = 0xF9; // (0b11111001); // 
       if (temp2== 2) a_secment = 0xA4; // (0b10100100); // 
       if (temp2== 3) a_secment = 0xB0; // (0b10110000); // 
       if (temp2== 4) a_secment = 0x99; // (0b10011001); // 
       if (temp2== 5) a_secment = 0x92; // (0b10010010); // 
       if (temp2== 6) a_secment = 0x82; // (0b10000010); // 
       if (temp2== 7) a_secment = 0xF8; // (0b11111000); // 
       if (temp2== 8) a_secment = 0x80; // (0b10000000); //
       if (temp2== 9) a_secment = 0x90; // (0b10010000); // 
      }
     if (a_digit == 3) //
      {
       temp2 = (temp2/10); // time for this is about 100uS
       if (temp2 > 9)
        {
         while (temp2 > 9)
         temp2 = temp2 - 10;
        }
			 if (temp2== 0) a_secment = 0xC0; // (0b11000000); //
       if (temp2== 1) a_secment = 0xF9; // (0b11111001); // 
       if (temp2== 2) a_secment = 0xA4; // (0b10100100); // 
       if (temp2== 3) a_secment = 0xB0; // (0b10110000); // 
       if (temp2== 4) a_secment = 0x99; // (0b10011001); // 
       if (temp2== 5) a_secment = 0x92; // (0b10010010); // 
       if (temp2== 6) a_secment = 0x82; // (0b10000010); // 
       if (temp2== 7) a_secment = 0xF8; // (0b11111000); // 
       if (temp2== 8) a_secment = 0x80; // (0b10000000); //
       if (temp2== 9) a_secment = 0x90; // (0b10010000); // 	
      }
     if (a_digit == 2) //
      {
       temp2 = (temp2/100); // time for this is about 100uS
			 if (temp2== 0) a_secment = 0xC0; // (0b11000000); //
       if (temp2== 1) a_secment = 0xF9; // (0b11111001); // 
       if (temp2== 2) a_secment = 0xA4; // (0b10100100); // 
       if (temp2== 3) a_secment = 0xB0; // (0b10110000); // 
       if (temp2== 4) a_secment = 0x99; // (0b10011001); // 
       if (temp2== 5) a_secment = 0x92; // (0b10010010); // 
       if (temp2== 6) a_secment = 0x82; // (0b10000010); // 
       if (temp2== 7) a_secment = 0xF8; // (0b11111000); // 
       if (temp2== 8) a_secment = 0x80; // (0b10000000); //
       if (temp2== 9) a_secment = 0x90; // (0b10010000); // 	
	    }
     if ((a_digit == 5) || (a_digit == 1)) a_secment = 0xFF; // (0b11111111); // colon and digit 1 off 
	  }		 
	}
}
void show_status()
{
 if ((stop_pb == 1) || (stop_lid_pb == 1) || (stop_disp_pb == 1))
 {
 if (next_status == 0)
  {
   next_status =1; // is stop pushbutton activated
   status_para++;
   if (status_para == 3) status_para = 1; // there is only 2 status parameters
  }
 }
else // stop push button not activated
 {
  next_status =0;
 }
switch (status_para)
 {
 case 1:
  count_status(); //
  break;
 case 2: 
  show_L_errors(); //
  break;
 default:
  break;  
 }
}
//
void count_status()
{
 uint8_t a;
 uint8_t b;
 uint8_t c;
 EE_read(EE_count_b1);
 a = temp;
 EE_read(EE_count_b2);
 b = temp;
 EE_read(EE_count_b3);
 c = temp;
 switch (gdv1)
 {
	 case 0:
    // E_counter = make32(0, c, b, a);
    E_counter = (a | (b << 8) | (c << 16)); // erstatning for make32	 
    count_1 = E_counter % 10; // separate in digits
    count_10 = (E_counter/10) % 10;
    count_100 = (E_counter/100) % 10;
    count_1000 = (E_counter/1000) % 10;
    count_10000 = (E_counter/10000) % 10 ;
    count_100000 = (E_counter/100000) % 10;
    count_1000000 = (E_counter/1000000) %10;
    if (count_msd == 0xFF) // show most significant 3 digits 03-04-2024
     {
      if (a_digit == 1) a_secment = 0xC6; // (0b11000110); //
      else if (a_digit == 2)
       {
        temp = count_100000;
        secment_value();
       }
      else if (a_digit == 3)
       {
        temp = count_10000;
        secment_value();
       }
      else if (a_digit == 4)
       {
        temp = count_1000;
        secment_value();
       }
      else if (a_digit == 5) a_secment = 0xFF; // (0b11111111); //
     }
    else // show least 3 significant digits
     {
      if (a_digit == 1) 
       {
        temp = count_100;
        secment_value();
       }
      else if (a_digit == 2)
       {
        temp = count_10;
        secment_value();
       }
      else if (a_digit == 3)
       {
        temp = count_1;
        secment_value();
       }
      else if (a_digit == 4) a_secment = 0xF0; // (0b11110000); //
      else if (a_digit == 5) a_secment = 0xFF; // (0b11111111); //
     }
    break;
   case 1:	 
    ds_byte[0] = 0;  //Run mode display
    ds_byte[1] = 101; // Door Cycle Counts
    ds_byte[2] = 0;   // no value is saved over 24 bit	so max i 16777216
    ds_byte[3] = c;   // value of door cycle counts
    ds_byte[4] = b;	 //
    ds_byte[5] = a;	 //	
    // E_counter = make32(0, c, b, a);  // old CCS function
    // erstatning for make men skal ikke bruges. E_counter = (a | (b << 8) | (c << 16));
	  break;
   default:
    break;		 
 }	 
}
//
void show_L_errors()
{
 // Denne rutine skal laves helt om da vi nemt kan vise fejlnummer og fejl kode i display på inverter.	
 if (error_counter == 0) // 14-04-2010
 {
	if (gdv1 == 1)
	 {		
	  ds_byte[0] = 0;  //Run mode display
    ds_byte[1] = 80; // No newer tekst
   }
  else
	 {
    if (a_digit == 1) a_secment = 0xFE; // (0b11111110); // upper symbol
    if (a_digit == 2) a_secment = 0xFE; // (0b11111110); // upper symbol 
    if (a_digit == 3) a_secment = 0xFE; // (0b11111110); // upper symbol
    if (a_digit == 4) a_secment = 0xFE; // (0b11111110); // upper symbol
    if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // no colon symbol
   }		
  EE_read(EE_error_1); // 15-11-2010
  if (((open_pb == 1) | (open_disp_pb == 1)) && (temp != 0) && (open_long_tim == 0)) // 15-11-2010
   {
    temp = 176;
    for (temp=176;temp<=185;temp++)
    EE_write(temp,0);
   } 
 }
else if (error_counter == 11) // 09-04-2010
 {
	if (gdv1 == 1)
	 { 
	  ds_byte[0] = 0;  //Run mode display
    ds_byte[1] = 91; // No older error
   }
  else
	 {
    if (a_digit == 1) a_secment = 0xF7; // (0b11110111); // lower symbol
    if (a_digit == 2) a_secment = 0xF7; // (0b11110111); // lower symbol 
    if (a_digit == 3) a_secment = 0xF7; // (0b11110111); // lower symbol
    if (a_digit == 4) a_secment = 0xF7; // (0b11110111); // lower symbol
    if (a_digit == 5) a_secment = 0xFF; // (0b11111111); // no colon symbol
   }		
 }
else 
 {
  // error_counter 1 -10 , adjustet later
 }
//
if ((open_pb == 1) | (open_disp_pb == 1))
 {
  if (push_timer == 0)
   {
    if (error_counter > 0) error_counter--; // 14-04-2010
    push_timer = 150; // changed to from 250 to 150 at new main cycle time
		hide_timer = 60; // turn off a litte time 30-10-2023 
   }
 }
else
 {
 if ((close_pb) || (close_disp_pb))
  {
   if (push_timer == 0)
    {
     error_counter++;
     if (error_counter > 11) error_counter = 11;
     push_timer = 150; // changed to from 250 to 150 at new main cycle time
		 hide_timer = 60; // turn off a litte time 30-10-2023	
    }
  }
 }
if ((error_counter == 11) || (error_counter == 0))// 09-04-2010
 {
  // do nothing, it´s already set by lines above
 }
else
 {
  switch (error_counter)
  {
   case 1:
   EE_read(EE_error_1);
   select_error(temp);
   break;
   case 2:
   EE_read(EE_error_2);
   select_error(temp);
   break;
   case 3:
   EE_read(EE_error_3);
   select_error(temp);
   break;
   case 4:
   EE_read(EE_error_4);
   select_error(temp);
   break;
   case 5:
   EE_read(EE_error_5);
   select_error(temp);
   break;
   case 6:
   EE_read(EE_error_6);
   select_error(temp);
   break;
   case 7:
   EE_read(EE_error_7); 
   select_error(temp);
   break;
   case 8:
   EE_read(EE_error_8); 
   select_error(temp);
   break;
   case 9:
   EE_read(EE_error_9); 
   select_error(temp);
   break;
   case 10:
   EE_read(EE_error_10);
   select_error(temp);
   break;
   case 11:
	 ds_byte[0] = 0;  //Run mode display
   ds_byte[1] = 91; // No older error 		
   break;
   default:
   break;
  }
}
}
void select_error(uint8_t temp)
{
 if (hide_timer > 0) temp = 50;	
 switch (temp)
  {
	case 0: 
  no_error();
  break;	
  case 1: 
  e1();
  break;
  case 2: 
  e2();
  break;
  case 3: 
  e3();
  break;
  case 4: 
  e4();
  break;
  case 5: 
  e5();
  break;
  case 6: 
  e6();
  break;
  case 7: // 
  e7();
  break;
  case 8: // 
  e8();
  break;
  case 9: // 
  e9();
  break;
	case 10: // 10-11-2021
  e10();
  break;
  case 20: // 
  e20();
  break;
  case 21: // 
  e21();
  break;
  case 22: // 
  e22();
  break;
  case 23: // 
  e23();
  break;
	case 25: // 
  enc_ecode7();
  break;
  case 27: // 
  e27();
  break;
  case 28: // 
  e28();
  break;
	case 29: // 
  e29();
  break;
	case 30: // 
  e30();
  break;
	case 50:
	e50();
  break;	
  default:
  break;
  }
}

void chk_inverter_status()
{
 status = getModbusState();
 if (status == HAL_BUSY)
	{
  }			 
 else
	{
	 switch (inv_com_state)
		{
		 case 0:
			status = getModbusParam(F0330_DRIVE_STATUS, &inv_run_status_1005); // get inverter parameter in non blocking mode
		  if (status != HAL_OK)
		   {
        // fejlmedelelse 
		    break; 
       }			
		  else
		   {
		    inv_com_state = 1;
				break; 
			 }	
     case 1:
		  inv_com_state = 0;
		  INV_status_temp = inv_run_status_1005; // 16 bit var update 24-06-2020
		  //INV_status_temp = 4; // debug simulate error 24-06-2020
      break;
     default:
      break;			 
	  } 
   
  }
	
 if (INV_status_temp > 0) // 24-06-2020
	{
	 if (bit_test(INV_status_temp,0) == 1) inv_fail_active = 0xB0;
	 if (bit_test(INV_status_temp,1) == 1) inv_fail_active = 0xB1;
   if (bit_test(INV_status_temp,2) == 1) inv_fail_active = 0xB2;
   if (bit_test(INV_status_temp,3) == 1) inv_fail_active = 0xB3;
   if (bit_test(INV_status_temp,4) == 1) inv_fail_active = 0xB4;
	 if (bit_test(INV_status_temp,5) == 1) inv_fail_active = 0xB5;
   if (bit_test(INV_status_temp,6) == 1) inv_fail_active = 0xB6;
   if (bit_test(INV_status_temp,7) == 1) inv_fail_active = 0xB7;
	 if (bit_test(INV_status_temp,8) == 1) inv_fail_active = 0xB8;
   if (bit_test(INV_status_temp,9) == 1) inv_fail_active = 0xB9;
   if (bit_test(INV_status_temp,10) == 1) inv_fail_active = 0xBA; // bit 10 = 1
   if (bit_test(INV_status_temp,11) == 1) inv_fail_active = 0xBB;
	 if (bit_test(INV_status_temp,12) == 1) inv_fail_active = 0xBC;
   if (bit_test(INV_status_temp,13) == 1) inv_fail_active = 0xBD;
   if (bit_test(INV_status_temp,14) == 1) inv_fail_active = 0xBE;
	 if (bit_test(INV_status_temp,15) == 1) inv_fail_active = 0xBF;
   //	
	 e31_inverter = 1; // for last error update only
   if (gdv1 == 1) //19-11-2024
	  { 
		 ds_byte[0] = 0;    // Run mode active
	   ds_byte[1] = 130;   // show parameter 130
	   ds_byte[2] = inv_fail_active;    // value 
	   ds_byte[3] = 0;                  // value
     ds_byte[4] = 0;                  // value	
	  }
	 else
	  {
     if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 symbol
     if (a_digit == 2) a_secment = 0x86; // (0b10000110) digit2 symbol 	
     if (a_digit == 3) a_secment = 0xB0; // (0b10110000) digit4 = 3
     if (a_digit == 4) a_secment = 0xF9; // (0b11111001) digit4 = 1
     if (a_digit == 5) a_secment = 0x7F; // (0b01111111) colon symbol
	  }		 
	}
 else
  {
   e31_inverter = 0; // for last error update only 	
	 inv_fail_active = 0; // 25-11-2024
  }
}
//************************************************************************
void show_led_p_num() // 16-12-2021
{
 if (timer_p == 0) timer_p = 500; // toggle between hundreds and below hundreds
 else timer_p--;	
 if (parameter > 99)
  {
   if (timer_p > 300)
	  {
		 if (a_digit == 1) a_secment = 0xFF; // (0b11111111) digit1 = -
     if (a_digit == 2) a_secment = 0xF9; // (0b11111001) digit2 = 1 
	  }
	 else
	  {
		 temp = parameter - 100;
		 if (a_digit == 1) calc_a_secment_p();
		 if (a_digit == 2) calc_a_secment_p();	
	  }
  }
 else
  {
   if (a_digit == 1) calc_a_secment_p();
	 if (a_digit == 2) calc_a_secment_p();
  }	 
}
//************************************************************************
void calc_a_secment_p() //calculate a_secment value for parameter > 100 
{
 if (a_digit == 2)  //
  {
   if (temp > 9)
    {
     while (temp > 9)
     temp = temp - 10;
    }
   if (temp == 0) a_secment = 0xC0; // (0b11000000) 
   if (temp == 1) a_secment = 0xF9; // (0b11111001) 
   if (temp == 2) a_secment = 0xA4; // (0b10100100) 
   if (temp == 3) a_secment = 0xB0; // (0b10110000) 
   if (temp == 4) a_secment = 0x99; // (0b10011001) 
   if (temp == 5) a_secment = 0x92; // (0b10010010) 
   if (temp == 6) a_secment = 0x82; // (0b10000010) 
   if (temp == 7) a_secment = 0xF8; // (0b11111000) 
   if (temp == 8) a_secment = 0x80; // (0b10000000)
   if (temp == 9) a_secment = 0x90; // (0b10010000) 
  }
 if (a_digit == 1) //
  {
   temp = (temp/10);
    if (temp == 0) a_secment = 0xC0; // (0b11000000)
    if (temp == 1) a_secment = 0xF9; // (0b11111001) 
    if (temp == 2) a_secment = 0xA4; // (0b10100100) 
    if (temp == 3) a_secment = 0xB0; // (0b10110000)
		if (temp == 4) a_secment = 0x99; // (0b10011001) 
    if (temp == 5) a_secment = 0x92; // (0b10010010) 
    if (temp == 6) a_secment = 0x82; // (0b10000010) 
    if (temp == 7) a_secment = 0xF8; // (0b11111000) 
    if (temp == 8) a_secment = 0x80; // (0b10000000)
    if (temp == 9) a_secment = 0x90; // (0b10010000) 
  }
}
//************************************************************************
void calc_a_secment() //calculate a_secment value for temp < 100 
{
 if (a_digit == 4)  //
  {
   if (temp > 9)
    {
     while (temp > 9)
     temp = temp - 10;
    }
   if (temp == 0) a_secment = 0xC0; // (0b11000000) 
   if (temp == 1) a_secment = 0xF9; // (0b11111001) 
   if (temp == 2) a_secment = 0xA4; // (0b10100100) 
   if (temp == 3) a_secment = 0xB0; // (0b10110000) 
   if (temp == 4) a_secment = 0x99; // (0b10011001) 
   if (temp == 5) a_secment = 0x92; // (0b10010010) 
   if (temp == 6) a_secment = 0x82; // (0b10000010) 
   if (temp == 7) a_secment = 0xF8; // (0b11111000) 
   if (temp == 8) a_secment = 0x80; // (0b10000000)
   if (temp == 9) a_secment = 0x90; // (0b10010000) 
  }
 if (a_digit == 3) //
  {
   temp = (temp/10);
    if (temp == 0) a_secment = 0xC0; // (0b11000000)
    if (temp == 1) a_secment = 0xF9; // (0b11111001) 
    if (temp == 2) a_secment = 0xA4; // (0b10100100) 
    if (temp == 3) a_secment = 0xB0; // (0b10110000)
		if (temp == 4) a_secment = 0x99; // (0b10011001) 
    if (temp == 5) a_secment = 0x92; // (0b10010010) 
    if (temp == 6) a_secment = 0x82; // (0b10000010) 
    if (temp == 7) a_secment = 0xF8; // (0b11111000) 
    if (temp == 8) a_secment = 0x80; // (0b10000000)
    if (temp == 9) a_secment = 0x90; // (0b10010000) 
  }
}

void calc_a_secment_main() // calculate a_secment for main version
{
 if (a_digit == 2)  //
  {
   if (temp > 9)
    {
     while (temp > 9)
     temp = temp - 10;
    }
   if (temp == 0) a_secment = 0xC0; // (0b11000000) 
   if (temp == 1) a_secment = 0xF9; // (0b11111001) 
   if (temp == 2) a_secment = 0xA4; // (0b10100100) 
   if (temp == 3) a_secment = 0xB0; // (0b10110000) 
   if (temp == 4) a_secment = 0x99; // (0b10011001) 
   if (temp == 5) a_secment = 0x92; // (0b10010010) 
   if (temp == 6) a_secment = 0x82; // (0b10000010) 
   if (temp == 7) a_secment = 0xF8; // (0b11111000) 
   if (temp == 8) a_secment = 0x80; // (0b10000000)
   if (temp == 9) a_secment = 0x90; // (0b10010000)
  }
 if (a_digit == 1) //
  {
   temp = (temp/10);
   if (temp == 0) a_secment = 0xC0; // (0b11000000) 
   if (temp == 1) a_secment = 0xF9; // (0b11111001) 
   if (temp == 2) a_secment = 0xA4; // (0b10100100) 
   if (temp == 3) a_secment = 0xB0; // (0b10110000) 
   if (temp == 4) a_secment = 0x99; // (0b10011001) 
   if (temp == 5) a_secment = 0x92; // (0b10010010) 
   if (temp == 6) a_secment = 0x82; // (0b10000010) 
   if (temp == 7) a_secment = 0xF8; // (0b11111000) 
   if (temp == 8) a_secment = 0x80; // (0b10000000)
   if (temp == 9) a_secment = 0x90; // (0b10010000) 
  }
}
//************************************************************************
//************************************************************************
void secment_value()
{
 if (temp == 0) a_secment = 0xC0; // (0b11000000) 
 if (temp == 1) a_secment = 0xF9; // (0b11111001) 
 if (temp == 2) a_secment = 0xA4; // (0b10100100) 
 if (temp == 3) a_secment = 0xB0; // (0b10110000) 
 if (temp == 4) a_secment = 0x99; // (0b10011001) 
 if (temp == 5) a_secment = 0x92; // (0b10010010) 
 if (temp == 6) a_secment = 0x82; // (0b10000010) 
 if (temp == 7) a_secment = 0xF8; // (0b11111000) 
 if (temp == 8) a_secment = 0x80; // (0b10000000)
 if (temp == 9) a_secment = 0x90; // (0b10010000) 
}
//************************************************************************

//************************************************************************
void IC7_load() // manuel serial transfering a_secment to IC7 storage register
 {
  disp_clk_off; // ready for new date
  disp_str_off;// ready for new data
  //
  if (bit_test(a_secment,7)) disp_data_on; //
  else disp_data_off; //
  disp_clk_on; //
  disp_clk_off; //
  //
  if( bit_test(a_secment,6)) disp_data_on; //
  else disp_data_off; //
  disp_clk_on; //
  disp_clk_off; //
  //
  if( bit_test(a_secment,5)) disp_data_on; //
  else disp_data_off; //
  disp_clk_on; //
  disp_clk_off; //
  //
  if( bit_test(a_secment,4)) disp_data_on; //
  else disp_data_off; //
  disp_clk_on; //
  disp_clk_off; //
  //
  if( bit_test(a_secment,3)) disp_data_on; //
  else disp_data_off; //
  disp_clk_on; //
  disp_clk_off; //
  //
  if( bit_test(a_secment,2)) disp_data_on; //
  else disp_data_off; //
  disp_clk_on; //
  disp_clk_off; //
  //
  if( bit_test(a_secment,1)) disp_data_on; //
  else disp_data_off; //
  disp_clk_on; //
  disp_clk_off; //
  //
  if( bit_test(a_secment,0)) disp_data_on; //
  else disp_data_off; //
  disp_clk_on; //
  disp_clk_off; //
  //
  disp_str_on; // transfer to storage register
  disp_str_off; //
 }
//************************************************************************

