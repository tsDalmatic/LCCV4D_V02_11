#include "stm32f0xx_hal.h"
#include "Global_Var.h"
// Status flags
uint8_t kip_stop_ready; // testbit for special kip function - 0 by powerup
uint8_t kip_down_ready; // testbit for special kip function - 0 by powerup
uint8_t safety_close_ok; // 0 by powerup
uint8_t ols;         // open limit switch after software filter - 0 by powerup
uint8_t ols_out;     // Open limit status from encoder function
uint8_t cls;         // close limit switch after software filter - 0 by powerup
uint8_t cls_out;     // close limit status from encoder function
uint8_t edge_photo_owr;    // 0 by powerup
uint8_t close_out;         // output testbit command for closing in modbus routine - old input_state(close_K1)
uint8_t run_prog_tb;	     // testbit for just programming select
uint8_t autoclose_dis = 0; // reset by powerup
uint8_t chkbit_pc = 0;     //
uint8_t pulse_chk = 0;     // used for welding check
uint8_t pulse_counter = 0; //
uint8_t phase_inv;         // Door direction testbit 04-11-2021

//
uint8_t ser_count_d;       // testbit for servicecounter down counting

// Push-buttons
uint8_t e_stop_pb;       // must by 0 by power up to prevent emergency stop of inverter
uint8_t stop_pb = 1;     // stop push-button after sw-filter - active by powerup
uint8_t stop_lid_pb;
uint8_t stop_safety_chain; // stop by safety chain disconnect 15-11-2016
uint8_t stop_disp_pb;    // stop on display unit
uint8_t safe_stop;       // safe stop command from encoder
uint8_t open_pb;         // open push-button after sw-filter - 0 by powerup
uint8_t open_disp_pb;    // open push-button on display unit after sw-filter - 0 by powerup
uint8_t close_pb;        // close push-button after sw-filter - 0 by powerup
uint8_t close_disp_pb;   // close push-button on display unit after sw-filter - 0 by powerup
uint8_t kip_pb;          // kip push-button after sw-filter - 0 by powerup
uint8_t e_h_ols_sw;      // electronic half open limit switch status for on/off select of the function or as pushbutton
// uint8_t ref_sw;       // ref switch for different functions
uint8_t ex_auto_sw = 1;  // external auto close switch. No autoclose by powerup
//
uint8_t free_1;          // free input 1 after software filter 03-11-2021
uint8_t free_2;          // free input 1 after software filter 03-11-2021
uint8_t free_3;          // free input 1 after software filter 03-11-2021

uint8_t E_limit = 1;     // control bit for electronic limits selected (for future use in LCC V3 Selected by powerup
uint8_t fac;             // testbit for factory settings reset - 0 by powerup
uint8_t s_speed_learn;   // single speed learn 
uint8_t photo_learn;     // testbit for photo in frame learning. When no value the variabel should be 0 by powerup
uint8_t photo_break;     // testbit for photo in frame activated
uint8_t welded;          // welding observed bit. for future option in LCC V3 - 0 by powerup
uint8_t s_encoder_tst;   // test bit for special encoder test feature, set in parameter 82 by push for test. When no value the variabel should be 0 by powerup
uint8_t ecode1;          // 1 = No answer from encoder
uint8_t ecode2;          // 2 = limits not learned
uint8_t ecode3;          // 3 = output fail - door runing but should be stopped
uint8_t ecode4;          // 4 = wrong DIL setup fail in turning direction or cls = ols = user fail or encoder fail
uint8_t ecode5;          // 5 = communicationsfail
uint8_t ecode7;          // 7 = Kostal pot fail or Dal/Feig encoder position fail (out of area)
uint8_t ecode8;          // 8 = Kostal fail in supply voltage
uint8_t ecode9;          // 9 = EEPROM safety check - flash = stop cause of fail
uint8_t half_ols_active; // half open limit active (electronic type)
uint16_t limit_indi;     // limit indication startet from encoder function
uint8_t enc_low_sp_p;    // encoder low speed point. 1 = low speed must be activated. old low_sp on encoder 
uint8_t temp;            // generel register til flere formål
uint8_t temp1;           // generel register til flere formål
uint8_t temp4;           //                   
uint8_t temp3;           // EEPROM safety check 26-10-2011
uint16_t adr;            // EPROM address for 16 bit write!
uint32_t data;           // Data for 16 bit read and write!
uint8_t tim_out_cnt = 30; // time out counter for encoder request. Filter for time out of encoder answer 200mS by powerup else 20mS.
uint8_t status_para = 1;   // status parameter
uint8_t error_counter = 1; //
uint8_t light_code;        // light codes for traffic light control
uint8_t lock_switch;       // 


// Status flags
uint8_t e1_mon;            // error code for  monitoring fail - 0 by powerup
uint8_t e2_speed;          // error - speed control stop - 0 by powerup
uint8_t e3_run;            // error - runtime - 0 by powerup
uint8_t e4_ser;            // error - service counter decremented to 0 - 0 by powerup
uint8_t e5_photo;          // error - failure in photo circuit - 0 by powerup
uint8_t e6_edge;           // error - failure in edge circuit - 0 by powerup
uint8_t e7_tacho;          // error - tacho failure no pulses or very slow pulses - 0 by powerup
uint8_t e8_wear;           // error - adap speed wear - 0 by powerup
uint8_t e9_pos_change;     // no position change error - 0 by powerup
uint8_t e10_edge;          // error - failure in 2. edge circuit or wicket Door circuit 10-11-2021
uint8_t e20_fail;          // clear EEPROM counter failure at powerup - 0 by powerup
uint8_t e21_fail;          // EEPROM safety check failed
uint8_t e22_stop;          // error in stop circuit
uint8_t e23_chain;         // error in safety chain circuit
uint8_t e27_modbus;        // error code for modbus timeout fail 16-03-2017
uint8_t e28_wdt;           // error code for Watchdog timeout occured cleared by power off
uint8_t e29_xtal;	         // processor main X-tal failed
uint8_t e31_inverter;      // inverter failure occured - for last error register only. Changed from e30 to e31 04-11-2021
uint8_t e32_weld;          // c_monitor have observed welded contactor added 04-11-2021
uint8_t e33_unlock_fail;    // unlock fail after activating relay output for this 04-03-2026
uint8_t e35_output_fail;   // program fail if this happen. open and close command at the same time. added 04-11-2021

uint8_t e1_mon_old;        // old error code for  monitoring fail for last 10 error saving
uint8_t e2_speed_old;      // old error - speed control stop for last 10 error saving
uint8_t e3_run_old;        // old error - runtime for last 10 error saving
uint8_t e4_ser_old;        // old error - service counter decremented to 0 for last 10 error saving
uint8_t e5_photo_old;      // old error - failure in photo circuit for last 10 error saving
uint8_t e6_edge_old;       // old error - failure in edge circuit for last 10 error saving
uint8_t e7_tacho_old;      // old error - tacho failure no pulses or very slow pulses for last 10 error saving
uint8_t e8_wear_old;       // old error - adap speed wear for last 10 error saving
uint8_t e9_pos_change_old; // older error - no position change error
uint8_t e10_edge_old;      // old error - failure in 2. edge circuit or wicket Door circuit 10-11-2021
uint8_t e20_fail_old;      // older error 
uint8_t e21_fail_old;      // older error
uint8_t ecode7_old;	       // encoder out of range error
uint8_t e22_stop_old;      // error in stop circuit
uint8_t e23_chain_old;     // error in safety chain circuit
uint8_t e27_modbus_old;    // error code for modbus timeout fail 22-03-2017
uint8_t e28_wdt_old;       // error code for Watchdog timeout occured cleared by power off
uint8_t e29_xtal_old;	     // processor main X-tal failed
uint8_t e31_inverter_old;  // inverter failure occured - for last error register only. Changed from e30 to e31 04-11-2021
uint8_t e32_weld_old;      // c_monitor have observed welded contactor added 04-11-2021
uint8_t e33_unlock_fail_old; // unlock fail after activating relay output for this 04-03-2026
//
uint8_t imp_open;          // impulse open testbit - 0 by powerup
uint8_t imp_close;         // impulse close testbit - 0 by powerup
uint8_t err_edge = 0;      // no error at powerup
uint16_t edge_err_show;     // 0 by powerup 
uint8_t photoclose;        // 0 by powerup
uint8_t interlock;         // checkbit for interlock function
uint8_t edge_photo_fail;   // testbit for service LED, showing fail when close P/B is activated. Keeped for LCC V3 purpose
uint8_t dw_ok;             //
uint8_t ok_down;           //
uint8_t photo1_dis;        // testbit used by photo frame function 
uint8_t photo2_dis;        // testbit used by photo frame function
uint8_t colon_on;          // controlbit for colon flash
uint8_t relearn_ph1;       // testbit in ram for photo in frame error
uint8_t relearn_ph2;       // testbit in ram for photo in frame error
uint8_t d_flash_tim;       // testbit for display flashing

uint8_t moving_fail;            // 1  = fail, no encoder position movement controlled in encoder function
uint8_t st_speed_stop = 0;      // testbit stop command in singleturn speedcontrol - no speed stop by powerup
uint8_t st_wear_observed;       // old sk2,6 on encoder - 0 by powerup
uint8_t photo_pos_observed = 0; // old s_byte2,5 on encoder - Shall be cleared by powerup so photo is not disabled by first closing after powerup
uint8_t low_bat = 0;            // test bit for low battery. Also old s_byte2,6 on encoder - no low bat by powerup
uint8_t spec_encoder_test;      // special encoder test. Old s_byte3 on encoder	
uint8_t tighten;	              // 
uint8_t tighten_ready = 1;      // disable wire tighten by powerup
uint8_t tighten_time = 0;       // wire tighten time - Should be 0 by power up but double safe
uint16_t autoclose_timer = 1200;   // 4 sec by powerup
uint8_t force_closing_time_ext;    // Forced closing time extender
uint16_t force_closing_time = 240; // 2 min by powerup - Autoclose override


uint8_t para_test;                 // controlbit for programming mode - 0 by powerup
uint8_t p_value;	                 // 0 by power up
uint8_t par_val;                   // controlbit for programming mode. checking about stop pushbutton still depressed or have been released
uint8_t disp_status;               // 0 by power up
uint8_t prog_mode_tb = 0;          // after powerup always start with parametermeter 1 in progmode.
uint8_t prev_para;                 // controlbit for selecting previous parameter
uint8_t confirm_learn;             // testbit for confirming learned
uint8_t new_EE;                    // testbit for EE update needed
uint8_t next_para;                 // control bit for selecting next parameter

uint8_t parameter = 1;             // parameter number used in programming mode parameter 1 by powerup

int16_t ver_show = 300;            // 1.0 sec version show at powerup

// DIP-switches
uint8_t run_prog;      // Status of DIL switch 1. When no value the variabel should be 0 by powerup 
uint8_t dil_2;
uint8_t dil_3;

// Safety related variables
uint8_t dw8k2;           // dw/8k2 safety edge input after sw-filter
uint8_t ex_dw8k2;        // dw/8k2 safety edge input after sw-filter 10-11-2021 
uint8_t fraba;           // fraba safety edge input after sw-filter
uint8_t photo1;          // photo 1 input after sw-filter - 0 by powerup
uint8_t photo2;          // photo 2 input after sw-filter - 0 by powerup
uint8_t edge_state;			        // state on safety edge
uint8_t edge_state_2;			      // state on safety edge 2 10-11-2021
uint8_t fraba_timer = 3;        // timer for optical edge
uint8_t fraba_pulse = 1;        // 1 by powerup
uint8_t safety_tst_time;        // 
uint8_t safety_after_time;      // 
uint16_t after_w_test;          // time from stop to safety test 

// variables where single bit operations is used
uint8_t sk1;                    // control register for adaptive speed
                                //   bit 0 = control for 2 measures
                                //   bit 1 = capture has occured
                                //   bit 2 = control bit for values saved in EEprom cause of wear
                                //   bit 3 = control bit about values adjusted for this door cycle
                                //   bit 4 = control bit for 2 measures on wear before accept
                                //   bit 5 = control bit for reset for new learning
                                //   bit 6-7 = free
uint8_t sk2;                    // control register for adaptive speed
                                // bit 0, 1=areas OK (ok_omr)
                                // bit 1, 1=capture has occured (cap)
                                // bit 2, 1=position counter active (cnt_ak)
                                // bit 3, 1=setpoint values is learned (v_ok)
                                // bit 4, 1=old values is learned (g_ok)
                                // bit 5, 1=counter position is saved in EEPROM (cnt_gem)
                                // bit 6, 1=wear is observed, cleared by learning of new values (wear)
                                // bit 7, 1=Initial values is learned (inival)
//
// variables used by other functions also
uint8_t ft_stop_pb  = 70;        // changed from 40 to 70 to be sure block_open is working if open push by power-up 28-03-2023
uint8_t ft_stop_disp_pb = 40;    // stop on display unit
uint8_t ft_open_pb  = 60;        // changed from 20 to 60 02-05-2017
uint8_t ft_open_disp_pb  = 60;   // open on display unit. Changed from 20 to 60 02-05-2017
uint8_t edgetype;    // Edge list type selected in prog. mode
                            // 1=t_dw, 2=t_8k2, 3=t_fraba, 4=t_lindab, 5=t_teleco_opt
//*******************************************														
														
uint8_t w_edge_ok;             // testbit for wireless edge
uint8_t rev_spec = 0;          // special reverseing testbit used in parameter 2303 - Should be 0 by power up but double safe 
uint8_t re_open_tb = 0;        // testbit for special autoclose secure - Should be 0 by power up but double safe
uint8_t warn_started;          // controlbit for warning already started
uint8_t warning;               // controlbit for warning selected on lamp PCB
uint8_t lp_req_doub;		       // testbit for only request lamp_pcb every second time to fit frequency.
                               // with this doubling the request time will be 4 mS. 
uint8_t e_tst_result;          // test bit for special encoder test feature
															 
uint8_t speed_opb = 0;         // speed open pull back time - Should be 0 by power up but double safe
uint8_t pull_back_time;        // special minimum speed pullback time so the will always be a little
                               // reverse no matter deadman select.
uint8_t test_delay = 100;      // time from stop to safety test. 200mS by powerup to prevent locking og stop when stop is active by power up 
                               // value shall be > 2 to work. Because safety_req is done before stop_pb is handled in interpreter routine
uint16_t max_w_tst;            // Max wireless test time 
uint16_t small_move = 0;       // only small movement step by E:09 failure
uint16_t temp2;                // general temp 16 bit register
uint16_t rev_time = 4;         // reverse time - 4 by powerup
uint16_t run_timer;            // run timer
uint16_t interlock_tim;        // 5 sec. interlock time, Stop press time
uint16_t e_tst_tim;            // timer for encoder test time. Normally 1 sec. 13-01-2012	
uint16_t confirm_timer;        // confirm timer for display programming feedback  - 0 by powerup
uint8_t flash_timer;           //

uint16_t show_clr_tim = 0;     // prvent long time before flash can be active
uint16_t st_speed_stop_tim = 0; // 
uint8_t push_code = 0;         // push code register for remember activated buttons
uint16_t photo_learn_tim = 0;   // for photo in frame learning purpose 27-06-2024
uint16_t bat_timer = 0;        // low battery showing flash timer
uint8_t half_ols_tim = 150;    // filter to prevent new speed area calc when electronic half open is active
uint16_t hide_timer = 0;       // 
uint16_t re_open_tim = 0;      // reopen timer - 0 by powerup
uint8_t pc_tim = 0;            //
uint16_t afterrun_time;        // afterrun time. 0 - by powerup becase it could be a long time at powerup if afterrun is not selected
uint8_t templed = 0;           // clear flash counts by powerup
uint8_t edge_fail_timer;       // new name of dw_fail timer
uint16_t warn_timer;           // warning timer, trafic light
uint16_t open_long_tim;        // long push of open P/B for factory reset and last 10 errors reset
uint8_t spec_edge_tim;         // timer for error in edgesetup (before fail)
uint16_t stop_long_tim;        // long push of stop push-button check timer
uint8_t push_hold_tim;         //timer for fast step of parametervalue
uint16_t push_code_tim;        // max time to make the push code
uint8_t blinktimer;            // LED on time by flash
uint8_t pause1;                // pausetime between LED flash
uint16_t pause2sek;            // long pause before next period - set to approx. 1.5 sec by prototype program
uint8_t ch_timer;              // change timer used in parameter 15
uint8_t push_timer;            //
uint8_t hide_learn_finish;     //
uint16_t photo_setp_tim;       // timer for 3 double push related to photo disable setpoint  30-04-2019
uint16_t limits_show = 600;    // 2 sec limits show at powerup after version show 30-04-2019 for use with Dall display
//
//**** Speed related variabels *********************************
uint16_t a_speed;              // current measured value of speed control
uint16_t speed_delay;          // speed delay
uint16_t speed_sp;             // calculated speed setpoint manual speed
uint16_t speed_test;           // debug test timer for a_speed read
uint16_t time;                 // check register for tacho fail	
uint8_t measure_new;           // testbit for 2 measures in speed kontrol	
uint16_t measure_1;            // First measurement register for 2 measurements use in speed control
uint16_t wear_value;           // wear value used in adaptive speed
uint16_t speedsetp;            // speed setpoint, adaptive speed control
uint16_t i_speedsetp;          // speed setpoint, adaptive speed control initial value
uint16_t pv05;                 // 0.5% percent calculated value for adaptive speedcontrol based on learned ce1 value
uint16_t sp_adj = 0;           // clear this because the value is random before force control learning else it will go wrong in function
                               // value_upd first time learning. 0 by powerup. 0.25% speed adjusment value for wear
uint8_t motor_rpm_1500;				 // motor select by parameter												
uint8_t t1_con_0 = 0;          // testbit for speed control
uint8_t speed_f_timer;         // initial speed filter
uint16_t m_speed;              // manuel speed temp register
uint16_t counterpos;           //
uint16_t calc_counterpos;      //
uint8_t div_2;                 // used for adaptive area calculate
uint8_t speed_o;               // adaptive speed control check reg.
uint8_t speed_c;               // adaptive speed control check reg.
uint8_t code_spd_rev;          // special code for speed reverse
uint8_t open_cnt;              // open counter for adaptive speed adjustment
uint8_t omsk12;                // value for area change from area 1 to area 2
uint8_t omsk23;                // value for area change from area 2 to area 3
uint8_t omsk34;                // value for area change from area 3 to area 4
uint8_t mso;                   // move setpoint controlreg. adaptive speedcontrol
uint8_t msc;                   // move setpoint controlreg. adaptive speedcontrol
uint16_t calc_a_speed;				 //
uint8_t st_speed_learn_ok;     // singleturn speed learning OK - old s_byte2,2
uint8_t new_m_speed;           // new manuel speed adjusment wanted for future use in LCC V3 only

//
uint16_t ce1;                  // adaptive speed control
uint16_t ce2;                  // adaptive speed control
uint16_t ce3;                  // adaptive speed control
uint16_t ce4;                  // adaptive speed control
uint16_t oe1;                  // adaptive speed control
uint16_t oe2;                  // adaptive speed control
uint16_t oe3;                  // adaptive speed control
uint16_t oe4;                  // adaptive speed control
//
uint16_t ice1;                 // adaptive speed control
uint16_t ice2;                 // adaptive speed control
uint16_t ice3;                 // adaptive speed control
uint16_t ice4;                 // adaptive speed control
uint16_t ioe1;                 // adaptive speed control
uint16_t ioe2;                 // adaptive speed control
uint16_t ioe3;                 // adaptive speed control
uint16_t ioe4;                 // adaptive speed control
//
uint32_t a;										 // debug or RX sync
uint32_t b;										 // debug or RX sync
uint32_t u2time;							 // debug or RX sync
uint8_t byte_count;            // RX sync
