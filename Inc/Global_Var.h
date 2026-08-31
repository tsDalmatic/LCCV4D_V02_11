
#include "stm32f0xx_hal.h"
// Status flags
extern uint8_t kip_stop_ready; //
extern uint8_t kip_down_ready;
extern uint8_t safety_close_ok;
extern uint8_t ols;         // open limit switch after software filter
extern uint8_t ols_out;     // Open limit status from encoder function
extern uint8_t cls;         // close limit switch after software filter
extern uint8_t cls_out;     // close limit status from encoder function
extern uint8_t edge_photo_owr;     //
extern uint8_t close_out;   // output testbit command for closing in modbus routine - old input_state(close_K1)
extern uint8_t run_prog_tb;	      // testbit for just programming select
extern uint8_t autoclose_dis;     // reset by powerup
extern uint8_t chkbit_pc;         //
extern uint8_t pulse_chk;         // used for welding check
extern uint8_t pulse_counter;     //
extern uint8_t phase_inv;         // Door direction testbit 04-11-2021

//
extern uint8_t ser_count_d;       // testbit for servicecounter down counting

// Push-buttons
extern uint8_t e_stop_pb;
extern uint8_t stop_pb;     // stop push-button after sw-filter
extern uint8_t stop_lid_pb;
extern uint8_t stop_safety_chain; // stop by safety chain disconnect 15-11-2016
extern uint8_t stop_disp_pb;    // stop on display unit after sw-filter
extern uint8_t safe_stop;     // safe stop command from encoder
extern uint8_t open_pb;       // open push-button after sw-filter
extern uint8_t open_disp_pb;  // open push-button on display unit after sw-filter
extern uint8_t close_pb;      // close push-button after sw-filter
extern uint8_t close_disp_pb; // close push-button on display unit after sw-filter
extern uint8_t kip_pb;      // kip push-button after sw-filter
extern uint8_t e_h_ols_sw;  // electronic half open limit switch status for on/off select of the function or as pushbutton
// extern uint8_t ref_sw;      // ref switch for different functions
extern uint8_t ex_auto_sw;  // external auto close switch
//
extern uint8_t free_1;          // free input 1 after software filter 03-11-2021
extern uint8_t free_2;          // free input 1 after software filter 03-11-2021
extern uint8_t free_3;          // free input 1 after software filter 03-11-2021

extern uint8_t E_limit;         // control bit for electronic limits selected (for future use in LCC V3
extern uint8_t fac;             // testbit for factory settings reset
extern uint8_t s_speed_learn;   // single speed learn 
extern uint8_t photo_learn;     // testbit for photo in frame learning. When no value the variabel should be 0 by powerup
extern uint8_t photo_break;     // testbit for photo in frame activated
extern uint8_t welded;          // welding observed bit. for future option in LCC V3
extern uint8_t s_encoder_tst;   // test bit for special encoder test feature, set in parameter 82 by push for test. When no value the variabel should be 0 by powerup
extern uint8_t ecode1;          // 1 = No answer from encoder
extern uint8_t ecode2;          // 2 = limits not learned
extern uint8_t ecode3;          // 3 = output fail - door runing but should be stopped
extern uint8_t ecode4;          // 4 = wrong DIL setup fail in turning direction or cls = ols = user fail or encoder fail
extern uint8_t ecode5;          // 5 = communicationsfail
extern uint8_t ecode7;          // 7 = Kostal pot fail or Dal/Feig encoder position fail (out of area)
extern uint8_t ecode8;          // 8 = Kostal fail in supply voltage
extern uint8_t ecode9;          // 9 = EEPROM safety check - flash = stop cause of fail
extern uint8_t half_ols_active; // half open limit active (electronic type)
extern uint16_t limit_indi;     // limit indication startet from encoder function
extern uint8_t enc_low_sp_p;    // encoder low speed point. 1 = low speed must be activated. old low_sp on encoder 
extern uint8_t temp;            // generel register til flere formål
extern uint8_t temp1;           // generel register til flere formål
extern uint8_t temp4;           //                   
extern uint8_t temp3;           // EEPROM safety check 26-10-2011
extern uint16_t adr;            // EPROM address for 16 bit write!
extern uint32_t data;           // Data for 16 bit read and write!
extern uint8_t tim_out_cnt;     // time out counter for encoder request
extern uint8_t status_para;     // status parameter
extern uint8_t error_counter;   //
extern uint8_t light_code;      // light codes for traffic light control
extern uint8_t lock_switch;     // 


// Status flags
extern uint8_t e1_mon;        // error code for  monitoring fail
extern uint8_t e2_speed;      // error - speed control stop
extern uint8_t e3_run;        // error - runtime
extern uint8_t e4_ser;        // error - service counter decremented to 0
extern uint8_t e5_photo;      // error - failure in photo circuit
extern uint8_t e6_edge;       // error - failure in edge circuit
extern uint8_t e7_tacho;      // error - tacho failure no pulses or very slow pulses
extern uint8_t e8_wear;       // error - adap speed wear
extern uint8_t e9_pos_change; // no position change error
extern uint8_t e10_edge;      // error - failure in 2. edge circuit or wicket Door circuit 10-11-2021
extern uint8_t e20_fail;      // clear EEPROM counter failure at powerup - 0 by powerup
extern uint8_t e21_fail;      // EEPROM safety check failed 
extern uint8_t e22_stop;      // error in stop circuit
extern uint8_t e23_chain;     // error in safety chain circuit
extern uint8_t e27_modbus;        // error code for modbus timeout fail 16-03-2017
extern uint8_t e28_wdt;       // error code for Watchdog timeout occured cleared by power off
extern uint8_t e29_xtal;	    // processor main X-tal failed
extern uint8_t e31_inverter;      // inverter failure occured - for last error register only. Changed from e30 to e31 04-11-2021
extern uint8_t e32_weld;          // c_monitor have observed welded contactor added 04-11-2021
extern uint8_t e33_unlock_fail;    // unlock fail after activating relay output for this 04-03-2026
extern uint8_t e35_output_fail;   // program fail if this happen. open and close command at the same time. added 04-11-2021

extern uint8_t e1_mon_old;        // old error code for  monitoring fail for last 10 error saving
extern uint8_t e2_speed_old;      // old error - speed control stop for last 10 error saving
extern uint8_t e3_run_old;        // old error - runtime for last 10 error saving
extern uint8_t e4_ser_old;        // old error - service counter decremented to 0 for last 10 error saving
extern uint8_t e5_photo_old;      // old error - failure in photo circuit for last 10 error saving
extern uint8_t e6_edge_old;       // old error - failure in edge circuit for last 10 error saving
extern uint8_t e7_tacho_old;      // old error - tacho failure no pulses or very slow pulses for last 10 error saving
extern uint8_t e8_wear_old;       // old error - adap speed wear for last 10 error saving
extern uint8_t e9_pos_change_old; // older error - no position change error
extern uint8_t e10_edge_old;      // old error - failure in 2. edge circuit or wicket Door circuit 10-11-2021
extern uint8_t e20_fail_old;      // older error 
extern uint8_t e21_fail_old;      // older error
extern uint8_t ecode7_old;	      // encoder out of range error
extern uint8_t e22_stop_old;      // error in stop circuit
extern uint8_t e23_chain_old;     // error in safety chain circuit
extern uint8_t e27_modbus_old;    // error code for modbus timeout fail 22-03-2017
extern uint8_t e28_wdt_old;       // error code for Watchdog timeout occured cleared by power off
extern uint8_t e29_xtal_old;	    // processor main X-tal failed
extern uint8_t e31_inverter_old;  // inverter failure occured - for last error register only. Changed from e30 to e31 04-11-2021
extern uint8_t e32_weld_old;      // c_monitor have observed welded contactor added 04-11-2021
extern uint8_t e33_unlock_fail_old; // unlock fail after activating relay output for this 04-03-2026
//
extern uint8_t imp_open;      // impulse open testbit
extern uint8_t imp_close;     // impulse close testbit
extern uint8_t err_edge;      //no error at powerup
extern uint16_t edge_err_show; // 0 by powerup 
extern uint8_t photoclose;    // 0 by powerup
extern uint8_t interlock;         // checkbit for interlock function
extern uint8_t edge_photo_fail;   // testbit for service LED, showing fail when close P/B is activated. Keeped for LCC V3 purpose
extern uint8_t dw_ok;             //
extern uint8_t ok_down;           //
extern uint8_t photo1_dis;        // testbit used by photo frame function 
extern uint8_t photo2_dis;        // testbit used by photo frame function 
extern uint8_t colon_on;          // controlbit for colon flash
extern uint8_t relearn_ph1;       // testbit in ram for photo in frame error
extern uint8_t relearn_ph2;       // testbit in ram for photo in frame error
extern uint8_t d_flash_tim;       // testbit for display flashing


extern uint8_t moving_fail;    // 1  = fail, no encoder position movement controlled in encoder function
extern uint8_t st_speed_stop;  // testbit stop command in singleturn speedcontrol
extern uint8_t st_wear_observed;   // old sk2,6 on encoder
extern uint8_t photo_pos_observed; // old s_byte2,5 on encoder
extern uint8_t low_bat;            // test bit for low battery. Also old s_byte2,6 on encoder
extern uint8_t spec_encoder_test;  // special encoder test. Old s_byte3 on encoder
extern uint8_t tighten;	           // 
extern uint8_t tighten_ready;      // disable wire tighten by powerup
extern uint8_t tighten_time;       // 
extern uint16_t autoclose_timer;   // 4 sec by powerup
extern uint8_t force_closing_time_ext;  // Forced closing time extender
extern uint16_t force_closing_time;     // 2 min by powerup - Autoclose override

extern uint8_t para_test;          // controlbit for programming mode
extern uint8_t p_value;
extern uint8_t par_val;            // controlbit for programming mode. checking about stop pushbutton still depressed or have been released
extern uint8_t disp_status;
extern uint8_t prog_mode_tb;       // after powerup always start with parametermeter 1 in progmode. -  0 by powerup
extern uint8_t prev_para;          // controlbit for selecting previous parameter
extern uint8_t confirm_learn;      // testbit for confirming learned
extern uint8_t new_EE;             // testbit for EE update needed
extern uint8_t next_para;          // control bit for selecting next parameter

extern uint8_t parameter;          // parameter number used in programming mode

extern int16_t ver_show;


// DIP-switches
extern uint8_t run_prog;      // Status of DIL switch 1. When no value the variabel should be 0 by powerup 
extern uint8_t dil_2;
extern uint8_t dil_3;

// Safety related variables
extern uint8_t dw8k2;           // dw/8k2 safety edge input after sw-filter
extern uint8_t ex_dw8k2;        // dw/8k2 safety edge input after sw-filter 10-11-2021
extern uint8_t fraba;           // fraba safety edge input after sw-filter
extern uint8_t photo1;          // photo 1 input after sw-filter
extern uint8_t photo2;          // photo 2 input after sw-filter
extern uint8_t edge_state;			   // state on safety edge
extern uint8_t edge_state_2;			 // state on safety edge 2 10-11-2021
extern uint8_t fraba_timer;        // timer for optical edge
extern uint8_t fraba_pulse;        //
extern uint8_t safety_tst_time;    // 
extern uint8_t safety_after_time;  // 
extern uint16_t after_w_test;      // time from stop to safety test 
//
// variables where single bit operations is used
extern uint8_t sk1;             // control register for adaptive speed
                                //   bit 0 = control for 2 measures
                                //   bit 1 = capture has occured
                                //   bit 2 = control bit for values saved in EEprom cause of wear
                                //   bit 3 = control bit about values adjusted for this door cycle
                                //   bit 4 = control bit for 2 measures on wear before accept
                                //   bit 5 = control bit for reset for new learning
                                //   bit 6-7 = free
extern uint8_t sk2;             // control register for adaptive speed
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
extern uint8_t ft_stop_pb;
extern uint8_t ft_stop_disp_pb;    // stop on display unit
extern uint8_t ft_open_pb;
extern uint8_t ft_open_disp_pb;    // open on display unit
extern uint8_t edgetype;    // Edge list type selected in prog. mode
                            // 1=t_dw, 2=t_8k2, 3=t_fraba, 4=t_lindab, 5=t_teleco_opt
//*******************************************														
														
extern uint8_t w_edge_ok;             // testbit for wireless edge
extern uint8_t rev_spec;              // special reverseing testbit used in parameter 2303 - Should be 0 by power up but double safe 
extern uint8_t re_open_tb;            // testbit for special autoclose secure - Should be 0 by power up but double safe
extern uint8_t warn_started;          // controlbit for warning already started
extern uint8_t warning;               // controlbit for warning selected on lamp PCB
extern uint8_t lp_req_doub;	          // testbit for only request lamp_pcb every second time to fit frequency.
                                      // with this doubling the request time will be 4 mS. 
extern uint8_t e_tst_result;          // test bit for special encoder test feature																			

extern uint8_t speed_opb;             // speed open pull back time - Should be 0 by power up but double safe
extern uint8_t pull_back_time;        // special minimum speed pullback time so the will always be a little 
                                      // reverse no matter deadman select.
extern uint8_t tighten_time;          // wire tighten time - Should be 0 by power up but double safe
extern uint8_t test_delay;            // time from stop to safety test
extern uint16_t max_w_tst;            // Max wireless test time 
extern uint16_t small_move;           // only small movement step by E:09 failure
extern uint16_t temp2;                // general temp 16 bit register
extern uint16_t rev_time;             // reverse time
extern uint16_t run_timer;            // run timer
extern uint16_t interlock_tim;        // 5 sec. interlock time, Stop press time
extern uint16_t e_tst_tim;            // timer for encoder test time. Normally 1 sec. 13-01-2012	
extern uint16_t confirm_timer;        // confirm timer for display programming feedback
extern uint8_t flash_timer;           //

extern uint16_t show_clr_tim;         // prvent long time before flash can be active	
extern uint16_t st_speed_stop_tim;    //
extern uint8_t push_code;             // push code register for remember activated buttons
extern uint16_t photo_learn_tim;       // for photo in frame learning purpose 27-06-2024
extern uint16_t bat_timer;            // low battery showing flash timer
extern uint8_t half_ols_tim;          // filter to prevent new speed area calc when electronic half open is active
extern uint16_t hide_timer;           // 
extern uint16_t re_open_tim;          // reopen timer
extern uint8_t pc_tim;                //
extern uint16_t afterrun_time;        // afterrun time. 0 - by powerup
extern uint8_t templed;               // clear flash counts by powerup
extern uint8_t edge_fail_timer;       // new name of dw_fail timer
extern uint16_t warn_timer;           // warning timer, trafic light
extern uint16_t open_long_tim;        // long push of open P/B for factory reset and last 10 errors reset
extern uint8_t colon_f_timer;         // timer for colon flash
extern uint8_t spec_edge_tim;         // timer for error in edgesetup (before fail)
extern uint16_t stop_long_tim;        // long push of stop push-button check timer
extern uint8_t push_hold_tim;         //timer for fast step of parametervalue
extern uint16_t push_code_tim;        // max time to make the push code
extern uint8_t blinktimer;            // LED on time by flash
extern uint8_t pause1;                // pausetime between LED flash
extern uint16_t pause2sek;            // long pause before next period - set to approx. 1.5 sec by prototype program
extern uint8_t ch_timer;              // change timer used in parameter 15
extern uint8_t push_timer;             //
extern uint8_t hide_learn_finish;     // 30-04-2019 was not made i h-file
extern uint16_t photo_setp_tim;       // timer for 3 double push related to photo disable setpoint  30-04-2019
extern uint16_t limits_show;          // showing encoder by powerup 30-04-2019
//
//**** Speed related variabels *********************************
extern uint16_t a_speed;              // current measured value of speed control
extern uint16_t speed_delay;          // speed delay
extern uint16_t speed_sp;             // calculated speed setpoint manual speed
extern uint16_t speed_test;           // debug test timer for a_speed read
extern uint16_t time;                 // check register for tacho fail
extern uint8_t measure_new;           // testbit for 2 measures in speed kontrol
extern uint16_t measure_1;            // First measurement register for 2 measurements use in speed control
extern uint16_t wear_value;           // wear value used in adaptive speed
extern uint16_t speedsetp;            // speed setpoint, adaptive speed control
extern uint16_t i_speedsetp;          // speed setpoint, adaptive speed control initial value
extern uint16_t pv05;                 // 0.5% percent calculated value for adaptive speedcontrol based on learned ce1 value
extern uint16_t sp_adj;               // clear this because the value is random before force control learning else it will go wrong in function
                                      // value_upd first time learning. 0 by powerup. 0.25% speed adjusment value for wear
extern uint8_t motor_rpm_1500;				// motor select by parameter	
extern uint8_t t1_con_0;              // testbit for speed control
extern uint8_t speed_f_timer;         // initial speed filter
extern uint16_t m_speed;              // manuel speed temp register
extern uint16_t counterpos;           //
extern uint16_t calc_counterpos;      //
extern uint8_t div_2;                 // used for adaptive area calculate
extern uint8_t speed_o;               // adaptive speed control check reg.
extern uint8_t speed_c;               // adaptive speed control check reg.
extern uint8_t code_spd_rev;          // special code for speed reverse
extern uint8_t open_cnt;              // open counter for adaptive speed adjustment
extern uint8_t omsk12;                // value for area change from area 1 to area 2
extern uint8_t omsk23;                // value for area change from area 2 to area 3
extern uint8_t omsk34;                // value for area change from area 3 to area 4
extern uint8_t mso;                   // move setpoint controlreg. adaptive speedcontrol
extern uint8_t msc;                   // move setpoint controlreg. adaptive speedcontrol
extern uint16_t calc_a_speed;			 	  //
extern uint8_t st_speed_learn_ok;     // singleturn speed learning OK - old s_byte2,2
extern uint8_t new_m_speed;           // new manuel speed adjusment wanted for future use in LCC V3 only
//
extern uint16_t ce1;                  // adaptive speed control
extern uint16_t ce2;                  // adaptive speed control
extern uint16_t ce3;                  // adaptive speed control
extern uint16_t ce4;                  // adaptive speed control
extern uint16_t oe1;                  // adaptive speed control
extern uint16_t oe2;                  // adaptive speed control
extern uint16_t oe3;                  // adaptive speed control
extern uint16_t oe4;                  // adaptive speed control
//
extern uint16_t ice1;                 // adaptive speed control
extern uint16_t ice2;                 // adaptive speed control
extern uint16_t ice3;                 // adaptive speed control
extern uint16_t ice4;                 // adaptive speed control
extern uint16_t ioe1;                 // adaptive speed control
extern uint16_t ioe2;                 // adaptive speed control
extern uint16_t ioe3;                 // adaptive speed control
extern uint16_t ioe4;                 // adaptive speed control
//
extern uint32_t a;										// debug or RX sync
extern uint32_t b;										// debug or RX sync
extern uint32_t u2time;								// debug or RX sync
extern uint8_t byte_count;            // RX sync

