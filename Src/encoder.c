// Program for Dalmatic LCC V3 Inverter udviklet af Melko Electronic
// Software er oprindeligt skrevet i C til V7E styring hos Dalmatic hvor dette er programmet i den lille processor til enkoder.
// Det oprindelige program hedder 7E-IC4K63-1.
// Compiler var CCS-C til PIC18F25K22 fra Microchip.
// Program omskrives til ARM processor fra ST og der benyttes compiler fra Keil.
// Programmet ændres så det kan integreres sammen med eksisterende portstyring software da LCC V3 Inverter kun har 1 processor.
// Oprindelige program arbejdede med fast program gennemløbstid på 6.3 mS der blev styret af timer0.
// Program ændres til at arbejde med den 2mS program programgennemløbstid som portstyring software har.
// Timer Interrupt (med prioitet 2) styrer forspørselstid på så der forespørges på position hvert 6.5 mS.
// 
// EUSART benyttes til kommunikation idet TX og RX føres til RS485 kreds. (8 bit data + 1 stopbit vælges for at
// være kombatibel med Feig)
// Kommunikationsform er asynkron idet enkodere har deres egen clock. Der vælges 19.2k som baud rate.
//
// Den overordnede programstruktur ses af tegning 4-0523, 4-0524, 4-0526
// Beregning af vejlængde for endestop og endestopsvinduer ses af tegning 4-0263
// Forklaring til positionsbestemmelse ses af tegning 4-0259
// Tidligere program arbejdede med 3 interrupt. Dette program arbejder på kun med 1 synligt interrupt. Processor styrer selv
// at modtage alle bytes uden at buffer skal tømmes.
// Programlog tekst fra programlog i tidligere software ses nederst i programmet
// ********************************************************************************************************

//********************************************************************************************************************
//********************************************************************************************************************
#include "stm32f0xx_hal.h"
#include "eprom.h"
#include "Defines.h"
#include "Global_Var.h"
#include "encoder.h"
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim7;
//
void save_new_pos_bat(void);
void enc_test_moving(void);	
void setup_controlbits(void);
void read_inputs(void);
void bat_test(void);
void co_buffer(void); 
void clr_buffer(void);
void enc_mtimer(void); 
void error_codes(void); 
void chk_weld(void); 
void prog_mode(void);  
void norm_mode(void);
void timeout_fail(void);
void clear_fails(void);
void kostal_moving_chk(void);
void dall_moving_chk(void);
void curtain_check(void);     // 30-04-2019
void inv_meas_save(void);     // 03-12-2021
void inv_speed_moni(void);    // 03-12-2021
void setup_uart_19200(void);  // 14-12-2021
void setup_uart_100000(void); // 14-12-2021
//
extern void stop_open_or_close(void); 
extern void delay_us(uint32_t delay_us); // 05-11-2021
extern uint8_t open_active(void);           // special inverter functions
extern uint8_t close_active(void);          // special inverter functions
extern uint8_t i_open;                      // inactive open testbit 30-04-2019
extern uint8_t i_close;                     // inactive close testbit 30-04-2019 
extern uint8_t i_kip;                       // inactive kip testbit 30-04-2019
//
//************************************************************************
#define maxpos 0xFFFFFFFF // max værdi for positionscounter Dal og Feig enkoder
#define maxposK 0x2000 // max værdi for positionscounter Kostal enkoder
#define DIL_tst_value 0x20000000 // max specialværdi til test run_range + 2 x limit_win skal kunne være der
#define dummybyte 0b01010101 //
//************************************************************************
#define LEDper 225              // 1.5 sek. LED periodepause
#define LED_on_tid 45          // LED ON tid ved blink = 45 x 6.66 mS = 300 mS
#define LED_off_tid 45         // LED OFF tid ved blink
//*******************************************************
#define dal     0x00  // Dalmatic enkoder valgt
#define kostal  0x01  // Kostal enkoder valgt
#define feig    0x02  // Feig enkoder valgt
#define avago   0x03  // Avago enkoder valgt
#define sce     0x04  // SCE RS485 enkoder valgt
//
// ****************RS485 opsætning**************************
#define  RS485_MASTER_ID        0x01 // her skal nok stå 0x01 for master adresse
#define  RS485_ENKODER_ID       0xA0 // Dall og Feig  0xA0 for enkoder adresse
#define  RS485_KOSTAL_I         0xB3 // Kostal control byte for increasing values by clockwice rotation
#define  RS485_KOSTAL_D         0xB0 // Kostal control byte for decreasing values by clockwice rotation
#define  RS485_BAT_STATE_ID     0xA5 // Dal encoder batteri voltage request 26-03-2010
#define  d_stopped_tim_set 308   // reload time for door stopped timer 25-01-2022
 static uint8_t tx1[2];              // 2 bytes	 
 uint8_t uart2_buf[7];         // uart 2 buffer
 uint8_t rx_buffer[7];         // 7 bytes needed to copy uart2_buf
 static uint8_t rx_buffer2;    // used for checking movement
 extern uint8_t uart3_buf[1];         // uart 3 buffer
 static uint8_t bat_req_prepare;     // control bit for battery req RS485 else pos req.
 uint8_t pack_56[3];          // array for SCE encoder package with 56 header 14-01-2021
 uint8_t pack_3a[7];          // array for SCE encoder package with 3a header 14-01-2021
 uint8_t pack_3b[6];          // array for SCE encoder package with 3b header 14-01-2021
 //
//
 // uint8_t temp5;                               // debug formål	
 uint16_t temp16;	                            // gen. purouse reg.
 uint32_t temp32;                             // temp for areaxx calculation	
 static uint8_t st_value_adj;           // old sk1,3
 
 //
 static uint8_t out_of_range;                 // kontrolbit for test om positioner helt ude af indlært område	
 
 
 uint8_t en_svar;                      // kontrolbit for om enkoder har svaret
 static uint8_t motor_run;                    // kontrolbit for enkoder bevægelse = Motor kører
 static uint8_t torque_singleturn;            // 1 = singleturn torgue selected by parameter 11
 //
 static uint8_t programming_mode;             // special programming mode used in encoder rutine
 static uint8_t e_limit_open_learning_active; // 
 static uint8_t e_limit_close_learning_active; // 
 static uint8_t e_limit_curt_rep_learning_active;// 12-02-2020
 static uint8_t save_active_learning_limit;   //
 static int8_t fine_ols;                      // must be signed 0 by power-up if no value is set
 static int8_t fine_cls;                      //	must be signed 0 by power-up if no value is set
 static uint8_t	open_ls_setp = 30;            // 30 by powerup
 static uint8_t	close_ls_setp =20;            // 20 by powerup	
 static	uint8_t speed_clr = 1;                // testbit for new learning wanted
 static uint16_t moving_timer;                // til brug ved stop hvis position ikke ændrer sig 
 static uint16_t mt_preset_value;             // til brug ved stop hvis position ikke ændrer sig 16-02-2017
 //  
 static uint8_t save_open = 0;  							// tryk for preset af OPEN LIMIT
 static uint8_t save_close = 0;               // tryk for preset af CLOSE LIMIT	
 static uint8_t learn_active = 0;             //
 static uint8_t save_curtain = 0;  						// tryk for preset af curtain rep position 12-02-2020
 static uint8_t new_limits;                   // cleared by power up and by programming mode select
 static uint8_t right_turn;                   // 
 static uint8_t use_half_open = 0;            // kontrolbit til styring af impulstryk for 1/2 åbne kommando	
 static uint8_t speed_lo_chk;                 // testbit ved singleturn speedkontrol
 static uint8_t speed_lc_chk;                 // testbit ved singleturn speedkontrol
 static uint8_t speed_filt = 0;               // control bit for 2 following lav speed measures for low speed stop - filter 0 ved powerup
 static uint8_t speed_filt_ini = 0;           // also filter on initialvalues - filter 0 ved powerup
 static uint8_t open_cnt_tb;                  // kontrolbit for open_cnt optælling 
 static uint8_t photo_pos_save_ready = 0;     // testbit, ready for saving on next stop
 static uint8_t ols_old;                      // control bit for only one request of battery voltage on OLS
 static uint8_t read_bat = 0;                 // for controlling interrupt RDA - no read by powerup
 static uint8_t e_mov_ok;                     // check bit for encoder move
 static uint8_t pwr_timer = 15;               // 100mS powerup timer så alle DIL switchse er læst inden request startes
 static uint8_t ecode7_timer = 100;           // 600mS filtertimer for out of range som på V7E
                                              // must be longer than tim_out_cnt at powerup  
 static uint16_t stop_tim;                    // stop filtertimer så stop ikke når at trække ved skift fra mek. endestop til el. endestop for fremtidig brug i LCC V3D
 static uint16_t bat_value = 2000;            // enkoder batteri værdi læst via rs485 - battery OK by powerup 
 static uint8_t E9mov_clr_open = 0;           // test register for sletning af moving_fail (E:09) hvis endestop kan findes
 static uint8_t E9mov_clr_close = 0;          // test register for sletning af moving_fail (E:09) hvis endestop kan findes
 static uint8_t fac_clr_tim;                  // factory clear filtertimer
 static uint8_t low_bat_filt = 0;             // filter for low batt observing, only 1 measure on each open
 uint16_t d_stopped_tim = 308;                // door stoppped timer 2 sec. by power-up 
 static uint16_t ecode3_tim;                  // filter for ecode3 
 static uint8_t e_tst_running;                // test byte for special encodercheck
 
 static uint32_t position;                    // position
 static uint32_t position_old ;               // tidligere position til stop hvis position ikke ændrer sig
 static uint32_t position_old2 ;              // tidligere position til stop hvis position ikke ændrer sig 2. måling 16-02-2017
 static uint32_t pos_chk;                     // position check for contactor welding test 
 static uint32_t limit_win;                   // endestopswindue - normalt 1/8 vejlængde
 static uint32_t limit_win_tst;               // special variabel der bruges til test af fejl i DILopsætning
 static uint32_t run_range;                   // beregnet vejlængde
 static uint32_t cls_min;                     // værdi for cls min som også er gemt i EEPROM 
 static uint32_t cls_max;                     // værdi for cls max som også er gemt i EEPROM
 static uint32_t ols_min;                     // værdi for ols min som også er gemt i EEPROM
 static uint32_t ols_max;                     // værdi for ols max som også er gemt i EEPROM
 static uint32_t ols_min_old;                 // værdi for ols min old til brug ved 1/2 ols visning på hovedprocessor 
 static uint32_t ols_max_old;                 // værdi for ols max old til brug ved 1/2 ols visning på hovedprocessor
 static uint32_t offset;                      // offsetværdi for justering af område så over/underflow undgås
 static uint32_t ex_limit_win;                // extra øgning af limit window for at kompensere for overløb
 static uint32_t fine_adj_unit;               // finjusteringenheder
 static uint32_t fine_adj_unit;               // finjusteringenheder
 static uint32_t open_low_sp;                 // low speed deceleration start ved åbne
 static uint32_t close_low_sp;                // low speed deceleration start ved lukke
 static uint32_t photo_pos;                   // temp to hold learned photo position until EEPROM saving
 static uint32_t p5_run_range;                // 5% of run_range
 static uint32_t e_chk_pos_1;                 // first saved position by encodercheck 21-12-2011
 static uint32_t e_chk_pos_2;                 // second saved position by encodercheck
 static uint8_t offset_sign;                  // offset fortegn - 1=positiv 0=negativ (1 byte fordi der skal gemmes i EE)
 static uint8_t open_saved;                   // testbit for ikke at gemme igen i EEPROM
 static uint8_t close_saved;                  // testbit for ikke at gemme igen i EEPROM
 uint8_t new_calc = 0;                        // Controlbit for new calculation required. Shall be cleared at powerup so already learned values can be used
                                              // EE_limit_ready is also used for this purpose.
 static uint8_t enc_blinktimer;               // LED on tid under blink
 static uint8_t enc_pause1;                   // pausetid mellem LED blink
 static uint16_t enc_pause2sek;               // lang pause inden næste periode - sat til ca 1.5 sek ved prototypeprogram
 static uint16_t batt_tst_tim = 2400;         // delay time before battery voltage test 16 sek. preset by power-up
 static uint16_t enc_hide_timer = 150;        // fejl skjules mens der gemmes i EEPROM
 uint8_t e_type = sce;                        // kontrolregister for enkodertype. powerup as sce 30-04-2019 18-11-2020
 static uint8_t e_type_old;                   // kontrolregister for enkodertype
 uint8_t EE_par_11_old;												// check register for changes made 13-03-2017
 uint8_t info_by_learn;                       // special testbit for showing missing encoderanswer by learning
 uint16_t show_E09_before_reset;              // timer for showing E09 error in display 2 sec. before reset
 uint8_t position_ready;											// 0 by power-up 02-05-2017
 //*********************Singleturn speed control *****************************
 static uint8_t s_buf_ready;                  // testbit for speed ring buffer filled first time
 static uint32_t s_buf_array[6];              // Declaring of speed ringbuffer array, 6 = 6 bytes
 static uint8_t r_buf_point;                  // speed ring buffer pointer
 static uint8_t enc_speed_delay;              // local speed delay 
 static uint16_t meas_interval;               //measure intervals for singleturn speed control
 static uint8_t move_down;                    // testbit for reversering til at forhindre speedfejl ved hurtig reversering
 static uint8_t st_learn_confirm = 0;         // timer for confirm learning - no confirming by powerup
 static uint32_t e_mso;                       // move setpoint controlreg. adaptive speedcontrol
 static uint32_t e_msc;                       // move setpoint controlreg. adaptive speedcontrol
 
 uint16_t enc_speed_o;                 // adaptive speed control check reg.
 uint16_t enc_speed_c;                 // adaptive speed control check reg.
 uint8_t enc_open_cnt;                 // open counter for adaptive speed adjustment 
 uint8_t st_area_ok;                   // old sk2,0 - 0 by powerup
 uint8_t st_values_learned;            // old sk2,3 - 0 by powerup
 uint8_t st_init_learned;				       // old sk2,7 - 0 by powerup
 //
 uint8_t curtain_sw;                   // curtain switch bt ref input 30-04-2019
 extern uint8_t curtain_learn;				 // test bit for curtain lerning active 12-02-2020
 uint32_t ols_curt_rep;                // temp. reg. for curtain position saving 12-02-2020
 uint32_t curt_rep_diff;               // calculated difference between ols and wanted curtian repair position 
 uint8_t ecode7_timer2 = 30;           // Filter added on kostal mechanical fail 30-04-2019
 uint8_t ecode8_timer = 30;            // Filter added on kostal mechanical fail 30-04-2019
 //
 uint32_t area12;                      // value for area change from area 1 to area 2
 uint32_t area23;                      // value for area change from area 2 to area 3
 uint32_t area34;                      // value for area change from area 3 to area 4
 uint32_t area45;                      // value for area change from area 4 to area 5
 uint32_t area56;                      // value for area change from area 5 to area 6
 uint32_t area67;                      // value for area change from area 6 to area 7
 uint32_t area78;                      // value for area change from area 7 to area 8
 uint32_t area89;                      // value for area change from area 8 to area 9
 uint32_t area910;                     // value for area change from area 9 to area 10
 
 uint32_t enc_ols;                     // indlært åbne endestop grænse
 uint32_t enc_cls;                     // indlært lukke endestop grænse
 
 uint16_t st_ce1;                      // adaptive speed control (single turn)
 uint16_t st_ce2;                      // adaptive speed control
 uint16_t st_ce3;                      // adaptive speed control
 uint16_t st_ce4;                      // adaptive speed control
 uint16_t st_ce5;                      // adaptive speed control
 uint16_t st_ce6;                      // adaptive speed control
 uint16_t st_ce7;                      // adaptive speed control
 uint16_t st_ce8;                      // adaptive speed control
 uint16_t st_ce9;                      // adaptive speed control
 uint16_t st_ce10;                     // adaptive speed control
//
 uint16_t st_oe1;                      // adaptive speed control
 uint16_t st_oe2;                      // adaptive speed control
 uint16_t st_oe3;                      // adaptive speed control
 uint16_t st_oe4;                      // adaptive speed control
 uint16_t st_oe5;                      // adaptive speed control
 uint16_t st_oe6;                      // adaptive speed control
 uint16_t st_oe7;                      // adaptive speed control
 uint16_t st_oe8;                      // adaptive speed control
 uint16_t st_oe9;                      // adaptive speed control
 uint16_t st_oe10;                     // adaptive speed control
//
 uint16_t st_ice1;                     // adaptive speed control
 uint16_t st_ice2;                     // adaptive speed control
 uint16_t st_ice3;                     // adaptive speed control
 uint16_t st_ice4;                     // adaptive speed control
 uint16_t st_ice5;                     // adaptive speed control
 uint16_t st_ice6;                     // adaptive speed control
 uint16_t st_ice7;                     // adaptive speed control
 uint16_t st_ice8;                     // adaptive speed control
 uint16_t st_ice9;                     // adaptive speed control
 uint16_t st_ice10;                    // adaptive speed control
//
 uint16_t st_ioe1;                     // adaptive speed control
 uint16_t st_ioe2;                     // adaptive speed control
 uint16_t st_ioe3;                     // adaptive speed control
 uint16_t st_ioe4;                     // adaptive speed control
 uint16_t st_ioe5;                     // adaptive speed control
 uint16_t st_ioe6;                     // adaptive speed control
 uint16_t st_ioe7;                     // adaptive speed control
 uint16_t st_ioe8;                     // adaptive speed control
 uint16_t st_ioe9;                     // adaptive speed control
 uint16_t st_ioe10;                    // adaptive speed control
 uint16_t speed_unit;                  //
 uint16_t sense;                       // 
 uint16_t st_i_speedsetp;              //   
 uint16_t st_speedsetp;                //
 //
 uint32_t learn_pos;                   // for learning routine 30-04-2019
 //
 uint16_t enc_select_tim = 120;		     // timer for aut. encoder type setup - 0.8 sec 30-04-2019	 note that down count starts when main while cycle is running
 uint8_t enc_select_ok;                // 0 by powerup 30-04-2019
 //
 uint8_t show_photo_dis_point;         // testbit for showing in display 30-04-2019
 uint16_t chk_weld_value;              // check weld value for different encoders 14-08-2019
 uint16_t pwr_timer_SER = 757;        // special powerup timer til clearing af SER fejl efter 5 sek. hvis mellem endestop PU 14-08-2019
 uint8_t sce_bat_state;			          // SCE enkoder battery state. 0 = OK, 1 = below alarm level, 2 = to low level 05-01-2021
 uint8_t sce_fail_count;              // sce fail filter 05-01-2021
 uint8_t encod_safe;                  // sce power fail observing 05-01-2021
 //
 uint8_t door_pos_half;               // test bit for half open 24-11-2021
 uint8_t door_pos_high;               // test bit for door above half open 24-11-2021
 uint8_t door_pos_low;                // test bit for door below half open 24-11-2021
 uint8_t h_pos_save_ready = 0;        // test bit for door below half open 24-11-2021
 extern uint8_t dock_54_active;       // controlbit for active dock leveler control with half open function 24-11-2021
 uint8_t car_wash_active = 0;         // test bit for car wash function active above half open 02-12-2021
 uint8_t save_h_open_learn = 0;       // test bit for half open learning setpoint 24-11-2021
 uint32_t h_open_pos;                 // learned half open position 18-05-2021
 uint32_t h_open_pos_max;             // learned half open position 18-05-2021
 uint32_t h_open_pos_min;             // learned half open position 18-05-2021
 uint32_t ls_h_o_opening_setp;        // calculated low speed hafl open opening set point 13-02-2026
 uint32_t ls_h_o_closing_setp;        // calculated low speed hafl open opening set point 13-02-2026
 //
 uint8_t encoder_errors;              // summerized encoder error 26-11-2021
 //
 uint32_t bef_close_lim;              // before close limit setpoint for AUX relays 02-12-2021
 uint32_t bef_open_lim;               // before open limit setpoint for AUX relays 02-12-2021
 uint8_t bol_on = 0;                  // before open limit activated 01-12-2021
 uint8_t bcl_on = 0;                  // before open limit activated 01-12-2021
 uint8_t bl_close_setp;               // variabel before limit close setpoint 03-12-2021
 uint8_t bl_open_setp;                // variabel before limit close setpoint 03-12-2021
 //
 uint32_t cls_comp;              // used for cls limit calculation 03-12-2021
 uint32_t inv_num_0_value;       // 03-12-2021
 uint32_t inv_num_1_value;       // 03-12-2021
 uint32_t inv_num_2_value;       // 03-12-2021
 uint32_t inv_num_3_value;       // 03-12-2021
 uint32_t inv_num_4_value;       // 03-12-2021
 uint8_t inv_meas_num = 0;       // 03-12-2021
 uint8_t inv_meas_interval = 31; // 03-12-2021
 uint8_t inv_meas_ok = 0;        // 03-12-2021
 uint8_t inv_low_sp = 0;         // 03-12-2021
 uint8_t stop_old = 0;           // used for disable SER function 25-01-2022
 uint8_t no_move = 1;            // used for disable SER function 25-01-2022
 uint8_t enable_ser;             // disable SER function 25-01-2022
 uint8_t run_prog_old = 1;       // must be 1 for not resetting welding by powerup if prog is selected
 extern uint8_t inverter_use;		 // 1 = inverter use is active 26-01-2022
 uint8_t tacho_fail = 0;         // when tacho is in use for welding check with inverter 27-01-2022
 uint8_t direc = 0;              // testbit for welding 27-01-2022
 uint8_t direc_old = 0;          // testbit for welding 27-01-2022
 uint8_t diff_moni_old_saved = 0; // testbit for welding 27-01-2022
 uint32_t diff_moni;             // difference between positions used in SER noise filter 27-01-2022
 uint32_t diff_moni_old;         // difference between positions used in SER noise filter 27-01-2022
 uint8_t learn_h_open_active = 0; // test bit for half open learning setpoint 09-12-2025
//*******************************************************************************************************
void encoder()
{
 curtain_check(); // 30-04-2019
 //	
 if (e_type == 0x01) chk_weld_value = 13; // er Kostal enkoder valgt 14-08-2019 28-03-2023
 else chk_weld_value = 300; // Dal or Feig encoder selected
 //	
 EE_read(EE_par_11);  // adjust E_limit
 if (temp == 7)
  {
   E_limit = 0; // 24-01-2022
	 EE_read(EE_par_26);	// 22-11-2023
	 if (temp != 1) EE_write(EE_par_26, 1);  // set this to 1 for mechanical limits	
   EE_read(EE_par_27);	// 22-11-2023
	 if (temp != 1) EE_write(EE_par_27, 1);  // set this to 1 for mechanical limits			
	}
 else E_limit = 1;	
 if (E_limit == 1)  // 
  {	
	 if (ecode4 == 1) // 16-02-2017
	  {
     EE_read(EE_ecode4);
     if (temp == 0) EE_write(EE_ecode4,1);	// adjust in EEPROM if necsesary 14-02-2017 - Calc error limits
	  }
   else
	  {
		 EE_read(EE_ecode4);
     if (temp == 1) EE_write(EE_ecode4,0);	// adjust in EEPROM if necsesary 14-02-2017
	  }
	 //	
	 if ((enc_select_tim == 0) || (enc_select_ok == 1))
    {
	   EE_read(EE_par_11);	
     if (e_type == kostal)	
	    {
       if ((temp == 5) || (temp == 6))
		    {
		    }
       else EE_write(EE_par_11,5);			
		  }
     else if (e_type == sce)
		  {
			 if ((temp == 8) || (temp == 9))
		    {
		    }
       else EE_write(EE_par_11,8);
		  }
		 else	 
	    {
		   if (((temp == 1) || (temp == 2)) || ((run_prog == 1) && (parameter == 11)))
		    {
		    }
       else EE_write(EE_par_11,1);			
	    }
	  }	
	 //	
 if (e_type != e_type_old)
  {
	 if (e_type == 1)
	  {
		 huart2.Init.Parity = UART_PARITY_EVEN;
		 huart2.Init.WordLength = UART_WORDLENGTH_9B;
		 HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0);	// this must be made manually with STM32F407 02-11-2021	
    }
	 else if (e_type == 4) // probebly the same as Dal encoder but if not it is easy to change
	  {
		 huart2.Init.Parity = UART_PARITY_NONE;
		 huart2.Init.WordLength = UART_WORDLENGTH_8B;
     HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0);	// this must be made manually with STM32F407 02-11-2021			
	  }	
	 else
	  {
		 huart2.Init.Parity = UART_PARITY_NONE;
		 huart2.Init.WordLength = UART_WORDLENGTH_8B;
     HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0);	// this must be made manually with STM32F407 02-11-2021			
	  }
  }
 e_type_old = e_type;	
 	 
 EE_read(EE_par_41); //is Electronic limit wanted
 if (temp == 4) torque_singleturn = 1; // speed singleturn adaptiv selected			
//********************************************************************************		
	 
	 //	
	 if (en_svar == 1)
	  {	
		 if (enc_select_tim < 105) // to prevent false encoder change by powerup first 100mS (noise) 13-04-2018 30-04-2019
		 {			 
		  enc_select_ok = 1; // encoder has been observed
		 }	 
     if (position_ready < 2) position_ready++; // position i received after power-up 04-05-2017			
		 EE_read(EE_learning_ok);	// 06-01-2017
		 if (temp == 1)
		  {
			 EE_read(EE_right_turn);	
       if (temp == 1) // right turn is measured
			  {
				 EE_read(EE_par_11);	
				 switch(temp)
				  {
					 case 1:
						// do noting Dal right turn is already active 
					  break;
					 case 2:
						EE_write(EE_par_11, 1);  // switch back to Dal encoder right turn 
					  break;
					 case 3:
						// do noting, not in use
					  break;
					 case 4:
						// do noting, not in use
					  break;
					 case 5:
						// do noting Kostal right turn is already active 
					  break;
					 case 6:
						EE_write(EE_par_11, 5);  // switch back to Kostal encoder right turn 
					  break;
					 case 8:
						// do noting SCE right turn is already active 18-11-2020
					  break;
					 case 9:
						EE_write(EE_par_11, 8);  // switch back to SCE encoder right turn 18-11-2020
					  break;
					 default:
						 break; // should not be possible
					}
			  }
       else // left turn is measured
			  {
         EE_read(EE_par_11);	
				 switch(temp)
				  {
					 case 1:
						EE_write(EE_par_11, 2);  // switch back to Dal encoder left turn 
						// do noting Dal right turn is already active 
					  break;
					 case 2:
						// do noting Dal left turn is already active 
						break;
					 case 3:
						// do noting, not in use
					  break;
					 case 4:
						// do noting, not in use
					  break;
					 case 5:
						EE_write(EE_par_11, 6);  // switch back to Kostal encoder left turn  
						// do noting Kostal right turn is already active 
					  break;
					 case 6:
						// do noting Kostal left turn is already active
						break;
					 case 8:
						EE_write(EE_par_11, 9);  // switch back to SCE encoder left turn  18-11-2020
						// do noting Kostal right turn is already active 
					  break;
					 case 9:
						// do noting SCE left turn is already active 18-11-2020
						break;
					 default:
						 break; // should not be possible
					}
			  }				 
		  }			 	
//		
		 ecode1 = 0; // update is finish because en_svar == 1 is set by callback routine
		 info_by_learn = 0; // also special check bit clear 13-03-2017			
   	 save_new_pos_bat();
		 learn_pos = position; // save for learning routine 30-04-2019	 
		 enc_test_moving();	
		 setup_controlbits();	// 30-04-2019 moved
     read_inputs();
		 //
     if (programming_mode == 1) stop_tim = 150; // 1 s. delay til stop ved skift til el.endestop, ved manglende svar eller endestop ikke indlært
     if ((e_type == 0) && (pwr_timer == 0)) // only dalmatic encoder have battery test feature
      {
       if ((ols_out == 1) || ((e_tst_running == 3) && (batt_tst_tim < 2))) // is ols active 26-03-2010 or special encoder test running 21-12-2011
        {
				 //ols_old = 1; // debug - skal fjernes efter debug enkoder timing skal rettes
         if ((ols_old == 1) | ((programming_mode == 1) & (s_encoder_tst == 0))) 
				  { 
				   ols_old = 0;
           bat_req_prepare = 0; // request position prepare in next TX	
				  }	 
         else
          {
           low_bat_filt++;
           ols_old = 1; // 
           //read_bat = 1; // set en save_new_pos_bat
           bat_req_prepare = 1;// request battery prepare
          }
        }
       else
        {
         ols_old = 0;
         bat_req_prepare = 0;	// request position prepare in next TX
        }
       bat_test(); // 
      }
     else
		  {
			 bat_req_prepare = 0;	// request position prepare in next TX			
		  }
     en_svar = 0; // kontrolbit cleares by each encoder request
		 //
    co_buffer(); // kopier buffer til buffer2 - kun 2 element som er LSB
    clr_buffer(); // nulstil buffer klar til nye data 
    tim_out_cnt = 10;
	 }
  else
   {
		EE_read(EE_right_turn); //be sure that right_turn is updated to prevent out of range fail 20-04-2017
    if (temp == 1) right_turn = 1;
    else right_turn = 0; 
		//		 
    if (tim_out_cnt == 0) ecode1 = 1; // there is timer out fail
		if (ecode1 == 1) ecode7_timer = 30; // 200mS preset filtertimer of range fail so disconnecting not gives out of range fail
                                        // when reconnected 25-01-2017 
		
		if (enc_select_ok == 1) // If 0, keep the observed encoder after powerup after 1 sec. 
		 {
     }
    else if (enc_select_tim > 110) // keep sce enkoder 66mS after powerup 120-110 = 10 x 6,66mS 18-11-2020 - requires no request
		 {
		 }
		else if (enc_select_tim > 30) // Kostal encoder trial req 66mS to 600mS after powerup 
		 {
			e_type = kostal; 
		 }
		else 
		 {
			e_type = dal;  // use then dal 18-11-2020 
		 } 
	 } 
	// 
	if (tim_out_cnt == 0) ecode1 = 1; // there is timer out fail 
	if (ecode1 == 1) info_by_learn = 1; // 13-03-2017
  else info_by_learn = 0;		 
	if (ecode1 == 1) ecode7_timer = 30; // 200mS preset filtertimer of range fail so disconnecting not gives out of range fail
                                      // when reconnected 26-01-2017  
  enc_mtimer(); // opdatering af multitimere
  // 
  //error_codes(); // blink med LED hvis der er nogle fejl not in use anymore with display
	if ((ecode1 == 0) && (tim_out_cnt < 9)) 
	 {
		HAL_Delay(0); // position is not new and correct for further calculation wait for new
	 }
	else
	 {
    if (version == 1) chk_weld(); // Check for welded contactors 26-09-2011 05-12-2011
    else ecode3 = 0; // version 0 selected
		//****************************************** 
		EE_read(EE_par_11); // 13-03-2017 by parameter 11 change, new limits must be learned both
    if (temp != EE_par_11_old)		 
		 {
      if (new_limits == 1)
       {
       }
      else
       {
        EE_write(EE_limit_ready,255); //værdier ikke klar til kørsel
        new_limits = 1;
        EE_write(EE_o_limit_ok, 255); // open limit er nu ikke klar  
        EE_write(EE_c_limit_ok, 255); // open limit er nu ikke klar  
        moving_fail = 0; // moving fejl cleares også ved factory reset ellers kan ikke køres altid
        EE_write(EE_moving_fail, 0); // gem også i EE
        EE_write(EE_photo_pos_saved, 0);
        calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 26-10-2011
        EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 26-10-2011
				ecode4 = 0; // 13-03-2017 
       }
		 }
		EE_read(EE_par_11); 
    EE_par_11_old = temp; 		 
		//******************************************** 
    //******* Factory clear ********************
    if (fac == 1)
     {
      if (new_limits == 1)
       {
       }
      else
       {
        if (fac_clr_tim == 0)
         {
          EE_write(EE_limit_ready,255); //værdier ikke klar til kørsel
          new_limits = 1;
          EE_write(EE_o_limit_ok, 255); // open limit er nu ikke klar  
          EE_write(EE_c_limit_ok, 255); // open limit er nu ikke klar  
          moving_fail = 0; // moving fejl cleares også ved factory reset ellers kan ikke køres altid
          EE_write(EE_moving_fail, 0); // gem også i EE
          EE_write(EE_photo_pos_saved, 0);
          calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 26-10-2011
          EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 26-10-2011
					ecode4 = 0; // 13-03-2017 
         }
		   }
     }
    else fac_clr_tim = 30; // 200 mS filter mtimer is runnning in 6.66 mS cycles
//**********************************************	
    if (ecode1 == 1) timeout_fail(); // var det time out som brød venteløkken 28-08-2012
    else
     {
			if (position_ready > 1) // 02-05-2017
			 {
        ecode1 = 0; // 
        if (batt_tst_tim !=0) batt_tst_tim--; // 21-12-2011 when encoder is answering downcount for batt measure ready
        //  crc = buffer[0] ^ buffer[1]; // beregning af XOR checksum
        //  crc = crc ^ buffer[2]; der arbejdes uden CRC indtil videre
        //  crc = crc ^ buffer[3];
        //   if (buffer[4] == crc)
        // {
        ecode5 = 0; // clear fejlkode 5 kom. fejl
				if ((save_h_open_learn == 1) || (learn_h_open_active == 1)) norm_mode(); //31-05-2021 09-12-2025 
        else if (programming_mode == 1) prog_mode(); // 
        else norm_mode();
        // }
        //  else
        //  {
        // debug - ecode5 = 1; // sæt fejlkode 5 kom. fejl
        //  }
			 }	
      else
			 {	
        // not received 2 positions after powerup yet 02-05-2017 
       }
	   }
	 }
 } 
 else
 {
  // disable 6.5mS req interrupt timer !! skal laves
	tim_out_cnt = 10;
	clear_fails(); 
  // future purpose	 
 }
 
}
//
void st_clr_val()
{
 // disable_interrupts(INT_TIMER1); // så der ikke kommer interruptfejl under clearing - fjernet ved ARM
 enc_speed_o = 0;
 enc_speed_c = 0;
 st_ce1 =65535;
 st_ce2 =65535;
 st_ce3 =65535;
 st_ce4 =65535;
 st_ce5 =65535;
 st_ce6 =65535;
 st_ce7 =65535;
 st_ce8 =65535;
 st_ce9 =65535;
 st_ce10 =65535;
 st_oe1 =65535;
 st_oe2 =65535;
 st_oe3 =65535;
 st_oe4 =65535;
 st_oe5 =65535;
 st_oe6 =65535;
 st_oe7 =65535;
 st_oe8 =65535;
 st_oe9 =65535;
 st_oe10 =65535;
 st_area_ok = 0;                // old sk2,0
 st_values_learned = 0;         // old sk2,3
 st_wear_observed = 0;          // old sk2,6
 st_init_learned = 0;					  // old sk2,7
 st_ice1 =65535;
 st_ice2 =65535;
 st_ice3 =65535;
 st_ice4 =65535; 
 st_ice5 =65535;
 st_ice6 =65535;
 st_ice7 =65535;
 st_ice8 =65535;
 st_ice9 =65535;
 st_ice10 =65535;
 st_ioe1 =65535;
 st_ioe2 =65535;
 st_ioe3 =65535;
 st_ioe4 =65535;
 st_ioe5 =65535;
 st_ioe6 =65535;
 st_ioe7 =65535;
 st_ioe8 =65535;
 st_ioe9 =65535;
 st_ioe10 =65535;
 speed_clr = 1; // kontrolbit for ny indlæring ønsket
 st_speed_learn_ok = 0;
 //
}
//***********************************************************

//**********************************************************
// flyttet til MODBUS.c void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (huart->Instance == USART2) // was it uart 2 rx call back
// 	 {
//	  rx_buffer[0] = uart2_buf[0]; // copy array
//    rx_buffer[1] = uart2_buf[1];
//    rx_buffer[2] = uart2_buf[2];
//	  rx_buffer[3] = uart2_buf[3];
//	  rx_buffer[4] = uart2_buf[4];
//    en_svar = 1; // 5 byte are received
//		
//	 }
//	else
//	 {
//	 }
//}	
//***********************************************************
// *************** RS485 end ********************************
//

//************************************************************************************************

void enc_mtimer() // multitimer function can be used when fixed program cycletime 3.33 ms but encoder routine
	                // is only runned half only cause of slow encoder resoends time. 
{
 if (pwr_timer !=0) pwr_timer--;
 if (enc_blinktimer !=0) enc_blinktimer--;
 if (enc_pause1 !=0) enc_pause1--;
 if (enc_pause2sek !=0) enc_pause2sek--;
 if (ecode7_timer !=0) ecode7_timer--;
 if (limit_indi !=0) limit_indi--;
 if (stop_tim !=0) stop_tim--;
 if (moving_timer !=0) moving_timer--;
 if (enc_hide_timer !=0) enc_hide_timer--;
 if (fac_clr_tim !=0) fac_clr_tim--; // 20-11-09
 if (enc_speed_delay !=0) enc_speed_delay--;
 if (meas_interval !=0) meas_interval--;
 if (st_learn_confirm !=0) st_learn_confirm--;
 if (pc_tim !=0) pc_tim--; // 26-09-2011
 if (d_stopped_tim !=0) d_stopped_tim--; // 26-09-2011
 if (ecode3_tim !=0) ecode3_tim--; // 26-09-2011
 if (tim_out_cnt !=0) tim_out_cnt--; // 13/5-2016
 if (show_E09_before_reset !=0) show_E09_before_reset--; // 22-03-2017
 if (enc_select_tim !=0) enc_select_tim--; // 30-04-2019	
}
void read_inputs()
{
 //
 if ((save_active_learning_limit == 1) & (e_limit_open_learning_active == 1)) save_open = 1; // 
 else save_open = 0;
 if ((save_active_learning_limit == 1) & (e_limit_close_learning_active == 1)) save_close = 1;
 else save_close = 0;
 if ((save_active_learning_limit == 1) & (e_limit_curt_rep_learning_active == 1)) save_curtain = 1;
 else save_curtain = 0;	
 //
 if (((e_limit_open_learning_active == 1) || (e_limit_close_learning_active == 1)) & 
	 (!save_active_learning_limit)) learn_active = 1;
 else learn_active = 0;
 //
 EE_read(EE_par_11); //is Electronic limit wanted
 if ((temp == 1)|| (temp == 3) || (temp == 5) || (temp == 8)) right_turn = 1; // 05-01-2020 24-01-2922 7 shall not be there
 else right_turn = 0;
 //
}
//
void clr_buffer() //nulstil buffer
{
uint8_t i;
for (i=0;i<=4;i++) pack_3a[i]=0; // 14-01-2021
}
//
void co_buffer() // kopier buffer array element 2 i array kopieres for at test bevægelse
{
rx_buffer2 = pack_3a[1]; // 14-01-2021
}
//
void calc_limit_win()  
 {
  limit_win = run_range/8;
  ex_limit_win = run_range/128; // extra øgning af limit window for at kompensere for overløb - 290108
  fine_adj_unit = ex_limit_win/2; // fin justerings enheder er run_range/256
  //
  data = fine_adj_unit; //  værdi skrives til EEPROM moved 13-02-2026
  adr = EE_fine_adj_unit;
  write_int32_eeprom();
//
  data = ex_limit_win; //  værdi skrives til EEPROM 13-02-2026 saved for later use
  adr = EE_ex_limit_win;
  write_int32_eeprom();
// 
  data = limit_win; // - 01-09-09
  adr = EE_limit_win;
  write_int32_eeprom();
  data = run_range; // - 04-02-2010
  adr = EE_run_range;
  write_int32_eeprom();
  p5_run_range = run_range/20; // calculate for 5% of run_range 27-10-2011
  data = p5_run_range; // 
  adr = EE_p5_run_range;
  write_int32_eeprom();  
 } 
//
void save_ls_o_setp() // til brug ved low speed udgang på hovedprocessor
 {
  read_int32_eeprom(EE_ex_limit_win); // get value for calc 13-02-2026
  ex_limit_win = data; 
  enc_hide_timer = 150; // skjul LED fejlvisning 1 sek.
	EE_write(EE_open_ls_setp, open_ls_setp);
  if (right_turn == 1) // er højredrejning valgt
   {
    read_int32_eeprom(EE_ols_min); 
    ols_min = data;
    data = (ols_min - ((run_range/100) * open_ls_setp)); // low speed ca 10 -40% fra OLS 20-01-2010
    adr = EE_open_low_sp;
    write_int32_eeprom();
    //
    temp = EE_read(EE_h_open_pos_saved); // check whether half open position is saved 13-02-2022     
    if (temp == 0)
     {
      read_int32_eeprom(EE_h_open_pos); // must be recalculated when open_ls_setp is changed 13-02-2026
      h_open_pos = data;
      data = (h_open_pos - (4 * ex_limit_win) - (run_range/100 * open_ls_setp)); // must be the same as in chk_half_open routine
      if ((data < cls_max) || (data > 500000000)) data = cls_min; // overlap CLS adjust to CLS
      adr = EE_ls_h_o_opening_setp;
      write_int32_eeprom(); 
     }
   }
  else // left turning selected
   {
    read_int32_eeprom(EE_ols_max); 
    ols_max = data;
    data = (ols_max + ((run_range/100) * open_ls_setp)); // low speed ca 10 - 40% fra OLS 20-01-2010
    adr = EE_open_low_sp;
    write_int32_eeprom();
    //
    temp = EE_read(EE_h_open_pos_saved); // check whether half open position is saved 13-02-2022
    if (temp == 0)
     {
      read_int32_eeprom(EE_h_open_pos); // must be recalculated when open_ls_setp is changed 13-02-2022
      h_open_pos = data;
      data = (h_open_pos + (4 * ex_limit_win) + (run_range/100 * open_ls_setp));
      if (data > cls_min) data = cls_max; // overlap CLS adjust to CLS 
      adr =EE_ls_h_o_opening_setp;
      write_int32_eeprom();
     }  
   }
  }
//
void save_ls_c_setp() // til brug ved low speed udgang på hovedprocessor
 {
  read_int32_eeprom(EE_ex_limit_win); // get value for calc 13-02-2026
  ex_limit_win = data; 
  enc_hide_timer = 150; // skjul LED fejlvisning 1 sek.
  EE_write(EE_close_ls_setp,close_ls_setp);
  if (right_turn == 1) // er højredrejning valgt
   {
    save_ls_o_setp(); // function for saving low speed open setpoint 
    read_int32_eeprom(EE_cls_max); 
    cls_max = data;
    data = (cls_max + ((run_range/100) * close_ls_setp)); // low speed ca 10 - 40% fra cls
    adr = EE_close_low_sp;
    write_int32_eeprom();
    //
    temp = EE_read(EE_h_open_pos_saved); // check whether half open position is saved 13-02-2022
    if (temp == 0)
     {
      read_int32_eeprom(EE_h_open_pos); // must be recalculated when open_ls_setp is changed 13-02-2022
      h_open_pos = data;
      data = (h_open_pos + (4 * ex_limit_win) + (run_range/100 * close_ls_setp));
      if (data > ols_min) data = ols_max; // overlap OLS adjust to OLS
      adr =EE_ls_h_o_closing_setp;
      write_int32_eeprom();
     }     
   }
  else // left turning selected
   {
    read_int32_eeprom(EE_cls_min); 
    cls_min = data;
    data = (cls_min - ((run_range/100) * close_ls_setp)); // low speed ca 10.0 - 40% fra cls
    adr = EE_close_low_sp;
    write_int32_eeprom();
    //
    temp = EE_read(EE_h_open_pos_saved); // check whether half open position is saved 13-02-2026
    if (temp == 0)
     {
      read_int32_eeprom(EE_h_open_pos); // must be recalculated when open_ls_setp is changed 13-02-2026
      h_open_pos = data;
      data = (h_open_pos - (4 * ex_limit_win) - (run_range/100 * close_ls_setp));
      if (data > ols_max) data = ols_min; // overlap OLS adjust to OLS
      adr =EE_ls_h_o_closing_setp;
      write_int32_eeprom();
     } 
   }
 }
//
void save_bl_close_setp() // 15-06-2020
 {
  hide_timer = 154; // skjul LED fejlvisning 1 sek.
  EE_write(EE_bl_close_setp,bl_close_setp);
  if (right_turn == 1) // er højredrejning valgt
   {
    //bl_open_setp(); // function for saving low speed open setpoint !OBS Hvorfor denne
    read_int32_eeprom(EE_cls_max); 
    cls_max = data;
    data = (cls_max + ((run_range/100) * bl_close_setp)); //  5 - 95% fra cls 12-11-2018 04-02-2022
    adr = EE_bef_close_lim;
    write_int32_eeprom();
   }
  else // left turning selected
   {
    read_int32_eeprom(EE_cls_min); 
    cls_min = data;
    data = (cls_min - ((run_range/100) * bl_close_setp)); // low speed ca 5.0 - 95% fra cls 12-11-2018 04-02-2022
    adr = EE_bef_close_lim;
    write_int32_eeprom();
   }
 }
void save_bl_open_setp() // 15-06-2020
 {
  hide_timer = 154; // skjul LED fejlvisning 1 sek.
  EE_write(EE_bl_open_setp,bl_open_setp);
  if (right_turn == 1) // er højredrejning valgt
   {
    read_int32_eeprom(EE_ols_min); 
    ols_min = data;
    data = (ols_min - ((run_range/100) * bl_open_setp)); // low speed ca 10 -95% fra OLS 20-01-2010 04-02-2022
    adr = EE_bef_open_lim;
    write_int32_eeprom();
   }
  else // left turning selected
   {
    read_int32_eeprom(EE_ols_max); 
    ols_max = data;
    data = (ols_max + ((run_range/100) * bl_open_setp)); // low speed ca 10 - 95% fra OLS 20-01-2010 04-02-2022
    adr = EE_bef_open_lim;
    write_int32_eeprom();
   }
 }
//
void calc_cls_comp() // 18-06-2021
{
 // For the kostal encoer the maximum useable range is approximately 315 angle degrees because there must be some place for two limit windows.
 // Limit window are normalle 45 degrees but half window is also useable.
 // 315 degrees is 7000 counts for the Kostal encoder.
 if ((e_type == 0x01) && (condoor == 1)) // is Kostal enkoder selected and Condoor valgt i define.h 
  {
   if (run_range < 1750) cls_comp = (ex_limit_win + ex_limit_win + fine_adj_unit); // 1,95% subtracted
   if ((run_range > 1749) && (run_range < 3500)) cls_comp = (ex_limit_win + ex_limit_win); // 1,56% subtracted
   if ((run_range > 3549) && (run_range < 5250)) cls_comp = (ex_limit_win + fine_adj_unit); // 1,17% subtracted
   if (run_range > 5249)  cls_comp = (ex_limit_win); // 0,78% subtracted
  }
 else // by not kostal enkoder
  {
   cls_comp = (ex_limit_win + ex_limit_win); // 1,56% subtracted
  }
} 
 
// 
void calc_windows()
{
 //
  moving_fail = 0; // moving fejl cleares kun ved ny indlæring.
	read_int32_eeprom(EE_enc_ols); // get old learned values when curtain rep 12-02-2020
	enc_ols = data;
	read_int32_eeprom(EE_enc_cls); // get old learned values when curtain rep 12-02-2020
	enc_cls = data;
//	
  EE_write(EE_moving_fail, 0); // gem også i EE
//
  if (right_turn == 1) // er højredrejning valgt
 {
	if (ols_curt_rep > enc_ols) curt_rep_diff = ols_curt_rep - enc_ols; //12-02-2020
	else 
	{
   if (e_type == 0x01) // er kostal enkoder valgt
    curt_rep_diff = (maxposK - enc_ols) + ols_curt_rep;
   else
    curt_rep_diff = (maxpos - enc_ols) + ols_curt_rep;
	} 
	//
	if (enc_ols > enc_cls) // ja, kontroller at ols er størst så efterfølgende beregning ikke bliver negativ
  run_range = enc_ols - enc_cls;
  else
  {
  if (e_type == 0x01) // er kostal enkoder valgt
    run_range = (maxposK - enc_cls) + enc_ols;
  else
    run_range = (maxpos - enc_cls) + enc_ols;
  }
  calc_limit_win(); //
//
  data = fine_adj_unit; //  værdi skrives til EEPROM
  adr = EE_fine_adj_unit;
  write_int32_eeprom();
//
  // Test for fejl i omdrejningsopsætning Feig og Dal enkoder er lavet lidt speciel fordi der ikke
  // kan kontrolleres for værdier større end 32 bit. Der bruges derfor limit_win som er en 1/8 af
  // run_range
  if ((e_type == 0x00) || (e_type == 0x04)) // er Dal Feig eller SCE enkoder valgt 05-01-2021
   {
    limit_win_tst = limit_win/8;
    if ((limit_win + limit_win_tst + limit_win_tst) > DIL_tst_value)
    ecode4=1;
    else
    ecode4=0;
   }
  else // Kostal
	 {
		if ((run_range > 7050) || (run_range < 650)) // 16-02-2017
		 {
			ecode4=1;
		 }	 
    else
     ecode4=0;
	 }
  if (enc_cls > limit_win)
  {
  offset = enc_cls - limit_win;
  offset_sign = 0; // ofsetværdi skal da trækkes fra
  }
  else
  {
  offset = limit_win - enc_cls;
  offset_sign = 1;
  }
	//led_speed_on;	//debug
  data = offset; // offset værdi skrives til EEPROM
  adr = EE_offset;
  write_int32_eeprom();
  //
  EE_write(EE_offset_sign, offset_sign);
  //
  data = 0; // CLS min værdi skrives til EEPROM
  adr = EE_cls_min;
  write_int32_eeprom();
  //
  data = (limit_win + ex_limit_win); // OLS max værdi skrives til EEPROM - 290108
  adr = EE_cls_max;
  write_int32_eeprom(); 
  //
  data = (run_range + limit_win - ex_limit_win); // - 290108
  adr = EE_ols_min;
  write_int32_eeprom();
  //
	data = ((run_range + (limit_win * 2)) - (2 * ex_limit_win)); // - 30-04-2019, changed to curtain learn limit 12-02-2020
  adr = EE_ols_min_curt_learn;
  write_int32_eeprom();
	//
	data = (run_range + (limit_win - ex_limit_win) + curt_rep_diff); // 12-02-2020
	adr = EE_ols_curt_rep;
  write_int32_eeprom(); // save value for curtain rep	 
	//
  data = ((limit_win * 2) + run_range); // OLS max værdi skrives til EEPROM
  adr = EE_ols_max;
  write_int32_eeprom();
  //
	//led_speed_off;	//debug
  // *** for low speed deceleration 30-06-09 **************************
  save_ls_o_setp(); // function for saving low speed open setpoint 
  save_ls_c_setp(); // function for saving low speed close setpoint
	save_bl_close_setp(); // genberegn når endestop ændrer sig 11-12-2025_d
	save_bl_open_setp(); // genberegn når endestop ændrer sig 11-12-2025_d
	// *********************************************************
  //
  EE_write(EE_limit_ready, 0); //værdier klar til kørsel
  new_calc = 0; // værdier er gemt
 }
  else  // left direction 
  {
	if (ols_curt_rep < enc_ols) // 12-02-2020
  curt_rep_diff = enc_ols - ols_curt_rep;
  else
  {
   if (e_type == 0x01) // er kostal enkoder valgt
   curt_rep_diff = (enc_ols + (maxposK - ols_curt_rep ));
   else
   curt_rep_diff = (enc_ols + (maxpos - ols_curt_rep));
  }
  //		
	if (enc_cls > enc_ols) // left direction, kontroller at cls er størst så efterfølgende beregning ikke bliver negativ
  run_range = enc_cls - enc_ols;
  else
  {
   if (e_type == 0x01) // er kostal enkoder valgt
   run_range = (enc_cls + (maxposK - enc_ols));
   else
   run_range = (enc_cls + (maxpos - enc_ols));
  }
  calc_limit_win(); //
//
  data = fine_adj_unit; //  værdi skrives til EEPROM
  adr = EE_fine_adj_unit;
  write_int32_eeprom();
//  
  // Test for fejl i omdrejningsopsætning Feig og Dal enkoder er lavet lidt speciel fordi der ikke
  // kan kontrolleres for værdier større end 32 bit. Der bruges derfor limit_win som er en 1/8 af
  // run_range
  if (e_type == 0x00) // er Dall eller Feig enkoder valgt samme dataformat
   {
    limit_win_tst = limit_win/8;
    if ((limit_win + limit_win_tst + limit_win_tst) > DIL_tst_value)
    ecode4=1;
    else
    ecode4=0;
   }
  else // Kostal
   {
	 if ((run_range > 7050) || (run_range < 650)) // 16-02-2017
		{
		 ecode4=1;
		}	 
   else
    ecode4=0;	
   }
  //
  if (enc_ols > limit_win)
  {
  offset = enc_ols - limit_win;
  offset_sign = 0; // ofsetværdi skal da trækkes fra
  }
  else
  {
  offset = limit_win - enc_ols;
  offset_sign = 1;
  }
  //  
  data = offset; // offset værdi skrives til EEPROM
  adr = EE_offset;
  write_int32_eeprom();
  //
  EE_write(EE_offset_sign, offset_sign);
  //
  data = 0; // OLS min værdi skrives til EEPROM
  adr = EE_ols_min;
  write_int32_eeprom();
  //
  data = (limit_win + ex_limit_win); // OLS max værdi skrives til EEPROM - 290108
  adr = EE_ols_max;
  write_int32_eeprom(); 
  //
	data = (2 * ex_limit_win); // - 30-04-2019
  adr = EE_ols_max_curt_learn; // changed to curtain learn limit 12-02-2020
  write_int32_eeprom();
	//
	data = ((limit_win + ex_limit_win) - curt_rep_diff); // 12-02-2020
	adr = EE_ols_curt_rep;
  write_int32_eeprom(); // save value for curtain rep	 
	//
  data = (run_range + limit_win + limit_win); // 
  adr = EE_cls_max;
  write_int32_eeprom();
  //
  data = (run_range + limit_win - ex_limit_win); // - 290108
  adr = EE_cls_min;
  write_int32_eeprom();
  //
  // *** for low speed deceleration 30-06-09 **************************
  save_ls_o_setp(); // function for saving low speed open setpoint
  save_ls_c_setp(); // function for saving low speed close setpoint
	save_bl_close_setp(); // genberegn når endestop ændrer sig 11-12-2025_d
	save_bl_open_setp(); // genberegn når endestop ændrer sig 11-12-2025_d
  // *********************************************************
  EE_write(EE_limit_ready, 0); //værdier klar til kørsel
  new_calc = 0; // værdier er gemt
  }
 }

void limit_calc() // Beregningsrutine for vejlængde, limit grænseværdier og offsetværdi 
{
 if (new_calc == 1)
 {
  EE_read(EE_c_limit_ok); // 13-03-2017
	temp1 = temp; 
  EE_read(EE_o_limit_ok);
  if ((temp == 0) && (temp1 == 0)) // er begge endestop gemt mindst 1 gang
  {
  calc_windows(); 
  calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 26-10-2011
  EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 26-10-2011
  }
 }
} 
//
void load_spec_encoder_test()
{
 if (ecode1==1) spec_encoder_test = 1; //12-01-2012
 else if (e_mov_ok == 0) spec_encoder_test = 3;
 else if (bat_value < 681) spec_encoder_test = 2;
 else // adjust battery voltage for display showing on uP1 
  {
   spec_encoder_test = (bat_value - 638);
   spec_encoder_test = spec_encoder_test/2;
   spec_encoder_test = spec_encoder_test - 4;
  }
}
//
void encoder_chk() // 21-12-2011
{
 if (e_tst_running < 4)
  {
   if (e_tst_running > 0)
    {
     if ((e_tst_running > 1) && (!open_pb) && (!open_disp_pb) && (!close_pb) && (!close_disp_pb)) // e_tst_running >1 and no OPEN or CLOSE P/B
      {
       if (e_tst_running == 3)
        {
        }
       else
        {
         e_chk_pos_2 = position; // save 2. position value         
         if (e_chk_pos_2 > e_chk_pos_1)
          {
           if ((e_chk_pos_2 - e_chk_pos_1) > 500)
            {
             e_mov_ok = 1;
             e_tst_running = 3;
            }
           else
            {
             e_mov_ok = 0;
             e_tst_running = 3;
            } 
          }
         else
          {
           if ((e_chk_pos_1 - e_chk_pos_2) > 500)
            {
             e_mov_ok = 1;
             e_tst_running = 3;
            }
           else
            {
             e_mov_ok = 0;
             e_tst_running = 3;
            } 
          }   
        }
       if ((batt_tst_tim == 0) & (read_bat == 1))
        {
         load_spec_encoder_test();
         e_tst_running = 4;
        } 
      }
     else
      {
       if ((open_pb) || (open_disp_pb) || (close_pb) || (close_disp_pb)) e_tst_running = 2; // is OPEN or CLOSE P/B activated.
      }
    }
   else
    {
     e_chk_pos_1 = position; // save 1. position value
     e_tst_running = 1;
    }
  } 
}
//
void prog_mode() 
{
 if (s_encoder_tst == 1) encoder_chk(); // 21-12-2011
 else
  {
   spec_encoder_test = 0; // 21-12-2011
   e_tst_running = 0; // 21-12-2011
   if ((read_bat == 1) && (s_encoder_tst == 0))  read_bat = 0; // clear if set. not in use in prog mode 30-03-2010 21-12-2011
   if ((e_limit_open_learning_active == 1) || (e_limit_close_learning_active == 1)) moving_fail = 0; //skal være der, ellers kan der ikke genindlæres
   safe_stop = 0; // deaktiver stop ved prog mode
   //
   temp = EE_read(EE_c_limit_ok);
   temp1 = EE_read(EE_o_limit_ok);
   if ((temp == 0) && (temp1 == 0)) // er begge endestop gemt mindst 1 gang
    {
		 EE_read(EE_par_115);
     open_ls_setp = temp; // low speed decell. 
		 EE_read(EE_open_ls_setp);
     if (temp != open_ls_setp) save_ls_o_setp(); // gem hvis der er ændringer 
     //
		 EE_read(EE_par_125);
     close_ls_setp = temp; // low speed decell. 
		 EE_read(EE_close_ls_setp);
     if (temp != close_ls_setp) save_ls_c_setp(); // gem hvis der er ændringer 
		 //
		 EE_read(EE_par_85);	// 04-02-2022
		 bl_close_setp = temp; // xx % before end close limit for AUX relays (7x:12) and (7x:12)  
     EE_read(EE_bl_close_setp);
     if (temp != bl_close_setp) save_bl_close_setp(); // gem hvis der er ændringer 
     //
		 EE_read(EE_par_86);	// 04-02-2022
		 bl_open_setp = temp; // xx % before end open limit for AUX relays (7x:12) and (7x:12)  	
     EE_read(EE_bl_open_setp);
     if (temp != bl_open_setp) save_bl_open_setp(); // gem hvis der er ændringer 	
    }
    //
    ecode7 = 0; // slet denne fejlkode ved ny indlæring 
    //
   if (learn_active == 1)
    {
     if (new_limits == 1)
      {
      }
     else
      {
       EE_write(EE_limit_ready,255); //værdier ikke klar til kørsel
       EE_write(EE_photo_pos_saved, 0);
       new_limits = 1;
      }
    }
   else new_limits = 0; 
   //
   temp = EE_read(EE_limit_ready);
   if (temp == 255)
    {
    ecode2 = 1; // denne LED fejlkode sættes
		st_area_ok = 0;                // old sk2,0
    st_values_learned = 0;         // old sk2,3
    st_wear_observed = 0;          // old sk2,6
    st_init_learned = 0;					  // old sk2,7	
		}
   else
    {
    if (new_calc == 0) // EE_limit_ready er da 0, er new_calc da også 0 =Endestop græser er klar
    ecode2 = 0; // denne LED fejlkode cleares
    }
   if (motor_run == 0) // hvis 0 er motor stoppet.
    {
		 if (save_curtain == 1) // 12-02-2020
	    {
	     EE_read(EE_curtain_rep_saved); // only save once
	     if (temp == 1) 
	      {
	      }				 
	     else
	      {				 
		     ols_curt_rep = position;
         confirm_learn = 1; // confirm with 2 sec fixed run showing
	       EE_write(EE_curtain_rep_saved, 1); 
         EE_write(EE_curtain_sw, 0); // ready for normal running	
         curtain_learn = 0; // clear curtain learn - when curtain rep point is saved
				 new_calc = 1; // klar til at gemme i EE senere under limit_calc som er nødvendig	
        } 
        cls_out = 0;          // deaktiver CLS (aktiv lav) Der vises open limit som bekræftelse
        ols_out = 1;          // aktiver OLS (aktiv lav)
        limit_indi = indi_time; //  1.50 sek. LED indikering
			}						
		 else if (save_open == 1) // er save open limit aktiveret 
      {
       if (open_saved == 1)
        {
        }
      else
       {
        enc_ols = position; // gem ols position i ram
        data = enc_ols; // - 27-11-09
        adr = EE_enc_ols;
        write_int32_eeprom();
        open_saved = 1; // set testbit så der ikke gemmes igen
        EE_write(EE_o_limit_ok, 0); // open limit er nu klar til at blive gemt 
        new_calc = 1; // klar til at gemme i EE senere under limit_calc
       }
       cls_out = 0;          // deaktiver CLS (aktiv lav)
       ols_out = 1;          // aktiver OLS (aktiv lav)
       limit_indi = indi_time; //  1.50 sek. LED indikering
      }
     else
      {
       open_saved = 0; // open limit tryk sluppet
       if (save_close == 1) // er save close limit aktiveret 
        {
         if (close_saved == 1)
          {
          }
         else
          {
           enc_cls = position; // gem cls position i ram
           data = enc_cls; // - 27-11-09
           adr = EE_enc_cls;
           write_int32_eeprom();
           close_saved = 1; // set testbit så der ikke gemmes igen
           EE_write(EE_c_limit_ok, 0); // close limit er nu klar at blive gemt
           new_calc = 1; // klar til at gemme i EE senere under limit_calc
          }
          ols_out = 0;    // deaktiver OLS 
          cls_out = 1;    // aktiver CLS (aktiv lav)
          limit_indi = indi_time; // 225 gennemløb a 6.66 mS skulle blive 1.50 sek. LED indikering
        }
       else close_saved = 0; // tryk close sluppet
      }
			//
			inv_speed_moni(); //03-12-2021
      limit_calc(); // 
     }
    else
     {
     }
    if (limit_indi == 0)
     {
      ols_out = 0;          //deaktiver ols - indikering er slut
      cls_out = 0;          //deaktiver cls - indikering er slut
     }
  } 
}
//
uint8_t out_of_range_tst()
{
 if (right_turn == 1) // er højredrejning valgt
   {
   if (position > ols_max) 
      out_of_range = true; 
   else 
      out_of_range = false;
   }
 else
   {
   if (position > cls_max) 
      out_of_range = true; 
   else 
      out_of_range = false;
   }
return out_of_range;
}
//
void calc_fine_adj()
 {
	EE_read(EE_par_13); //get finadjust value for open limit
  fine_ols = temp;    //	 
  //
  switch (fine_ols)
  { 
  case 1: fine_ols = -4;
  break;
  case 2: fine_ols = -3;
  break;
  case 3: fine_ols = -2;
  break;
  case 4: fine_ols = -1;
  break;
  case 6: fine_ols = 1;
  break;
  case 7: fine_ols = 2;
  break;  
  case 8: fine_ols = 3;
  break;  
  case 9: fine_ols = 4;
  break; 
  default: fine_ols = 0;
  break; // 
  }
  //
	EE_read(EE_par_15); //get finadjust value for close limit
	fine_cls = temp;    //
  //
  switch (fine_cls)
  { 
  case 1: fine_cls = -4;
  break;
  case 2: fine_cls = -3;
  break;
  case 3: fine_cls = -2;
  break;
  case 4: fine_cls = -1;
  break;
  case 6: fine_cls = 1;
  break;
  case 7: fine_cls = 2;
  break;  
  case 8: fine_cls = 3;
  break;  
  case 9: fine_cls = 4;
  break; 
  default: fine_cls = 0;
  break; // 
  } 
 }
// 

void chk_deceleration()
{
 if (right_turn == 1)
  {
   enc_low_sp_p = 0; // low speed ikke aktiv hvis den ikke settes af følgende 2 linier
   if ((open_active() == 1) && (position > open_low_sp)) enc_low_sp_p = 1; // når port kører op
   if ((close_active() == 1) && (position < close_low_sp)) enc_low_sp_p = 1; // når port kører op
   if ((use_half_open == 1) && (open_active() == 1) && (position < h_open_pos) && (position > ls_h_o_opening_setp))
    {
     enc_low_sp_p = 1;   // if half open is wanted and door running up and position i between ls_h_o_opening_setp and half_open position - the set low speed 13-02-2026
    }
   if ((use_half_open == 1) && (close_active() == 1) && (position > h_open_pos) && (position < ls_h_o_closing_setp))
    { 
     enc_low_sp_p = 1; // if half open is wanted and door running up and position i between ls_h_o_closing_setp and half_open position - the set low speed 13-02-2026
    } 
  }
 else // left turn selected
  {
   enc_low_sp_p = 0; // low speed ikke aktiv hvis den ikke settes af følgende 2 linier
   if ((open_active() == 1) && (position < open_low_sp)) enc_low_sp_p = 1; // når port kører op
   if ((close_active() == 1) && (position > close_low_sp)) enc_low_sp_p = 1; // når port kører op
   if ((use_half_open == 1) && (open_active() == 1) && (position > h_open_pos) && (position < ls_h_o_opening_setp)) enc_low_sp_p = 1; // 13-02-2026
   if ((use_half_open == 1) && (close_active() == 1) && (position < h_open_pos) && (position > ls_h_o_closing_setp)) enc_low_sp_p = 1; // 13-02-2026
   // if half open is wanted and door running up and position i between ls_h_o_closing_setp and half_open position - the set low speed 13-02-2026 
  }
}
//
void chk_before_limits() // 15-06-2020
{
 bol_on = 0; // before open limit shall be ON if not set by the following 22-06-2020
 bcl_on = 0; // before close limit shall be ON if not set by the following 22-06-2020
 if (right_turn == 1)
  {
   if (position > bef_open_lim) bol_on = 1; // når port kører op
   if (position < bef_close_lim) bcl_on = 1;  // når port kører op
   //
   if (position > (ols_min - (run_range/2) - p5_run_range)) car_wash_active = 1; // is position > 45% open for car wash function active 17-06-2020
   else car_wash_active = 0;
  }
 else // left turn selected
  {
   if (position < bef_open_lim) bol_on = 1; // når port kører op
   if (position > bef_close_lim) bcl_on = 1;  // når port kører op
   //
   if (position < (ols_max + (run_range/2) + p5_run_range))  car_wash_active = 1; // 17-06-2020
   else car_wash_active = 0;
  }
}
//
void get_values()
{
 read_int32_eeprom(EE_offset); 
 offset = data; 
 read_int32_eeprom(EE_cls_min); 
 cls_min = data;
 read_int32_eeprom(EE_cls_max); 
 cls_max = data;
 if (right_turn == 1) // er højredrejning valgt 30-04-2019
  {
	 read_int32_eeprom(EE_ols_max); 
   ols_max = data;
	 EE_read(EE_curtain_rep_saved); // 12-02-2020
   temp1 = temp;		
	 EE_read(EE_curtain_sw);	
	 if (curtain_learn == 1)
	  {
     read_int32_eeprom(EE_ols_min_curt_learn); // use curtain learn limits 12-02-2020
     ols_min = data;
    }
   else if ((temp == 1) && (temp1 == 1))
	  {
     read_int32_eeprom(EE_ols_curt_rep); // use repair limits
     ols_min = data; 
		}
	 else
	  {
     read_int32_eeprom(EE_ols_min); // use normal lilimts
     ols_min = data;
		}		 
  }
 else // left turn selected 30-04-2019
  {
	 read_int32_eeprom(EE_ols_min); 
   ols_min = data;
   EE_read(EE_curtain_rep_saved); // 12-02-2020
   temp1 = temp;			
	 EE_read(EE_curtain_sw);
	 if (curtain_learn == 1)
	  {
     read_int32_eeprom(EE_ols_max_curt_learn); // use curtain learn limits 12-02-2020
     ols_max = data;
    }
   else if ((temp == 1) && (temp1 == 1)) 	
		{
     read_int32_eeprom(EE_ols_curt_rep); // use repair limits
     ols_max = data;
		}
   else
	  {
     read_int32_eeprom(EE_ols_max); 
     ols_max = data;	
		}	 
  }	
 read_int32_eeprom(EE_limit_win); 
 limit_win = data;
 read_int32_eeprom(EE_run_range); // 04-02-2010
 run_range = data;
 read_int32_eeprom(EE_p5_run_range); // 27-10-2011
 p5_run_range = data;
 //*********** 30-06-09
 read_int32_eeprom(EE_open_low_sp); 
 open_low_sp = data;
 read_int32_eeprom(EE_close_low_sp); 
 close_low_sp = data;
 //************** 
 temp = EE_read(EE_h_open_pos_saved); // check whether half open position is saved 13-02-2026
 if (temp == 0)
  {
   read_int32_eeprom(EE_h_open_pos); //
   h_open_pos = data;
   read_int32_eeprom(EE_ls_h_o_opening_setp); 
   ls_h_o_opening_setp = data; 
   read_int32_eeprom(EE_ls_h_o_closing_setp); 
   ls_h_o_closing_setp = data;
  }
 //*********** 
 read_int32_eeprom(EE_bef_close_lim); // 04-02-2022 
 bef_close_lim = data;
 read_int32_eeprom(EE_bef_open_lim); // 04-02-2022 
 bef_open_lim = data;	
 //**************	
 read_int32_eeprom(EE_fine_adj_unit); 
 fine_adj_unit = data;
 offset_sign = EE_read(EE_offset_sign);
 //
 read_int32_eeprom(EE_ex_limit_win); //13-02-2026 So we can use this later if needed
 ex_limit_win = data; 
}

void chk_half_open()
{
 EE_read(EE_par_16); //is 1/2 open Electronic limit wanted	
 if (dock_54_active == 1)
  {
   if ((temp > 2) && (e_h_ols_sw == 1)) // 18-05-2021 // before in V7E SR ((bit_test(r_byte2,4) == 1) & (bit_test(r_byte2,5) == 1))
		                                    // 22-11-2023
    {
     use_half_open = 1; // set kontrolbit hvis tryk og valg
    }
   else use_half_open = 0; // clear when door_half_cmd on uP1 is cleared
  }
 else
  {
   if ((temp > 2) && (e_h_ols_sw == 1)) // 18-05-2021 // before in V7E SR((bit_test(r_byte2,4) == 1) & (bit_test(r_byte2,5) == 1))
		                                    // 22-11-2023
    {
     use_half_open = 1; // set kontrolbit hvis tryk og valg
    }
   if ((open_pb == 1) | (close_pb == 1)) use_half_open = 0; // clear kontrolbit ved åbne eller lukke tryk
  }
 //
 if ((save_h_open_learn == 1) && (open_active() == 0) && (close_active() == 0)) // is save half open position learn wanted from uP1 and door stopped
  {
   if (h_pos_save_ready == 1)
    {
    }
   else
    {
     h_pos_save_ready = 1;
     data = position; // half open position in EEPROM
     adr = EE_h_open_pos;
     write_int32_eeprom();
     EE_write(EE_h_open_pos_saved, 0);
     // 
     if (right_turn == 1) // 13-02-2026 low speed before half open calc and save - opening
      {
       data = (position - (4 * ex_limit_win) - (run_range/100 * open_ls_setp));
       if ((data < cls_max) || (data > 500000000)) data = cls_min; // overlap CLS adjust to CLS
      }
     else // left turn
      {
       data = (position + (4 * ex_limit_win) + (run_range/100 * open_ls_setp));
       if (data > cls_min) data = cls_max; // overlap CLS adjust to CLS 
      }
     adr = EE_ls_h_o_opening_setp;
     write_int32_eeprom();
     //
     if (right_turn == 1) // 13-02-2026 low speed before half open calc and save - closing
      {
       data = (position + (4 * ex_limit_win) + (run_range/100 * close_ls_setp));
       if (data > ols_min) data = ols_max; // overlap OLS adjust to OLS
      }
     else // left turn
      {
       data = (position - (4 * ex_limit_win) - (run_range/100 * close_ls_setp)); 
       if (data > ols_max) data = ols_min; // overlap OLS adjust to OLS
      }
     adr = EE_ls_h_o_closing_setp;
     write_int32_eeprom(); 
     // 
     limit_indi = 400;
    }
  }
 else h_pos_save_ready = 0; //
 //
 EE_read(EE_par_16); //is 1/2 open Electronic limit wanted
 if ((temp == 2) && (open_active() == 1) && (e_h_ols_sw == 1)) // 24-11-2021	27-01-2022
  {
   use_half_open = 1; //
  }
 temp = EE_read(EE_limit_ready); // 25-05-2021
 if ((use_half_open == 1) && (temp == 0))// skal halv åbne aktiveres 
  {
   temp = EE_read(EE_h_open_pos_saved);
   if (temp > 0)
    {
     if (right_turn == 1) h_open_pos = (ols_min - (run_range/2));
     else h_open_pos = (ols_max + (run_range/2));
    }
   else
    {
     read_int32_eeprom(EE_h_open_pos); //
     h_open_pos = data;
    }
  } 
  //
 else
  {
  }
 temp = EE_read(EE_h_open_pos_saved);
 if (temp == 0)
  {
   read_int32_eeprom(EE_h_open_pos); //
   h_open_pos = data;
  }
 h_open_pos_max = h_open_pos + ex_limit_win + ex_limit_win + ex_limit_win + ex_limit_win; // 1/32 runrange added 13-02-2026
 h_open_pos_min = h_open_pos - ex_limit_win - ex_limit_win - ex_limit_win - ex_limit_win; // 1/32 runrange subtracted 13-02-2026 
  
//***
 if (right_turn == 1)
  {
   if ((position < h_open_pos_max) && (position > h_open_pos_min))
    {
     door_pos_high = 0;
     door_pos_half = 1;
     door_pos_low = 0;			
    }
   else if ((position > h_open_pos_max) || (position == h_open_pos_max))
    {
     door_pos_high = 1;
     door_pos_half = 0;
     door_pos_low = 0;			
    } 
   else
    {
     door_pos_high = 0;
     door_pos_half = 0;
     door_pos_low = 1;			
    } 
  } 
 else
  {
   if ((position < h_open_pos_max) && (position > h_open_pos_min))
    {
     door_pos_high = 0;
     door_pos_half = 1;
     door_pos_low = 0;			
    }
   else if ((position < h_open_pos_min) || (position == h_open_pos_min))
    {
     door_pos_high = 1;
     door_pos_half = 0;
     door_pos_low = 0;			
    } 
   else
    {
     door_pos_high = 0;
     door_pos_half = 0;
     door_pos_low = 1;			
    } 
  } 
 //**
 if ((open_active() == 1) && (door_pos_high == 1))  // is door moving up and in high area
  {
   use_half_open = 0;
  }
 if ((close_active() == 1) && (door_pos_low == 1))  // is door moving down and in low area
  {
   use_half_open = 0;
  }
 if ((close_active() == 0) && (open_active() == 0) && (door_pos_half == 1)) // is door stopped and already on half open
  {
   use_half_open = 0;
  }	
}
//******************************************
void st_ok_save()
{
 data = st_ce1; //
 adr = EEst_ce1;
 write_int16_eeprom();
 //
 data = st_ce2; //
 adr = EEst_ce2;
 write_int16_eeprom();
 //
 data = st_ce3; //
 adr = EEst_ce3;
 write_int16_eeprom();
 //
 data = st_ce4; //
 adr = EEst_ce4;
 write_int16_eeprom();
 //
 data = st_ce5; //
 adr = EEst_ce5;
 write_int16_eeprom();
 //
 data = st_ce6; //
 adr = EEst_ce6;
 write_int16_eeprom();
 //
 data = st_ce7; //
 adr = EEst_ce7;
 write_int16_eeprom();
 //
 data = st_ce8; //
 adr = EEst_ce8;
 write_int16_eeprom();
 //
 data = st_ce9; //
 adr = EEst_ce9;
 write_int16_eeprom();
 //
 data = st_ce10; //
 adr = EEst_ce10;
 write_int16_eeprom();
 //
 data = st_oe1; //
 adr = EEst_oe1;
 write_int16_eeprom();
 //
 data = st_oe2; //
 adr = EEst_oe2;
 write_int16_eeprom();
 //
 data = st_oe3; //
 adr = EEst_oe3;
 write_int16_eeprom();
 //
 data = st_oe4; //
 adr = EEst_oe4;
 write_int16_eeprom();
 //
 data = st_oe5; //
 adr = EEst_oe5;
 write_int16_eeprom();
 //
 data = st_oe6; //
 adr = EEst_oe6;
 write_int16_eeprom();
 //
 data = st_oe7; //
 adr = EEst_oe7;
 write_int16_eeprom();
 //
 data = st_oe8; //
 adr = EEst_oe8;
 write_int16_eeprom();
 //
 data = st_oe9; //
 adr = EEst_oe9;
 write_int16_eeprom();
 //
 data = st_oe10; //
 adr = EEst_oe10;
 write_int16_eeprom();
 //
 st_values_learned = 1;
 st_learn_confirm = 45; // 300mS singleturn learn finish confirming 
 //
 speed_unit = (st_ce5/110);
 data = speed_unit; //
 adr = EEspeed_unit;
 write_int16_eeprom();
 //
 if (st_init_learned == 1) // is initial values learned
  {
   // yes, don't save these
  }
 else
  {
   st_ice1 = st_ce1;
   data = st_ce1; //
   adr = EEst_ice1;
   write_int16_eeprom();
   //
   st_ice2 = st_ce2;
   data = st_ce2; //
   adr = EEst_ice2;
   write_int16_eeprom();
   //
   st_ice3 = st_ce3;
   data = st_ce3; //
   adr = EEst_ice3;
   write_int16_eeprom();
   //
   st_ice4 = st_ce4;
   data = st_ce4; //
   adr = EEst_ice4;
   write_int16_eeprom();
   //
   st_ice5 = st_ce5;
   data = st_ce5; //
   adr = EEst_ice5;
   write_int16_eeprom();
   //
   st_ice6 = st_ce6;
   data = st_ce6; //
   adr = EEst_ice6;
   write_int16_eeprom();
   //
   st_ice6 = st_ce6;
   data = st_ce6; //
   adr = EEst_ice6;
   write_int16_eeprom();
   //
   st_ice7 = st_ce7;
   data = st_ce7; //
   adr = EEst_ice7;
   write_int16_eeprom();
   //
   st_ice8 = st_ce8;
   data = st_ce8; //
   adr = EEst_ice8;
   write_int16_eeprom();
   //
   st_ice9 = st_ce9;
   data = st_ce9; //
   adr = EEst_ice9;
   write_int16_eeprom();
   //
   st_ice10 = st_ce10;
   data = st_ce10; //
   adr = EEst_ice10;
   write_int16_eeprom();
   //
   st_ioe1 = st_oe1;
   data = st_oe1; //
   adr = EEst_ioe1;
   write_int16_eeprom();
   //
   st_ioe2 = st_oe2;
   data = st_oe2; //
   adr = EEst_ioe2;
   write_int16_eeprom();
   //
   st_ioe3 = st_oe3;
   data = st_oe3; //
   adr = EEst_ioe3;
   write_int16_eeprom();
   //
   st_ioe4 = st_oe4;
   data = st_oe4; //
   adr = EEst_ioe4;
   write_int16_eeprom();
   //
   st_ioe5 = st_oe5;
   data = st_oe5; //
   adr = EEst_ioe5;
   write_int16_eeprom();
   //
   st_ioe6 = st_oe6;
   data = st_oe6; //
   adr = EEst_ioe6;
   write_int16_eeprom();
   //
   st_ioe7 = st_oe7;
   data = st_oe7; //
   adr = EEst_ioe7;
   write_int16_eeprom();
   //
   st_ioe8 = st_oe8;
   data = st_oe8; //
   adr = EEst_ioe8;
   write_int16_eeprom();
   //
   st_ioe9 = st_oe9;
   data = st_oe9; //
   adr = EEst_ioe9;
   write_int16_eeprom();
   //
   st_ioe10 = st_oe10;
   data = st_oe10; //
   adr = EEst_ioe10;
   write_int16_eeprom();
   //
   st_init_learned = 1; 
  }
}
//******************************************
void st_save_speed()
{
 if (st_init_learned == 0) // is initial values not learned yet
  {
   st_ok_save();
  }
 else
  {
   read_int16_eeprom(EEst_ce1);
   temp16 = data;
   if (temp16 == 65535) // is value save first time after reset
    {
     st_ok_save();
    }
   else
    {
     if (enc_open_cnt >9)
      {
       enc_open_cnt = 0;
       st_ok_save();
      }
    }
  } 
}
//******************************************
void st_value_upd()
{
 enc_speed_o = 0;
 enc_speed_c = 0;
 if (bit_test(e_msc,0) == 1)
  {
   if (bit_test(e_msc,1) == 1) st_ce1 = (st_ce1 - speed_unit);
  }
 else
  {
   if (st_ce1 == 65535) // keep value if not learned course of fast moving
    {
    }
   else st_ce1 = (st_ce1 + speed_unit);
  }
 //
 if (bit_test(e_msc,2) == 1)
  {
   if (bit_test(e_msc,3) == 1) st_ce2 = (st_ce2 - speed_unit);
  }
 else st_ce2 = (st_ce2 + speed_unit);
 //
 if (bit_test(e_msc,4) == 1)
  {
   if (bit_test(e_msc,5) == 1) st_ce3 = (st_ce3 - speed_unit);
  }
 else st_ce3 = (st_ce3 + speed_unit);
 //
 if (bit_test(e_msc,6) == 1)
  {
   if (bit_test(e_msc,7) == 1) st_ce4 = (st_ce4 - speed_unit);
  }
 else st_ce4 = (st_ce4 + speed_unit);
 //
 if (bit_test(e_msc,8) == 1)
  {
   if (bit_test(e_msc,9) == 1) st_ce5 = (st_ce5 - speed_unit);
  }
 else st_ce5 = (st_ce5 + speed_unit);
 //
 if (bit_test(e_msc,10) == 1)
  {
   if (bit_test(e_msc,11) == 1) st_ce6 = (st_ce6 - speed_unit);
  }
 else st_ce6 = (st_ce6 + speed_unit);
 //
 if (bit_test(e_msc,12) == 1)
  {
   if (bit_test(e_msc,13) == 1) st_ce7 = (st_ce7 - speed_unit);
  }
 else
  {
   if (st_ce7 == 65535) // keep value if not learned course of fast moving
    {
    }
   else st_ce7 = (st_ce7 + speed_unit);
  }
 //
 if (bit_test(e_msc,14) == 1)
  {
   if (bit_test(e_msc,15) == 1) st_ce8 = (st_ce8 - speed_unit);
  }
 else
  {
   if (st_ce8 == 65535) // keep value if not learned course of fast moving
    {
    }
   else st_ce8 = (st_ce8 + speed_unit);
  }
 //
 if (bit_test(e_msc,16) == 1)
  {
   if (bit_test(e_msc,17) == 1) st_ce9 = (st_ce9 - speed_unit);
  }
 else 
  {
   if (st_ce9 == 65535) // keep value if not learned course of fast moving
    {
    }
   else st_ce9 = (st_ce9 + speed_unit);
  }
 //
 if (bit_test(e_msc,18) == 1)
  {
   if (bit_test(e_msc,19) == 1) st_ce10 = (st_ce10 - speed_unit);
  }
 else 
  {
   if (st_ce10 == 65535) // keep value if not learned course of fast moving
    {
    }
   else st_ce10 = (st_ce10 + speed_unit);
  }
 //
 if (bit_test(e_mso,0) == 1)
  {
   if (bit_test(e_mso,1) == 1) st_oe1 = (st_oe1 - speed_unit);
  }
 else
  {
   if (st_oe1 == 65535) // keep value if not learned course of fast moving
    {
    }
   else st_oe1 = (st_oe1 + speed_unit);
  }
 //
 if (bit_test(e_mso,2) == 1)
  {
   if (bit_test(e_mso,3) == 1) st_oe2 = (st_oe2 - speed_unit);
  }
 else
  {
   if (st_oe2 == 65535) // keep value if not learned course of fast moving
    {
    }
   else st_oe2 = (st_oe2 + speed_unit);
  }
 //
 if (bit_test(e_mso,4) == 1)
  {
   if (bit_test(e_mso,5) == 1) st_oe3 = (st_oe3 - speed_unit);
  }
 else
  {
   if (st_oe3 == 65535) // keep value if not learned course of fast moving
    {
    }
   else st_oe3 = (st_oe3 + speed_unit);
  }
 //
 if (bit_test(e_mso,6) == 1)
  {
   if (bit_test(e_mso,7) == 1) st_oe4 = (st_oe4 - speed_unit);
  }
 else 
  {
   if (st_oe4 == 65535) // keep value if not learned course of fast moving
    {
    }
   else st_oe4 = (st_oe4 + speed_unit);
  }
 //
 if (bit_test(e_mso,8) == 1)
  {
   if (bit_test(e_mso,9) == 1) st_oe5 = (st_oe5 - speed_unit);
  }
 else st_oe5 = (st_oe5 + speed_unit);
 //
 if (bit_test(e_mso,10) == 1)
  {
   if (bit_test(e_mso,11) == 1) st_oe6 = (st_oe6 - speed_unit);
  }
 else st_oe6 = (st_oe6 + speed_unit);
 //
 if (bit_test(e_mso,12) == 1)
  {
   if (bit_test(e_mso,13) == 1) st_oe7 = (st_oe7 - speed_unit);
  }
 else st_oe7 = (st_oe7 + speed_unit);
 //
 if (bit_test(e_mso,14) == 1)
  {
   if (bit_test(e_mso,15) == 1) st_oe8 = (st_oe8 - speed_unit);
  }
 else st_oe8 = (st_oe8 + speed_unit);
 //
 if (bit_test(e_mso,16) == 1)
  {
   if (bit_test(e_mso,17) == 1) st_oe9 = (st_oe9 - speed_unit);
  }
 else st_oe9 = (st_oe9 + speed_unit);
 //
 if (bit_test(e_mso,18) == 1)
  {
   if (bit_test(e_mso,19) == 1) st_oe10 = (st_oe10 - speed_unit);
  }
 else
  {
   if (st_oe10 == 65535) // keep value if not learned course of fast moving
    {
    }
   else st_oe10 = (st_oe10 + speed_unit);
  }
 //
 e_mso = 0;
 e_msc = 0;
 st_values_learned = 1;
}
//******************************************
void clr_s_buf_array() //nulstil speed control buffer
{
uint8_t i;
for (i=0;i<=5;i++) s_buf_array[i]=0;
}
//******************************************
void chk_ini_speed_open()
{
 speed_filt = 0;
 if ((s_buf_array[0] - s_buf_array[4]) < (st_i_speedsetp - (sense * 2))) // 10-12-09
  {
   if (speed_filt_ini == 1)
    {
     st_speed_stop = 1;
     st_wear_observed = 1; // wear observed, debug hvis frakoblet
    }
   else speed_filt_ini = 1;
  }
 else speed_filt_ini = 0;
}
//******************************************
void chk_ini_speed_close()
{
 speed_filt = 0;
 if ((s_buf_array[4] - s_buf_array[0]) < (st_i_speedsetp - (sense * 2))) // 10-12-09
  {
   if (speed_filt_ini == 1)
    {
     st_speed_stop = 1;
     st_wear_observed = 1; // wear observed, debug hvis frakoblet
    }
   else speed_filt_ini = 1;
  }
 else speed_filt_ini = 0;
}
//******************************************
void stop_by_speed() // flyttet til funktion grundet debug formål
{
 if (speed_filt == 1) st_speed_stop = 1;
 else speed_filt = 1;
}
//******************************************
void compare_increasing()
{
if ((s_buf_array[0] - s_buf_array[1]) < (temp16 - sense)) stop_by_speed(); // fast stop if very low speed
else if ((s_buf_array[0] - s_buf_array[2]) < ((temp16 * 2) - sense)) stop_by_speed();
else if ((s_buf_array[0] - s_buf_array[3]) < ((temp16 * 3) - sense)) stop_by_speed();
else if ((s_buf_array[0] - s_buf_array[4]) < (st_speedsetp - sense)) stop_by_speed();
else chk_ini_speed_open();
}
//******************************************
void compare_decreasing()
{
 if ((s_buf_array[1] - s_buf_array[0]) < (temp16 - sense)) stop_by_speed(); // fast stop if very low speed
 else if ((s_buf_array[2] - s_buf_array[0]) < ((temp16 * 2) - sense)) stop_by_speed();
 else if ((s_buf_array[3] - s_buf_array[0]) < ((temp16 * 3) - sense)) stop_by_speed();
 else if ((s_buf_array[4] - s_buf_array[0]) < (st_speedsetp - sense)) stop_by_speed();
 else chk_ini_speed_close();
}
//******************************************
void speed_compare()
{
 if (ols_out == 1) // is ols active
  {
   // skip speed stop
  }
 else if ((enc_speed_delay == 0) && (s_buf_ready == 1))
  {
   temp16 = (st_speedsetp/4);
   if (open_active() == 1) // is door moving up
    {
     if (right_turn == 1) compare_increasing();
     else compare_decreasing();
    }
   else // by closing
    {
     if (right_turn == 1) compare_decreasing();
     else compare_increasing();
    } 
  }
 else
  {
  } 
}
//******************************************
void array_subtract_open()
{
 if (right_turn == 1) temp32 = (s_buf_array[0] - s_buf_array[4]);
 else temp32 = (s_buf_array[4] - s_buf_array[0]);
}
//******************************************
void array_subtract_close()
{
 if (right_turn == 1) temp32 = (s_buf_array[4] - s_buf_array[0]);
 else temp32 = (s_buf_array[0] - s_buf_array[4]);
}
//******************************************
void area_1()
{
 if (open_active() == 1) // is door moving up
  {
   array_subtract_open();
   if (temp32 >= st_oe1)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_oe1 = temp32;
      }
     else
      {
       if (bit_test(e_mso,0) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_mso,1); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_mso,0);
        }
      }
    }
   st_i_speedsetp = st_ioe1;
   bit_set(enc_speed_o,0);
   st_speedsetp = st_oe1;
  }
 //
 else // door moving down
  {
   array_subtract_close();
   if (temp32 >= st_ce1)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_ce1 = temp32;
      }
     else
      {
       if (bit_test(e_msc,0) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_msc,1); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_msc,0);
        }
      }
    }
   st_i_speedsetp = st_ice1;
   bit_set(enc_speed_c,0);
   st_speedsetp = st_ce1;
  }
}
//******************************************
void area_2()
{
 if (open_active() == 1) // is door moving up
  {
   array_subtract_open();
   if (temp32 >= st_oe2)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_oe2 = temp32;
      }
     else
      {
       if (bit_test(e_mso,2) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_mso,3); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_mso,2);
        }
      }
    }
   st_i_speedsetp = st_ioe2;
   bit_set(enc_speed_o,1);
   st_speedsetp = st_oe2;
  }
 //
 else // door moving down
  {
   array_subtract_close();
   if (temp32 >= st_ce2)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_ce2 = temp32;
      }
     else
      {
       if (bit_test(e_msc,2) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_msc,3); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_msc,2);
        }
      }
    }
   st_i_speedsetp = st_ice2;
   bit_set(enc_speed_c,1);
   st_speedsetp = st_ce2;
  }
}
//******************************************
void area_3()
{
 if (open_active() == 1) // is door moving up
  {
   array_subtract_open();
   if (temp32 >= st_oe3)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_oe3 = temp32;
      }
     else
      {
       if (bit_test(e_mso,4) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_mso,5); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_mso,4);
        }
      }
    }
   st_i_speedsetp = st_ioe3;
   bit_set(enc_speed_o,2);
   st_speedsetp = st_oe3;
  }
 //
 else // door moving down
  {
   array_subtract_close();
   if (temp32 >= st_ce3)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_ce3 = temp32;
      }
     else
      {
       if (bit_test(e_msc,4) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_msc,5); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_msc,4);
        }
      }
    }
   st_i_speedsetp = st_ice3;
   bit_set(enc_speed_c,2);
   st_speedsetp = st_ce3;
  }
}
//******************************************
void area_4()
{
 if (open_active() == 1) // is door moving up
  {
   array_subtract_open();
   if (temp32 >= st_oe4)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_oe4 = temp32;
      }
     else
      {
       if (bit_test(e_mso,6) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_mso,7); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_mso,6);
        }
      }
    }
   st_i_speedsetp = st_ioe4;
   bit_set(enc_speed_o,3);
   st_speedsetp = st_oe4;
  }
 //
 else // door moving down
  {
   array_subtract_close();
   if (temp32 >= st_ce4)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_ce4 = temp32;
      }
     else
      {
       if (bit_test(e_msc,6) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_msc,7); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_msc,6);
        }
      }
    }
   st_i_speedsetp = st_ice4;
   bit_set(enc_speed_c,3);
   st_speedsetp = st_ce4;
  }
}
//******************************************
void area_5()
{
 if (open_active() == 1) // is door moving up
  {
   array_subtract_open();
   if (temp32 >= st_oe5)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_oe5 = temp32;
      }
     else
      {
       if (bit_test(e_mso,8) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_mso,9); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_mso,8);
        }
      }
    }
   st_i_speedsetp = st_ioe5;
   bit_set(enc_speed_o,4);
   st_speedsetp = st_oe5;
  }
 //
 else // door moving down
  {
   array_subtract_close();
   if (temp32 >= st_ce5)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_ce5 = temp32;
      }
     else
      {
       if (bit_test(e_msc,8) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_msc,9); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_msc,8);
        }
      }
    }
   st_i_speedsetp = st_ice5;
   bit_set(enc_speed_c,4);
   st_speedsetp = st_ce5;
  }
}
//******************************************
void area_6()
{
 if (open_active() == 1) // is door moving up
  {
   array_subtract_open();
   if (temp32 >= st_oe6)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_oe6 = temp32;
      }
     else
      {
       if (bit_test(e_mso,10) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_mso,11); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_mso,10);
        }
      }
    }
   st_i_speedsetp = st_ioe6;
   bit_set(enc_speed_o,5);
   st_speedsetp = st_oe6;
  }
 //
 else // door moving down
  {
   array_subtract_close();
   if (temp32 >= st_ce6)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_ce6 = temp32;
      }
     else
      {
       if (bit_test(e_msc,10) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_msc,11); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_msc,10);
        }
      }
    }
   st_i_speedsetp = st_ice6;
   bit_set(enc_speed_c,5);
   st_speedsetp = st_ce6;
  }
}
//******************************************
void area_7()
{
 if (open_active() == 1) // is door moving up
  {
   array_subtract_open();
   if (temp32 >= st_oe7)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_oe7 = temp32;
      }
     else
      {
       if (bit_test(e_mso,12) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_mso,13); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_mso,12);
        }
      }
    }
   st_i_speedsetp = st_ioe7;
   bit_set(enc_speed_o,6);
   st_speedsetp = st_oe7;
  }
 //
 else // door moving down
  {
   array_subtract_close();
   if (temp32 >= st_ce7)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_ce7 = temp32;
      }
     else
      {
       if (bit_test(e_msc,12) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_msc,13); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_msc,12);
        }
      }
    }
   st_i_speedsetp = st_ice7;
   bit_set(enc_speed_c,6);
   st_speedsetp = st_ce7;
  }
}
//******************************************
void area_8()
{
 if (open_active() == 1) // is door moving up
  {
   array_subtract_open();
   if (temp32 >= st_oe8)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_oe8 = temp32;
      }
     else
      {
       if (bit_test(e_mso,14) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_mso,15); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_mso,14);
        }
      }
    }
   st_i_speedsetp = st_ioe8;
   bit_set(enc_speed_o,7);
   st_speedsetp = st_oe8;
  }
 //
 else // door moving down
  {
   array_subtract_close();
   if (temp32 >= st_ce8)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_ce8 = temp32;
      }
     else
      {
       if (bit_test(e_msc,14) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_msc,15); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_msc,14);
        }
      }
    }
   st_i_speedsetp = st_ice8;
   bit_set(enc_speed_c,7);
   st_speedsetp = st_ce8;
  }
}
//******************************************
void area_9()
{
 if (open_active() == 1) // is door moving up
  {
   array_subtract_open();
   if (temp32 >= st_oe9)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_oe9 = temp32;
      }
     else
      {
       if (bit_test(e_mso,16) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_mso,17); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_mso,16);
        }
      }
    }
   st_i_speedsetp = st_ioe9;
   bit_set(enc_speed_o,8);
   st_speedsetp = st_oe9;
  }
 //
 else // door moving down
  {
   array_subtract_close();
   if (temp32 >= st_ce9)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_ce9 = temp32;
      }
     else
      {
       if (bit_test(e_msc,16) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_msc,17); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_msc,16);
        }
      }
    }
   st_i_speedsetp = st_ice9;
   bit_set(enc_speed_c,8);
   st_speedsetp = st_ce9;
  }
}
//******************************************
void area_10()
{
 if (open_active() == 1) // is door moving up
  {
   array_subtract_open();
   if (temp32 >= st_oe10)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_oe10 = temp32;
      }
     else
      {
       if (bit_test(e_mso,18) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_mso,19); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_mso,18);
        }
      }
    }
   st_i_speedsetp = st_ioe10;
   bit_set(enc_speed_o,9);
   st_speedsetp = st_oe10;
  }
 //
 else // door moving down
  {
   array_subtract_close();
   if (temp32 >= st_ce10)
    {
    }
   else
    {
     if (st_values_learned == 0) // is value not ok yet
      {
       st_ce10 = temp32;
      }
     else
      {
       if (bit_test(e_msc,18) == 1)
        {
         if (enc_open_cnt == 10) bit_set(e_msc,19); // both 9th and 10 th open says less sensivity
        }
       else
        {
         if (enc_open_cnt == 9) bit_set(e_msc,18);
        }
      }
    }
   st_i_speedsetp = st_ice10;
   bit_set(enc_speed_c,9);
   st_speedsetp = st_ce10;
  }
}
//******************************************
void st_sort_area()
{
 if ((open_active() == 1) && (position < area12)) speed_lo_chk = 1; // is door moving up and position in area_1
 if ((close_active() == 0) && (position > area910)) speed_lc_chk = 1; // is door moving down and position in area_10
//
 if (s_buf_ready == 1)
  {
   if (right_turn == 1) // er højredrejning valgt
    {
     if (s_buf_ready == 1) // is ring buffer full
      {
       if (position < area12) area_1();
       else if (position < area23) area_2();
       else if (position < area34) area_3();
       else if (position < area45) area_4();
       else if (position < area56) area_5();
       else if (position < area67) area_6();
       else if (position < area78) area_7();
       else if (position < area89) area_8();
       else if (position < area910) area_9();
       else area_10();   
      }
    }
   else // left turning selected
    {
     if (position > area12) area_1();
     else if (position > area23) area_2();
     else if (position > area34) area_3();
     else if (position > area45) area_4();
     else if (position > area56) area_5();
     else if (position > area67) area_6();
     else if (position > area78) area_7();
     else if (position > area89) area_8();
     else if (position > area910) area_9();
     else area_10();   
    }
  }
}
//******************************************
void area_fill()
{
 if (st_oe1 == 65535) // is open setpoint 1 still on reset
  {
   if (st_oe2 == 65535)
    {
     if (st_oe3 == 65535)
      {
       if (st_oe4 == 65535)
        {
         st_oe4 = st_oe5;
         st_oe3 = st_oe5; 
         st_oe2 = st_oe5;
         st_oe1 = st_oe5; 
        }
       else
        {
         st_oe3 = st_oe4;
         st_oe2 = st_oe4;
         st_oe1 = st_oe4;
        } 
      }
     else
      {
       st_oe2 = st_oe3;
       st_oe1 = st_oe3;
      }
    }
   else 
    {
     st_oe1 = st_oe2;
    }
  }
 if (st_oe10 == 65535) st_oe10 = st_oe9;
 //
 if (st_ce10 == 65535) // is close setpoint 10 still on reset
  {
   if (st_ce9 == 65535)
    {
     if (st_ce8 == 65535)
      {
       if (st_ce7 == 65535)
        {
         st_ce7 = st_ce6;
         st_ce8 = st_ce6; 
         st_ce9 = st_ce6;
         st_ce10 = st_ce6; 
        }
       else
        {
         st_ce8 = st_ce7;
         st_ce9 = st_ce7;
         st_ce10 = st_ce7;
        } 
      }
     else
      {
       st_ce9 = st_ce8;
       st_ce10 = st_ce8;
      }
    }
   else 
    {
     st_ce10 = st_ce9;
    }
  }
 if (st_ce1 == 65535) st_ce1 = st_ce2;
}
//******************************************
void chk_area_ok()
{
 st_value_adj = 0; // clear control bit for about values adjusted for this door cycle
 if ((speed_lo_chk == 1) & (speed_lc_chk == 1) && (st_area_ok == 1)) // has continoursly run taken place and area testbit ok
  {
   st_value_upd();
   area_fill();
   st_save_speed();
  }
 else
  {
   e_mso = 0;
   e_msc = 0;
  }
}
//******************************************
void chk_v_just()
{
 if (open_active() == 1) // is door moving up
  {
  }
 else
  {
   if (st_value_adj == 0) // is values adjusted on this door cycle
    {
    }
   else
    {
     if (st_values_learned == 1) // is value learned
      {
       if (enc_open_cnt > 10) chk_area_ok();
      }
     else chk_area_ok();
    }
  }
}
//******************************************
void rotate_buffer()
{
 s_buf_array[4] = s_buf_array[3];
 s_buf_array[3] = s_buf_array[2];
 s_buf_array[2] = s_buf_array[1];
 s_buf_array[1] = s_buf_array[0];
 s_buf_array[0] = position;
 meas_interval = 90; // 600mS interval
}
//******************************************
void speed_check()
{
 if (e_type == 0x01) // Kostal encoder (kostal følsomhed kan måske sættes lidt bedre da den er mere liniær)
  {
	 EE_read(EE_par_44);	
   if (temp == !2) sense = (speed_unit * 6); // low sensivity
   else sense = (speed_unit * 4); // high sensivity 
  }
 else // Dalmatic og Feig enkoder
  {
	 EE_read(EE_par_44);	
   if (temp == !2)
    {
     sense = (speed_unit * 8); // low sensivity
     if (sense < 370) sense = 380;
    }
   else
    {
     sense = (speed_unit * 4); // high sensivity (speed_unit er ca 1/2 enkoderenhed)
     if (sense < 185) sense = 190;
    }
  }
 //
 if (s_speed_learn == 0) speed_clr = 0;
 if ((s_speed_learn == 1) & (speed_clr == 0) & (run_prog == 1)) // 07-12-09 11-02-2022
  {
   st_clr_val(); // new learning wanted
  }
 //
 EE_read(EE_limit_ready); //	
 if ((torque_singleturn == 1) && (temp == 0))
  {
   if ((close_active() == 1) & (open_active() == 0)) move_down = 1; // 10-12-09 
   if ((open_active() == 1) & (close_active() == 0) & (move_down == 1))
    {
     stop_open_or_close();
		 //gate_off;
     //close_gate_off;
    }
   if ((open_active() == 0) & (close_active() == 0))
    {
     move_down = 0;
     clr_s_buf_array(); // clear speed buffer array, ready for new start
     enc_speed_delay = 60; // reload speed delay with 0.4 sec.
     r_buf_point = 0; // clear ring buffer pointer
     s_buf_ready = 0; // clear speed buffer ready testbit
     meas_interval = 0; // clear timer for measure testintervals
     st_speed_stop = 0; // clear speed stop testbit. Door already stopped.
     if ((cls_out == 1) | (ols_out == 1)) // is it a stop by limits
      {
      }
     else
      {
       speed_lo_chk = 0;
       speed_lc_chk = 0;
      }
    }
   else
    {
     if (enc_speed_delay == 0)
      {
       if (open_active() == 1) st_value_adj = 1; // set controlbit when door moving up
       if (meas_interval == 0)
        {
         if (s_buf_ready == 1) rotate_buffer();
         else
          {
           if (r_buf_point == 4)
            {
             r_buf_point = 0;
             s_buf_ready = 1;
            }
           else r_buf_point++;
           rotate_buffer();
          } 
        }
       if (cls_out == 1) // is cls active
        {
         chk_v_just();
        }
       else if (meas_interval == 90) // 
        {
         st_sort_area();
         if (((open_active() == 1) | (close_active() == 1)) && (st_values_learned == 1))
          {
           speed_compare(); // only speed compare when new buffervalue is loaded
          }
        }
       else
        {
        }
      }
    }
  }
 else
  {
   st_speed_stop = 0; 
  }
}
//******************************************
void reload_moving_timer() // 16-02-2017
 {
	EE_read(EE_par_81); // selected value for E:09 failure (moving fail on Encoder processor)	
  if (temp == 0) moving_timer = 150; // 1 Sec. to failure 
  if (temp == 1) moving_timer = 300; // 2 Sec. to failure
  if ((temp == 2) || (temp == 3)) moving_timer = 600; // 4 Sec. to 
	mt_preset_value = (moving_timer/2); 
  position_old = position;
	position_old2 = position;
 }

void chk_moving()
{
 if ((open_active() == 1) | (close_active() == 1)) // kører port op eller ned
  { 
	 show_E09_before_reset = 300; // sec show of E09 before reset if parameter 81 = 3 22-03-2017	
   if (moving_timer < mt_preset_value) // measure interval 1 16-02-2017
    {
     if (e_type == kostal) kostal_moving_chk(); // 20-03-2017
		 else dall_moving_chk();	// 20-03-2017
	  }
  }
 else // port kører ikke
  {
   // singleturn area shift saving ***********
   if (st_area_ok == 0)
    {
     if (right_turn == 1) // er højredrejning valgt
      {
       temp32 = (ols_min - cls_max)/10;
       area12 = (cls_max + temp32);
       area23 = (area12 + temp32);
       area34 = (area23 + temp32);
       area45 = (area34 + temp32);
       area56 = (area45 + temp32);
       area67 = (area56 + temp32);
       area78 = (area67 + temp32);
       area89 = (area78 + temp32);
       area910 = (ols_min - temp32);
      }
     else // left turn selected
      {
       temp32 = (cls_min - ols_max)/10;
       area12 = (cls_min - temp32);
       area23 = (area12 - temp32);
       area34 = (area23 - temp32);
       area45 = (area34 - temp32);
       area56 = (area45 - temp32);
       area67 = (area56 - temp32);
       area78 = (area67 - temp32);
       area89 = (area78 - temp32);
       area910 = (ols_max + temp32);
      }
     //
     data = area12; //  værdi skrives til EEPROM
     adr = EEarea12;
     write_int32_eeprom();
     data = area23; //  værdi skrives til EEPROM
     adr = EEarea23;
     write_int32_eeprom();
     data = area34; //  værdi skrives til EEPROM
     adr = EEarea34;
     write_int32_eeprom();
     data = area45; //  værdi skrives til EEPROM
     adr = EEarea45;
     write_int32_eeprom();
     data = area56; //  værdi skrives til EEPROM
     adr = EEarea56;
     write_int32_eeprom();
     data = area67; //  værdi skrives til EEPROM
     adr = EEarea67;
     write_int32_eeprom();
     data = area78; //  værdi skrives til EEPROM
     adr = EEarea78;
     write_int32_eeprom();
     data = area89; //  værdi skrives til EEPROM
     adr = EEarea89;
     write_int32_eeprom();
     data = area910; //  værdi skrives til EEPROM
     adr = EEarea910;
     write_int32_eeprom();
     //
     st_area_ok = 1;
    }
   //*********************************
	 EE_read(EE_par_81); // selected value for E:09 failure (moving fail on Encoder processor)	
   if (temp == 3) // er der valgt 4 sek. tid og tilladt genkørsel 4 sek. 09-03-2010
    {
		 if (show_E09_before_reset == 0) 
		  {
       moving_fail = 0; // 22-03-2017
       EE_read(EE_moving_fail);
       if (temp == 1) EE_write(EE_moving_fail,0);
		  }	 
    }
   // moving fail må ikke slettes her. Kun ved genindlæring eller ovenstående special genkørsel valg . 18-01-2010
	 reload_moving_timer();	// 16-02-2017
	}  
}
//******************************************
void req_photo_frame() // 11-03-2010
{
 if (photo_learn == 1) // is photo_learn activated
  {
   // wait for stop is activated to stop learning in req_photo_learn.c 27-06-2024
  }
 else
  {
	 EE_read(EE_par_31);
   temp3 = temp;
	 EE_read(EE_photo_pos_saved);	
	 if ((temp3 == 4) && (temp == 0) && (run_prog == 1) && (parameter == 31)) // is photo in frame selected and photo pos not saved 27-06-2024
		                                                                        // and prog is selected and parameter 31 is active
	  {
     photo_pos = position; // save position in RAM
     photo_pos_observed = 1;
		 show_photo_dis_point = 1; //30-04-2019
		 confirm_learn = 1; 
	   photo_pos_save_ready = 1;
		 confirm_timer = 600; // confirm learning	moved 09-11-2017 
     EE_read(EE_relearn_ph1);
     if (temp == 1) EE_write(EE_relearn_ph1, 0); // no photo1 relearn not needed	17-06-2024			
    }
 //  if (confirm_timer == 0) show_photo_dis_point = 0; //30-04-2019		moved 27-06-2024
	 if (photo_pos_save_ready == 1)
    {
     if ((open_active() == 0) & (close_active() == 0)) // is the door stopped
      {
       read_int32_eeprom(EE_run_range); // 
       run_range = data;
	   if (right_turn == 1) data = ((photo_pos) + (run_range/100)); // 16-08-2010
       else data = ((photo_pos) - (run_range/100)); // 16-08-2010
       adr = EE_photo_pos;
       write_int32_eeprom();
       photo_pos_save_ready = 0;
       EE_write(EE_photo_pos_saved, 1); // gem testbit i EEprom
      }
    }
   else
    {
     temp = EE_read(EE_photo_pos_saved);
     if (temp == 1)
      {
       read_int32_eeprom(EE_photo_pos);
       photo_pos = data;
       if (right_turn == 1) // er højredrejning valgt
        {
         if (position > photo_pos)
          {
           if (open_active() == 1) photo_pos_observed = 0; // if door moving up and position > learned photo position
          }
         else // position <= photo position
          {
           if (close_active() == 1) photo_pos_observed = 1; // if door moving down and position <= learned photo position
          }
        }
       else
        {
         if (position < photo_pos)
          {
           if (open_active() == 1) photo_pos_observed = 0; // if door moving up and position > learned photo position
          }
         else // position <= photo position
          {
           if (close_active() == 1) photo_pos_observed = 1; // if door moving down and position <= learned photo position
          }
        }
      }
     else photo_pos_observed = 0; // 23-03-2010 
    }
  }
}
//******************************************
void norm_mode()
{
 //*************************************************
 if ((open_active() == 0) & (close_active() == 0)) enc_low_sp_p = 0; // if door not moving clear low speed decelerete
 temp = EE_read(EE_limit_ready);
 if ((temp == 0) && (new_calc == 0)) // 0 = limits klar, new_calc = 0 = grænser er beregnet.
 {
  ecode2 = 0; // denne LED fejlkode cleares
 }
 else
 {
 ecode2 = 1; // denne LED fejlkode sættes
 st_area_ok = 0;                // old sk2,0 speed singleturn kan ikke være indlært når endestop ikke er indlært
 st_values_learned = 0;         // old sk2,3
 st_wear_observed = 0;          // old sk2,6
 st_init_learned = 0;					  // old sk2,7	 
 ols_out = 0;  //deaktiver ols så der kan indlæres
 cls_out = 0;  //deaktiver cls så der kan indlæres
 }
 //
 //ecode2 = 0; // debug så stop ikke trækkes
 //ecode7 = 0; // debug så stop ikke trækkes
 if ((ecode1==1) | (ecode2==1) | (ecode3==1) | (ecode4==1) | (ecode5==1) | (ecode7==1) | (ecode8==1) | (ecode9==1)) // 18-10-2011 26-10-2011
  {
   if (stop_tim == 0)
    {
     safe_stop = 1; // stop aktiveres - ingen LED kode endnu for dette
    }
  }
 else if (read_bat == 1) 
  {
   read_bat = 0; // This was battery request, battery voltage saved by posmake function 30-03-2010
  }
 else
 {
  safe_stop = 0; // deaktiver stop - der er ikke fejlkoder aktive
  
 // hent offset og limit grænser i EEPROM og gem i ram
 get_values();
 //
 calc_fine_adj();
 //
  if (right_turn == 1) // er højredrejning valgt
  {
  switch (offset_sign)
   {
  case 1: position = position + offset; //offset_sign == 1, offset skal lægges til
  break;
  case 0: //offset_sign == 0, offset skal trækkes fra
     if (e_type == 0x01) // er kostal enkoder valgt
     {
       if (position < offset)
       position = (maxposK - offset) + position;
       else    
       position = position - offset;
       break;
     }
     else // Feig eller Dal enkoder maxpos benyttes
     {
      if (position < offset)
      position = (maxpos - offset) + position;
      else    
      position = position - offset;
      break;
     }
   }
  }
  else // venstre drejning er valgt 
  {
  switch (offset_sign)
   {
  case 1:
  position = position + offset; //offset_sign == 1, offset skal lægges til
  break;
  case 0: //offset_sign == 0, offset skal trækkes fra
     if (e_type == 0x01) // er kostal enkoder valgt
     {
       if (position < offset)
       position = (maxposK - offset) + position;
       else    
       position = position - offset; 
       break;
     }
     else // Feig eller Dal enkoder maxpos benyttes
     {
      if (position < offset)
      position = (maxpos - offset) + position;
      else    
      position = position - offset;
      break;
     }
    }
  }
  if (out_of_range_tst()) // test om position er helt ude af område
  {
   if (ecode7_timer == 0) // hvis ecode7_timer aksepteres fejlen.
     { 
      if (stop_tim == 0)
       {
        safe_stop = 1; // stop aktiveres
       }
      ecode7 = 1; // fejl - position ude af indlært område - stop derfor dette er farligt
     }
  }  
  else
   { 
    if (right_turn == 1) 
     {
      ols_min = (ols_min + (fine_ols * fine_adj_unit)); // højredrejning valgt
      cls_max = (cls_max + (fine_cls * fine_adj_unit)); // højredrejning valgt
     }
    else // left turn selected
     {
      ols_max = (ols_max - (fine_ols * fine_adj_unit)); // venstedrejning valgt
      cls_min = (cls_min - (fine_cls * fine_adj_unit)); // venstedrejning valgt
     }
    //*****************************************
		chk_before_limits(); // 04-02-2022
    //***************************************** 
    // 4/8, 5/8, 6/8 eller 7/8 open justering af ols_min eller ols_max 01-09-09
    chk_half_open();
    //************** low speed deceleration 30-06-09 ***************************
    chk_deceleration();
    //*****************************************
    chk_moving(); 
    //*****************************************
    req_photo_frame();
    //*****************************************
    if (((position <= ols_max) && (position >= ols_min)) || ((use_half_open == 1) && (d_stopped_tim > 100) && (door_pos_half == 1) && (open_active() == 1))) // 
     {
      ecode7_timer = 30; // 200mS preset filtertimer til out of range fejl 26-01-2017
      cls_out = 0;       // deaktiver cls
      ols_out = 1;          // aktiver ols
			EE_read(EE_limits_check); // 30-04-2019
			if (bit_test(temp,0) == 1) // open limet is not seen yet after curtain switch activated
			 {
        temp = temp - 1;
				EE_write(EE_limits_check, temp); 
			 }	 
			E9mov_clr_open = 1; // 
      if (E9mov_clr_open && E9mov_clr_close) // 18-01-2010
       {
        moving_fail = 0;
        temp = EE_read(EE_moving_fail);
        if (temp == 1) EE_write(EE_moving_fail,0);
       }
      if (open_cnt_tb == 0) // 07-12-09 
       {
        enc_open_cnt++;
        open_cnt_tb = 1;
       }
      if ((position <= ols_max_old) && (position >= ols_min_old)) half_ols_active = 0; // half ols is not active, the normal ols is reached
     }
    else if (((position <= cls_max) && (position >= cls_min)) || ((use_half_open == 1) && (d_stopped_tim > 100) && (door_pos_half == 1) && (close_active() == 1))) //
     {
      ecode7_timer = 30; // 200mS preset filtertimer til out of range fejl 26-01-2017 
      ols_out = 0;       // deaktiver ols
      cls_out = 1;       // aktiver cls
			EE_read(EE_limits_check); // 30-04-2019
			if (bit_test(temp,1) == 1) // close limet is not seen yet after curtain switch activated
			 {
        temp = temp - 2;
				EE_write(EE_limits_check, temp); 
			 } 
			E9mov_clr_close = 1; // 
      if (E9mov_clr_open && E9mov_clr_close) // 18-01-2010
       {
        moving_fail = 0;
        temp = EE_read(EE_moving_fail);
        if (temp == 1) EE_write(EE_moving_fail,0);
       }
      open_cnt_tb = 0; // 07-12-09
     }
    else 
     {
      ecode7_timer = 30; // 200mS preset filtertimer til out of range fejl 26-01-2017 
      ols_out = 0;       // deaktiver ols
      cls_out = 0;       // deaktiver cls
     }
    //
    speed_check(); // Singleturn speedcontrol
    //
   }
 } 
}
//******************************************************************************

//**************************************************************************
void avago_com()
{

}
//**************************************************************************
//**************************************************************************
void timeout_fail()
{
// T1_kontrol = get_timer1();
 if (programming_mode == 0) // ingen stop aktivering ved manglende svar i prog mode da man da går i baglås
                    // hvis der ingen encoder er tilsluttet.
  {
   if (new_calc == 0) // stop skal ikke aktiveres hvis der er ved at gemmes i EEPROM
    {
     if ((stop_tim == 0) && (ecode1 == 1)) // 28-08-2012
      {
       safe_stop = 1;  // ja, stop aktiveres  - der er timeout fejl
      }
    }
  }
 else 
  {
   //ecode1 = 0; // 12-05-2016 shall be removed for showing no answer 29-03-2017
   safe_stop = 0;   // deaktiver stop ved prog mode og timeout fejl
   batt_tst_tim = 8000; // min. 16 sec. delay before measure
   load_spec_encoder_test();
   e_tst_running = 4;
  }
}
//**************************************************************************
void bat_test() // 26-03-2010
{
 if (bat_value < 681)
  {
   if (low_bat_filt > 1) 
    {
     low_bat = 1;
     low_bat_filt = 0;
    }
  }
 else
  {
   low_bat = 0;
   low_bat_filt = 0;
  }
}
//**************************************************************************
void ecode3_save()
{
 if (pwr_timer_SER == 0) //  Wait until power clear is tested 14-08-2019
 {	 
  temp = EE_read(EE_ecode3);
  if (ecode3 == 1)
   {
    if (temp == 1)
     {
     }
    else EE_write(EE_ecode3,1);
   }
  else
   {
    if (temp == 0)
     {
     }
    else EE_write(EE_ecode3,0);
   }
 }
}
//**************************************************************************
void clear_fails()
{
 ecode1 = 0; // slet disse fejlkoder når mekaniske endestop er valgt
 ecode2 = 0;
 ecode4 = 0;
 EE_read(EE_ecode4);
 if (temp == 1) EE_write(EE_ecode4,0);	// 16-02-2017	
 ecode7 = 0;
 ecode8 = 0;
 safe_stop = 0; // deactivate safe stop 
 if (ecode3 == 1) safe_stop = 1; // activate safe stop - motor running but should be stoppped (welded contacts)
 moving_fail = 0;
}
//***************************************************************************
void new_chk_weld()
 {
  pos_chk = position;
	pc_tim = 46; // 300mS
	ecode3_tim = 340; // 2200mS
	diff_moni_old_saved = 0;
 }	
//***************************************************************************
void chk_diff_moni() 
 {
	if (e_type == kostal)
	 {
		if (diff_moni_old_saved == 1)
		 {
			if (diff_moni >= diff_moni_old)
			 {
				if ((diff_moni - diff_moni_old) > 100) new_chk_weld();
			 }
			else
			 {
				if ((diff_moni_old - diff_moni) > 100) new_chk_weld(); 
			 }
		 }
		else
		 {
		  diff_moni_old = diff_moni;
      diff_moni_old_saved = 1;			 
		 }
	 }
	else // Dall/Feig
	 {
		if (diff_moni_old_saved == 1)
		 {
			if (diff_moni >= diff_moni_old)
			 {
				if ((diff_moni - diff_moni_old) > 1000) new_chk_weld();
			 }
			else
			 {
				if ((diff_moni_old - diff_moni) > 1000) new_chk_weld(); 
			 }
		 }
		else
		 {
		  diff_moni_old = diff_moni;
      diff_moni_old_saved = 1;			 
		 } 
	 }
 }
//***************************************************************************  
void noise_check()
 {
	if (direc == 1)
	 {
		if (direc_old == 1)
		 {
			diff_moni = position - pos_chk;
			chk_diff_moni(); 
			direc_old = 1; 
		 }
		else
		 {
			direc_old = 1;
			new_chk_weld();
		 }
	 }
	else
	 {
		if (direc_old == 0)
		 {
			diff_moni = pos_chk - position;
			chk_diff_moni(); 
			direc_old = 0; 
		 }
		else
		 {
			direc_old = 0;
			new_chk_weld();
		 } 
	 }
 }
//***************************************************************************
void chk_tacho()
 {
	if (d_stopped_tim == 0)
	 {
		if (chkbit_pc == 1)
		 {
			if (pc_tim == 0)
			 {
				if (pulse_counter > 3)
				 {
					tacho_fail = 1;
					pulse_counter = 0;
					pc_tim = 15;
				 }
				else
				 {
					pulse_counter = 0;
					pc_tim = 15;
				 }
			 }
			else if (pulse_chk == 1)
			 {
				pulse_counter++;
				pulse_chk = 0;
			 }
			else
			 {
			 }
		 }
		else
		 {
		  pulse_counter = 0;
      pc_tim = 15; // 100mS when tacho is used
      chkbit_pc = 1;			 
		 }
	 }
 }
//**************************************************************************************
void chk_weld() // 26-09-2011
{
 if (inverter_use == 1) // 27-01-2022
  {	
   if ((stop_pb == 1) | (stop_lid_pb == 1) | (stop_disp_pb == 1) | (stop_safety_chain == 1) | (e_stop_pb == 1))
    {
     if (d_stopped_tim < (d_stopped_tim_set - 75)) // 31-10-2019
      {
       if (stop_old == 0)
        {
         stop_old = 1;
         if (no_move == 1) 
          {
           enable_ser = 0; // disable SER is allowed the motor is not running when stop was activated
          }       
        }
       else
        {
         stop_old = 1;
        }
      }
    }
   else
    {
     stop_old = 0;
     enable_ser = 1;
    }  
  //
   if ((open_active() == 0) & (close_active() == 0))  // is door stopped 25-01-2022
    {
     if (E_limit == 0)
      {
		   chk_tacho(); // 26-01-2022 
       if (tacho_fail == 1) // 26-01-2022
        {
         no_move = 0; // 31-10-2019
         if ((ecode3_tim == 0) && (enable_ser == 1)) ecode3 = 1;
        } 
       else
        {
         no_move = 1;
         ecode3_tim = 200; // 154 = 1000mS filter to secure reset first time (by mechannical limits) 169 = 1100mS 30-08-2019 340 = 2210mS 12-03-2020
					                 // 200 = 1300mS for faster reaction 28-03-2023 
        } 
      }
     else if (chkbit_pc == 1) // is position check saved
      {
       if (pc_tim == 0)
        {
         if (position > pos_chk)
          {
           if ((position - pos_chk) > chk_weld_value) // chk_weld_value 300 dal encoder and 6 by Kostal encoder 07-11-2018
            {
             direc = 1; // 17-03-2020
             if (d_stopped_tim > 0) ecode3_tim = 200; // 154 = 1000mS reload as long as d_stopped_tim not 0 340 = 2210mS 12-03-2020
							                                        // 200 = 1300mS for faster reaction 28-03-2023
             no_move = 0; // door is still moving
             if ((ecode3_tim == 0) && (enable_ser == 1))
              {
               ecode3 = 1; // filter because communication takes 50-200mS when start motor
              }
             pos_chk = position;
             pc_tim = 46; // 180 mS 12-11-2018 300mS 30-08-2019
             noise_check(); // 17-03-2020
            }
           else
            {
             no_move = 1;
             new_chk_weld(); // 17-03-2020
            }
          }
         else
          {
           if ((pos_chk - position) > chk_weld_value) // chk_weld_value 300 dal encoder and 6 by Kostal encoder 07-11-2018
            {
             direc = 0; // 17-03-2020
             if (d_stopped_tim > 0) ecode3_tim = 200; // 154 = 1000mS reload as long as d_stopped_tim not 0 340 = 2210mS 12-03-2020
							                                        // 200 = 1300mS for faster reaction 28-03-2023
             no_move = 0; // door is still moving
             if ((ecode3_tim == 0) && (enable_ser == 1))
              {
               ecode3 = 1; // filter because communication takes 50-200mS when start motor
              }
             pos_chk = position;
             pc_tim = 46; // 180 mS 12-11-2018 300 mS 30-08-2019
             noise_check(); // 17-03-2020
            }
           else
            {
             no_move = 1;
             new_chk_weld();
            }
          }
        }
      }
     else
      {
       pos_chk = position;
       pc_tim = 46; // 180 mS 12-11-2018 between observations of positions 300mS 30-08-2019
       chkbit_pc = 1; // first position observed after stop
       ecode3_tim = 200; // 154 = 1000mS filter 18-10-2011 12-11-2018 169 = 1100mS 30-08-2019 340 = 2210mS 12-03-2020
				                 // 200 = 1300mS for faster reaction 28-03-2023
       diff_moni_old_saved = 0; // 17-03-2020
      }
   ecode3_save();
   }
  else
   {
    d_stopped_tim = d_stopped_tim_set; // (2 sec. "6.5mS cycle time) 30-08-2019 30-10-2019
    chkbit_pc = 0;
    no_move = 0; // door is activated
   }
  if ((run_prog == 1) && (pwr_timer == 0))
   {
    if (run_prog_old == 0)
     {
      ecode3 = 0; // clear of ecode3 when change from normal mode to prog. mode
	    tacho_fail = 0;	
      run_prog_old = run_prog;
      ecode3_tim = 200; // make 1.3 sek. delay for accecpt of new ecode3 set
     }
   }
  else run_prog_old = run_prog;
 //
  if (pwr_timer_SER == 1) //  14-08-2019
   { 
    if ((ols_out == 0) && (cls_out == 0))
     {
      ecode3 = 0; // clear of ecode3 when powerup and no limit are active 14-08-2019
     }
    else
     {
		  EE_read(EE_ecode3); // 
      if (temp == 1) ecode3 = 1; // set ecode3
      else ecode3 = 0;   	
		 }
   }	
  }
 else
 {
  //
 }	 
}
//**************************************************************************************
void save_new_pos_bat() // save newest position or battery values
 // running is still without using checksum byte5. Feig is uknown and Kostal have no checksum. 	
 {
	position = 0; // clear position - ready for new values
	switch(e_type) // change to switch routine 17-12-2020
	 {
		case 0: // Dal encoder
		 {
      if (bat_req_prepare == 1) // 
	     {
		    bat_value = rx_buffer[1] << 8; // 14-01-2021
		    bat_value += rx_buffer[0]; // 14-01-2021
		    read_bat = 1; // set this to prevent using values in norm_mode position control
	     }
	    else
	     {
				position = rx_buffer[3] << 24;
	  	  position += rx_buffer[2] << 16;
		    position += rx_buffer[1] << 8; 
        position += rx_buffer[0]; 
			 }
     }	
     break; 		 
		case 1: // is Kostal encoder selected (no battey request possible)
		 {
			position = rx_buffer[1] << 8; // get rx_buffer array 1 and place it in byte 2 in variabel
      position += rx_buffer[2]; // add rx_buffer[1] in byte 0 position variabel	 
	   }	
		 break;
		case 2: 
		 {
			position = rx_buffer[3] << 24;
	  	position += rx_buffer[2] << 16;
		  position += rx_buffer[1] << 8; 
      position += rx_buffer[0];  
		 }
	   break;
	  case 3: 
		 {
			// Avago encoder - not made yet 
		 }
	   break;
		case 4: // case 4 changed a lot 14-01-2021
		 {
			if (pack_56[0] == 0x56) // 
			 {
				temp = pack_56[0]; // debug only
			  // first powerup send from enkoder - not used yet
			 }
		  if (pack_3b[0] == 0x3B) // battery voltage on SCE RS485 encoder
		   {
				encod_safe = pack_3b[3]; // save for use later
				EE_read(EE_encod_safe);
        if (temp != encod_safe)
				 {
				  if (sce_fail_count == 2) sce_bat_state = 3; // SCE encoder power fail value changed since learning
					else
					 {
						sce_fail_count = 2;
           } 
				 }
		    else if (pack_3b[2] < 29)
				 {
				  sce_bat_state = 1; // battery voltage below alarm level
					sce_fail_count = 0; // shall not be easy to get other faults
         }					 
				else if	((pack_3b[2] < 26) || (pack_3b[2] > 40))
				 { 
				  if (sce_fail_count == 2) sce_bat_state = 2; // battery too low
					else
					 {
						sce_fail_count = 2; 
					 }						
         }
				else
				 {
				  sce_bat_state = 0; // battery OK
					sce_fail_count = 0;  
         } 					 
		   }
		  if (pack_3a[0] == 0x3A) // Er det en position 
		   {
			  position = pack_3a[4] << 24;
	      position += pack_3a[3] << 16;
		    position += pack_3a[2] << 8; 
        position += pack_3a[1];
		   }
      else
			 {
        temp = pack_3a[0]; // debug only
			 }				
		 }
		 break;
	  default:
		 break; // should not be possible
	 }	
 } 
//********* 
void enc_test_moving()
 {	
  if ((e_type == 0x00)||(e_type == 0x02)) // er DAL eller Feig enkoder valgt
   {
    if (bat_req_prepare == 1) // was it battery request
     {
      // don't check movement 
     }
    else
     {
      temp = pack_3a[1]; // save in temp register  for test later 14-01-2021
      if (rx_buffer2 > 250)
       {
        rx_buffer2 = rx_buffer2 - 10; // values moved a little to fit in 1 byte without roll over
        temp = temp - 10; // 
       }
      if (rx_buffer2 < 5)
       {
        rx_buffer2 = rx_buffer2 + 10; // values moved a little to fit in 1 byte without roll over
        temp = temp + 10; // 
       }
      if ((temp <= (rx_buffer2 - 2 )) || (temp >= (rx_buffer2 + 2 )))
                                      // byte 1 is 2. msb - changes approx. 1/500 rpm.
       {   
        motor_run = 1; // interval on +/- 2 - Encoder ia moving - motor runing
       }    
      else motor_run = 0;
     }
   }

if (e_type == 0x01) // is Kostal encoder selected, 
   {
    if (pack_3a[1] == 0xFA)
		 {
		  if (ecode7_timer2 !=0) ecode7_timer2--; 	// 30-04-2019
		  if (ecode7_timer2 == 0) ecode7=1; // kostal pot. fail
		 }
		else ecode7_timer2 = 30; // preset filter
		// 
		if (pack_3a[1] == 0xFB) 
		 {	
			if (ecode8_timer !=0) ecode8_timer--;  // 30-04-2019
			if (ecode8_timer == 0) ecode8=1; // kostal fail in power supply
		 }
		else ecode8_timer = 30; // preset filter
    //			 
		temp = pack_3a[1]; // save in temp register  for test later 
    if (rx_buffer2 > 250)
   {
    rx_buffer2 = rx_buffer2 - 10; // values moved a little to fit in 1 byte without roll over
    temp = temp - 10; // 
   }
   if (rx_buffer2 < 5)
   {
    rx_buffer2 = rx_buffer2 + 10; // values moved a little to fit in 1 byte without roll over
    temp = temp + 10; // 
   }
    if ((temp <= (rx_buffer2 - 2 )) 
    ||
       (temp >= (rx_buffer2 + 2 ))) // byte 1 is 2. msb - changes approx. 1/500 rpm.
    {   
        motor_run = 1; // interval på +/- 2 - Encoder ia moving - motor runing
    }    
    else motor_run = 0;
   }
}   		
 //********************************************************************************
 
void setup_controlbits()
{	
// ********** setup of kontrol bits for encoder ********************************** 
	programming_mode = 0; // 
	if (run_prog == 1) programming_mode = 1; // run_prog is not the same in interpreter and encoder
  if ((para_test == 1) && ((parameter == 13) || (parameter == 15) || (parameter == 51))) programming_mode = 0; 
  if ((para_test == 1) && (parameter == 41)) programming_mode = 0; // need electronic limits to work if these active
	if ((para_test == 1) && (parameter == 70)) programming_mode = 0; // need electronic limits to work if these active 12-02-2020
	if ((run_prog == 1) && (parameter == 31)) programming_mode = 0; // need elec. limits to work for photo learn 27-06-2024
  //if ((para_test == 1) && (parameter == 31)) programming_mode = 0; // need elec. limits to work for photo learn 11-03-2010
  if ((p_value == 1) && ((parameter == 43) || (parameter == 42))) programming_mode = 0; //need electronic limits to work	
	e_limit_open_learning_active = 0;  
  if ((parameter == 12) && ((confirm_timer > 0) || (para_test == 1))) e_limit_open_learning_active = 1;
	e_limit_close_learning_active = 0; 
  if ((parameter == 14) & ((confirm_timer > 0) || (para_test == 1))) e_limit_close_learning_active = 1;
	e_limit_curt_rep_learning_active = 0;
	if ((parameter == 60) & ((confirm_timer > 0) || (para_test == 1))) e_limit_curt_rep_learning_active = 1; // 07-02-2022
	save_active_learning_limit = 0;
	if ((confirm_timer > 0) && (parameter != 82))
	{
	save_active_learning_limit = 1; // confirming is 2 seconds 
	}
	// 
	if ((st_values_learned == 1) & (st_learn_confirm > 0)) st_speed_learn_ok = 1; // speed learning is OK if values is OK
	else st_speed_learn_ok = 0; // 04-01-2018
//	
  save_h_open_learn = 0; // must be cleared if not set by the following lines	
	if ((parameter == 16) && (confirm_timer > 0)) save_h_open_learn = 1; // save half open point 20-12-2021
  EE_read(EE_par_71); //is dock leveler  type 54 selected on any relay
  if ((parameter == 71) && (temp == 54) && (confirm_timer > 0) && run_prog == 1) save_h_open_learn = 1; // save half open point uP2 23-06-2021
	EE_read(EE_par_72); //is dock leveler  type 54 selected on any relay
  if ((parameter == 72) && (temp == 54) && (confirm_timer > 0) && run_prog == 1) save_h_open_learn = 1; // save half open point uP2 23-06-2021
	EE_read(EE_par_74); //is dock leveler  type 54 selected on any relay
  if ((parameter == 74) && (temp == 54) && (confirm_timer > 0) && run_prog == 1) save_h_open_learn = 1; // save half open point uP2 23-06-2021
  EE_read(EE_par_75); //is dock leveler  type 54 selected on any relay
  if ((parameter == 75) && (temp == 54) && (confirm_timer > 0)) save_h_open_learn = 1; // save half open point uP2 23-06-2021 
	//
	if ((ecode1==1) | (ecode2==1) | (ecode3==1) | (ecode4==1) | (ecode5==1) | (ecode7==1) | (ecode8==1)) encoder_errors = 1;
	else encoder_errors = 0; // summerized encoder error used for relays output signal	
 } 
//*********************************************************************************	
void encoder_TX()
{
 if ((pwr_timer == 0) && (e_type != sce)) // 18-11-2020
  {	
	 //usart2_de_man_on; // 02-11-2021 prepare for sending. 10-03-2022 Not in use when RS485 can be set up in Cubemx
   //delay_us(1);		// wait for stabilize	
	 if (bat_req_prepare == 1)
	  {
	   tx1[0] = RS485_BAT_STATE_ID;
		 if (HAL_UART_Transmit(&huart2, tx1, 1, 1) == HAL_OK)
			{
       // Everything good!
      }
     else
  	  {
       // Error message!
      }		
		}	
	 else if ((e_type == 0x00)||(e_type == 0x02))   // is DAL or Feig encoder selected 
		{
     if (USART2->BRR == 0x4E2) // is baud rate change to 19200 - value found by debug
		  { 
		   tx1[0] = RS485_ENKODER_ID;	
       if (HAL_UART_Transmit(&huart2, tx1, 1, 1) == HAL_OK)
			  {
         // Everything good!
        }
       else
  	    {
         // Error message!
        }
		  }	
     else
		  {
       setup_uart_19200();
		  }			 
		}	
	 else		
	  {
		 if (USART2->BRR == 0x4E2) // is baud rate change to 19200
		  {	
		   tx1[0] = 0xA2;	  					    // Kostal request header
       tx1[1] = RS485_KOSTAL_I;        // Kostal control byte for increasing values by clockwice rotation
       if (HAL_UART_Transmit(&huart2, tx1, 2, 1) == HAL_OK) // Kostal encoder
			  {
         // Everything good!
        }
       else
  	    {
         // Error message!
        }
		  }
		 else
		  {
       setup_uart_19200();
      }			 
	}
	//if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == 0) 	skal vist ikke bruges
  //usart2_de_man_off; // 02-11-2021 ready for receive. 10-03-2022 Not in use when RS485 can be slected in Cubemx
  //delay_us(1);		// wait for stabilize		
	HAL_UART_AbortReceive(&huart2); // clear receiver if of of sync cause of noise from motor	
	//HAL_UART_AbortReceive_IT(&huart2); // er det ikke den som skal være der? 03-11-2021
	//__HAL_UART_FLUSH_DRREGISTER(&huart2);	
	__HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST); // skal også være der for at nulstille RDR register hvis der 
		                                                       // ligger en byte 02-05-2018	- Findes vist kke ved STM32F407 er den nødvendig når abort er der?		
 	//
	if ((e_type == 0x00)||(e_type == 0x02))   // is DAL or Feig encoder selected 	
	 {
    HAL_UART_Receive_IT(&huart2, uart2_buf, 5); // preparing receive of 5 bytes		
	 }
	else if (e_type == 1)
	 {
		HAL_UART_Receive_IT(&huart2, uart2_buf, 3); // preparing receive of 2 bytes	by kostal 
	 }
	else // 
	 {
		// 
	 }		
	//
	}
 else
  {
	 // usart2_de_man_off; // 02-11-2021 always ready for receive for this encodertype 13-12-2021.  10-03-2022 Not in use when RS485 can be slected in Cubemx
   if (USART2->BRR == 0xF0) // is baud rate change to 100000 - value found by debug
		{		
	   b = __HAL_TIM_GET_COUNTER(&htim7); // get timer which count in 10uS step
	   if (b > 48000) // check whether the SCE com is stopped debug
	    {		 
       HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0);	// restart not available in STM32F407
       //HAL_UART_AbortReceive_IT(&huart2); // er det ikke den som skal være der? 03-11-2021
	     //__HAL_UART_FLUSH_DRREGISTER(&huart2);			 
      HAL_UART_AbortReceive(&huart2);                           // clear receiver if of of sync cause of noise from motor	
      __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST); // skal også være der for at nulstille RDR register hvis der 
      HAL_UART_Receive_IT(&huart2, uart2_buf, 1); // preparing receive of 1 bytes	by SCE RS485 speedtech 18-11-2020
      }
	    //__HAL_TIM_SetCounter(&htim7,0); //debug 30-11-2020	
	    // no setup here for sce encoder	
    }
	 else
	  {
     setup_uart_100000(); // change to 100000 baud
    }		 
  }
 
} 

void kostal_moving_chk()
{
 if ((position_old < (position + 5)) && (position_old > (position - 5)))
  {
   if (moving_timer == 0) // measure interval 2
    {
     if ((position_old2 < (position + 5)) && (position_old2 > (position - 5)))
		  {
			 moving_fail = 1;
       temp = EE_read(EE_moving_fail); // test om der er gemt i EE  18-01-2010
       if (temp == 0) EE_write(EE_moving_fail, 1); // gem så i EE
	     E9mov_clr_open = 0; // clear testregister for sletning når moving fail observeres
       E9mov_clr_close = 0; // clear testregister for sletning når moving fail observeres 
		  }
     else
	  	{
			 reload_moving_timer();	
		  } 
		}
	}
 else
  {
	 reload_moving_timer();	
	}
}
//
void dall_moving_chk()
{
 if ((position_old < (position + 451)) && (position_old > (position - 451)))
  {
   if (moving_timer == 0) // measure interval 2
    {
     if ((position_old2 < (position + 451)) && (position_old2 > (position - 451)))
		  {
			 moving_fail = 1;
       temp = EE_read(EE_moving_fail); // test om der er gemt i EE  18-01-2010
       if (temp == 0) EE_write(EE_moving_fail, 1); // gem så i EE
	     E9mov_clr_open = 0; // clear testregister for sletning når moving fail observeres
       E9mov_clr_close = 0; // clear testregister for sletning når moving fail observeres 
		  }
     else
	  	{
			 reload_moving_timer();	
		  } 
		}
	}
 else
  {
	 reload_moving_timer();	
	}
}
//
void curtain_check() // 30-04-2019 
{
 	EE_read(EE_par_60); // 26-11-2021
	if (temp == 1)
	 {
		curtain_sw = !free_3; // Curtain is NC switch 29-05-2024
    if (curtain_sw == 1)
		 {
			stop_open_or_close(); 
			if ((close_pb) || (close_disp_pb)) i_close = 1; // set inactive close if close pushbutton is active
			if (kip_pb == 1) i_kip = 1;                     // set inactive kip if kip pushbutton is active 
			if ((open_pb) || (open_disp_pb)) i_open = 1;    // set inactive close if close pushbutton is active 
		 }
	  EE_read(EE_curtain_sw);	 
		if ((curtain_sw == 1) && (temp == 0)) 
		 {	
		  EE_write(EE_curtain_sw, 1); // save in EEPROM if not already saved
			EE_write(EE_limits_check, 3); // preset - both limits must be seen before impulse ok again - open limit = bit 0, close limit = bit 1
     }
		EE_read(EE_curtain_sw);	 
		if ((temp > 0) && (curtain_sw == 0))
		 {			
		  EE_read(EE_limits_check);
      if (temp == 0) EE_write(EE_curtain_sw, 0); // save in EEPROM if not already saved		 
		 }
	 }
	else
	 {
    EE_read(EE_curtain_sw);	 // EE_curtain not in use 
		if (temp > 0) 
		 {			
		  EE_write(EE_curtain_sw, 0); // clear in EEPROM if not already saved
			EE_write(EE_limits_check, 0); // clear also this in EEPROM if not already  
		 }
		EE_read(EE_limits_check); // 28-03-2023
		if (((bit_test(temp,0) == 0) &  (bit_test(temp,1) == 0)) && (temp > 0)) // check for faulty value of EE_limits_check
		 {
      EE_write(EE_limits_check, 0); // clear because value is faulty 28-03-2023
		 }	  
	 }		
}
//*************************************
void inv_meas_save() // 30-06-2021
{
 inv_meas_interval = 31; // 31 = 200mS
 switch (inv_meas_num)
  {
   case 0: inv_num_0_value = position;
           inv_meas_num = 1;
           break;
   case 1: inv_num_1_value = position;
           inv_meas_num = 2;
           break;
   case 2: inv_num_2_value = position;
           inv_meas_num = 3;
           break;
   case 3: inv_num_3_value = position;
           inv_meas_num = 4;
           break;
   case 4: inv_num_4_value = position;
           if (right_turn == 1)
            {
             if ((inv_num_2_value - inv_num_1_value < 27) && (inv_num_1_value - inv_num_0_value < 27))
              {
               inv_low_sp = 1;
               inv_meas_ok = 1;
               break; 
              }
             else if ((inv_num_4_value - inv_num_3_value < 27) && (inv_num_2_value - inv_num_1_value < 27))
              {
               inv_low_sp = 1;
               inv_meas_ok = 1;
               break; 
              }
             else
              {
               inv_low_sp = 0;
               inv_meas_ok = 1;
               break;  
              }

            }
           else
            {
             if ((inv_num_1_value - inv_num_2_value < 27) && (inv_num_0_value - inv_num_1_value < 27))
              {
               inv_low_sp = 1;
               inv_meas_ok = 1;
               break; 
              }
             else if ((inv_num_3_value - inv_num_4_value < 27) && (inv_num_1_value - inv_num_2_value < 27))
              {
               inv_low_sp = 1;
               inv_meas_ok = 1;
               break; 
              }
             else
              {
               inv_low_sp = 0;
               inv_meas_ok = 1;
               break;  
              }
            }
   default: 
           break; // 
  }
}
//*************************************
void inv_speed_moni() // 30-06-2021
{
 if (inv_meas_ok == 1)
  {
  }
 if (open_active() == 1) // kører port op 
  {
   if (inv_meas_interval == 0)
    {
     inv_meas_save();
    }
  }
 else
  {
   inv_meas_num = 0;
   inv_meas_interval = 31;
  }
}
//**************************************
void setup_uart_19200()
{
 huart2.Instance = USART2;
 huart2.Init.BaudRate = 19200;
 if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }	
}
//**************************************
void setup_uart_100000()
{
 huart2.Instance = USART2;
 huart2.Init.BaudRate = 100000;
 if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }	
}
//

