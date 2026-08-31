#include "stm32f0xx_hal.h"
#include "Global_Var.h"
#include "Defines.h"
#include "main.h"
#include "eprom.h"
#include "encoder.h"
#include "modbus.h"
static HAL_StatusTypeDef status = HAL_OK; // definering af status og start initialisering er HAL_OK
extern UART_HandleTypeDef huart2;
extern IWDG_HandleTypeDef hiwdg;
extern void st_clr_val(void);
void clr_val(void);
void new_inverter_setup(void);
void inverter_para_check(void);
extern uint8_t EE_par_11_old;	// check register for changes made 13-03-2017
extern uint8_t something_saved;   // Test for something saved in EEPROM 14-03-2017
extern uint8_t inv_com_state; // used for inverter_para_check 21-05-2017
extern uint16_t inverter_param; // used for inverter_para_check 21-05-2017
extern uint8_t uart2_buf[7];    // uart 2 buffer 18-11-2020
extern uint8_t curtain_sw;           // curtain switch bt ref input 30-04-2019
extern uint8_t edge_setup;           // 02-12-2021
extern uint8_t inverter_use;			   // 1 = inverter use is active - else contactor use 09-06-2022
uint16_t temp6; // used for inverter_para_check 21-05-2017

//
void start_ini() // Start initialyze
{
 digit_1_on; // T9 OFF no LED digit active in start of factory reset 09-06-2022
 digit_2_on; // T8 OFF
 digit_3_on; // T7 OFF	
 digit_4_on; // T5 OFF	
 // init_eeprom();      // EEPROM read and save i RAM array moved to main at start 28-03-2023	
 //
 EE_read(EE_new_chk_1); // double check for new EEPROM cause normal EE_check can fail 29-03-2017
 temp1 = temp;
 EE_read(EE_new_chk_2);	
 if ((temp1 != 0xAA) | (temp != 0x55))	// double check for new EEPROM cause normal EE_check can fail 29-03-2017
  {
	 EE_write(EE_new_chk_1,0xAA);
	 EE_write(EE_new_chk_2,0x55);
   EE_write(EE_check,0xFF);
   EE_read(EE_new_chk_1);	
   temp1 = temp; // debug	
   EE_read(EE_new_chk_2);	
   temp1 = temp; // debug			
  }
 //	
 safety_test_off;	
 //EE_read(EE_par_11); // adjust for kostal encoder
 //if ((temp == 5) || (temp == 6)) // Is Kostal encoder selected, cubemx set to none
 // {
 //	 huart2.Init.Parity = UART_PARITY_EVEN;
 //	 huart2.Init.WordLength = UART_WORDLENGTH_9B;	
 //	 HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0);	
 // }
 //	
 huart2.Init.Parity = UART_PARITY_NONE; // start with sce encoder setup
 huart2.Init.WordLength = UART_WORDLENGTH_8B;	
 // HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0); // findes ikke i STM32F407	
 //HAL_UART_AbortReceive(&huart2);    // clear receiver if of of sync cause of noise from motor		
 HAL_UART_AbortReceive_IT(&huart2); // er det ikke den som skal være der? 05-11-2021
	__HAL_UART_FLUSH_DRREGISTER(&huart2);		// 05-11-2021
 //__HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST); // skal også være der for at nulstille RDR register hvis der - duer ikke med STM32F407 05-11-2021
 HAL_UART_Receive_IT(&huart2, uart2_buf, 1); // preparing receive of 7 bytes	by SCE RS485 speedtech 18-11-2020		                                                       // ligger en byte 21-09-2018		
 //	
 EE_read(EE_par_11); 
 EE_par_11_old = temp; // must be the same at powerup to prevent reset of limits 13-03-2017
 EE_read(EE_par_21); // 02-05-2011
 //	
 EE_read(EE_ecode4);
 if (temp == 1) ecode4 = 1; // set ecode 4 again if there was power off when ecode4 was set 16-02-2017
 //
 EE_read(EE_par_5); // 09-06-2022 be sure to setup inverter parameter if factory reset
 if (temp > 1)  inverter_use = 1;
 else inverter_use = 0;	
 //
 //**** check of new EEprom
 EE_read(EE_check);
 if (temp == 255)
  {
	 EE_write(EE_ecode3,0);	
   temp = 0;
   for (temp=0;temp<=239;temp++) // electronic counter is also cleared 31-03-2010
   EE_write(temp,0); // all EE adresses 0 to 239 is cleared
	 //
   EE_write(EE_par_5, 0);  // save default value default LED version 09-11-2023		
  }
 //**** check of factory clear 31-03-2010
 EE_read(EE_factory);
 if (temp == 0)
 {
  for (temp=0;temp<=159;temp++) // electronic counter is not cleared
  EE_write(temp,0); // all EE adresses 0 to 159 is cleared (0x00 to 0x9F)
  EE_write(EE_check, 255); 
	edge_setup = 0; // 02-12-2021
 }
 //**********************encoder related powerup from EEPROM *******************************************
	
 read_int32_eeprom(EE_enc_cls); // hent gemte cls værdi hvis der skal ændres senere på endestop 27-11-09
 enc_cls = data;
 read_int32_eeprom(EE_enc_ols); // hent gemte ols værdi hvis der skal ændres senere på endestop 27-11-09
 enc_ols = data;
 temp = EE_read(EE_moving_fail);
 if (temp == 0) moving_fail = 0;
 else moving_fail = 1;
 //
read_int16_eeprom(EEst_ce1);
temp16 = data;
if (temp16 == 0) st_clr_val();
else
 {
  read_int16_eeprom(EEst_ce1);
  st_ce1 = data;
  read_int16_eeprom(EEst_ce2);
  st_ce2 = data;
  read_int16_eeprom(EEst_ce3);
  st_ce3 = data;
  read_int16_eeprom(EEst_ce4);
  st_ce4 = data;
  read_int16_eeprom(EEst_ce5);
  st_ce5 = data;
  read_int16_eeprom(EEst_ce6);
  st_ce6 = data;
  read_int16_eeprom(EEst_ce7);
  st_ce7 = data;
  read_int16_eeprom(EEst_ce8);
  st_ce8 = data;
  read_int16_eeprom(EEst_ce9);
  st_ce9 = data;
  read_int16_eeprom(EEst_ce10);
  st_ce10 = data;
  //
  read_int16_eeprom(EEspeed_unit);
  speed_unit = data;
  //
  read_int16_eeprom(EEst_oe1);
  st_oe1 = data;
  read_int16_eeprom(EEst_oe2);
  st_oe2 = data;
  read_int16_eeprom(EEst_oe3);
  st_oe3 = data;
  read_int16_eeprom(EEst_oe4);
  st_oe4 = data;
  read_int16_eeprom(EEst_oe5);
  st_oe5 = data;
  read_int16_eeprom(EEst_oe6);
  st_oe6 = data;
  read_int16_eeprom(EEst_oe7);
  st_oe7 = data;
  read_int16_eeprom(EEst_oe8);
  st_oe8 = data;
  read_int16_eeprom(EEst_oe9);
  st_oe9 = data;
  read_int16_eeprom(EEst_oe10);
  st_oe10 = data;
  //
  read_int16_eeprom(EEst_ice1);
  st_ice1 = data;
  read_int16_eeprom(EEst_ice2);
  st_ice2 = data;
  read_int16_eeprom(EEst_ice3);
  st_ice3 = data;
  read_int16_eeprom(EEst_ice4);
  st_ice4 = data;
  read_int16_eeprom(EEst_ice5);
  st_ice5 = data;
  read_int16_eeprom(EEst_ice6);
  st_ice6 = data;
  read_int16_eeprom(EEst_ice7);
  st_ice7 = data;
  read_int16_eeprom(EEst_ice8);
  st_ice8 = data;
  read_int16_eeprom(EEst_ice9);
  st_ice9 = data;
  read_int16_eeprom(EEst_ice10);
  st_ice10 = data;
  //
  read_int16_eeprom(EEst_ioe1);
  st_ioe1 = data;
  read_int16_eeprom(EEst_ioe2);
  st_ioe2 = data;
  read_int16_eeprom(EEst_ioe3);
  st_ioe3 = data;
  read_int16_eeprom(EEst_ioe4);
  st_ioe4 = data;
  read_int16_eeprom(EEst_ioe5);
  st_ioe5 = data;
  read_int16_eeprom(EEst_ioe6);
  st_ioe6 = data;
  read_int16_eeprom(EEst_ioe7);
  st_ioe7 = data;
  read_int16_eeprom(EEst_ioe8);
  st_ioe8 = data;
  read_int16_eeprom(EEst_ioe9);
  st_ioe9 = data;
  read_int16_eeprom(EEst_ioe10);
  st_ioe10 = data;
  //
  read_int32_eeprom(EEarea12);
  area12 = data;
  read_int32_eeprom(EEarea23);
  area23 = data;
  read_int32_eeprom(EEarea34);
  area34 = data;
  read_int32_eeprom(EEarea45);
  area45 = data;
  read_int32_eeprom(EEarea56);
  area56 = data;
  read_int32_eeprom(EEarea67);
  area67 = data;
  read_int32_eeprom(EEarea78);
  area78 = data;
  read_int32_eeprom(EEarea89);
  area89 = data;
  read_int32_eeprom(EEarea910);
  area910 = data;
  //
	enc_speed_o = 0;              // adaptive speed control check reg.
  enc_speed_c = 0;              // adaptive speed control check reg.
	enc_open_cnt = 5;             // open counter for adaptive speed adjustment - only 5 opening before new update 
  //
  st_area_ok = 1; // areas OK
  st_values_learned = 1; // values OK
  st_init_learned = 1; // initial values learned
 }	
//*************************************************************************************************
 EE_read(EE_check);
 if (temp == 255)
  {
	 if ((inverter_use == 1) && (e27_modbus == 0)) new_inverter_setup(); // 29-08-2017 only if modbus is working 09-06-2022	
	 //  
   EE_write(EE_par_1, 3);  // save default value
   EE_write(EE_par_2, 1);  // save default value 08-06-2022
	 //EE_write(EE_par_5, 0);  // save default value default LED version 09-11-2021	08-06-2022 no change by factory clear
   EE_write(EE_par_11, 1);  // save default value
   EE_write(EE_par_13, 5);  // save default value
   EE_write(EE_par_15, 5);  // save default value
   EE_write(EE_par_16, 0);  // save default value
   EE_write(EE_par_17, 0);  // save default value
   EE_write(EE_par_21, 1);  // save default value
   EE_write(EE_par_22, 0);  // save default value
	 EE_write(EE_par_24, 5);  // save default value 12-11-2021
   EE_write(EE_par_25, 2);  // save default value 16-03-2022
   EE_write(EE_par_26, 0);  // save default value 12-11-2021
   EE_write(EE_par_27, 0);  // save default value 12-11-2021
   EE_write(EE_par_29, 0);  // save default value
   EE_write(EE_par_31, 1);  // save default value 08-06-2022
   data = 0; //
   adr = EE_par_32;
   write_int16_eeprom();
   EE_write(EE_par_33, 0);  // save default value
   EE_write(EE_par_34, 0);  // save default value 23-08-2011
   EE_write(EE_par_35, 3);  // save default value 20-12-2021
   EE_write(EE_par_36, 0);  // save default value 16-05-2013 01-12-2021
   EE_write(EE_par_41, 0);  // save default value
   EE_write(EE_par_42, 50);  // save default value
   EE_write(EE_par_43, 50);  // save default value
   EE_write(EE_par_44, 2);  // save default value
   EE_write(EE_par_51, 2);  // save default value
   EE_write(EE_par_52, 1);  // save default value
   EE_write(EE_par_53, 30);  // save default value
	 EE_write(EE_par_55, 0);  // save default value 03-04-2024
   EE_write(EE_par_58, 0);  // save default value
   EE_write(EE_par_59, 0);  // save default value
   EE_write(EE_par_60, 0);  // save default value	01-12-2021 curtain switch 
	 EE_write(EE_par_71, 0);  // save default value AUX1
	 EE_write(EE_par_72, 25);  // save default value AUX2 22-11-2025
	 EE_write(EE_par_74, 0);  // save default value AUX4
	 EE_write(EE_par_75, 0);  // save default value AUX5
	 EE_write(EE_par_77, 0);  // save default value 
	 EE_write(EE_par_78, 5);  // save default value 
	 EE_write(EE_par_79, 3);  // save default value 
	 EE_write(EE_par_80, 1);  // save default value 08-12-2025
   EE_write(EE_par_81, 3);  // save default value 09-03-2010 03-11-2010 22-03-2013 
	 EE_write(EE_par_84, 0);  // save default value 09-05-2017 29-04-2019
	 EE_write(EE_par_85, 5);  // save default value 04-02-2022 11-12-2025_d
	 EE_write(EE_par_86, 5);  // save default value 04-02-2022 11-12-2025_d
	 EE_write(EE_par_91, 15);  // save default value
	 EE_write(EE_par_92, 15);  // save default value
	 EE_write(EE_par_93, 5);  // save default value
	 EE_write(EE_par_94, 5);  // save default value
	 EE_write(EE_par_115, 30);  // save default value 09-05-2017
   EE_write(EE_par_125, 30);  // save default value 09-05-2017
   //
   EE_write(EE_check, 0);  // not default saving by next powerup
   clr_val();
   calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 26-10-2011
   EE_write(EE_pu_check, temp3); // save in EE_pu_check for later test 26-10-2011
	 prog_mode_tb = 0; // after powerup always start with parametermeter 1 in progmode. 29-04-2019	
	}
 else
  {
	 //EE_read(EE_ecode3); // 
   //if (temp == 1) ecode3 = 1; // set ecode3
   //else ecode3 = 0;           //  -		
	 //*****************************************	
		
		
	 		
   EE_read(EE_warning); //get and save warning if active
   warning = temp; // 
   warn_started = 0; // warning can be runing by powerup
   interlock = 0;
   EE_read(EE_interlock); //get and save interlock
   if (temp == 1) interlock = 1; //
   EE_read(EE_par_41);
   if (temp == 4)
    {
     EE_read(EE_s_speed_saved);
     if (temp == 10)
			{
       bit_set(sk2,0); // values ok and wear clear by powerup if singleturn speed is selected
       bit_set(sk2,3);
       bit_set(sk2,7);
      }
    }
   else
    {
     read_int16_eeprom(EEce1);
     temp2 = data;
     if (temp2 == 0) clr_val();
     else
      {
       read_int16_eeprom(EEce1);
       ce1 = data;
       read_int16_eeprom(EEce2);
       ce2 = data;
       read_int16_eeprom(EEce3);
       ce3 = data;
       read_int16_eeprom(EEce4);
       ce4 = data;
       read_int16_eeprom(EEoe1);
       oe1 = data;
       read_int16_eeprom(EEoe2);
       oe2 = data;
       read_int16_eeprom(EEoe3);
       oe3 = data;
       read_int16_eeprom(EEoe4);
       oe4 = data;
       //
       read_int16_eeprom(EEice1);
       ice1 = data;
       read_int16_eeprom(EEice2);
       ice2 = data;
       read_int16_eeprom(EEice3);
       ice3 = data;
       read_int16_eeprom(EEice4);
       ice4 = data;
       read_int16_eeprom(EEioe1);
       ioe1 = data;
       read_int16_eeprom(EEioe2);
       ioe2 = data;
       read_int16_eeprom(EEioe3);
       ioe3 = data;
       read_int16_eeprom(EEioe4);
       ioe4 = data;
       //
       read_int16_eeprom(EEomsk12);
       omsk12 = data;
       read_int16_eeprom(EEomsk23);
       omsk23 = data;
       read_int16_eeprom(EEomsk34);
       omsk34 = data;
       EE_read(EE_div_2); // 21-12-2010 only 8 bit value
       div_2 = temp;
       read_int16_eeprom(EEpv05);
       pv05 = data;
       read_int16_eeprom(EE_sp_adj); // 21-12-2010
       sp_adj = data;
       //
       speed_o = 0;
       speed_c = 0;
       open_cnt = 5; // only 5 opening before new update 
       //
       EE_read(EE_c_point_pos);
       EE_read(EEcntpos + temp); //get and save warning if active
       counterpos = temp; //
       temp = div_2;
       while (temp > 0)
        {
         counterpos = (counterpos * 2); // adjust counterpos according to div_2
         temp--;
        }
       //
       bit_set(sk2,0);
       bit_set(sk2,3);
       bit_set(sk2,7);
      }
    }
  }
 if ((inverter_use == 1) && (e27_modbus == 0)) inverter_para_check(); // 29-08-2017 no check if modbus not work	09-06-2022	
 calc_EE_cksum(); // calculate safety EEPROM adresses and saved in temp3 26-10-2011
 // EE_write(EE_pu_check, temp3); // debug save in EE_pu_check for later test 26-10-2011, skal fjernes efter debug	
 EE_read(EE_pu_check); // read and saved in temp 26-10-2011
 // temp3++; // debug test
 if (temp3 != temp) e21_fail = 1; // 26-10-2011
 else e21_fail = 0; // 26-10-2011
 //
 EE_read(EE_par_60);
 if (temp == 1) curtain_sw = 1; // must be set to 1 at power up when NC switch is used for curtain sw 14-08-2019	
 e5_photo = 0; // extra clear at power up because photo circuit is slow to power ON. 11-04-2023	
 if (inverter_use == 1) door_up_on; // turn ON inverter by powerup. 11-04-2023	
//	
}
//
void clr_val()
{
 speed_o = 0;
 speed_c = 0;
 ce1 =0;
 ce2 =0;
 ce3 =0;
 ce4 =0;
 oe1 =0;
 oe2 =0;
 oe3 =0;
 oe4 =0;
 bit_set(sk1,5);
 sk2 =0;
 ice1 =0;
 ice2 =0;
 ice3 =0;
 ice4 =0;
 ioe1 =0;
 ioe2 =0;
 ioe3 =0;
 ioe4 =0; 
 e8_wear = 0;	
}
//
void new_inverter_setup() // setup of other inverter parameters except 115 and 125  
 { 
	do
	 {		
	  HAL_IWDG_Refresh(&hiwdg); // 14-03-2017 refresh to prevent WDT timeout of 50mS
	  something_saved = 1; // test bit for saving in EEPROM 
	  status = getModbusState();
	 }
	// 
	while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F110E_RATED_POWER, 5); // 0.4 kW motor, Parameter 100 LSis 14-09-2018 5 = 1.5 kW 27-10-2021
	data = 5; // Save in LCC V3 Inverter EEPROM also 21-05-2017 27-10-2021	 
	adr = EE_par_100;
  write_int16_eeprom(); 
	do
	 {
	  HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS	
    status = getModbusState();
	 }
	// 
  while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F120F_RATED_VOLT, 230); // 230 V motor, Parameter 101
	data = 230; // Save in LCC V3 Inverter EEPROM also 21-05-2017
  adr = EE_par_101;
  write_int16_eeprom(); 
	do
	 {
	  HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS	
    status = getModbusState();
	 }
  //
  while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F120D_RATED_CURR, 65); // rated current motor, Parameter 102 27-10-2021
	data = 65; // Save in LCC V3 Inverter EEPROM also 21-05-2017 6.5A 27-10-2021	 
  adr = EE_par_102;
  write_int16_eeprom(); 
	do
	 {
	  HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS	
    status = getModbusState();
	 }
  //
  while (status == HAL_BUSY);
	HAL_Delay(3);
  // rated speed motor, Parameter 103
	data = 1380; // Save in LCC V3 Inverter EEPROM also 21-05-2017
  adr = EE_par_103;
  write_int16_eeprom(); 
	do
	 {
	  HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS	
    status = getModbusState();
	 }
  //
	// 
	while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F120B_MOTOR_POLES, 4); // 4 motor pole counts by factory reset, 26-09-2018
	do
	 {
	  HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS	
    status = getModbusState();
	 } 
	// 
	while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F120C_RATED_SPEED, 120); // 4 Hz rated slip speed by factory reset, 26-09-2018
	do
	 {
	  HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS	
    status = getModbusState();
	 } 
	//  
	while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F1112_RATED_FREQ, 5000); // rated speed motor, Parameter 104 LSis base frequency 14-09-2018
	data = 5000; // Save in LCC V3 Inverter EEPROM also 21-05-2017
  adr = EE_par_104;
  write_int16_eeprom(); 
	do
	 {
	  HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS	
    status = getModbusState();
	 }
  // 
//	while (status == HAL_BUSY);
//	HAL_Delay(3);
//  setModbusParam(F800_AUTOTUNE, 0); // rated speed motor, Parameter 105
//	data = 0; // Save in LCC V3 Inverter EEPROM also 21-05-2017
//  adr = EE_par_105;
//  write_int16_eeprom(); 
//	do
//	 {
//	  HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS	
//    status = getModbusState();
//	 }
  //  
	while (status == HAL_BUSY); // Already defined by F0005 by LSis - set it anyway? 14-09-2018
	HAL_Delay(3);
  setModbusParam(F1101_TARGET_FREQ, 5000); // 50 Hz parameter 110 LSis 14-09-2018
	data = 5000; // Save in LCC V3 Inverter EEPROM also 21-05-2017
  adr = EE_par_110;
  write_int16_eeprom(); 
	do
	 {
	  HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS	
    status = getModbusState();
	 }
	// 
  while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F1232_STP_1_FREQ, 1000); //35 Hz parameter 111 09-05-2017 21-09-2018 29-04-2019
	data = 1000; // Save in LCC V3 Inverter EEPROM also
  adr = EE_par_111;
  write_int16_eeprom();  
  do
	 {
		HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS 
    status = getModbusState();
	 }
	// 
  while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F1103_ACC_TIME, 10); // 1 sec. parameter 112 LSis 14-09-2018
	data = 10; // Save in LCC V3 Inverter EEPROM also
  adr = EE_par_112;
  write_int16_eeprom();  
  do
   {
		HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS 
    status = getModbusState();
	 }
	// 
  while (status == HAL_BUSY);
	HAL_Delay(3);
	setModbusParam(F1247_STEP_1_DEC, 10); // 1 sec. parameter 113 09-05-2017 29-04-2019
	data = 10; // Save in LCC V3 Inverter EEPROM also
  adr = EE_par_113;
  write_int16_eeprom();  
  do
   {
		HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS 
    status = getModbusState();
	 }
	// 
  while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F1104_DEC_TIME, 3); // 0.3 sec. parameter 114 09-05-2017 LSis 14-09-2018 29-04-2019
	data = 3; // Save in LCC V3 Inverter EEPROM also
  adr = EE_par_114;
  write_int16_eeprom();  
  do
   {
		HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS 
    status = getModbusState();
	 }
	// 
  while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F1233_STP_2_FREQ, 3000); // 30 Hz parameter 120  09-05-2017 29-04-2019
	data = 3000; // Save in LCC V3 Inverter EEPROM also
  adr = EE_par_120;
  write_int16_eeprom();  
  do
   {
		HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS 
    status = getModbusState();
	 }
	// 
  while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F1234_STP_3_FREQ, 1000); //10 Hz parameter 121  09-05-2017 21-09-2018 29-04-2019
	data = 1000; // Save in LCC V3 Inverter EEPROM also
  adr = EE_par_121;
  write_int16_eeprom();  
  do
   {
		HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS 
    status = getModbusState();
	 }
	// 
  while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F1248_STEP_2_ACC, 10); // 1 sec. parameter 122
	data = 10; // Save in LCC V3 Inverter EEPROM also
  adr = EE_par_122;
  write_int16_eeprom();  
  do
   {
		HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS 
    status = getModbusState();
	 }
	// 
  while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F124B_STEP_3_DEC, 20); // 2 sec. parameter 123  09-05-2017 29-04-2019
	data = 20; // Save in LCC V3 Inverter EEPROM also
  adr = EE_par_123;
  write_int16_eeprom();  
  do
   {
		HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout of 50mS 
    status = getModbusState();
	 }
	// 
  while (status == HAL_BUSY);
	HAL_Delay(3);
  setModbusParam(F1114_FREQ_UP_LIMIT, 10000); // 100 Hz max frequency. Must be higher than 110,111,120,121 
	data = 10000; // Save in LCC V3 Inverter EEPROM also
  adr = EE_par_109;
  write_int16_eeprom();   
	do
   {
		HAL_IWDG_Refresh(&hiwdg); // refresh to prevent WDT timeout of 50mS 
    status = getModbusState();
	 }
	//
  while (status == HAL_BUSY);
	 
 }
 //
 void EEPROM_check(uint16_t EE_adr, enum modbusParam EURA_EEPROM_adr)
  {
	 // getModbusParam take approx. 1.5 mS to send. After this there is approx. 2 mS pause.
   // Answer of this take approx. 2 mS. This means that parameter is first ready 5.5mS after start request
   // setModbusParam take also about 1.5 mS and modbus respond the same as send so this take
   // also about 2 mS pause and 2 sek. answer. We don't know the minimum time between to setModbusParam.
   // However in this routine there is a getModbusParam between to of these. 		
   read_int16_eeprom(EE_adr);
   temp2 = data;
	 do	
	 status = getModbusState();
	 while (status == HAL_BUSY);
	 status = getModbusParam(EURA_EEPROM_adr, &inverter_param); // get inverter parameter in blocking mode
	 HAL_IWDG_Refresh(&hiwdg); // refresh to prevent WDT timeout of 50mS
	 do
	 status = getModbusState();
	 while (status == HAL_BUSY);
	 HAL_Delay(3);	// wait 3 x 1.66 mS = 5 mS before modbus result is ready. should be ready but we don't know
                  // whether this delay can be lower
   if (temp2 != inverter_param)
	  {	
     status = setModbusParam(EURA_EEPROM_adr, temp2);  // Value is forgotton save again
		 do	
	   status = getModbusState();
	   while (status == HAL_BUSY);	
		 HAL_Delay(3);	// wait 5 mS again to be sure inverter is ready for a new command
	  } 
	 else
	  {
	  }			
  }	 
 //
 void inverter_para_check() // 21-05-2017
 {
	HAL_IWDG_Refresh(&hiwdg); // refresh to prevent WDT timeout of 50mS 
	EEPROM_check(EE_par_100, F110E_RATED_POWER);
	if ((temp2 < 2) || (temp2 > 6)) // check if motor is in selected range else reset to factory by next power-up
		                              // new power-up is maybe started by watchdog timer 12-06-2017 14-09-2018 LSis
	                                // range for M100 is 0.4 kW to 2.2 kW 27-10-2021	 
	 {
		EE_write(EE_overv,255);
    EE_write(EE_factory,0); 
		ver_show = 450; // Show version and factory reset there is no new complete new powerup 	
    start_ini();    // false power setup  - new Inverter setup needed 09-06-2022  
	 }
	//	
  EEPROM_check(EE_par_101, F120F_RATED_VOLT);
	EEPROM_check(EE_par_102, F120D_RATED_CURR);		 
  EEPROM_check(EE_par_103, F120C_RATED_SPEED);		 
	EEPROM_check(EE_par_104, F1112_RATED_FREQ);		 
 // EEPROM_check(EE_par_105, F800_AUTOTUNE);	 
  EEPROM_check(EE_par_109, F1114_FREQ_UP_LIMIT);		 
  EEPROM_check(EE_par_110, F1101_TARGET_FREQ);		 
  EEPROM_check(EE_par_111, F1232_STP_1_FREQ);		 
  EEPROM_check(EE_par_112, F1103_ACC_TIME);		 
  EEPROM_check(EE_par_113, F1247_STEP_1_DEC);		 
  EEPROM_check(EE_par_114, F1104_DEC_TIME);	// LSis 14-09-2018	 
  EEPROM_check(EE_par_120, F1233_STP_2_FREQ);		 
  EEPROM_check(EE_par_121, F1234_STP_3_FREQ);		 
  EEPROM_check(EE_par_122, F1248_STEP_2_ACC);		 
  EEPROM_check(EE_par_123, F124B_STEP_3_DEC);		 
 }
 
