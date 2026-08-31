
#include "stm32f0xx_hal.h"
#include "Global_Var.h"
#include <Defines.h>
#include <main.h>
#include <eprom.h>

void save_error(uint8_t);

void last_error_upd() 
{
// error is set directly when it occurs in the program. 
// When new error occur, the error is save in EEPROM and older error is pushed in EEPROM.
// Last 10 errors can be shown.
if (safety_test == 1) // is safety test running 13-12-2016
 {
	// don't register safety test errors. it's shown in display
 }
else
 {
 if (e1_mon == 0) e1_mon_old = 0; //
 else
  {
   if (e1_mon_old == 1) 
    {
    }
   else
    {
     save_error(1); // error no 1 must be saved
     e1_mon_old = 1;
    }
  }
//
 if (e2_speed == 0) e2_speed_old = 0; //
 else
  {
   if (e2_speed_old == 1) 
    {
    }
   else
    {
     save_error (2); // error no 2 must be saved
     e2_speed_old = 1;
    }
  }
//
 if (e3_run == 0) e3_run_old = 0; //
 else
  {
   if (e3_run_old == 1) 
    {
    }
   else
    {
     save_error (3); // error no 3 must be saved
     e3_run_old = 1;
    }
  }
//
 if (e4_ser == 0) e4_ser_old = 0; //
 else
  {
   if (e4_ser_old == 1) 
    {
    }
   else
    {
     save_error (4); // error no 4 must be saved
     e4_ser_old = 1;
    }
  }
//
 if (e5_photo == 0) e5_photo_old = 0; //
 else
  {
   if (e5_photo_old == 1) 
    {
    }
   else
    {
     save_error (5); // error no 5 must be saved
     e5_photo_old = 1;
    }
  }
//
 if (e6_edge == 0) e6_edge_old = 0; //
 else
  {
   if (e6_edge_old == 1) 
    {
    }
   else
    {
     save_error (6); // error no 6 must be saved
     e6_edge_old = 1;
    }
  }
//
 if (e7_tacho == 0) e7_tacho_old = 0; //
 else
  {
   if (e7_tacho_old == 1) 
    {
    }
   else
    {
     save_error (7); // error no 7 must be saved
     e7_tacho_old = 1;
    }
  }
//
 if (e8_wear == 0) e8_wear_old = 0; //
 else
  {
   if (e8_wear_old == 1) 
    {
    }
   else
    {
     save_error (8); // error no 8 must be saved
     e8_wear_old = 1;
    }
  }
//
 if (e9_pos_change == 0) e9_pos_change_old = 0; //
 else
  {
   if (e9_pos_change_old == 1) 
    {
    }
   else
    {
     save_error (9); // error no 9 must be saved
     e9_pos_change_old = 1;
    }
  }
//
 if (e10_edge == 0) e10_edge_old = 0; // 10-11-2021
  else
   {
    if (e10_edge_old == 1) 
     {
     }
    else
     {
      save_error (10); // error no 10 must be saved
      e10_edge_old = 1;
     }
   }
//	
 if (e20_fail == 0) e20_fail_old = 0; // 14-04-2010
 else
  {
   if (e20_fail_old == 1) 
    {
    }
   else
    {
     save_error (20); // error no 20 must be saved
     e20_fail_old = 1; // 14-04-2010
    }
  }  
 //
 if (e21_fail == 0) e21_fail_old = 0; // 26-10-2011
 else
  {
   if (e21_fail_old == 1) 
    {
    }
   else
    {
     save_error (21); // error no 21 must be saved
     e21_fail_old = 1; // 26-10-2011
    }
  }  
 if (ecode7 == 0) ecode7_old = 0; // 11-11-2016
 else
  {
   if (ecode7_old == 1) 
    {
    }
   else
    {
     save_error (25); // error no 25 must be saved
     ecode7_old = 1; // 11-11-2016
    }
  }  
 //	
 if (e22_stop == 0) e22_stop_old = 0; // 18-11-2016
 else
  {
   if (e22_stop_old == 1) 
    {
    }
   else
    {
     save_error (22); // error no 22 must be saved
     e22_stop_old = 1; // 18-11-2016
    }
  }	
 //
 if (e23_chain == 0) e23_chain_old = 0; // 18-11-2016
 else
  {
   if (e23_chain_old == 1) 
    {
    }
   else
    {
     save_error (23); // error no 23 must be saved
     e23_chain_old = 1; // 18-11-2016
    }
  }	
 //
 if (e27_modbus == 0) e27_modbus_old = 0; // 22-03-2017
 else
  {
   if (e27_modbus_old == 1) 
    {
    }
   else
    {
     save_error (27); // error no 27 must be saved
     e27_modbus_old = 1; // 22-03-2017
    }
  }	
 //	
 if (e28_wdt == 0) e28_wdt_old = 0; // 22-03-2017
 else
  {
   if (e28_wdt_old == 1) 
    {
    }
   else
    {
     save_error (28); // error no 28 must be saved
     e28_wdt_old = 1; // 22-03-2017
    }
  }		
 //
 if (e29_xtal == 0) e29_xtal_old = 0; // 22-03-2017
 else
  {
   if (e29_xtal_old == 1) 
    {
    }
   else
    {
     save_error (29); // error no 29 must be saved
     e29_xtal_old = 1; // 22-03-2017
    }
  }	
//
 if (e31_inverter == 0) e31_inverter_old = 0; // 22-03-2017 04-11-2021
 else
  {
   if (e31_inverter_old == 1) 
    {
    }
   else
    {
     save_error (31); // error no 30 must be saved
     e31_inverter_old = 1; // 22-03-2017
    }
  }		
//	
 if (e32_weld == 0) e32_weld_old = 0; // 22-03-2017 04-11-2021 1-01-2022
 else
  {
   if (e32_weld_old == 1) 
    {
    }
   else
    {
     save_error (32); // 
     e32_weld_old = 1; // 22-03-2017
    }
  }		
 //		
 if (e33_unlock_fail == 0) e33_unlock_fail_old = 0; // 04-03-2026
 else
  {
   if (e33_unlock_fail_old == 1) 
    {
    }
   else
    {
     save_error (33); // 
     e33_unlock_fail_old = 1; // 
    }
  }		
 //		
 }
}
//
//***********************************************************************
void save_error(uint8_t new_E) // routine time about 10 x 4 mS = 40 mS. WDT should be 256 mS befor reset
                            // If an error occur and the door should stop by that, the door is stopped
                            // before this rutine is executing. This means that the 40 mS delay don't
                            // delay a stop command.
{
//disable_interrupts(GLOBAL); // all interrupts OFF -Behøves ikke ifølge Anders
EE_read(EE_error_9);
EE_write(EE_error_10,temp);
EE_read(EE_error_8);
EE_write(EE_error_9,temp);
EE_read(EE_error_7);
EE_write(EE_error_8,temp);
EE_read(EE_error_6);
EE_write(EE_error_7,temp);
EE_read(EE_error_5);
EE_write(EE_error_6,temp);
EE_read(EE_error_4);
EE_write(EE_error_5,temp);
EE_read(EE_error_3);
EE_write(EE_error_4,temp);
EE_read(EE_error_2);
EE_write(EE_error_3,temp);
EE_read(EE_error_1);
EE_write(EE_error_2,temp);
EE_write(EE_error_1,new_E); // Saving newest error
//enable_interrupts(GLOBAL); // enable interrupts again after EEPROM writeing -Behøves ikke ifølge Anders
}
