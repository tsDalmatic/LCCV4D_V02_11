#include "stm32f0xx_hal.h"
#include "Global_Var.h"
#include <Defines.h>
#include <main.h>
#include <eprom.h>
//
#define turn_off 1         // 1 = activated, 0 = deactivated
#define turn_off_tim 90000 // 3.33ms x 90000 = 5 min , debug 9000 = 30 sec.
//
void green_timers(void);
void r_green_tim(uint16_t);
extern void flash_aux0(void);
extern void flash_aux1(void);
//
uint8_t down_count; //
uint8_t M1_sig; //
uint8_t M2_sig; //
uint8_t M3_sig; //
uint8_t M4_sig; //
uint8_t flash_active; //
uint16_t time_green_in; //
uint16_t time_green_out; //
uint16_t after_green_in; //
uint16_t after_green_out; //
uint32_t tim_turn_off = 90000; // turn lamp off on CLS after time ended if activated by define 5 min by powerup
//
void spec_traffic_light()
 // made from ladder diagram from Dalmatic PLC control	
{
 down_count = !down_count; // toggle to double timer times
 // 	
 if (cls == 0) tim_turn_off = turn_off_tim; // Preset time 3.33ms x 90000 = 5 min
 else
  {
   if ((tim_turn_off != 0) && (turn_off == 1)) tim_turn_off--; //
  }
 //	
 if (((free_1) || (M1_sig) || (M3_sig)) && (!M2_sig) && (time_green_in > 0)) M1_sig = 1;
 else M1_sig = 0;
 //	
 if (((free_2) || (M2_sig) || (M4_sig)) && (!M1_sig) && (time_green_out > 0)) M2_sig = 1;
 else M2_sig = 0;
 //
 if ((((free_1) && (M2_sig)) || (M3_sig)) && (!M1_sig)) M3_sig = 1; 
 else M3_sig = 0;
 //
 if ((((free_2) && (M1_sig)) || (M4_sig)) && (!M2_sig)) M4_sig = 1; 
 else M4_sig = 0;
 //	
 if (((M1_sig) || (M2_sig)) && (ols == 0)) flash_active = 1;
 else flash_active = 0;
 //
 if ((M1_sig) || (M2_sig) || (aux2) || (aux3)) open_pb = 1; // simulate open push
 else 
  {
   // do nothing, it's turned off by door control norml running
	}	
 //
 if ((M2_sig) && (ols) && (!aux2) && (after_green_in == 0)) aux3_on; //
 else aux3_off;	
 //
 if ((aux3) && (!free_2))	
  {
	 if ((time_green_out != 0) && (down_count == 1)) time_green_out--; // downcount tmer
	 EE_read(EE_par_93); // preset timer
	 r_green_tim(temp);		
   after_green_out = data;	
	}
 else
  {
	 EE_read(EE_par_91); // preset timer
	 r_green_tim(temp);	
   time_green_out = data;
	 if ((after_green_out != 0) && (down_count == 1)) after_green_out--;		
  }
 //
 if (((M1_sig) && (!aux3)) || (aux2)) aux1_on;	// red ON outside
 else if ((!flash_active) && (!aux3) && (tim_turn_off > 0)) aux1_on; //
 else if ((flash_active) && (!aux3)) flash_aux1(); //aux1 flash
 else aux1_off;	// red off inside
 //
 if ((M1_sig) && (ols) && (!aux3) && (after_green_out == 0)) aux2_on; //
 else aux2_off;		
 //
 if ((aux2) && (!free_1))	
  {
	 if ((time_green_in != 0) && (down_count == 1)) time_green_in--; // downcount tmer
	 EE_read(EE_par_94); // preset timer
	 r_green_tim(temp);		
   after_green_in = data;	
	}
 else
  {
	 EE_read(EE_par_92); // preset timer
	 r_green_tim(temp);	
   time_green_in = data;
	 if ((after_green_in != 0) && (down_count == 1)) after_green_in--;		
  }
 //
 if (((M2_sig) && (!aux2)) || (aux3)) aux0_on;	// red ON inside
 else if ((!flash_active) && (!aux2) && (tim_turn_off > 0)) aux0_on; //
 else if ((flash_active) && (!aux2)) flash_aux0();	// flash
 else aux0_off; // red off inside	 
}
//
void r_green_tim(uint16_t new_data)
 {
	data = new_data * 150; //
	return;  
 }
//************************************** 
