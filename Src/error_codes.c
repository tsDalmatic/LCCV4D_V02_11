#include "stm32f0xx_hal.h"
#include "Global_Var.h"
#include <Defines.h>
#include <main.h>
//#include <eprom.h>
//
#define LEDper 0x1C7    // 1.5 sek. LED periodepause 455 x 3.3mS 29-12-2021
#define LED_on_tid 0x5B // LED ON tid ved blink = 91 x 3.3 mS = 300 mS 29-12-2021
#define LED_off_tid 0x5B // LED OFF tid ved blink 
extern uint8_t wic_slack;              // filter testbit for wicket door and slack rope 10-11-2021
//
void reload_led_per() // 24-01-2012
{
 if (e9_pos_change == 1) templed = 9; // position don't move. unless stop active e1_mon have higest priority
 if (e8_wear == 1) templed = 8; // 
 if ((bit_test(low_bat,0) == 1) & (cls == 1)) templed = 6; //
 if (e31_inverter == 1) templed = 5; // 25-11-2024	
 if (e4_ser == 1) templed = 4;
 if (e3_run == 1) templed = 3;
 if (e2_speed == 1) templed = 2;
 if (e1_mon == 1) templed = 1;
 if (templed > 0) blinktimer = LED_on_tid;
 led_pad_off;   //deactivate LED if set ON by stop_pb
} 
//
void error_codes()
{
if ((safety_test != 1) && (safety_after_time == 0))// only when not safety test 
 {	
	if ((stop_pb == 1) | (e_stop_pb == 1) | (stop_lid_pb == 1) | (stop_disp_pb == 1) | 
		(stop_safety_chain == 1) | (wic_slack == 1) | (safe_stop == 1)) led_pad_on; 
	  // fail on wicket door input also activate LED 14-12-2018 22-11-2023
  else if (edge_photo_fail == 1)
   {
    if ((pause1 == 0) && (blinktimer == 0)) blinktimer = 75; // fast flash
    if (blinktimer > 0) // flash period not ended
     {
      led_pad_on; //activate LED
      pause1 = 75; // preload of pause1 time - fast flash
     }
    else led_pad_off; //deactivate LED 
   }
  else if ((blinktimer == 0) && (pause1 == 0) && (pause2sek == 0) && (templed == 0)) reload_led_per();  // is flash period ended
  else
   {
    if (blinktimer > 0) // flash period not ended
     {
      led_pad_on; //activate LED
      pause1 = LED_off_tid; // preload of pause1 tid
     }
    else
     {
      led_pad_off; // deactivate LED
      if (pause1 == 0)
       {
        if (templed > 0) templed--;
        if (templed > 0) blinktimer = LED_on_tid;
       }
      else pause2sek = LEDper;
     }
   }
 }
 }
 
