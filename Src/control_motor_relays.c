#include "stm32f0xx_hal.h"
#include <Defines.h>
#include "Global_Var.h"
#define con_off_time 4   //(7 x 3.33mS = 23.3 mS) Main contactor off time before activate small direction relays
                         // 4 x 3.33mS = 13.2 mS
#define up_off_time 3    // time from switching off up direction relay - to switching on down direction relay
#define down_off_time 3  // time from switching off up direction relay - to switching on down direction relay
#define con_on_time 0    // time from switching on, up or down direction relay - to switching on main contactor
#define  d_stopped_tim_set 308   // reload time for door stopped timer 25-01-2022

extern uint8_t contactor_off_tim;				 // preset at startup to prevent uintended activate
extern uint8_t up_relay_tim;       //
extern uint8_t down_relay_tim;     //
extern uint8_t weldtimer;          //
extern uint8_t con_on_delay;       //
extern uint8_t start_close;			  	 // Variabel for start running down. Should be 0 by powerup 
extern uint8_t start_open;			  		 // Variabel for start running up. Should be 0 by powerup
extern uint8_t mon_ph1; // contactor monitor for welding check
extern uint8_t mon_ph2; // contactor monitor for welding check
extern uint8_t inverter_use;			   // 1 = inverter use is active - else contactor use 09-11-2021
extern uint16_t d_stopped_tim;       // door stoppped timer used in encoder.c 
//
extern void stop(void);
//
void chk_welding(void);
void motor_off(void);

//
void control_motor_relays()
{
 if (inverter_use == 0) // 09-11-2021
  {	 
   if (contactor == 1) contactor_off_tim = con_off_time;
   if (door_up == 1) up_relay_tim = up_off_time;
   if (door_down == 1) down_relay_tim = down_off_time;
   //
   if ((start_open == 0) && (start_close == 0))	
    {
	   motor_off(); 
    }
   else if ((start_open == 1) && (start_close == 1))
    {
     e35_output_fail = 1;
	   motor_off();	
    }	
   else if ((start_open == 1) && (start_close == 0))
    {
     if ((contactor == 1) && (door_up == 1)) 
	    {
		   // keep running up 
	    }
	   else if ((contactor == 1) && (door_down == 1))
		  {
		   // aux3_on; // activate brake 25-08-2017	changed from AUX1 to AUX3 04-11-2021. Controlled from relæ funktions setup 10-01-2022
		   contactor_off;	
		  }
	   else if (contactor_off_tim == 0)
		  {
		   if (door_down == 1)
		    {
         door_down_off;
        }
       else if (down_relay_tim == 0)
		    {
         if (door_up == 1)
			    {
           if (con_on_delay == 0)
				    {
					   //aux3_off;	  // release brake 25-08-2017. Controlled from relæ funktions setup 10-01-2022
				     contactor_on;
				    }
           else
					  {
						 //aux3_off; // release brake a little time before 25-08-2017. Controlled from relæ funktions setup 10-01-2022					
					  }
				  }
         else
			    {
				   door_up_on;
				   con_on_delay = con_on_time;
			    }				 
		    }
       else
		    {
        }			 
		  }
	   else
		  {
		  }			
    }	 
   else // start_open == 0 and start_close == 1 is active
	  {
     if ((contactor == 1) && (door_down == 1)) 
	    {
		   // keep running down 
	    }
	   else if ((contactor == 1) && (door_up == 1))
		  {
		   //aux3_on; // activate brake 25-08-2017. Controlled from relæ funktions setup 10-01-2022	
		   contactor_off;	
		  }
	   else if (contactor_off_tim == 0)
		  {
		   if (door_up == 1)
		    {
         door_up_off;
        }
       else if (up_relay_tim == 0)
		    {
         if (door_down == 1)
			    {
           if (con_on_delay == 0)
				    {
					   //aux3_off; // release brake 25-08-2017. Controlled from relæ funktions setup 10-01-2022	
				     contactor_on;
				    } 
				   else
					 {
					  //aux3_off; // release brake a little time before 25-08-2017. Controlled from relæ funktions setup 10-01-2022
           }						 
				  }
         else
			    {
           door_down_on;
					 con_on_delay = con_on_time;
			    }				 
		    }
       else
		    {
        }			 
		  }
	   else
		  {
		  }			
    }	
   chk_welding();
  }
 else // inverter use
  {
	 //
  }	 
}

//*********************
void motor_off()
{
 //aux3_on; // activate brake 25-08-2017. Controlled from relæ funktions setup 10-01-2022	
 if ((contactor == 0) && (contactor_off_tim == 0))
  {
	 door_up_off;
   door_down_off;		
  }
 else
	{
	 contactor_off;	
	}
}
//**********************
void chk_welding() // 31-01-2017
 {
	if (contactor == 0)
	 {
		if (weldtimer == 0)
		 {
			if ((mon_ph1 == 1) || (mon_ph2 == 1))
			 {
        e32_weld = 1; // set fail code and switch off small relays - only clear at powerup and new stop activation
        door_up_off;
        door_down_off;	
        stop();				 
			 }
			else
			 {
			 }
		 }
		else
		 {
		 }
	 }
	else
	 {
		weldtimer = 60; // 200mS by 3.3mS cycle time 
		d_stopped_tim = d_stopped_tim_set; // used in encoder.c for half open checking 27-01-2022
	 }

if ((stop_pb == 1) | (stop_lid_pb == 1) | (stop_disp_pb == 1)) 	e32_weld = 0 ; // clear this fail be stop PB or power-up 13-01-2022
 }
