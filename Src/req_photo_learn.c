
#include "stm32f0xx_hal.h"
#include <Defines.h>
#include "Global_Var.h"
#include <eprom.h>

extern void stop(void); 
extern uint8_t open_active(void);           // special inverter functions
extern uint8_t close_active(void);          // special inverter functions

void req_photo_learn() // 11-03-2010
{
 if (photo_learn == 1)
  {
   if (photo_learn_tim == 0) // 23-03-2010
    {
     if ((stop_pb == 1) | (stop_lid_pb == 1) | (stop_disp_pb == 1)) // is stop activated after photo_learn is activated 27-06-2024
      {
       if ((close_active() == 1) | (open_active() == 1)) stop(); // if door is moving 
       else 
        {
         photo_learn = 0;
         para_test = 0;
         p_value = 0;
         par_val = 0;
         next_para = 0; // prevent next para select
        }
      }
    }
   else
	  {
     EE_read(EE_photo_pos_saved);
		 if (temp == 1) EE_write(EE_photo_pos_saved,0); // clear this for new learning 27-06-2024  
	   stop(); // Photo learn timer is not 0, stop door
	  }
  }
}
