
#include "stm32f0xx_hal.h"
#include "Global_Var.h"
#include <Defines.h>
//

uint8_t reg_fill; // testbit for push code

void push_code_req() // 10-03-2010
{
 static uint8_t push_count; // push code counter	29-03-2017	
 	
 if ((stop_pb == 1) | (stop_lid_pb == 1) | (stop_disp_pb == 1))
  {
   if ((close_pb) || (close_disp_pb))
    {
     if (reg_fill == 1)
      {
      }
     else
      {
       push_code <<= 1; // 0 will be put in bit 0 after shift
       bit_set(push_code,0);
       push_count++;
       reg_fill = 1;
      }
    }
   else if ((open_pb == 1) | (open_disp_pb == 1))
    {
     if (reg_fill == 1)
      {
      }
     else
      {
			 push_code <<= 1; // 0 will be put in bit 0 after shift
			 bit_clear(push_code,0);	// corrected 29-03-2017	
       push_count++;
       reg_fill = 1;
      }
    }
   else reg_fill = 0; 
  }
 else
  {
   push_code = 0;
   push_count = 0;
   push_code_tim = 1200; // 4 sec
  }
 //
 if ((push_code == 56) && (push_count == 6)) edge_photo_owr = 1;
 if (push_count == 7) edge_photo_owr = 0; //wrong push count, clear again
 if (push_code_tim == 0) push_code = 0; // timer is runout, clear push_code
}
