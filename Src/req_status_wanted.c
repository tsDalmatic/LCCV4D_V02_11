
#include "stm32f0xx_hal.h"
#include "Global_Var.h"
#include "Defines.h"

extern uint8_t i_open;                       // inactive open testbit
extern void stop_open_or_close(void);
void req_status_wanted() // 
 {
  if ((cls == 1) & (dil_2 == 1))
   {
    disp_status = 1;
    //close_gate_off; // stop motor up and down
    //open_gate_off; //  -      -      -
		stop_open_or_close(); 
    imp_close = 0;
    imp_open = 0;
    i_open = 1;
   }
  else disp_status = 0;
 }
