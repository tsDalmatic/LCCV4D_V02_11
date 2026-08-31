#include "stm32f0xx_hal.h"
#include "Global_Var.h"
#include "eprom.h"
#include "Defines.h"
//
extern uint8_t open_active(void);           // special inverter functions
extern uint8_t close_active(void);          // special inverter functions
extern uint8_t h_ols;                       // 1/2 open limit switch after sw-filter 02-12-2021
//
void a_run_timer(void);
void tst_clr_c_b(void);

//
uint8_t adap_door_run;               // testbit for adaptive runtimer learning
uint16_t open_timer;            // measuring timer for adaptive runtime.

//
void adap_run_time()
{
 EE_read(EE_par_51);
 if (temp > 0)
  {
   if (temp == 3) // is adaptive runtime selected
    {
     EE_read(E_run);
     if (temp == 1)
      {
      }
     else
      {
       if ((run_prog == 1) && (parameter == 51)) // is prog mode and parameter 51 selected
        {
         run_timer = 34500; // preset run time temporary to max value 230 sec.
         if (cls == 1)
          {
           if (open_active() == 1) // is door moving up
            {
             adap_door_run = 1;
             a_run_timer();
            }
           else
            {
             tst_clr_c_b();
             a_run_timer();
            }
          }
         else
          {
           tst_clr_c_b();
           a_run_timer();
          }
        }
       else run_timer = 0; // it´s not possible to learn adaptiv runtimer in run mode - ready for stop
      }
    }
  }	
}
//
void a_run_timer()
{
 static uint8_t open_timer_doubler;                   // testbit for open_timer doubler to fit run_timer doubler	
 if (open_active() == 1) // is door moving up
  {
   if (cls == 1) open_timer = 0; // is close limit switch active
   else
    {
     open_timer_doubler = !open_timer_doubler; // toggle doubler bit
     if (open_timer_doubler == 1) open_timer++;
     if (open_timer > 34500) open_timer = 34500; // keep max value if learning to long
    }
  }	
}
//
void tst_clr_c_b()
{
 if ((close_active() == 1) | (open_active() == 1))
  {
   EE_read(EE_par_16);
    
   if ((ols == 1) | ((temp < 2) && (h_ols == 1)))
    {
     if (adap_door_run == 1)
      {
       adap_door_run = 0;
       run_timer = open_timer;
       data = run_timer; //
       adr = EE_run_timer;
 //      disable_interrupts(GLOBAL); // disable interupt mens der skrives til EEPROM - skulle ikke være nødvendig ifølge Anders
       write_int16_eeprom();
       EE_write(E_run,1);
 //      enable_interrupts(GLOBAL); // - skulle ikke være nødvendig ifølge Anders
       para_test = 0;
       p_value = 0;
       new_EE = 0;
       par_val = 0;
       next_para = 0; // prevent selecting next parameter because open is pressed
       confirm_learn = 1; // confirm with 2 sec fixed run showing
      }
    }
  }
 else
  {
   adap_door_run = 0;
   open_timer = 0;
  }	
}
//

