#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "Global_Var.h"
#include "Defines.h"
#include "main.h"
#include "eprom.h"
//
extern void clr_val(void);
void direction_setup(void);
//
extern uint8_t open_saved;  // testbit for ikke at gemme igen i EEPROM
extern uint8_t close_saved; // testbit for ikke at gemme igen i EEPROM
extern uint8_t save_open;   // tryk for preset af OPEN LIMIT
extern uint8_t save_close;  // tryk for preset af CLOSE LIMIT	
extern uint8_t start_close;	// Variabel for start running down. Should be 0 by powerup 
extern uint8_t start_open;	// Variabel for start running up. Should be 0 by powerup 
extern uint32_t learn_pos;  //
extern uint16_t edge_value; // measured analog edge_value
extern uint8_t e_limit_open_learning_active; // 
extern uint8_t e_limit_close_learning_active; // 
extern uint8_t encod_safe;
//
uint8_t stop_tb;            // test bit for long push to programming select LCC V3D
uint16_t meas_dir_interval;  // 11-04-2023
uint8_t direction_setup_ok = 0;
uint8_t measure_num = 0;
uint32_t num_0_value = 0;
uint32_t num_1_value = 0;
uint32_t num_2_value = 0;
uint32_t num_3_value = 0;
uint32_t num_4_value = 0;

//

void learning()
{
 //	
 if (meas_dir_interval !=0) meas_dir_interval--; //  	
 //
 if ((run_prog == 1) && (parameter == 14) && (para_test == 1))
  {
	 imp_close = 0; // to be sure if set
   imp_open = 0;	// to be sure if set
	 if (E_limit == 1)
	  {			 
		 if (direction_setup_ok == 1)
			{
			}
		 else
		  {
			 if (((start_close == 1) || (start_open == 1)) && ((close_pb == 1)|| (close_disp_pb))) // is door running and close PB active 27-03-2019
			  {
				 if (meas_dir_interval == 0)
				  {
	  			 direction_setup();
					}
        }
       else
			  {
         meas_dir_interval = 600; // start delay of 200mS between position measurements. Changed to 2000 mS 11-04-2023  
				 measure_num = 0;
        }						 
			}
	  }
	}
 else
	{
	 // mech_limits
	}
}     

void direction_setup()
 {
	meas_dir_interval = 75; // delay of 200mS between position measurements. Changed to 250 mS 11-04-2023
  switch(measure_num)
	 {
    case 0:
		 num_0_value = learn_pos;
		 measure_num = 1;
		 break;
		case 1:
		 num_1_value = learn_pos;
		 measure_num = 2;
		 break;	
    case 2:
		 num_2_value = learn_pos;
		 measure_num = 3;
		 break;
		case 3:
		 num_3_value = learn_pos;
		 measure_num = 4;
		 break;	
		case 4:
		 num_4_value = learn_pos;
		 if ((num_2_value < num_1_value) && (num_1_value < num_0_value))
		  {
			 EE_write(EE_right_turn,1);
       direction_setup_ok = 1;
			 EE_write(EE_encod_safe,encod_safe);	
       break;				
		  }
		 else if ((num_4_value < num_3_value) && (num_3_value < num_2_value))
		  {
			 EE_write(EE_right_turn,1);
       direction_setup_ok = 1;
			 EE_write(EE_encod_safe,encod_safe);					
       break;				
		  }
		 else
		  {
			 EE_write(EE_right_turn,0);
			 //EE_write(EE_debug,1); // debug test only	
       direction_setup_ok = 1;
       break;	
		  }
		 default:
			// failcode if this happen shall maybe be made
		 break;
	  }		
 }
 



 
