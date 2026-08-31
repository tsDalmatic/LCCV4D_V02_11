
#include "stm32f0xx_hal.h"
#include "Global_Var.h"
#include <Defines.h>
#include <main.h>
#include <eprom.h>

extern ADC_HandleTypeDef hadc;
//
//extern DMA_HandleTypeDef hdma_adc1;
//extern DMA_HandleTypeDef hdma_adc2;
//
extern uint8_t open_active(void);           // special inverter functions
extern uint8_t close_active(void);          // special inverter functions
extern uint8_t edge_select_tim;             // 02-12-2021
extern uint8_t encoder_errors;              // summerized encoder error 08-06-2022
//
void error_edge_setup(void);
void ok_edge_setup(void);
//void check_5_12_24(void);
void WD_circuit(void);

uint8_t WD_activated;        // 1 = Wicket Door activated. 10-11-2021
uint8_t SC_activated;        // 1 = slack cable activated. 10-11-2021
uint8_t Safety_2_short;      // 1 = safety_2 terminals are short-circuit  10-11-2021 
uint8_t wic_slack_range = 0; // testbit for showing missing circuit in terminals 12-11-2021
uint8_t edge_setup = 0;      // 02-12-2021
uint8_t edgetype_learn;      // 02-12-2021 learning edgetype
//uint8_t fail_5v = 0;       // no fail by powerup
uint8_t fail_12v = 0;      // no fail by powerup
uint8_t fail_24v = 0;      // no fail by powerup
//
uint16_t ad_dwpne_1;                         // measured analog edge_value
uint16_t ad_wicket;                          // measured analog edge_value
uint16_t Temp_sensor;                        // debug for AD function
uint16_t Vref;                               // debug for AD function
uint16_t toggle_time;                        // toggle between 12V and 24V fail showing
extern volatile uint16_t AD_values [4];				               // AD array of 4 values 08-11-2021
//

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) // controlled by timer 3. New update every 5 mS approximately.
{
	ad_dwpne_1 = AD_values[0]; // measured analog edge_value 
	ad_wicket = AD_values[1];
	Temp_sensor = AD_values[2];
	Vref = AD_values[3];
}
//

// 
void read_ad_inputs()
{
//
//	HAL_ADC_Start(&hadc); // turn on ADC and start 1 convertion in rank
//	HAL_ADC_PollForConversion(&hadc, 1000); // wait for convertion completed 
//  ad_dwpne_1 = HAL_ADC_GetValue(&hadc);
//  HAL_ADC_Start(&hadc); // start next ADC input in rank forward 18-05-2020
//	HAL_ADC_PollForConversion(&hadc, 1000); // 18-05-2020
//	ad_wicket = HAL_ADC_GetValue(&hadc); // 18-05-2020
//	HAL_ADC_PollForConversion(&hadc, 1000); // 18-05-2020
//	Temp_sensor = HAL_ADC_GetValue(&hadc); // 18-05-2020
//	HAL_ADC_Stop(&hadc); // stop convertion and disable ADC 18-05-2020
//
// Check for low voltage due to failure in 24V or 12V supply
if (safety_test == 1)
 {
  // no check when safety test 
 }
else
 {	
	if (ad_dwpne_1 < 1489) 
	 {
    if (toggle_time != 0) toggle_time--;
    else toggle_time = 600;	// showing 2 x 1 sec.	 
	  if (toggle_time > 300)
		 {
			fail_24v = 1;
			fail_12v = 0; 
		 }
		else
		 {
			fail_24v = 0;
			fail_12v = 1; 
		 }	
   }
 }
//	
if (edgetype != t_fraba)
 {
 fraba = 0; // fraba not selected, keep this off for kip is working 15-01-2010
 if ((edgetype == t_dw) || (edgetype == t_lindab))     // is PNE/DW selected or Lindab. Lindab is now the same in new hardware 11-12-2014 
  {
	 if (safety_test == 1)
    {
     if (ad_dwpne_1 < 1241) edge_state = 1; // Analog input is tested working (<1.0V). We don´t simulate breaking DW because we 
                                           // don't want active component in series with the safety input.(reliability)
     else edge_state = 0; // analog input fungerer ikke 
		 //
    }
   else
    {
    if (ad_dwpne_1 < 3227) edge_state = 1; // < 2.6V - edge activated, switches between 3.3V and 1.55V 
    else edge_state = 0; //
		// ***** SPECIAL TEST ***************** 
    // Both edge are tested if ex edge are selected. Edge must always be same type
		if ((ad_dwpne_1 <= 3600) && (ad_dwpne_1 >= 2730)) // if edge value in the interval 2.2 to 2.9V 
     {
      if (spec_edge_tim == 0) error_edge_setup();
     }
    else ok_edge_setup();
    if (run_prog == 1) 
     {
      if (fraba_timer > 0) error_edge_setup(); // Optical edge is connected
      if (ad_dwpne_1 < 3227) error_edge_setup();  // edge not connected or activated < 2.6V
     }
    }
  }
 //************************************************************************************************************************
 else if (edgetype == t_8k2) // 8k2 selected
  {
	 if (safety_test == 1)
    {
     if (ad_dwpne_1 < 1241) edge_state = 1; // Analog input is tested working (<1.0V). We don´t simulate breaking DW because we 11-12-2014
                                          // don't want active component in series with the safety input.(reliability)
     else edge_state = 0; // analog input fungerer ikke
    }
   else
    {
     if (ad_dwpne_1 > 3662) edge_state = 1; // > 2.95V - edge activated, switches between 2.59V and 3.35  
     else edge_state = 0; // edge not activated
     // ***** SPECIAL TEST *****************
     // Both edge are tested if extra edge is selected. Edge must always be same type
		 if (ad_dwpne_1 < 2730) // < 2.2V edge 8k2 not connected or an activated DW is connected 
      {
       if (spec_edge_tim == 0) error_edge_setup();
      }
		 else ok_edge_setup();
     if (run_prog == 1) 
      {
       if ((ad_dwpne_1 > 3662) || (ad_dwpne_1 < 2730)) error_edge_setup(); // > 2.95V or < 2.2V - edge activated or 
              														 //DW connected by a fail or nothing connected 11-12-2014
       if (fraba_timer > 0) error_edge_setup(); // Optical edge is connected
      }
    }
  }
 //************************************************************************************************************************
 
 }
else // optical edge selected, but test for connected other edges to PNE/8k2 EDGE input by a fail
 {
	edge_state_2 = 0;	// not in use - parameter 23 is 0		 
  ex_dw8k2 = 0; // set to off for kip working 15-01-2010
	if (ad_dwpne_1 < 2730) // dw8k2 test to be < 2.2V 
   {
    ok_edge_setup(); // < 2.2V
    dw8k2 = 0; // sættes til off for at kip fungerer 15-01-2010
   }
  else error_edge_setup(); 
 }
//***************************************************************************************************************************
WD_circuit();
// 
}
//
void error_edge_setup()
{
 err_edge = 1; // <= 2.85V and > 2.4V  or > 4.2V  then there is parameter failure setup
 edge_state = 1; // activate edge in case of failure in setup	
}
//
void ok_edge_setup()
{
 err_edge = 0;
 spec_edge_tim = 60; // 200 mS filter	
}
//

//************************************************************************************************************
//*****************************************************************************
void WD_circuit_ok()
{
 WD_activated = 0;
 SC_activated = 0;
 Safety_2_short = 0;
}
//*****************************************************************************
void WD_circuit_not_ok() // 20-08-2021
{
 WD_activated = 1;
 SC_activated = 1;
 Safety_2_short = 0;
}
//*****************************************************************************
void WD_circuit_short() // 27-08-2021
{
 WD_activated = 0;
 SC_activated = 0;
 Safety_2_short = 1;
}
//
//*****************************************************************************
void WD_circuit() // 12-11-2021
{
 // EE_read(EE_learn_wd_ready); not necesary here 30-08-2021
 //if (run_prog == 1)
 // {
 //  WD_circuit_ok();
 // } 
 if ((run_prog == 0) && (encoder_errors == 1) && (ad_wicket > 1135) && (E_limit == 1)) 
	 // Is there encoder errors wicket is not shown except circuit is > 15 KOhm 08-06-2022 
  {
   WD_circuit_ok(); // do not show door fitter a wicket fail because it will be correct when learned 
   wic_slack_range = 0;
  }	
 else if (safety_test == 1)
  {
   if (ad_wicket < 1055) WD_circuit_not_ok(); // Analog input is tested working (<0.85V). We don´t simulate breaking DW because we 11-12-2014
                                          // don't want active component in series with the safety input.(reliability)
   else WD_circuit_ok(); // analog input fungerer ikke 
  }	
 else  // is WD learned 30-08-2021
  {
   wic_slack_range = 0; // OK if not set by the following 30-08-2021 still used if mechannical limits is in use.
   EE_read(EE_par_24); // check again for easy program reading
   if (temp == 4) // 20-12-2018 16-06-2021
    {
     read_int16_eeprom(EE_WD_value);
     temp2 = data;
     if (ad_wicket < 1135) // value > 15 kOhm 20-08-2021
      {
       wic_slack_range = 1; // 30-08-2021
       WD_circuit_not_ok(); // 20-08-2021 no matter learned or not
      }
     else if ((temp2 > (1451 - 186)) && (temp2 < (1451 + 186))) // type F measured 12-11-2021 1,17V +/- 0,15V
      {
       if ((ad_wicket < 3500) && (ad_wicket > 1637)) // 3 kOhm shorted 12-11-2021
        {
         WD_circuit_short();
        }
       else if (ad_wicket > 4001) // check for total short of circuit 12-11-2021
        {
         WD_circuit_short();
        }
       else if ((ad_wicket > (temp2 - 186)) && (ad_wicket < (temp2 + 186))) // learned value +/- 186 = +/- 0,15 Volt = Circuit OK 12-11-2021
        {
         WD_circuit_ok();
        }
       else
        {
         WD_circuit_not_ok(); // 27-08-2021
        }
      }
     else if ((ad_wicket > (temp2 - 186)) && (ad_wicket < (temp2 + 186))) // learned value +/- 186 = +/- 0,15 Volt = Circuit OK
      {
       WD_circuit_ok();
      }
     else if (ad_wicket > (temp2 + 185)) // must the be terminals shorted
      {
       WD_circuit_short();
      }
     else // out of OK resistance range and not shorted
      {
       WD_circuit_not_ok(); // 20-08-2021
      }
    }
   else if (temp == 5) // 20-12-2018
    {
     if (ad_wicket > 3103) // > 2,5V  = NC contacts OK 12-11-2021 
      {
       WD_circuit_ok();
      }
     else
      {
       wic_slack_range = 1; // 
       WD_circuit_not_ok(); // 
      }
    }
   else
    {
     // should not be possible
    }
  }
}
//*****************************************************************************
void edge_detect() // 18-06-2020
{
 if ((close_active() == 0) & (open_active() == 0)) // is door stopped
  {
   edge_select_tim = 100;
   EE_read(EE_edge_setup);
   if ((temp == 0) && (edge_setup == 1))
    {
     EE_write(EE_par_21, edgetype_learn); 
     EE_write(EE_edge_setup, 1); // edge is deected and saved
    }
   else
    {
    }
  }
 else if ((open_active() == 0) && (edge_select_tim == 0 )) // is door running up and delay timer ended
  {
   EE_read(EE_edge_setup);
   if ((temp == 1) || (edge_setup == 1))
    {
     // edgetype already detected
    }
   else
    {
     if ((ad_dwpne_1 < 3472) && (ad_dwpne_1 > 2957)) // nominal value is 2,59V by 8k2 = 3215 and +/- 8% chosen
      {
       edgetype_learn = t_8k2;
       edge_setup = 1;
      }
     else if (ad_dwpne_1 > 3471) 
      {
       edgetype_learn = t_dw;
       edge_setup = 1;
      }
     else
      {
       edgetype_learn = t_fraba;
       edge_setup = 1;
      }
    } 
  }
}
//**************************************************************************


