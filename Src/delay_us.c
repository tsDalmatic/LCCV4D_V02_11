//Microsecond delay
#include "stm32f0xx_hal.h"

void delay_us(uint32_t delay_us) // 02-11-2021
{
  volatile unsigned int num;
  volatile unsigned int t;


  for (num = 0; num < delay_us; num++)
  {
    t = 20; // must be adjustet to Cortex clock time
    while (t != 0)
    {
      t--;
    }
  }
}
