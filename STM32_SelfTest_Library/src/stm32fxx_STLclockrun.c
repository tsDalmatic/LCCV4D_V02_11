/**
  ******************************************************************************
  * @file    stm32fxx_STLclockrun.c 
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    19-Jun-2017
  * @brief   Contains routines required to monitor CPU frequency during run-time.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32fxx_STLparam.h"
#include "stm32fxx_STLlib.h"

/** @addtogroup STM32FxxSelfTestLib_src
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function verifies the frequency from the last clock
  *   period measurement
  * @param  : None
  * @retval : ClockStatus = (LSI_START_FAIL, HSE_START_FAIL,
  *   HSI_HSE_SWITCH_FAIL, TEST_ONGOING, EXT_SOURCE_FAIL,
  *   CLASS_B_VAR_FAIL, CTRL_FLOW_ERROR, FREQ_OK)
  */
ClockStatus STL_MainClockTest(void)
{
  ClockStatus result = TEST_ONGOING; /* In case of unexpected exit */

  CtrlFlowCnt += CLOCKPERIOD_TEST_CALLEE;
  /* ==============================================================================*/
  /* MISRA violation of rule 12.4 - "&&" operand can't contain side effects in normal operation
   - all the following pairs of volatile variables are changed consistently at timer ISR only */
  #ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
    #pragma diag_suppress=Pm026              
  #endif /* __IAR_SYSTEMS_ICC__ */
  /* checking result of HSE measurement done at TIM5 interrupt */
  if (((PeriodValue ^ PeriodValueInv) == 0xFFFFFFFFuL)\
  &&  ((LSIPeriodFlag ^ LSIPeriodFlagInv) == 0xFFFFFFFFuL)\
  &&   (LSIPeriodFlag != 0u) )
  #ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
    #pragma diag_default=Pm026              
  #endif /* __IAR_SYSTEMS_ICC__ */
  /* ==============================================================================*/
   
  {
#ifdef HSE_CLOCK_APPLIED    
    if (PeriodValue < HSE_LimitLow(SYSTCLK_AT_RUN_HSE))
#else
    if (PeriodValue < HSI_LimitLow(SYSTCLK_AT_RUN_HSI))
#endif
    {
      /* Switch back to internal clock */
      RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
      HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
      result = EXT_SOURCE_FAIL;	/* Sub-harmonics: HSE -25% below expected */
    }
    else
    {
#ifdef HSE_CLOCK_APPLIED    
      if (PeriodValue > HSE_LimitHigh(SYSTCLK_AT_RUN_HSE))
#else
      if (PeriodValue > HSI_LimitHigh(SYSTCLK_AT_RUN_HSI))
#endif
      {
        /* Switch back to internal clock */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
        HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
        result = EXT_SOURCE_FAIL;	/* Harmonics: HSE +25% above expected */
      }
      else
      {
        result = FREQ_OK;         /* Crystal or Resonator started correctly */        
        /* clear flag here to ensure refresh LSI measurement result will be taken at next check */
        LSIPeriodFlag = 0u;
      } /* No harmonics */
    } /* No sub-harmonics */
  } /* Control Flow error */
  else
  {
    result = CLASS_B_VAR_FAIL;
  }

  CtrlFlowCntInv -= CLOCKPERIOD_TEST_CALLEE;

  return (result);
}

/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
