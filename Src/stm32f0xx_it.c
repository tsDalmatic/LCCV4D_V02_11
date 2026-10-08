/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2022 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

/* USER CODE BEGIN 0 */
#include "stm32fxx_STLparam.h"
#include "stm32fxx_STLlib.h"
#include "Global_Var.h"
#include <Defines.h> // debug
uint16_t tmpCC1_last;	/* keep last TIM14/CCR1 captured value */
uint16_t chk_timer17;    // debug
extern uint8_t pwr_timer;
extern uint8_t uart2_buf[7];        // uart 2 buffer 18-11-2020
uint16_t tmpCC1_last;	/* keep last TIM14/CCR1 captured value */
extern uint8_t en_svar;             // control bit for whether encoder have answered
extern uint8_t rx_buffer[7];        // 7 bytes needed to copy uart2_buf
extern uint8_t pack_56[3];          // array for SCE encoder package with 56 header 14-01-2021
extern uint8_t pack_3a[7];          // array for SCE encoder package with 3a header 14-01-2021
extern uint8_t pack_3b[6];          // array for SCE encoder package with 3b header 14-01-2021
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  FailSafePOR();
  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
		 
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  FailSafePOR();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */
  FailSafePOR();
  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
  FailSafePOR();
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
   /* Verify TickCounter integrity */
  if ((TickCounter ^ TickCounterInv) == 0xFFFFFFFFuL)
  {
    TickCounter++;
    TickCounterInv = ~TickCounter;

    if (TickCounter >= SYSTICK_10ms_TB)
    {
      uint32_t RamTestResult;
			#if defined STL_EVAL_MODE
        /* Toggle LED_VLM for debug purposes */
       BSP_LED_Toggle(LED_VLM);
      #endif  /* STL_EVAL_MODE */

      /* Reset timebase counter */
      TickCounter = 0u;
      TickCounterInv = 0xFFFFFFFF;

      /* Set Flag read in main loop */
      TimeBaseFlag = 0xAAAAAAAAuL;
      TimeBaseFlagInv = 0x55555555uL;

        ISRCtrlFlowCnt += RAM_MARCHC_ISR_CALLER;
			  __disable_irq();
        RamTestResult = STL_TranspMarch();
			  __enable_irq();
        ISRCtrlFlowCntInv -= RAM_MARCHC_ISR_CALLER;
      switch ( RamTestResult )
      {
        case TEST_RUNNING:
          break;
        case TEST_OK:
          #ifdef STL_VERBOSE
          /* avoid any long string output here in the interrupt, '#' marks ram test completed ok */
            putchar((int)'#');
          #endif  /* STL_VERBOSE */
          #ifdef STL_EVAL_MODE
            /* Toggle LED_VLM for debug purposes */
            BSP_LED_Toggle(LED_VLM);
          #endif /* STL_EVAL_MODE */
          #if defined(STL_EVAL_LCD)
            ++MyRAMCounter;
          #endif /* STL_EVAL_LCD */
          break;
        case TEST_FAILURE:
        case CLASS_B_DATA_FAIL:
        default:
          #ifdef STL_VERBOSE
            printf("\n\r >>>>>>>>>>>>>>>>>>>  RAM Error (March C- Run-time check)\n\r");
          #endif  /* STL_VERBOSE */
          FailSafePOR();
          break;
      } /* End of the switch */

      /* Do we reached the end of RAM test? */
      /* Verify 1st ISRCtrlFlowCnt integrity */
      if ((ISRCtrlFlowCnt ^ ISRCtrlFlowCntInv) == 0xFFFFFFFFuL)
      {
        if (RamTestResult == TEST_OK)
        {
  /* ==============================================================================*/
  /* MISRA violation of rule 17.4 - pointer arithmetic is used to check RAM test control flow */
	#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
		#pragma diag_suppress=Pm088
	#endif /* __IAR_SYSTEMS_ICC__ */
          if (ISRCtrlFlowCnt != RAM_TEST_COMPLETED)
	#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
		#pragma diag_default=Pm088
	#endif /* __IAR_SYSTEMS_ICC__ */
  /* ==============================================================================*/
          {
          #ifdef STL_VERBOSE
            printf("\n\r Control Flow error (RAM test) \n\r");
          #endif  /* STL_VERBOSE */
          FailSafePOR();
          }
          else  /* Full RAM was scanned */
          {
            ISRCtrlFlowCnt = 0u;
            ISRCtrlFlowCntInv = 0xFFFFFFFFuL;
          }
        } /* End of RAM completed if*/
      } /* End of control flow monitoring */
      else
      {
      #ifdef STL_VERBOSE
        printf("\n\r Control Flow error in ISR \n\r");
      #endif  /* STL_VERBOSE */
      FailSafePOR();
      }
      #if defined STL_EVAL_MODE
        /* Toggle LED_VLM for debug purposes */
        BSP_LED_Toggle(LED_VLM);
      #endif  /* STL_EVAL_MODE */
    } /* End of the 10 ms timebase interrupt */
  } 
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line 4 to 15 interrupts.
*/
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
   if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET) // 16-03-2022
	 {
	  fraba_pulse = 1;
	  fraba_timer = 2; // changed from 3 to 2 after new main cycle time 3.33mS
	 }
  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel 1 interrupt.
*/
void DMA1_Ch1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Ch1_IRQn 0 */

  /* USER CODE END DMA1_Ch1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc);
  /* USER CODE BEGIN DMA1_Ch1_IRQn 1 */

  /* USER CODE END DMA1_Ch1_IRQn 1 */
}

/**
* @brief This function handles TIM14 global interrupt.
*/
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */
  uint16_t tmpCC1_last_cpy;
   
  if ((TIM14->SR & TIM_SR_CC1IF) != 0u )
  {
    /* store previous captured value */
    tmpCC1_last_cpy = tmpCC1_last; 
    /* get currently captured value */
    tmpCC1_last = TIM14->CCR1;
    /* The CC1IF flag is already cleared here be reading CCR1 register */

    /* overight results only in case the data is required */
    if (LSIPeriodFlag == 0u)
    {
      /* take correct measurement only */
      if ((TIM14->SR & TIM_SR_CC1OF) == 0u  &&  tmpCC1_last >  tmpCC1_last_cpy)
      {
        /* Compute period length */
        PeriodValue = tmpCC1_last - tmpCC1_last_cpy;
        PeriodValueInv = ~PeriodValue;
      
        /* Set Flag tested at main loop */
        LSIPeriodFlag = 0xAAAAAAAAu;
        LSIPeriodFlagInv = 0x55555555u;
      }
      else
      {
        /* ignore computation in case of IC overflow */
        TIM14->SR &= (uint16_t)(~TIM_SR_CC1OF);
      }
    }
    /* ignore computation in case data is not required */
  }
  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/**
* @brief This function handles TIM17 global interrupt.
*/
void TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM17_IRQn 0 */
  a_speed = TIM17->CCR1; // get value
	if (motor_rpm_1500 == 0) a_speed = a_speed/2;       // when motor 3000 is active, see main 31-01-2017
	__HAL_TIM_SET_COUNTER(&htim17, 0);	//reset counter
	//__HAL_TIM_SetCounter(&htim17, 0);	//reset counter from LCCV3D but not accepted
	chk_timer17 = a_speed; // debug test
  bit_set(sk1,1);
  bit_set(sk2,1); // for position counter
  pulse_chk = 1; // welding check
	
	
  /* USER CODE END TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM17_IRQn 1 */

  /* USER CODE END TIM17_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

 
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles USART3 to USART8 global interrupts / USART3 wake-up interrupt through EXTI line 28.
*/
void USART3_8_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_8_IRQn 0 */

  /* USER CODE END USART3_8_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  HAL_UART_IRQHandler(&huart4);
  HAL_UART_IRQHandler(&huart5);
	HAL_UART_IRQHandler(&huart7);
  /* USER CODE BEGIN USART3_8_IRQn 1 */

  /* USER CODE END USART3_8_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/******************************************************************************/
/**
  * @brief Configure TIM14 to measure LSI period
  * @param  : None
  * @retval : ErrorStatus = (ERROR, SUCCESS)
  */
ErrorStatus STL_InitClock_Xcross_Measurement(void)
{
  ErrorStatus result = SUCCESS;
  TIM_HandleTypeDef  tim_capture_handle;
  TIM_IC_InitTypeDef tim_input_config;
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  __HAL_RCC_PWR_CLK_ENABLE(); // Corrected 16-03-2022 the macro used don't exist
  HAL_PWR_EnableBkUpAccess();

  /* Configue LSI as RTC clock soucre */
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  { 
    result = ERROR;
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    result = ERROR;
  }
  /* Enable RTC Clock */
  __HAL_RCC_RTC_ENABLE();
  
  /* TIM14 Peripheral clock enable */
	__HAL_RCC_TIM14_CLK_ENABLE(); // Corrected 16-03-2022 the macro used don't exist
    
  /* Configure the NVIC for TIM14 */
  HAL_NVIC_SetPriority(TIM14_IRQn, 4u, 0u);
  
  /* Enable the TIM14 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM14_IRQn);
  
  /* TIM14 configuration: Input Capture mode ---------------------
  The LSI oscillator is connected to TIM14 CH1 via RTC clock.
  The Rising edge is used as active edge, ICC input divided by 8
  The TIM14 CCR1 is used to compute the frequency value. 
  ------------------------------------------------------------ */
  tim_capture_handle.Instance = TIM14;
  tim_capture_handle.Init.Prescaler         = 0u; 
  tim_capture_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;  
  tim_capture_handle.Init.Period            = 0xFFFFFFFFul; 
  tim_capture_handle.Init.ClockDivision     = 0u;     
  tim_capture_handle.Init.RepetitionCounter = 0u; 
	tim_capture_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // added by Melko 23-03-2022
  /* define internal HAL driver status here as handle structure is defined locally */
  //__HAL_RESET_HANDLE_STATE(&tim_capture_handle);
	tim_capture_handle.State = HAL_TIM_STATE_RESET;
  if(HAL_TIM_IC_Init(&tim_capture_handle) != HAL_OK)
  {
    /* Initialization Error */
    result = ERROR;
  }
  /* Connect internally the TIM5_CH1 Input Capture to the LSI clock output */
  HAL_TIMEx_RemapConfig(&tim_capture_handle, TIM_TIM14_RTC);
  
  /* Configure the TIM14 Input Capture of channel 1 */
  tim_input_config.ICPolarity  = TIM_ICPOLARITY_RISING;
  tim_input_config.ICSelection = TIM_ICSELECTION_DIRECTTI;
  tim_input_config.ICPrescaler = TIM_ICPSC_DIV8;
  tim_input_config.ICFilter    = 0u;
  if(HAL_TIM_IC_ConfigChannel(&tim_capture_handle, &tim_input_config, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Initialization Error */
    result = ERROR;
  }
  
  /* Reset the flags */
  tim_capture_handle.Instance->SR = 0u;
  LSIPeriodFlag = 0u;
  
  /* Start the TIM Input Capture measurement in interrupt mode */
  if(HAL_TIM_IC_Start_IT(&tim_capture_handle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Initialization Error */
    result = ERROR;
  }
  return(result);
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
