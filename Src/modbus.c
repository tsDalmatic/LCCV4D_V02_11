
#include "stm32f0xx_hal.h"

#include <modbus.h>
#include "Defines.h"
#include "Global_Var.h"

#define MODBUS_TIMEOUT  200 // Modbus reply timeout value (in timerTicks) changed from 240 to 200 to LSis
#define MODBUS_DELAY      3 // Modbus delay between query (in timerTicks) oprindelig 3 13-09-2018
/* IC5 on the inverter is an STWD100xX with a 71 ms minimum timeout. */
#define MODBUS_HEARTBEAT_INTERVAL_MS   25U
#define MODBUS_HEARTBEAT_REGISTER     0x0004U
//*******************************************************
#define dal     0x00  // Dalmatic enkoder valgt
#define kostal  0x01  // Kostal enkoder valgt
#define feig    0x02  // Feig enkoder valgt
#define avago   0x03  // Avago enkoder valgt
#define sce     0x04  // SCE RS485 enkoder valgt
//*******************************************************

extern void delay_us(uint32_t delay_us); // 05-11-2021

extern IWDG_HandleTypeDef hiwdg;
extern UART_HandleTypeDef huart4; // 05-11-2021
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart2;

static uint8_t txFrame[8]; // Buffer for modbus transmit data frame.
static uint8_t rxFrame[8]; // Buffer for modbus receive data frame.

static uint16_t * dataPtr; // Pointer to destination for parameter read.
static uint32_t busyTimer; // Timer for measuring query round-trip time.

static HAL_StatusTypeDef uartState = HAL_OK; // Internal modbus state.
static uint32_t heartbeatTimer;
static uint16_t heartbeatValue;
extern uint8_t en_svar;             // control bit for whether encoder have answered
extern uint8_t e_type;              // 13-12-2021
//
extern uint8_t pwr_timer;
extern uint8_t uart2_buf[7];        // uart 2 buffer 18-11-2020
extern uint8_t rx_buffer[7];        // 7 bytes needed to copy uart2_buf
uint8_t uart3_buf[1];               // uart 3 buffer
#include <Defines.h> // debug
//
extern uint8_t pack_56[3];          // array for SCE encoder package with 56 header 14-01-2021
extern uint8_t pack_3a[7];          // array for SCE encoder package with 3a header 14-01-2021
extern uint8_t pack_3b[6];          // array for SCE encoder package with 3b header 14-01-2021
uint8_t byte_count_dis;             // byte count disable increment
extern uint8_t push_buttons;        //
extern uint8_t disp_svar;						// checkbit for display responds
// Public Function Definitions

// Function for obtaining present Modbus state - providing a
// timeout-check, dealing with the absence of a modbus reply.
HAL_StatusTypeDef getModbusState(void) {
	//HAL_IWDG_Refresh(&hiwdg); // must be refreshed because modbus init and settings take approx. 410mS + 50 mS
  if (uartState == HAL_BUSY && HAL_GetTick() - busyTimer > MODBUS_TIMEOUT)
    uartState = HAL_TIMEOUT;
  return uartState;
}


// Keep the inverter board's RS485 safety monitor alive outside inverter menus.
// The read is deliberately scheduled from the normal main loop: if the
// controller firmware stalls, requests stop and the inverter can time out.
void serviceModbusHeartbeat(uint8_t enabled) {

  uint32_t now = HAL_GetTick();

  if (enabled == 0U) {
    heartbeatTimer = now;
    return;
  }

  // Foreground parameter reads/writes always have priority. getModbusState()
  // also advances a missing reply from HAL_BUSY to HAL_TIMEOUT.
  if (getModbusState() == HAL_BUSY)
    return;

  if ((uint32_t)(now - heartbeatTimer) < MODBUS_HEARTBEAT_INTERVAL_MS)
    return;

  heartbeatTimer = now;

  // Register 4 is read-only from the controller's perspective. A valid frame
  // supplies RS485 activity without changing direction, speed or run commands.
  (void)getModbusParam((enum modbusParam)MODBUS_HEARTBEAT_REGISTER,
                       &heartbeatValue);
}


// Validate the new inverter board without writing legacy LSis parameters.
// This blocking check is used only during controller power-up and an explicit
// inverter restart. Normal main-loop communication remains non-blocking.
HAL_StatusTypeDef checkModbusSafetyLink(void) {

  HAL_StatusTypeDef status;

  do {
    status = getModbusState();
  } while (status == HAL_BUSY);

  HAL_Delay(MODBUS_DELAY);
  status = getModbusParam((enum modbusParam)MODBUS_HEARTBEAT_REGISTER,
                          &heartbeatValue);
  if (status != HAL_OK)
    return status;

  do {
    status = getModbusState();
  } while (status == HAL_BUSY);

  return status;
}


// Function for configuring inverter parameters. A list of valid parameter
// codes are present i the modbus.h file (and may be extended as required).
HAL_StatusTypeDef setModbusParam(enum modbusParam param, uint16_t value) {

  uint16_t crc = 0xFFFF; // Checksum calculation preload value
  //HAL_IWDG_Refresh(&hiwdg); // must be refreshed because modbus init and settings take approx. 410mS + 50 mS
  // Setup tx-frame for modbus RTU mode communication
  txFrame[0] = 0x01; // Inverter address, 1 per default
  txFrame[1] = 0x06; // Function = Preset single register
  txFrame[2] = param >> 8; // Register address, high byte
  txFrame[3] = param;      // Register address, low byte
  txFrame[4] = value >> 8; // Preset data, high order byte
  txFrame[5] = value;      // Preset data, low order byte

  // Calculating CRC checksum
  for (int i = 0; i<6; i++) {
    crc ^= txFrame[i];
    for (int s=0; s<8; s++)
      if (crc & 1)
        crc = crc >> 1 ^ 0xA001;
      else
        crc = crc >> 1;
  }
  txFrame[6] = crc;      // CRC, low byte
  txFrame[7] = crc >> 8; // CRC, high byte

  // If Modbus is already busy, preserve state and return HAL_BUSY.
  // Otherwise, transmit request and set state according to result.
  if (uartState == HAL_BUSY)
    return HAL_BUSY;
  else
	 {
		//usart6_de_man_on; // 05-11-2021 prepare for sending. 10-03-2022 Not in use when RS485 can be setup in CubeMx
    //delay_us(1);		  // wait for stabilize	
    __HAL_UART_CLEAR_FLAG(&huart4, UART_IT_TC);	// maybe not nedded	 
    switch (uartState = HAL_UART_Transmit_IT(&huart4, txFrame, 8)) // 05-11-2021
  	 {
      case HAL_OK:
        busyTimer = HAL_GetTick();
        uartState = HAL_BUSY;
        return HAL_OK;
      default:
        return uartState;
     }
	 }
}


// Function for obtaining inverter parameters. A list of valid parameter
// codes are present i the modbus.h file (and may be extended as required).
HAL_StatusTypeDef getModbusParam(enum modbusParam param, uint16_t* value) {

  uint16_t crc = 0xFFFF; // Checksum calculation preload value

  // Setup tx-frame for modbus RTU mode communication:
  txFrame[0] = 0x01; // Inverter address, 1 per default
  txFrame[1] = 0x03; // Function = Read holding register
  txFrame[2] = param >> 8; // Register address, high byte
  txFrame[3] = param;      // Register address, low byte
  txFrame[4] = 0x00;       // Register count, high byte
  txFrame[5] = 0x01;       // Register count, low byte

  // Calculating CRC checksum:
  for (int i = 0; i<6; i++) {
    crc ^= txFrame[i];
    for (int s=0; s<8; s++)
      if (crc & 1)
        crc = crc >> 1 ^ 0xA001;
      else
        crc = crc >> 1;
  }
  txFrame[6] = crc;      // CRC, low byte
  txFrame[7] = crc >> 8; // CRC, high byte

  // Storing pointer to destination variable (used in rx-callback).
  dataPtr = value;

  // If Modbus is already busy, preserve state and return HAL_BUSY.
  // Otherwise, transmit request and set state according to result.
  if (uartState == HAL_BUSY)
    return HAL_BUSY;
  else
	 {
		//usart6_de_man_on; // 05-11-2021 prepare for sending . 10-03-2022 Not in use when RS485 can be setup in CubeMx
    //delay_us(1); // wait for stabilize
    //__HAL_UART_CLEAR_FLAG(&huart6, UART_IT_TC); // maybe not needed
		switch (uartState = HAL_UART_Transmit_IT(&huart4, txFrame, 8)) // 05-11-2021
		 {
      case HAL_OK:
        busyTimer = HAL_GetTick();
        uartState = HAL_BUSY;
        return HAL_OK;
      default:
        return uartState;
     }
	 }
}


// Callback function called on end of modbus query.
void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart) 
 {
	if (huart->Instance == USART4) // was it uart 6 rx call back
 	 {
    // If changing baud-rate, also set that baud-rate
    // for controller in order to receive acknowledge.
    if (txFrame[2] == 0x17 && // Parameter F1702 is for
       txFrame[3] == 0x02) // setting the baud-rate.
	   {
      switch (txFrame[5]) 
			 {
        case 0:
         huart4.Init.BaudRate =  1200;
         break;
        case 1:
         huart4.Init.BaudRate =  2400;
         break;
        case 2:
         huart4.Init.BaudRate =  4800;
         break;
        case 3:
         huart4.Init.BaudRate =  9600;
         break;
        case 4:
         huart4.Init.BaudRate = 19200;
         break;
        case 5:
         huart4.Init.BaudRate = 38400;
         break;
        case 6:
         huart4.Init.BaudRate = 57600;
         break;
        default:
         huart4.Init.BaudRate =  9600;
       }
		  if (HAL_UART_Init(&huart4) != HAL_OK)
       {
       _Error_Handler(__FILE__, __LINE__);
       }
//      HAL_RS485Ex_Init(&huart4, UART_DE_POLARITY_HIGH, 0, 0); Not available in STM32F407 05-11-2021
			  HAL_Delay(2);
     }
    // First, receive 5 bytes (length of abnormal response)
	  __HAL_UART_CLEAR_FLAG(&huart4, UART_IT_TC); // ready for new TC
	  //usart6_de_man_off; // 05-11-2021 ready for receive 10-03-2022 Not in use when RS485 is availble in CubeMx
    //delay_us(1);		// wait for stabilize		
	  __HAL_UART_FLUSH_DRREGISTER(&huart4);	// 14-09-2017 05-11-2021
    HAL_UART_Receive_IT(&huart4, rxFrame, 5); // 05-11-2021
	}
 else
  {
   delay_us(1);		// for debug test only
	}	 
}


// Callback function called on end of modbus reply.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	if (huart->Instance == USART2) // was it uart 2 rx call back (encoder)
 	 {
		switch (e_type)
		 {
      case dal:
       rx_buffer[0] = uart2_buf[0]; // copy array
       rx_buffer[1] = uart2_buf[1];
       rx_buffer[2] = uart2_buf[2];
	     rx_buffer[3] = uart2_buf[3];
	     rx_buffer[4] = uart2_buf[4];
       en_svar = 1; // 5 byte are received
      break;
			//
			case kostal:
			 rx_buffer[0] = uart2_buf[0]; // copy array
       rx_buffer[1] = uart2_buf[1];
       rx_buffer[2] = uart2_buf[2];	
			 en_svar = 1; // 5 byte are received
			break;
			//
			case sce:
			 a = __HAL_TIM_GET_COUNTER(&htim7); // get timer which count in 10uS step
	     if (a > 2880) // test for pause since last interrupts >600 uS 
	      {
	       byte_count = 0;
	      }
	     //
       /* USER CODE END USART2_IRQn 0 */
       HAL_UART_IRQHandler(&huart2);
       /* USER CODE BEGIN USART2_IRQn 1 */
	     if (byte_count == 0) rx_buffer[0] = uart2_buf[0]; // 
       if (byte_count == 1) rx_buffer[1] = uart2_buf[0]; // 	 
	     if (byte_count == 2)
        {
		     rx_buffer[2] = uart2_buf[0]; //  
         if (rx_buffer[0] == 0x56) 
		      {
			     byte_count = 0; 
			     byte_count_dis = 1;
			     pack_56[0] = rx_buffer[0]; // 14-01-2021
			     pack_56[1] = rx_buffer[1]; 
			     pack_56[2] = rx_buffer[2]; 
		       HAL_UART_AbortReceive(&huart2);                          // clear receiver if of of sync cause of noise from motor	
					 __HAL_UART_FLUSH_DRREGISTER(&huart2);		
	         en_svar = 1; // 3 bytes are received 
	        }
	      }		
	     if (byte_count == 5) 
	      {
		     if (rx_buffer[0] == 0x3B) 
		      {
			     byte_count = 0;
			     byte_count_dis = 1; 
			     pack_3b[0] = rx_buffer[0]; // 14-01-2021
           pack_3b[1] = rx_buffer[1];
           pack_3b[2] = rx_buffer[2];
			     pack_3b[3] = rx_buffer[3];
           pack_3b[4] = rx_buffer[4];
           pack_3b[5] = uart2_buf[0];			 
		       HAL_UART_AbortReceive(&huart2);                          // clear receiver if of of sync cause of noise from motor	
					 __HAL_UART_FLUSH_DRREGISTER(&huart2);		
	         en_svar = 1; // 6 bytes are received
			    }
		     else
		      {
		       rx_buffer[5] = uart2_buf[0]; // it must be 3A header then
		      } 
	      }
       if (byte_count == 6) 
	      {
		     byte_count = 0;
		     byte_count_dis = 1; 
		     pack_3a[0] = rx_buffer[0]; // 14-01-2021
         pack_3a[1] = rx_buffer[1];
         pack_3a[2] = rx_buffer[2];
		     pack_3a[3] = rx_buffer[3];
         pack_3a[4] = rx_buffer[4];
         pack_3a[5] = rx_buffer[5]; 
		     pack_3a[6] = uart2_buf[0]; 
		     HAL_UART_AbortReceive(&huart2);                          // clear receiver if of of sync cause of noise from motor	
				 __HAL_UART_FLUSH_DRREGISTER(&huart2);	
	       en_svar = 1; // 7 bytes are received
	      }	
       if (byte_count == 3) rx_buffer[3] = uart2_buf[0]; // 
       if (byte_count == 4) rx_buffer[4] = uart2_buf[0]; // 
       //
       HAL_UART_Receive_IT(&huart2, uart2_buf, 1); // preparing receive of 1 bytes	at a time 	 
       if (byte_count_dis != 1) byte_count++; 
       byte_count_dis = 0;	 
       __HAL_TIM_SET_COUNTER(&htim7,0); 
			break;	
			 //	
    	 //emergency_out_off; // debug	
       //b = SysTick->VAL;
	     //u2time = a-b;
	     //if (u2time >247)
	     // {
	     //	u2time = u2time + 1; // debug
	     // }
       /* USER CODE END USART2_IRQn 1 */
		  default:
		  break;			
     }
			//
	 }
	else if (huart->Instance == USART3) // was it uart 2 rx call back  (display)
	 {
		push_buttons = uart3_buf[0]; 
		disp_svar = 1; // display har svaret 
	 }
	 
	else
	 {
  uint16_t crc = 0xFFFF;

  // Acting according to function code.
  switch (rxFrame[1]) {
    case 0x03:  // Parameter read, 1st part.
      rxFrame[1]++; // Prepare for 2nd part.
      HAL_UART_Receive_IT(&huart4, rxFrame+5, 2); // 05-11-2021
      return;
    case 0x04: // Parameter read, 2nd part.
      rxFrame[1]--;
      // Calculating CRC checksum:
      for (int i = 0; i<5; i++) {
        crc ^= rxFrame[i];
        for (int s=0; s<8; s++)
          if (crc & 1)
            crc = crc >> 1 ^ 0xA001;
          else
            crc = crc >> 1;
      }
      // Checking CRC checksum:
      if ((rxFrame[6] << 8 | rxFrame[5]) != crc) {
        uartState = HAL_ERROR;
        return;
      }
      // Transfer parameter value to destination:
      *dataPtr = rxFrame[3] << 8 | rxFrame[4];
      uartState = HAL_OK;
      return;

    case 0x06: // Parameter write, 1st part.
      rxFrame[1]++; // Prepare for 2nd part.
      HAL_UART_Receive_IT(&huart4, rxFrame+5, 3); // 05-11-2021
      return;
    case 0x07: // Parameter write, 2nd part.
      rxFrame[1]--;
		  // Check if responce match request:
      for (int i=0; i<8; i++)
        if (rxFrame[i] != txFrame[i]) {
          uartState = HAL_ERROR;
          return;
        }
      uartState = HAL_OK;
      return;

    case 0x86: // Abnormal responce.
    default:
      uartState = HAL_ERROR;
      return;
   }
  }
}


HAL_StatusTypeDef initModbusParam(void) {
  
  HAL_StatusTypeDef status = HAL_OK;

  HAL_Delay(MODBUS_DELAY);

  // Searching for correct baudrate
  // Standard baudrate is 38400 (5)
  // Note: Transmit function setup controllers
  // baud-rate in order to receive acknowledge.
	//
//  setModbusParam(F1703_BAUD_RATE, 0);
//  do
//    status = getModbusState();
//  while (status == HAL_BUSY);
//  if (status != HAL_OK) {
//    HAL_Delay(MODBUS_DELAY);
//    setModbusParam(F1703_BAUD_RATE, 1);
//    do
//      status = getModbusState();
//    while (status == HAL_BUSY);
//    if (status != HAL_OK) {
//      HAL_Delay(MODBUS_DELAY);
//      setModbusParam(F1703_BAUD_RATE, 2);
//      do
//        status = getModbusState();
//      while (status == HAL_BUSY);
//      if (status != HAL_OK) {
//        HAL_Delay(MODBUS_DELAY);
//        setModbusParam(F1703_BAUD_RATE, 3);
//        do
//          status = getModbusState();
//        while (status == HAL_BUSY);
//        if (status != HAL_OK) {
//          HAL_Delay(MODBUS_DELAY);
//          setModbusParam(F1703_BAUD_RATE, 4);
//          do
//            status = getModbusState();
//          while (status == HAL_BUSY);
//          if (status != HAL_OK) {
//            HAL_Delay(MODBUS_DELAY);
//            setModbusParam(F1703_BAUD_RATE, 5);
//            do
//              status = getModbusState();
//            while (status == HAL_BUSY);
//            if (status != HAL_OK) {
//              HAL_Delay(MODBUS_DELAY);
//              setModbusParam(F1703_BAUD_RATE, 6);
//              do
//                status = getModbusState();
//              while (status == HAL_BUSY);
//              if (status != HAL_OK)
//                return status;
//            }
//          }
//        }
//      }
//    }
//  }
//  HAL_Delay(MODBUS_DELAY);
  huart4.Init.BaudRate = 38400; // 38400 as start with LSis M100 which is the fastest in this. 05-11-2021
	if (HAL_UART_Init(&huart4) != HAL_OK)
   {
    _Error_Handler(__FILE__, __LINE__);
   }
	// HAL_RS485Ex_Init(&huart6, UART_DE_POLARITY_HIGH, 0, 0); // 05-11-2021 Not available in STM32F407 05-11-2021
	HAL_Delay(MODBUS_DELAY);
  status = setModbusParam(F1703_BAUD_RATE, 5); // 38400 er max for M100 fra LSis
	//status = setModbusParam(F1703_BAUD_RATE, 6); // 57600 is set by Cube MX setup - Fit for S100
  do
    status = getModbusState();
  while (status == HAL_BUSY);
  if (status != HAL_OK)
    return status;
  HAL_Delay(MODBUS_DELAY);

  // Starting Frequency (1/100 Hz)
  setModbusParam(F1311_DC_BREAK_FREQ, 100); // 50 er vist minimum 0.5 Hz 
  do
    status = getModbusState();
  while (status == HAL_BUSY);
  if (status != HAL_OK)
    return status;
  HAL_Delay(MODBUS_DELAY);

  // Minimum Frequency (1/100 Hz => 50 = 0.5 Hz)
  status = setModbusParam(F1319_FREQ_LOW_LIMIT, 50);
  do
    status = getModbusState();
  while (status == HAL_BUSY);
  if (status != HAL_OK)
    return status;
  HAL_Delay(MODBUS_DELAY);

  // Mode of Direction Setting (2 = terminal setting) 
  status = setModbusParam(F1F0D_MODE_OF_DIR, 0); // F = forward - 14-09-2018 forstås ikke af inverter 0 = Forward
  do
    status = getModbusState();
  while (status == HAL_BUSY);
  if (status != HAL_OK)
    return status;
  HAL_Delay(MODBUS_DELAY);

  // Main Frequency Source (0 = set by keypad 1 told by Mark Wester)
  //status = setModbusParam(F1F04_FREQ_SOURCE, 0); //- 14-09-2018 forstås ikke af inverter
  //do
  //  status = getModbusState();
  //while (status == HAL_BUSY);
  //if (status != HAL_OK)
  //  return status;
  //HAL_Delay(MODBUS_DELAY);
	
  // Relay Token Output (1 = invertaer fault protection)
  //                   (14 = Running state)
  //status = setModbusParam(F161F_RELAY_OUTPUT, 29); // S100 Set to trip. No invertaer fault protection is available in LSis
	status = setModbusParam(F161F_RELAY_OUTPUT, 17); // M100 Set to trip. No invertaer fault protection is available in LSis
  do
    status = getModbusState();
  while (status == HAL_BUSY);
  if (status != HAL_OK)
    return status;
  HAL_Delay(MODBUS_DELAY);
	//
	// Special fix - write 1 to this adress to reduce noise from inverter - Duer ikke på M100
  //status = setModbusParam(F1009_NOISE_REDUCT, 1); // 
  //do
  //  status = getModbusState();
  //while (status == HAL_BUSY);
  //if (status != HAL_OK)
  //  return status;
  //HAL_Delay(MODBUS_DELAY);
	
// P1 Terminal Function Setting (1 = Fx (=run)  *** INPUT er lavet med LSis DMP-file
//  status = setModbusParam(F1541_P1_TERM_FUN, 1);
//  do
//    status = getModbusState();
//  while (status == HAL_BUSY);
//  if (status != HAL_OK)
//    return status;
//  HAL_Delay(MODBUS_DELAY);

  // P2 Terminal Function Setting (2 = Rx (=direction)
//  status = setModbusParam(F1542_P2_TERM_FUN, 2);
//  do
//    status = getModbusState();
//  while (status == HAL_BUSY);
//  if (status != HAL_OK)
//    return status;
//  HAL_Delay(MODBUS_DELAY);

  // P3 Terminal Function Setting (5 = output block) Not in use yet
//  status = setModbusParam(F1543_P3_TERM_FUN, 5);
//  do
//    status = getModbusState();
//  while (status == HAL_BUSY);
//  if (status != HAL_OK)
//    return status;
//  HAL_Delay(MODBUS_DELAY);
	
	// P4 Terminal Function Setting (0 = No function) Not in use yet
//  status = setModbusParam(F1544_P4_TERM_FUN, 0);
//  do
//    status = getModbusState();
//  while (status == HAL_BUSY);
//  if (status != HAL_OK)
//    return status;
//  HAL_Delay(MODBUS_DELAY);

  // P5 Terminal Function Setting (7 = switch to low speed)
//  status = setModbusParam(F1545_P5_TERM_FUN, 7);
//  do
//    status = getModbusState();
//  while (status == HAL_BUSY);
//  if (status != HAL_OK)
//    return status;
//  HAL_Delay(MODBUS_DELAY);

  // DI-6 Terminal Function Setting (0 = no function)
//  status = setModbusParam(F1546_P6_TERM_FUN, 0);
//  do
//    status = getModbusState();
//  while (status == HAL_BUSY);
//  if (status != HAL_OK)
//    return status;
//  HAL_Delay(MODBUS_DELAY);

  // Terminal Filter Fimes
  //status = setModbusParam(F328_FILTER_TIMES, 5); Not available in LSis
  //do
  //  status = getModbusState();
  //while (status == HAL_BUSY);
  //if (status != HAL_OK)
  //  return status;
  //HAL_Delay(MODBUS_DELAY);

  // Stage Speed Mode Selection (1 = 0000 is stage 1) Not used in LSis
  //status = setModbusParam(F580_STG_SPD_MODE, 1);
  //do
  //  status = getModbusState();
  //while (status == HAL_BUSY);
  //if (status != HAL_OK)
  //  return status;
  //HAL_Delay(MODBUS_DELAY);

  // DC Braking Function Selection (2 = breaking during stopping)
  //status = setModbusParam(F600_BRAKING_FUNC, 0);  // DEBUG (disabled in Torbens inverter) Not in use
  //do
  //  status = getModbusState();
  //while (status == HAL_BUSY);
  //if (status != HAL_OK)
  //  return status;
  //HAL_Delay(MODBUS_DELAY);

  // Initial Frequency for DC Braking (1/100 Hz => 100 = 1 Hz)
  status = setModbusParam(F132C_BRAKING_FREQ, 100); // LSis - Brake release forward freq.
  do
    status = getModbusState();
  while (status == HAL_BUSY);
  if (status != HAL_OK)
    return status;
  HAL_Delay(MODBUS_DELAY);

  // Braking close freq during Stop (Hz)
  status = setModbusParam(F132F_BR_STOP_EFF, 10); // LSis - Brake engage frequency
  do
    status = getModbusState();
  while (status == HAL_BUSY);
  if (status != HAL_OK)
    return status;
  HAL_Delay(MODBUS_DELAY);

  // Braking close delay time During Stopping (1/100 sec => 50 = 0.5 sec)
  status = setModbusParam(F132E_BR_STOP_TIME, 50); // LSis - Brake engage delay time 
  do
    status = getModbusState();
  while (status == HAL_BUSY);
  if (status != HAL_OK)
    return status;
  HAL_Delay(MODBUS_DELAY);

  // Selection of Stalling Adjusting Function 
  status = setModbusParam(F1B32_STALL_ADJUST, 0); // LSis - Stall prevent
  do
    status = getModbusState();
  while (status == HAL_BUSY);
  if (status != HAL_OK)
    return status;
  HAL_Delay(MODBUS_DELAY);


  //
  return HAL_OK;
}


