
#ifndef MODBUS_H
#define MODBUS_H

#include "stm32f0xx_hal.h"


enum direction {
  FORWARD = 0,
  REVERSE = 1
};

// Note that address shall always be one lower because of a fail in LSis inverter 21-09-2018


enum modbusParam {
	F1009_NOISE_REDUCT   = 0x1008,  // Special fix - write 1 to this adress to reduce noise
  F1311_DC_BREAK_FREQ  = 0x1310,  // DC Breaking frequency
  F1319_FREQ_LOW_LIMIT = 0x1318,  // Frequency lower limit value	
  F1232_STP_1_FREQ     = 0x1231,  // Multi step speed frequency 1
  F1233_STP_2_FREQ     = 0x1232,  // Multi step speed frequency 2
  F1234_STP_3_FREQ     = 0x1233,  // Multi step speed frequency 3
	F1247_STEP_1_DEC     = 0x1246,  // Multi-step deceleration time 1
  F1248_STEP_2_ACC     = 0x1247,  // Multi-step acceleration time 2
	F1249_STEP_2_DEC     = 0x1248,  // Multi-step deceleration time 2
	F124B_STEP_3_DEC     = 0x124A,  // Multi-step deceleration time 3	
  F132C_BRAKING_FREQ = 0x132B,  // Brake release forward frequency
  F132F_BR_STOP_EFF  = 0x132E,  // Brake engage frequency
  F132E_BR_STOP_TIME = 0x132D,  // Brake engage delay time	
	F1703_BAUD_RATE		 = 0x1702,  // Build-in communication speed (BAUD_RATE)	
	//
//F1114_FREQ_UP_LIMIT  = 0x1113,  // Maximum frequency - S100
	F1114_FREQ_UP_LIMIT  = 0x1F07,  // Maximum frequency - M100 1F08
//F03E2_FACTORY_RESET  = 0x03E1,  // Parameter initialization (reset) - M100?	
//F1F0D_MODE_OF_DIR    = 0x1F0C,  // Select rotation direction - S100
	F1F0D_MODE_OF_DIR    = 0x1113,  // Select rotation direction - M100 1114
  F161F_RELAY_OUTPUT   = 0x161E,  // Multifunction relay 1 item (relay 2)
	F1103_ACC_TIME       = 0x1F00,  // Acceleration time (step 0) - M100 1F01 28-11-2018
  F1104_DEC_TIME       = 0x1F01,  // Deceleration time (step 0) - M100 1F02 28-11-2018
//F1B32_STALL_ADJUST = 0x1B31,  // Stall prevention motion and flux braking - S100
	F1B32_STALL_ADJUST = 0x1931,  // Stall prevention motion and flux braking - M100 1932
//F1101_TARGET_FREQ  = 0x1100,  // Target frequency - LSis Frequency high OPEN told by Mark Wester - S100
	F1101_TARGET_FREQ  = 0x1EFF,  // Target frequency - LSis Frequency high OPEN told by Mark Wester - M100 1F00
//F110E_RATED_POWER  = 0x110D,  // Motor capacity - S100
	F110E_RATED_POWER  = 0x1F04,  // Motor capacity - M100 1F05
//F120F_RATED_VOLT   = 0x120E,  // Motor rated voltage - S100
  F120F_RATED_VOLT   = 0x1F08,  // Motor rated voltage - M100 1F08   
//F120D_RATED_CURR   = 0x120C,  // Motor rated current - S100
	F120D_RATED_CURR   = 0x1F05,  // Motor rated current - M100 1F06
//F120C_RATED_SPEED  = 0x120B,  // Rated slip speed
	F120C_RATED_SPEED  = 0x120B,  // Rated slip speed - M100 1F07 - Mark says 120C
//F1112_RATED_FREQ   = 0x1111,  // Base frequency - S100
	F1112_RATED_FREQ   = 0x1F06,  // Base frequency - M100 1F07
//F0330_DRIVE_STATUS = 0x032F,  // Latch type trip infomation - DRIVE_STATUS - S100
	F0330_DRIVE_STATUS = 0x000E,  // Latch type trip infomation - DRIVE_STATUS - M100 000F 
//F120B_MOTOR_POLES  = 0x120A   // Motor pole counts S100 
  F120B_MOTOR_POLES  = 0x120A   // Motor pole counts M100 26-09-2018	
  
};


HAL_StatusTypeDef initModbusParam(void);

HAL_StatusTypeDef getModbusState(void);

HAL_StatusTypeDef setModbusParam(enum modbusParam, uint16_t);
HAL_StatusTypeDef getModbusParam(enum modbusParam, uint16_t *);

HAL_StatusTypeDef setStagepar(int, int, uint32_t, uint32_t, uint32_t);

#endif
