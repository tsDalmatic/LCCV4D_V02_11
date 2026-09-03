////////////////////////////////////////////////////////////////////////
//                                                                    //
//  The eprom.c file contains functions for accessing external EPROM  //
//  data via the STM32 peripheral I2C hardware interface. As reading  //
//  a single byte from an EPROM via the I2C bus takes approximately   //
//  0.4 ms (at 100 kHz) all EPROM data is loaded and maintained in a  //
//  local data buffer in RAM ensuring immediate read access. However, //
//  the write function is delayed by the time an actual write cycle   //
//  takes (including the device free-time as specified in the EPROM   //
//  data-sheet).                                                      //
//  Be aware of the inconsistent and extremely unconventional way     //
//  of passing address and data parameters using global variables!    //
//                                                                    //
//  Implemented by Anders Petersen, Dalmatic A/S, Marts 2016          //
//                                                                    //
////////////////////////////////////////////////////////////////////////

#include "stm32f0xx_hal.h"
#include "eprom.h"
#include "modbus.h"
#include "Global_Var.h"
#include "Defines.h"

#define EPROM_SIZE 4096 // The EPROM size used for local buffering 29-05-2024 32kbit EEPROM
//#define EPROM_SIZE 1024 // The EPROM size used for local buffering 8k EEPROM
#define EPROM_ADDR  160 // The EPROM Control byte (from data-sheet)

#define EPROM_FREE   6 // The EPROM free-time between write-cycles (ms) Datasheet says max 5 mS
                       // 6 x 1.11mS = 6.66mS. W.C can be 5 if saving just before ticktimer change so 5 x 1.11mS = 5.55mS 11-04-2023


// VARIABLE DEFINITIONS


extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;

extern uint8_t temp;              // Data register for 8 bit access.
extern uint8_t temp3;             // Data register for 8 bit access.
extern uint16_t adr;              // EPROM address for 16 bit write!
extern uint32_t data;             // Data for 16 bit read and write!
//uint32_t test_tick;               // test of ticktimer debug
//uint32_t test_tick_old;           // test of ticktimer debug
//uint32_t test_epromtimer;		      // test of epromtimer

extern uint8_t e21_fail;          // EPROM access error flag.
extern uint8_t something_saved;   // Test for something saved in EEPROM 
extern uint8_t inverter_use;       // 1 = inverter use is active, else contactor use


static uint8_t eprom[EPROM_SIZE]; // Defining array 

static uint32_t epromTimer;


// FUNCTION DECLARATIONS

static uint8_t read_eeprom(uint16_t addr);
static  void  write_eeprom(uint16_t addr, uint8_t data);


// FUNCTION DEFINITIONS

// Initialization function to be called once upon power-on.
void init_eeprom(void) 
 {
	// HAL_I2C_Mem_Read parameters are:
  // 1: I2C_HandleTypeDef (external, created by CubeMX)
  // 2: DevAddress (EPROM Control code from data sheet)
  // 3: MemAddress (reading starting from address zero)
	// 4: MemAddressSize (device has 1 addressing byte) This bigger 32k EEPROM 24LC32 has 2 bytes adressing 29-05-2024 
  // 4: MemAddressSize (device has 1 addressing byte)
  // 5: Memory pointer to the eprom data buffer
  // 6: Number of bytes to read (size of data buffer)
  // 7: Timeout in millisec (about 50 ms for 512 bytes)
	// 
  //if (HAL_I2C_Mem_Read(&hi2c1, EPROM_ADDR, 0, 1, eprom, EPROM_SIZE, 400) != HAL_OK) // the hole 8k EEPROM is read into the eprom array
  if (HAL_I2C_Mem_Read(&hi2c1, EPROM_ADDR, 0, 2, eprom, EPROM_SIZE, 400) != HAL_OK) // the hole 32k EEPROM is read into the eprom array 29-05-2024
    e21_fail = 1;
  else
    e21_fail = 0;
  epromTimer = HAL_GetTick();
 }


void calc_EE_cksum() // Xor of all safety related EEPROM adresses
{
 temp3 = EE_read(EE_par_1);	
 temp3 ^= EE_read(EE_par_2);
 temp3 ^= EE_read(EE_par_24);
 temp3 ^= EE_read(EE_par_25);	
 temp3 ^= EE_read(EE_par_31);
 temp3 ^= EE_read(EE_par_32);      // 2 bytes
 temp3 ^= EE_read(0x0E);           // 
 temp3 ^= EE_read(EE_par_52);      // 
 temp3 ^= EE_read(EE_cls_min);     // 4 bytes
 temp3 ^= EE_read(0x101);          //
 temp3 ^= EE_read(0x102);          //	
 temp3 ^= EE_read(0x103);          //	
 temp3 ^= EE_read(EE_cls_max);     // 4 bytes	
 temp3 ^= EE_read(0x105);          //	
 temp3 ^= EE_read(0x106);          //	
 temp3 ^= EE_read(0x107);          //	
 temp3 ^= EE_read(EE_ols_min);     // 4 bytes		
 temp3 ^= EE_read(0x109);          //
 temp3 ^= EE_read(0x10A);          //	
 temp3 ^= EE_read(0x10B);          //
 temp3 ^= EE_read(EE_ols_max);     // 4 bytes	
 temp3 ^= EE_read(0x10D);          //
 temp3 ^= EE_read(0x10E);          //
 temp3 ^= EE_read(0x10F);          //
 temp3 ^= EE_read(EE_offset);      // 4 bytes
 temp3 ^= EE_read(0x111);          //
 temp3 ^= EE_read(0x112);          //
 temp3 ^= EE_read(0x113);          //
 temp3 ^= EE_read(EE_offset_sign); //
 temp3 ^= EE_read(EE_limit_ready); // DATA EEPROM 1 byte bruges selvom behov kun er 1 testbit, 0=klar
 temp3 ^= EE_read(EE_o_limit_ok);  // open limit indlært hvis register er 0
 temp3 ^= EE_read(EE_c_limit_ok);  // close limit indlært hvis register er 0
 temp3 ^= EE_read(EE_run_range);   // 4 bytes
 temp3 ^= EE_read(0x121);          //
 temp3 ^= EE_read(0x122);          //
 temp3 ^= EE_read(0x123);          //
 temp3 ^= EE_read(EE_limit_win);   // 4 bytes
 temp3 ^= EE_read(0x129);          //
 temp3 ^= EE_read(0x12A);          //
 temp3 ^= EE_read(0x12B);          // result saved in temp3
}

// EPROM read and write functions - 8 bit

// Note: The temp variable is in fact a global parameter!
uint8_t EE_read(int16_t ad)
{
  temp = read_eeprom(ad);
  return temp;
}
//
// Note: The temp1 variable is in fact a global parameter!
uint8_t EE_read2(int16_t ad) // not in use yet - future use maybe
{
  temp1 = read_eeprom(ad);
  return temp1;
}
//
void EE_write(uint16_t ad, uint8_t da)
{
  write_eeprom (ad, da);
  // Seems like a hack that perhaps should be implemented elsewhere?
  if ((ad == EE_par_1) && (da < 3))
  {
   data = 0;
   adr = EE_par_32;
   write_int16_eeprom();
  }
}


// EPROM read and write functions - 16 bit

// Note: The data variable is in fact a global parameter!
uint16_t read_int16_eeprom(uint16_t ad)
{
  for (int i = 0; i < 2; i++)
    *(((uint8_t*)&data) + i) = read_eeprom(i + ad);
  return(data);
}

// Note: Address and data is passed via global variables!
void write_int16_eeprom(void)
{
  for (int i = 0; i < 2; i++)
    write_eeprom(i + adr, *(((uint8_t *)&data) + i));
}


// EPROM read and write functions - 32 bit

// Note: The data variable is in fact a global parameter!
uint32_t read_int32_eeprom(uint16_t ad) 
{
  for (int i = 0; i < 4; i++)
    *(((uint8_t*)&data) + i) = read_eeprom(i + ad);
  return(data);
}

// Note: Address and data is passed via global variables!
void write_int32_eeprom(void)
{
  for (int i = 0; i < 4; i++)
	 {
		if (tst_watchdog ==0) watchdog_on; // processor running monitor output to prevent timeout 02-02-2017
		else watchdog_off; // processor running monitor output to prevent timeout  
		write_eeprom(i + adr, *(((uint8_t *)&data) + i));
	 }
	watchdog_off; // processor running monitor output 
}


// Low level EPROM read and write

static uint8_t read_eeprom(uint16_t ad)
{
  return eprom[ad];
}

static void write_eeprom(uint16_t ad, uint8_t da)
{
  eprom[ad] = da; // Update local buffer
	tim_out_cnt = 10; // to prevent time out when saving i EEPROM
	//test_epromtimer = 0;

  while (HAL_GetTick() - epromTimer < EPROM_FREE) 
		{
		 //test_tick = HAL_GetTick();
     //test_epromtimer = epromTimer;
		 //HAL_IWDG_Refresh(&hiwdg); // debug	
		 // Wait for the EPROM free-time to pass
		} 
  //uint8_t ctrl = EPROM_ADDR + (ad >> 7 & 0x06); // 8k EEPROM not in use with 32 kbit EEPROM 29-05-2024
  //if (HAL_I2C_Mem_Write(&hi2c1, ctrl, ad, 1, &da, 1, 100) != HAL_OK) // 8k EEPROM
	if (HAL_I2C_Mem_Write(&hi2c1, EPROM_ADDR, ad, 2, &da, 1, 100) != HAL_OK)	// 32k EEPROM 29-05-2024
    e21_fail = 1;
  else
    e21_fail = 0;
  epromTimer = HAL_GetTick();
	HAL_IWDG_Refresh(&hiwdg); // 28-11-2016 refresh to prevent WDT timeout of 50mS this is needed especially when EEPROM is new
	// Long, multi-byte limit saves must not starve the inverter safety link.
	// Service only after the I2C write returns, so a stalled EEPROM transaction
	// still causes the external safety watchdog to time out.
	serviceModbusHeartbeat(inverter_use);
	something_saved = 1; // test bit for saving in EEPROM 02-02-2017
}
