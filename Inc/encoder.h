#include "stm32f0xx_hal.h"
extern uint8_t temp5;                               // debug formål	
extern uint16_t temp16;	                            // gen. purouse reg.
extern uint32_t temp32;                             // temp for areaxx calculation

extern uint8_t st_speed_learn_ok;            // singleturn speed learning OK - old s_byte2,2 
extern uint16_t enc_speed_o;                 // adaptive speed control check reg.
extern uint16_t enc_speed_c;                 // adaptive speed control check reg.
extern uint8_t enc_open_cnt;                 // open counter for adaptive speed adjustment 
extern uint8_t st_area_ok;                   // old sk2,0 - 0 by powerup
extern uint8_t st_values_learned;            // old sk2,3 - 0 by powerup
extern uint8_t st_init_learned;				       // old sk2,7 - 0 by powerup
extern uint8_t st_wear_observed;             // wear testbit cleared by 0 by powerup

extern uint32_t area12;                      // value for area change from area 1 to area 2
extern uint32_t area23;                      // value for area change from area 2 to area 3
extern uint32_t area34;                      // value for area change from area 3 to area 4
extern uint32_t area45;                      // value for area change from area 4 to area 5
extern uint32_t area56;                      // value for area change from area 5 to area 6
extern uint32_t area67;                      // value for area change from area 6 to area 7
extern uint32_t area78;                      // value for area change from area 7 to area 8
extern uint32_t area89;                      // value for area change from area 8 to area 9
extern uint32_t area910;                     // value for area change from area 9 to area 10

extern uint32_t enc_ols;                            // indlært åbne endestop grænse
extern uint32_t enc_cls;                            // indlært lukke endestop grænse

extern uint16_t st_ce1;                      // adaptive speed control (single turn)
extern uint16_t st_ce2;                      // adaptive speed control
extern uint16_t st_ce3;                      // adaptive speed control
extern uint16_t st_ce4;                      // adaptive speed control
extern uint16_t st_ce5;                      // adaptive speed control
extern uint16_t st_ce6;                      // adaptive speed control
extern uint16_t st_ce7;                      // adaptive speed control
extern uint16_t st_ce8;                      // adaptive speed control
extern uint16_t st_ce9;                      // adaptive speed control
extern uint16_t st_ce10;                     // adaptive speed control
//
extern uint16_t st_oe1;                      // adaptive speed control
extern uint16_t st_oe2;                      // adaptive speed control
extern uint16_t st_oe3;                      // adaptive speed control
extern uint16_t st_oe4;                      // adaptive speed control
extern uint16_t st_oe5;                      // adaptive speed control
extern uint16_t st_oe6;                      // adaptive speed control
extern uint16_t st_oe7;                      // adaptive speed control
extern uint16_t st_oe8;                      // adaptive speed control
extern uint16_t st_oe9;                      // adaptive speed control
extern uint16_t st_oe10;                     // adaptive speed control
//
extern uint16_t st_ice1;                     // adaptive speed control
extern uint16_t st_ice2;                     // adaptive speed control
extern uint16_t st_ice3;                     // adaptive speed control
extern uint16_t st_ice4;                     // adaptive speed control
extern uint16_t st_ice5;                     // adaptive speed control
extern uint16_t st_ice6;                     // adaptive speed control
extern uint16_t st_ice7;                     // adaptive speed control
extern uint16_t st_ice8;                     // adaptive speed control
extern uint16_t st_ice9;                     // adaptive speed control
extern uint16_t st_ice10;                    // adaptive speed control
//
extern uint16_t st_ioe1;                     // adaptive speed control
extern uint16_t st_ioe2;                     // adaptive speed control
extern uint16_t st_ioe3;                     // adaptive speed control
extern uint16_t st_ioe4;                     // adaptive speed control
extern uint16_t st_ioe5;                     // adaptive speed control
extern uint16_t st_ioe6;                     // adaptive speed control
extern uint16_t st_ioe7;                     // adaptive speed control
extern uint16_t st_ioe8;                     // adaptive speed control
extern uint16_t st_ioe9;                     // adaptive speed control
extern uint16_t st_ioe10;                    // adaptive speed control
extern uint16_t speed_unit;                  //
extern uint16_t sense;                       // 
extern uint16_t st_i_speedsetp;              //   
extern uint16_t st_speedsetp;                //
