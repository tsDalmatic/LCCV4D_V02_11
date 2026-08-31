
#include "stm32f0xx_hal.h"

// FUNCTION DECLARATIONS

void init_eeprom(void);
void calc_EE_cksum(void);

uint8_t EE_read(int16_t ad);
void EE_write(uint16_t ad, uint8_t da);

uint16_t read_int16_eeprom(uint16_t ad);
void write_int16_eeprom(void);

uint32_t read_int32_eeprom(uint16_t ad);
void write_int32_eeprom(void);


// DEFINING EPROM ADDRESSES (controler)

#define EE_par_1      0x01 // value of parameter 1 
#define EE_par_2      0x02 // value of parameter 2  21-12-2010 
//#define EE_par_5      0x03 // value of parameter 5  04-11-2021 09-06-2022
#define EE_par_11     0x04 // value of parameter 11
#define EE_par_13     0x05 // value of parameter 13
#define EE_par_15     0x06 // value of parameter 15
#define EE_par_16     0x07 // value of parameter 16
#define EE_par_17     0x08 // value of parameter 17
#define EE_par_21     0x09 // value of parameter 21
#define EE_par_22     0x0A // value of parameter 22
//not in use          0x0B // 
#define EE_par_24     0x0C // value of parameter 25 12-11-2021
#define EE_par_25     0x0D // value of parameter 25 04-11-2021
#define EE_par_26     0x0E // value of parameter 26 04-11-2021
#define EE_par_27     0x0F // value of parameter 27 04-11-2021
#define EE_par_29     0x10 // value of parameter 29
#define EE_par_31     0x11 // value of parameter 31
#define EE_par_32     0x12 // value of parameter 32
//                    0x13 is reserved for EE_par_32 msb
#define EE_par_33     0x14 // value of parameter 33 
#define EE_par_34     0x15 // value of parameter 34 23-08-2011
#define EE_par_35     0x16 // value of parameter 35 03-11-2010
#define EE_par_36     0x17 // value of parameter 35 16-05-2013
#define EE_par_41     0x18 // value of parameter 41 
#define EE_par_43     0x19 // value of parameter 43 manuel speed close setting
#define EE_par_42     0x1A // value of parameter 42 manuel speed open setting
#define EE_par_44     0x1B // value of parameter 44
#define EE_par_51     0x1C // value of parameter 51
#define EE_par_52     0x1D // value of parameter 52
#define EE_par_53     0x1E // value of parameter 53
#define EE_par_55     0x1F // value of parameter 53
#define EE_par_58     0x20 // value of parameter 58
#define EE_par_58_new 0x21 // old value parameter 58 used in service counter reset
#define EE_par_59     0x22 // value of parameter 59
//
#define EE_par_60     0x23 // value of parameter 60 26-11-2021
//
// Not in use         0x24 // 04-03-2026
#define EE_par_71     0x25 // value of parameter 71 26-11-2021
#define EE_par_72     0x26 // value of parameter 72 26-11-2021
// Not in use         0x27 // 04-03-2026
#define EE_par_74     0x28 // value of parameter 74 26-11-2021
#define EE_par_75     0x29 // value of parameter 75 26-11-2021
//
#define EE_par_77     0x2A // value of parameter 77 26-11-2021
#define EE_par_78     0x2B // value of parameter 78 26-11-2021
#define EE_par_79     0x2C // value of parameter 79 26-11-2021
//
#define EE_par_80     0x2D // value of parameter 80 01-07-2021
#define EE_par_81     0x2E // value of parameter 81 09-03-2010
#define EE_par_84     0x2F // value of parameter 81 06-11-2013
#define EE_par_85     0x30 // value of parameter 79 26-11-2021
#define EE_par_86     0x31 // value of parameter 79 26-11-2021
//
// EE_par_91 to EE_par_94 se below
//
#define EE_par_100    0x32 // value of parameter 100
//			EE_par_100    0x33 // -	-
#define EE_par_101    0x34 // value of parameter 101
//			EE_par_101    0x35 // -	-
#define EE_par_102    0x36 // value of parameter 102
//			EE_par_102    0x37 // -	-
#define EE_par_103    0x38 // value of parameter 103
//			EE_par_103    0x39 // -	-
#define EE_par_104    0x3A // value of parameter 104
//			EE_par_104    0x3B // -	-
#define EE_par_105    0x3C // value of parameter 105
//			EE_par_105    0x3D // -	-
#define EE_par_109    0x3E // value of parameter 109
//			EE_par_109    0x3F // -	-
#define EE_par_110    0x40 // value of parameter 110
//			EE_par_110    0x41 // -	-
#define EE_par_111    0x42 // value of parameter 111
//			EE_par_111    0x43 // -	-
#define EE_par_112    0x44 // value of parameter 112
//			EE_par_112    0x45 // -	-
#define EE_par_113    0x46 // value of parameter 113
//			EE_par_113    0x47 // -	-
#define EE_par_114    0x48 // value of parameter 114
//			EE_par_114    0x49 // -	-
//
#define EE_par_115    0x4A // value of parameter 115
//
#define EE_par_120    0x4B // value of parameter 120
//			EE_par_120    0x4C // -	-
#define EE_par_121    0x4D // value of parameter 121
//			EE_par_121    0x4E // -	-
#define EE_par_122    0x4F // value of parameter 122
//			EE_par_122    0x50 // -	-
#define EE_par_123    0x51 // value of parameter 123
//			EE_par_123    0x52 // -	-
#define EE_par_124    0x53 // value of parameter 124
//			EE_par_124    0x54 // -	-
//
#define EE_par_125    0x55 // value of parameter 125
// 0x56 - 0x57 is free
#define EE_CMD_freq   0x58 // value of command frequency 14-09-2018
//      EE_CMD_freq   0x59 // value of command frequency
//
#define EE_run_timer  0x5A // run timer 16 bit
//      EE_run_timer  0x5B 
#define EE_warning    0x5C //
//
#define EEce1         0x5D // min closing speed i area 1 16 bit
#define EEce2         0x5F // min closing speed i area 2 16 bit
#define EEce3         0x61 // min closing speed i area 3 16 bit
#define EEce4         0x63 // min closing speed i area 4 16 bit
//
#define EEoe1         0x65 // min opening speed i area 1 16 bit
#define EEoe2         0x67 // min opening speed i area 2 16 bit
#define EEoe3         0x69 // min opening speed i area 3 16 bit
#define EEoe4         0x6B // min opening speed i area 4 16 bit
//
#define EEice1        0x6D // initial min closing speed i area 1 16 bit
#define EEice2        0x6F // initial min closing speed i area 2 16 bit
#define EEice3        0x71 // initial min closing speed i area 3 16 bit
#define EEice4        0x73 // initial min closing speed i area 4 16 bit
//
#define EEioe1        0x75 // initial min opening speed i area 1 16 bit
#define EEioe2        0x77 // initial min opening speed i area 2 16 bit
#define EEioe3        0x79 // initial min opening speed i area 3 16 bit
#define EEioe4        0x7B // initial min opening speed i area 4 16 bit
//
#define EEomsk12      0x7D // value for area change from area 1 to area 2
#define EEomsk23      0x7E // value for area change from area 2 to area 3
#define EEomsk34      0x7F // value for area change from area 3 to area 4
//
#define E_run         0x80 // test register for adaptive runtimer learned or not
// 0x81 is free
#define EE_c_point_pos 0x82 // cell pointer for EEcntpos (20 cells)
#define EEcntpos      0x83 // 1. door position (cell pointer = 0) saved in EEPROM every time the door stops
//      EEcntpos      0x84 // reserved for door position for saving minumum 3 mill. times at 55 degrees Celsius (half of 25 degrees which is 1 mill minimum.
//      EEcntpos      0x85 // reserved for door position for saving minumum 3 mill. times
//      EEcntpos      0x86 // reserved for door position for saving minumum 3 mill. times
//      EEcntpos      0x87 // reserved for door position for saving minumum 3 mill. times
//      EEcntpos      0x88 // reserved for door position for saving minumum 3 mill. times
// 0x89 - 0x8C is free
#define EE_div_2      0x8D // counts of divisions save in EEPROM
#define EEpv05        0x8E // 0.5% pv05 value saved in EEPROM , 16 bit
//      EE_pv05       0x8F // 0.5% pv05 value saved in EEPROM , 16 bit
#define EE_sp_adj     0x90 // 0.25% speed adjust value for wear calc , 16 bit 21-12-2010
//      EE_sp_adj     0x91 // 0.25% speed adjust value for wear calc , 16 bit 21-12-2010
#define EE_ser_count  0x92 // service counter status saved in EEPROM 16 bit
//      EE_ser_count  0x93 // 
#define EE_interlock  0x94 // interlock is save for security if emergency stop is activated when power off
//
#define EE_s_speed_saved 0x95 // check adress for singleturn speed saved, value 10 = saved
//
#define EE_relearn_ph1 0x96 // testbit for photo1 in frame must be relearned 06-10-2010
#define EE_relearn_ph2 0x97 // testbit for photo2 in frame must be relearned 06-10-2010
#define EE_edge_setup  0x98 // 1 when edge is detected and saved. clared by factory clear only 02-12-2021
#define EE_right_turn  0x99 // if 1 - encoder right turning is learned 30-04-2019
#define EE_phase_inv   0x9A // phase inverting test bit saved i EEPROM 30-04-2019
#define EE_ecode4      0x9B // must be saved if power off before new learning 16-02-2017
#define EE_learning_ok 0x9C // set if learning is finished 30-04-2019
//
// EE 9D to 9E is not in use 31-03-2010 21-05-2017
#define EE_pu_check   0x9F // check adress for new EEprom - Cleared by factory clear 26-10-2011
//
#define EE_c_pointer  0xA0 // Electronic counter byte 1 pointer 31-03-2010
#define EE_count_b1   0xA1 // Electronic counter byte 1
//      EE_count_l    0xA2 // Electronic counter byte 1 endurance byte
//      EE_count_l    0xA3 // Electronic counter byte 1 endurance byte
//      EE_count_l    0xA4 // Electronic counter byte 1 endurance byte
//      EE_count_l    0xA5 // Electronic counter byte 1 endurance byte
//      EE_count_l    0xA6 // Electronic counter byte 1 endurance byte
//      EE_count_l    0xA7 // Electronic counter byte 1 endurance byte
//      EE_count_l    0xA8 // Electronic counter byte 1 endurance byte
//      EE_count_l    0xA9 // Electronic counter byte 1 endurance byte
//      EE_count_l    0xAA // Electronic counter byte 1 endurance byte
#define EE_count_b2   0xAB // Electronic counter byte 2
#define EE_count_b3   0xAC // Electronic counter byte 3
//
#define EE_error_1    0xB0 // last 10 errors save registers  moved 07-04-2010
#define EE_error_2    0xB1 // last 10 errors save registers
#define EE_error_3    0xB2 // last 10 errors save registers
#define EE_error_4    0xB3 // last 10 errors save registers
#define EE_error_5    0xB4 // last 10 errors save registers
#define EE_error_6    0xB5 // last 10 errors save registers
#define EE_error_7    0xB6 // last 10 errors save registers
#define EE_error_8    0xB7 // last 10 errors save registers
#define EE_error_9    0xB8 // last 10 errors save registers
#define EE_error_10   0xB9 // last 10 errors save registers
//
#define EE_learn_wd_ready 0xC0 // 11-11-2021
#define EE_WD_value 0xC1       // 11-11-2021
     // EE_WD_value 0xC2       // 16-06-2021 16 bits needed
//
#define EE_check      0xF0 // check adress for new EEprom
#define EE_new_chk_1  0xF1 // double check for new EEPROM
#define EE_new_chk_2  0xF2 // double check for new EEPROM
//
#define EE_par_5      0xF3 // value of parameter 5  09-06-2022 moved out of factory reset area 
//
#define EE_factory    0xFE // check register for reset to factory settings 
#define EE_overv      0xFF // check register for EE wear 


// DEFINING EPROM Addresses (encoder)

#define EE_cls_min       0x100 // DATA EEPROM 4 byte skal bruges
#define EE_cls_max       0x104 // DATA EEPROM 4 byte skal bruges
#define EE_ols_min       0x108 // DATA EEPROM 4 byte skal bruges
#define EE_ols_max       0x10C // DATA EEPROM 4 byte skal bruges
#define EE_offset        0x110 // DATA EEPROM 4 byte skal bruges
#define EE_open_low_sp   0x114 // Data EEPROM 4 byte skal bruges
#define EE_close_low_sp  0x118 // Data EEPROM 4 byte skal bruges
#define EE_offset_sign   0x11C // DATA EEPROM 1 byte skal bruges
#define EE_limit_ready   0x11D // DATA EEPROM 1 byte bruges selvom behov kun er 1 testbit, 0=klar
#define EE_o_limit_ok    0x11E // open limit indlært hvis register er 0
#define EE_c_limit_ok    0x11F // close limit indlært hvis register er 0
#define EE_run_range     0x120 // Run_range også gemt i EEPROM - 4 bytes skal bruges
#define EE_fine_adj_unit 0x124 // finjustering enhed gemmes
#define EE_limit_win     0x128 // beregnet limit_win gem i EE_prom da den anvendes til flere ting
#define EE_open_ls_setp  0x130 // for low speed brug
#define EE_close_ls_setp 0x131 // for low speed brug
#define EE_moving_fail   0x132 //
#define EE_ecode3        0x133 // ecode3 saved in EEPROM 26-09-2011
#define EE_enc_cls       0x134 // 27-11-09 4 byte shal be used
#define EE_enc_ols       0x138 // 27-11-09 4 byte shal be used
#define EE_cls           0x13C // 27-11-09 4 byte shal be used
#define EE_ols           0x140 // 27-11-09 4 byte shal be used

#define EE_p5_run_range  0x144 // 27-10-2011 5% of run_range for half open purpose, 4 byte shal be used
//
#define EE_h_open_pos    0x148    // value for photo frame position, 4 bytes shall be used 24-11-2021
#define EE_h_open_pos_saved 0x14C // 24-11-2021 1 byte
// 0x14D - 0x14F not in use
#define EE_ls_h_o_opening_setp 0x150 // low speed half open opening set point 4 byte shal be used 13-02-2026
#define EE_ls_h_o_closing_setp 0x154 // low speed half open closing set point 4 byte shal be used 13-02-2026
//
#define EE_ex_limit_win 0x158 // 13-02-2026 4 bytes
// 0x15C - 0x15F not in use
#define EEst_ce1     0x160 // min closing speed i area 1 16 bit
#define EEst_ce2     0x162 // min closing speed i area 2 16 bit
#define EEst_ce3     0x164 // min closing speed i area 3 16 bit
#define EEst_ce4     0x166 // min closing speed i area 3 16 bit
#define EEst_ce5     0x168 // min closing speed i area 3 16 bit
#define EEst_ce6     0x16A // min closing speed i area 3 16 bit
#define EEst_ce7     0x16C // min closing speed i area 3 16 bit
#define EEst_ce8     0x16E // min closing speed i area 3 16 bit
#define EEst_ce9     0x170 // min closing speed i area 3 16 bit
#define EEst_ce10    0x172 // min closing speed i area 3 16 bit
//
#define EEst_oe1     0x174 // min opening speed i area 1 16 bit
#define EEst_oe2     0x176 // min opening speed i area 2 16 bit
#define EEst_oe3     0x178 // min opening speed i area 3 16 bit
#define EEst_oe4     0x17A // min opening speed i area 3 16 bit
#define EEst_oe5     0x17C // min opening speed i area 3 16 bit
#define EEst_oe6     0x17E // min opening speed i area 3 16 bit
#define EEst_oe7     0x180 // min opening speed i area 3 16 bit
#define EEst_oe8     0x182 // min opening speed i area 3 16 bit
#define EEst_oe9     0x184 // min opening speed i area 3 16 bit
#define EEst_oe10    0x186 // min opening speed i area 3 16 bit
//
#define EEst_ice1    0x190 // initial min closing speed i area 1 16 bit
#define EEst_ice2    0x192 // initial min closing speed i area 2 16 bit
#define EEst_ice3    0x194 // initial min closing speed i area 3 16 bit
#define EEst_ice4    0x196 // initial min closing speed i area 3 16 bit
#define EEst_ice5    0x198 // initial min closing speed i area 3 16 bit
#define EEst_ice6    0x19A // initial min closing speed i area 3 16 bit
#define EEst_ice7    0x19C // initial min closing speed i area 3 16 bit
#define EEst_ice8    0x19E // initial min closing speed i area 3 16 bit
#define EEst_ice9    0x1A0 // initial min closing speed i area 3 16 bit
#define EEst_ice10   0x1A2 // initial min closing speed i area 3 16 bit
//
#define EEst_ioe1    0x1A4 // initial min opening speed i area 1 16 bit
#define EEst_ioe2    0x1A6 // initial min opening speed i area 2 16 bit
#define EEst_ioe3    0x1A8 // initial min opening speed i area 3 16 bit
#define EEst_ioe4    0x1AA // initial min opening speed i area 3 16 bit
#define EEst_ioe5    0x1AC // initial min opening speed i area 3 16 bit
#define EEst_ioe6    0x1AE // initial min opening speed i area 3 16 bit
#define EEst_ioe7    0x1B0 // initial min opening speed i area 3 16 bit
#define EEst_ioe8    0x1B2 // initial min opening speed i area 3 16 bit
#define EEst_ioe9    0x1B4 // initial min opening speed i area 3 16 bit
#define EEst_ioe10   0x1B6 // initial min opening speed i area 3 16 bit
//
#define EEarea12  0x1C0 // value for area change from area 1 to area 2, 4 bytes shal be used
#define EEarea23  0x1C4 // value for area change from area 2 to area 3, 4 bytes shal be used
#define EEarea34  0x1C8 // value for area change from area 3 to area 4, 4 bytes shal be used
#define EEarea45  0x1CC // value for area change from area 4 to area 5, 4 bytes shal be used
#define EEarea56  0x1D0 // value for area change from area 5 to area 6, 4 bytes shal be used
#define EEarea67  0x1D4 // value for area change from area 6 to area 7, 4 bytes shal be used
#define EEarea78  0x1D8 // value for area change from area 7 to area 8, 4 bytes shal be used
#define EEarea89  0x1Dc // value for area change from area 8 to area 9, 4 bytes shal be used
#define EEarea910 0x1E0 // value for area change from area 9 to area 10, 4 bytes shal be used
//
#define EEspeed_unit       0x1E4 // speed unit saved in EE for powerup use 2 bytes shall be used
//
#define EE_photo_pos       0x1E6 // value for photo frame position, 4 bytes shall be used
#define EE_photo_pos_saved 0x1EA // value for photo frame position, 4 bytes shall be used
#define EE_curtain_sw 0x1EE // Saved in EE_PROM in case of power off 01-04-2019 
#define EE_limits_check 0x1EF // check register for both limits have been reached after curtain sw activated 01-04-2019
// 0x200 - 0x26F not in use yet
#define EE_ols_curt_rep 0x270 // temporary OLS min for repair/placement of curtain, 4 bytes shal be used 01-04-2019 (right rotation)
#define EE_ols_min_curt_learn 0x274 // temporary OLS min for learning of curtain placement, 4 bytes shal be used 01-04-2019 (right rotation)
#define EE_ols_max_curt_learn 0x278 // temporary OLS min for learning of curtain placement, 4 bytes shal be used 01-04-2019 (left rotation)
#define EE_curtain_rep_saved 0x27C  // testbit for curtain repposition saved 12-02-2020
#define EE_encod_safe 0x27D         // test value for sce encoder power fail
// 0x27E to 0x2FF not in use yet
#define EE_bl_close_setp  0x300 // 15-06-2020 1 byte
#define EE_bl_open_setp   0x301 // 15-06-2020 1 byte
#define EE_bef_close_lim  0x302 // 15-06-2020 4 bytes
//                        0x303
//                        0x304
//                        0x305
#define EE_bef_open_lim   0x306 // 15-06-2020 4 bytes
//                        0x307
//                        0x308
//                        0x309
//
#define EE_par_91         0x330 // 01-02-2022
#define EE_par_92         0x331 // 01-02-2022
#define EE_par_93         0x332 // 01-02-2022
#define EE_par_94         0x333 // 01-02-2022


