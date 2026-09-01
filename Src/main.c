/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "Global_Var.h"
#include "Defines.h"
#include "eprom.h"
#include "modbus.h"
#include "stm32fxx_STLmain.h"
//static HAL_StatusTypeDef status = HAL_OK; // definering af status og start initialisering er HAL_OK - Er denne nødvendig. Spørg Anders
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//
extern uint8_t ft_dip_sw3; // long filter for this reset at power up
uint8_t req_encoder;       // for encoder request control
uint32_t timeTick1 = 0;    // For control loop timing
uint32_t timeTick2 = 0;    //   -   -
uint32_t timeTick3 = 0;    //   -   -
uint32_t timetick_calc;    //   -   -
uint32_t timetick_calc2;    //   -   -
uint8_t something_saved;   // Test for something saved in EEPROM
uint8_t timeout_modbus;    // timeout for init modbus by powerup 16-03-2017
volatile uint16_t AD_values [8];	 // AD array of 8 values 08-11-2021
uint8_t inverter_use; // 1 = inverter use is active - else contactor use 09-11-2021
uint16_t inv_timeout = 1501;   // inverter timeout for safety 5 Sec at power-up 28-03-2023 22-11-2024
//
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void); 
static void MX_TIM17_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_USART5_UART_Init(void);
static void MX_USART7_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_WWDG_Init(void); 
static void MX_TIM14_Init(void);
static void MX_TIM7_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
// *** first mentioned are external functions *********************************

extern void interpreter(void);
extern void encoder(void);
extern void start_ini(void);
extern void stop(void);
extern void mtimer(void);
extern void error_codes(void);
extern void read_inout(void);
extern void modbus(void);
extern void adap_run_time(void);
extern void read_ad_inputs(void);
extern void last_error_upd(void);
extern void display_keypad(void);
extern void push_code_req(void);
extern void req_photo_learn(void);
extern void req_status_wanted(void);
extern void encoder_TX(void);
extern void control_motor_relays(void); // 09-11-2021
extern void learning(void); // 30-04-2019
extern void inverter_safety(void); // 28-02-2023

//****************
extern void setModbusBaud(uint32_t baud);
void gen_ini(void);
//char mitArray[10] = {0,1,2,3,4,5,6,7,8,9}; // debug test af hard fault
//int minVariabel;                           // debug test af hard fault


void watchdog(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_IWDG_Init(); 
  MX_TIM17_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_USART5_UART_Init();
	MX_USART7_UART_Init();
  MX_CRC_Init();
  //MX_WWDG_Init();
  MX_TIM14_Init();
  MX_TIM7_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
		
  //**************** Program log **********************
  // Vision 2916 is first prototype program (version is setup in defines.h)
  //
	// HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/600); // debug test for Systick = 1.66mS
	//led_speed_on;	//debug
  //HAL_Delay(1000); // wait for inverter powerup	1.66 sec.
	//led_speed_off;	//debug
	//
	// 11-11-2016
	// DIL_4 ændret til DIL_3 in main.c fordi ny hardware kun har 3 DIL switches.
	// error_codes() og reload_led_per() er fjernet i encoder.c fordi disse LED codes er flyttet til display
	// Enkoder error koder er flyttet til display i display_keypad.c 
	// Enkoder error koder er kun vist i normal mode fordi der nok er forvirring hvis de vises under indlæring af endestop.
	// Error ecode7 registreres også i sidste 10 fejl register og som fejl nr. 25
	// dip_sw4 er fjernet i inout.c og nogle timere fra ft_dip_sw4 er flyttet til ft_dip_sw3.
	// Cubemx er ændret så PC3 er ændret fra input til output og til AUX3_RELAY.
  // Cubemx er ændret så PB12 er ændret fra output speed_ser LED til EX_AUTO_CLOSE.
  // Cubemx er ændret så PB13 er ændret fra fail_LED til EMERGENCY_OUT.
	// Cubemx er ændret så LED_STOP er ændret til LED_ERROR. 
  // Funktion of emergency-out og ex_auto_close er ikke lavet endnu.
  // Fail in electronic counter is corrected. 
	//
  // 14-11-2016 + 15-11-2016
  // Rettelser omkring safety test og safety_chain input som ikke var lavet endnu.
  // Safety chain ændret i display_keypad til at vises separat på display hvis denne er afbrudt	
	// I display_keypad er lavet så photo ikke vises når photo sidder i portramme og port er under indlært niveau.
	// 16-11-2016 + 18-11-2016
	// Safety test i interpreter er lavet på en lidt anden måde i henhold til tegning 4-0532 og 4-0533.
	// 21-11-2016
	// Parameter 87 og 89 indføres for aux_relay1 og AUX3_relay i display_keypad.
	//
	// 22-11-2016
	// ex_auto_close indført. Når switch er ON er autoclose frakoblet.
	// Det er konstateret at interlock aktiveres automatisk hvis der er power off når port er åben.
	// Der skyldes at der startes i stop situatiion og interlock_tim er 0. Om det altid også har været sådan ved V7E
	// vides ikke men det er i hvert fald det sikreste at port ikke begynder at køre ned ved power on efter endt autolukketid.
	// 23-11-2016
	// Emergency stop er tilføjet program, så optokobler emergency_out aktiveres ved tryk på emergency tryk.
	// 
	// 24-11-2016
	// X-tal drift og CSS aktiveret. e29_xtal indført. Clock skifter automatisk fra 48 MHz krystal til intern 8 MHz RC osc.
	// samtidig genereres et interrupt som vi bruger til at sætte fejlkode og til at geninitialisere display usart for at kunne
	// vise på display. Fejlkode aktiverer et normalt stop. Der er ændret i main.c - interpreter.c - display_keypad.c.
	// e29_xtal resettes kun ved reset. Hard_fault interrupt er også indført til stop ved denne fejl.
	// Hard fault standser programmet i en while 1 løkke. Det er watchdog der skal bringe program ud af denne løkke.
	// En evt. aktiv udgang vil derfor resettes fordi I/O bliver til indgang ved reset.
	// 28-11-2016
	// Watchdog ændres til 50mS da det er det sikkerhedssoftware skal køre med ifølge Anders.
	// I eprom.c er derfor indført refresh når der skal gemmes meget i EEPROM som tager måske 1 sek.
	// I cubeMX ændres watchdog opsætning derfor til prescaler 4 og 500 nedtælling 1/40k x 4 x 500 = 50mS
	// Der er afprøvet med 25 mS watchdog også som kontrol for at den ikke lige er på grænsen.
	// 29-11-2016
	// e28_wdt lavet for at vi kan se evt. watchdog timeout. Denne fejlkode aktiverer ikke stop fordi en reset
	// allerede deaktiverer evt. aktiv kørsel. e28 kan cleares ved åbne tryk eller power OFF.
	// 13-12-2016 Rettelse i last_error_upd.c omkring fejl som ikke skal registreres når det er under selve 
	// safety testen der foretages ved hvert stop fordi disse bliver sat og derefter resat når de er OK.
	// Dette er fordi safety testen er ændret lidt 18-11-2016.
	//
	// 14-12-2016
	// Start af watchdog flyttes fordi HAL_IWDG_Refresh(&hiwdg) ikke virker rigtigt i start_ini rutinen når hele
	// EEPROMen skal slettes ved ny processor. Den kan dog ikke sættes førend HAL_Init og da den kun er sat til 50mS
	// skal den også sættes efter pause for inverter powerup.
	// I modbus.c er indført 2 refresh også for at undgå wdt timeout.
	// Ved 50 mS WDT går det dog stadig galt ved init_eeprom i eprom.c fordi læsning af hele eprom ind i array tager ca 45mS
	// Vi skal lige have fundet ud af om der virkelig kræves at vi kun har 50mS i WDT ifølge krav fra sikkerhedssoftwaren.
	// indtil da stiller vi lige WDT til 1 sek. i cubemx så vi kan komme til at køre.
	//
	// 05-01-2017 program udbygges med AUX3 relæ styring i parameter 89.
	// Ved parameter 89 = 1 vil AUX3 aktiveres ved aktivering af: safety chain på enkoder - eller
  // ecode 1 til ecode9 på enkoder eller e9_pos_change.
	//
	// 26-01-2017 modbus.c ændres idet nogle linier udmaskes da de genindlæste nogle default motor parametre
	// 26-01-2017 out_out_range fail improvement is made in encoder.c
	// 26-01-2017 ecode7_timer ændres fra 40mS til 200mS og 660mS ved powerup som det var på V7E.
	// 26-01-2017 tim_out_cnt powerup værdi ændres da den skal være kortere end ecode7_timer
	//
	// 02-02-2017 Forbedringer i eprom.c omkring vedligeholdelse af ekstern watchdog fordi den løb ud når
	// mange værdier i forcecontrol gemmes. Skal nok også indføres i LCC V3 inverter.
	// 02-02-2017 Testvariabel indført til at sikre nok pause mellem positionsforespørgsler når der gemmes i EEPROM
	// Indført i main.c og eprom.c. Nærmere forklaring findes i konstruktionslog.
	// 16-02-2017 Rettelser i interpreter.c omkring safety_req hvis port står på CLS ved power up stoppede safety_req utilsigtet 
	// så der ikke blev testet færdigt med fejlkoder som resultat. Det er en rettelse taget fra LCCV3D.
	// 16-02-2017 Forbedring af moving_fail så der skal 3 målinger til idet man kunne være uheldig at ved reversering at
  // man netop ramte samme måling. Moving_fail er forbedret lidt ved kostal da den kunne svigte hvis position skifter lidt hele tiden selvom 
	// enkoder står stille. Det er rettelser taget fra LCCV3D.
	// 16-02-2017 ecode4 lavet for Kostal også basseret på at hvis man forsøger at indlære med arbejdsområde (run_range)
  // under 30 grader og over 310 vinkelgrader. Hvis man bruger ove 310 grader vil endestopsvinduer overlappe hinanden da
  // de er 1/8 af run_range. Det er rettelser taget fra LCCV3D.
  // 16-02-2017 ecode4 gemmes i EEPROM så den huskes ved power off da man ellers kan køre fejlagtigt. Rettelser er i ecoder.c og start_ini.c
	// et er rettelser taget fra LCCV3D.
	// ***********************************************************************************************************
	//************************************************************************************************************
	// 13-03-2017 Sikkerhedssoftware indføres.
	// ! OBS: Ved hver programændring skal der debugges og laves et breakpoint omkring linje 292 ved crc_result = CRC->DR;
	// i stm32fxx_STLstartup.c. her holdes musen over DR for at vise den beregnede checksum. Skriv denne værdi ned -
	// forlad debug og skriv værdi i startup_stm32f071xb.s i linje 278. compiler herefter som normalt.
	// Hvis værdi er forkert kkommer program slet ikke i gang og error LED og circuit error LED lyser.
	// ! OBS husk at ved evt. ny cubemx codegenerering skal MX_WWDG_Init(); udkommenteres igen hvis den ikke bruges.
	//*************************************
	// 13-03-2017 En del rettelser omkring enkoder drift udføres. Det er TO DO LISTEN af 10/3-2017 punkt 33, 34, 35, 36
	// og 38. Det er rettelser i encoder.c display_keypad.c og start_ini.c.
	// Displayvisning af inverterfejl indføres med en ny T005_DRIVE_STATUS modbus forespørgselsmulighed. punkt 37.
	// Der er også rettet i display program for at løse dette.
	// Der er lavet en del rettelser omkring factory reset i main.c og start_ini.c hvilket skulle løse punkt 39 også.
	// Startværdier og factory reset værdier indføres for 12 inverter parametre som ligger i inverterens EEPROM.
	// Watchdog skal refrshes under dette fordi en factory reset ikke ændrer på watchdog timeouttid der kun er 50mS
	// Parameter 89 ændres til 1 som factory reset værdi.
	// 16-03-2017 Ny parameter 109 indføres. Det er max frekvens for inverter. Fabriksindstilling 100 og indstillingsområde
  // 50-200. Det er punkt 40 på todo liste. Hvis parameter 110, 111, 120 eller 121 er højere end 109 justeres disse
  // automatisk ned til 109 værdi. Det gør Inverteren tilsyneladende selv.
	// Der indføres visning af modbus timeout hvis modbus ikke svarer ved powerup. 
	// 17-03-2017 opstart omkring modbus ændret da den ofte gik i modbus timeoutfejl. 1.6 sek. delay flyttet ud af
	// do while løkke. Der er valgt et opstartsdelay på 2.5 sek. inden modsbus initialisering da der er observeret
	// at inverter ikke er klar førend efter 1.9 sek. efter powerup. Der tillades herefter 2 cycles for modbus
	// initialisering førend det vises modbus timeout så der er 1 chance mere hvis den første skulle kikse.
	// 20-03-2017 Forbedring i encoder.c omkring moving_fail der styrer E9 no position change fejl. 
	// Problemet var at hvis man afmonterer 2 faser så der kun er 1 fase ud på motor så genererer inverter støj
	// som forstyrer enkoderens præcistion. der er set at den kan skifte 360 hvilket er 4 enheder på enkoderopløsningen.
	// Der er valgt at enkoder skal flytte sig mere end 450 = 5 enheder ligesom kostal.
	// Det er en gammel dall enkoderder er på den målte motor. Muligvis er MTM enkoder bedre på dette punkt.
  // LCC V3 INV-V01_01	
	// 21-03-2017 Ændringer omkring factory clear. Special reset af elektronisk tæller er indført til internt brug. 
	// Denne tæller resettes ved DIL SW3 ON + stop_disp_pb + 2 sec. open_pb eller open_disp_pb + close_pb på klemrække.
	// stm32fxx_STLmain.c ændres så reloadværdi er 16 for 100mS timeout. Det er sikkert nok til vores application
	// og det skyldes at init_eeprom() tager 50mS så ved factory reset er stor risiko for at watchdog resetter.
	// Inverterfejl stopper nu også controller så det ikke kun er inverter der stopper kørsel. Så skulle controller
	// ikke gå i E09 fejl. 
	// 22-03-2017 Rettelser omkring fejlkodeudlæsning. E09 fejl ændres så den vises nu i display og slettes først efter
	// 2 sek. når parameter 81 er 3. Prioitering for denne E09 fejlkode flyttes også så den kan vises på endestop.
	// Checksum 0x006B73FD
	//
	// LCC V3 INV-V01_02
	// 29-03-2017
	// Program udbygget så special push tryk kan dødmand lukke port 1 gang hvis foto eller kantliste er aktiveret.
	// Det er i interpreter.c
	// Fejl i push_kode tryk også rettet da der var en fejl i dette. Det er i push_code_req.
	// Ekstra check for ny EEPROM er indført da vi har set det almindelige check fejle. Lavet i start_ini.c
	// Fejl rettet omkring EEPROM skrivning da det nok egenligt har været årsagen til at skrivningen fejlede
	// ved ny processor. Det skyldes at i start_ini.c var ticktimer 1 mS hvorimod den er 1.66mS senere i den
	// normale while(1) løkke. Skrivning i af flere værdier i start_ini foregik da med 4 mS interval og der skal
  // være minimum 5mS. STL_InitRunTimeChecks() flyttes derfor om på den anden side af start_ini og ticktimer
  // justeres i stm32fxx_STLmain.c til af være de 1.66 mS. Vi skal dog dog bibeholde stilling af ticktimer tilbage
  // til 0 da STL_InitRunTimeChecks() sætter den til 192 og det kan give problemer med programafviklingen.
  // Mærkeligt at de ikke har styr på dette. Der bibholdes alligevel dobbelt check funktionen for check af ny
	// EEPROM da det anbefales på nettet at man ikke kan være 100% sikker på at EEPROM er FF som ny.
	// Rettelser omkring enkoder test funktion i parameter 82 da den ikke virkede. Rettelser er i encoder.c
	// og i display_keypad samt i selve display programmet.
	// 03-04-2017
	// Program udbygget så inverter parameter F118 automatisk følger inverter parameter F810 som er vores parameter 104.
	//
	// 02-05-2017 LCC V3 INV-V01_03
	// Rettelser i sikkerhedssoftware da den ikke startede op rigtigt. Der er indført en linie i stm32f0xx_it.c 
	// omkring linie 408. Fejlen er ikke observeret i dette program men i LCC V3D så det burde være samme så det indføres
	// for en sikkerheds skyld.
	// Rettelser omkring softwarefilter for OPEN_PB + KIP_PB ved powerup. Problem er set ved kostal enkoder hvis åbnetryk
	// er aktiveret ved power-up. Her kommer kortvarigt en åbne komando til inverteren selvom port er åben.
	// Det skyldes at SW-filter for åbnetryk var sat til 66 mS, men TX til enkoder starter først efter pwr_timer er
	// udløbet som er 100 mS. Envidere går der ca. 60mS mere inden Kostal enkoder skifter om fra 9600 til 19200 
	// baud-rate. Der går derfor 160mS inden der er en position vi kan regne med. Indtil de 160mS er gået skiftede 
	// position lidt tilfældigt i norm_mode i encoder.c hvilket giver ustabile OLS og cls værdier i den periode.
	// softwarefiltre for OPEN_PB + KIP_PB ændres derfor til 200mS ved power-up i Global_Var.c så der ikke køres op
	// førend ny position er læst.
	// Der rettes i encoder.c også så norm_mode ikke køres førend position er modtaget mindst 2 gange. Dette
	// for at programmet ikke forstyres med flere forskellige ols og cls værdier førend position er læst. Det er en ny 
	// variabel med navnet position_ready. OLS og CLS herefter altid være 0 ved power-up indtil position er læst.
	// rettelser omkring sw-filter og position_ready skal også laves i LCC V3D.
	// 
	// 09-05-2017 Specialprogram for UPS styringer hvor parameter 84 og 89 er fast = 1.
	// Derudover er ændret lidt på inverterens fabriks-frekvensindstillinger.
	//
	// 21-05-2017 LCC V3 INV-V50_01-21052017
	// Inverter indstillingsparametre for motor og drift gemmes i LCC V3 Inverterens EEPROM også da de tilsynedeladende periodisk glemmes efter power-off.
	// EURA drives Inverter 2000 er set glemme de indstillede værdier og tilbagestiller disse til EURA drives fabriksindstillinger.
	// Det er ikke i orden for driften at disse glemmes. Årsagen til at disse glemmes er ukendt og det sker også sjældent.
	// Vi gemmer derfor disse også i EEPROM om kontrollerer disse ved power-up og genindstiller disse om nødvendigt.
	// Der er set problemer med parameter 111, 113, 120, 122, 124 men vi må hellere kontrollere alle dem vi kan indstille på.
	// Det vil sige 100, 101, 102, 103, 104, 105, 109, 110, 111, 112, 113, 114, 120, 121, 122, 123, 124.
	// 115 og 125 er allerede i LCC V3 Inverter EEPROM'en.
	// Disse 17 parametre som er 16 bit optager således 32 bytes i EEPROM. Der er derfor ændret i EEPROM adresser.
  // !!! OBS Hvis tidligere programmerede enheder benyttes bør der efter programmering foretages en fabriksreset af LCC V3 Inverter.
  // Retttelse er i eprom.h display_keypad.c og start_ini.c 	
	// Parameter 100, 101, 102, 103, 104 og 105 tilføjes også som fabriksreset. 
	// 13-06-2017 "do" added in main.c
	//
	// 29-08-2017 LCC V3 INV-V50_02-29082017
	// Rettelse omkring opstart af program i tilfælde af at modbus ikke fungerer eller inverter ikke er tændt.
	// Styring gik før i stå fordi der under inverterens EEPROM check i start_ini ikke svares.
	// Dette rettes så der vises E27 MODBUS Timeout efter power-up. Rettelser er i start_ini.c
	//
	// 14-09-2017 LCC V3 INV-V50_03-14092017
	// Rettelse i Modbus.c så RX cleares ved hver forespørgsel, da powerup støjpulser til tider kunne få modbus
  // til at gå baglås.
	//
	// 14-10-2017 LCC V3 INV-V50_04-13102017
	// Fejlagtig dobbelterklæring af temp er fjernet i inout.c
	// AUX 1 relæ funktioner indført med 5 funktioner valgt i parameter 87. Både som AUX 2, men også
	// wireless edge. Special Vacon out (s_out_tim) for hurtig reversering fjernes i programmet da det ikke er relevant mere.
	// Der er lavet ændringer i display_keypad.c interpreter.c inout.c og mtimer.c.
	//
	// 04-01-2018 LCC V3 INV-V50_05-04012018
	// Program opdateres med rettelser fra LCC V3 INV-V01_07 og LCC V3 INV-V01_08. Dog ikke parameter 84:02 mulighed da denne er fast 1.
	// Kip funktion udbygget med et valg der hedder 35:03. Åbne – stop – lukke – stop – åbne – stop etc.
	// ændringer er i display_keypad.c og interpreter.c
	// Rettelse da port ikke ville reversere på åbne-tryk ved stilling 35:03. Rettet i interpreter.
	// Rettelse omkring 1/2 åbne funktion som ikke virkede. Oversat forkert i inout.c. og i encoder.c
	// Rettelser omkring parameter 88:03 og 87:03 som ikke virkede efter begge relæer kunne sættes op til denne 
	// dør lås. Ref. sw. input bruges til dette formål også som kontrolsignal for om der er låst op.
	// Rettelser er i interpreter.c og inout.c
	//
	// 21-04-2018 LCC V3 INV_LSS-V050_07-21092018
	// Første program hvor der skal køres med LSiS Inverter S100.
	// Modbus.c ændres til denne.
	// Program opdateres til nyeste V050_07 version fra EURA drive.
	// Program udbygges i parameter 84 med 02 valg så port altid vil lukke ved konstant lukke signal.
	// Taget fra V7E program. Parameter 111 og 121 ændres i fabriksindstilling fra 25 Hz til 35 Hz.
	// Forbedring for støjimmunitet m.h.t. encoder kommunikation indbygges ligesom i LCC V3D selvom fejl ikke er set her
	// endnu. Der er ændret i interpreter.c encoder.c display_keypad.c start_ini.c og defines.h for version.
	// Rettelse omkring STOP tryk idet rettelse fra 04-01-2018 kunne få port til at gå i baglås så den kørte videre over endestop
  // og ikke reagerede reagerede på stop. Fejlen kunne frembringes hvis man kørte dødmand og holdte OP tryk inde og så trykkede
  // meget kortvarigt på STOP. Software kunne da bringes i en stilling hvor safety test var aktiveret, men port kørte op så 
	// safety test ventede samtidigt på at port skulle stoppe. Det løses så på en anden måde i interpreter.c 
	// Rettelse omkring autolukke fra halv åben som ikke fungerede korrekt i encoder.c.
	// Rettelse omkring KIP da port kunne stoppe på vej op og begynde at køre ned ved aktivering af KIP hvis knap blev holt inde.
	// Ændring as vaskehalsfunktion så indstilling 33:01 betyder minimum tid der så sættes til 20mS i stedet for 100mS.
	// Dette fordi kunder oplyser at en cykelist kan slippe igennem uden aktivering. Ændring er i mtimer.c
	// LSis versionen har også ændringer i display_keypad og start_ini.c  
	//
	// 28-11-2018 LCC V3 INV_LSS-V050_08-28112018
	// Rettelser i modbus.h for at teste om decelleration kommer til at virke som den skal.
	// Rettelser omkring AUX relæ 1 som ikke virdede i parameter 87:03. Det er i interpreter.c
	// Checksum 0x006EA53C
	//
	// 29-04-2019 LCC V3 INV_LSS-V01_10-29042019
	// Med baggrund i LCC V3 INV_LSS-V050_08-28112018 laves et nyt program hvor der tilføjes rettelse fra LCC V3 INV-V50_09. Derefter fjernes brandventilationsopsætninger.
  // Efter dette laves de nye ting som egentlig var lavet i et Eura drive program. Det omdøbes og så tages ændringer fra dette program som hedder LCC V3 INV_LSS-V01_tst-01042019	
	// Rettelse omkring vaskehal som ikke virkede når der var valgt korteste fotoaktiverinstid og fotot var valgt i portramme. (i mtimer.c)
	// Rettelse omkring factory reset så programmering altid starter op i parameter 1. (i start_ini.c)
	// Ændring så det ikke er med brandventilations UPS setup. (ændres i display_keypad.c og start.ini) 
	// Nye ting til Dan-Door lægges som ændring 30-04-2019.
	// 30-04-2019 Program udbygget med specialfunktion for ref. SW input som benyttes til sikkerheds NC switch hvis dug i port er kørt ud. (Dan-Doors)
  // Når NC switch er aktiveret kan port ikke køre. Efter at dug manuelt er sat på plads og switch er deaktiveret kan port kun køre dødmand op indtil både åbne og lukke endestop
  // har været aktiveret. (også selom der har været slukket for strømmen. Det er lidt specielt for under denne dødmandskørsel skal kunne køres længere op end åbne endestop minimum.
	// der skal kunne køres på 90% af åbne endestopsvindue. Er lavet men ikke testet.
	// Hvis parameter 70 vælges som 70:01 vil:
  // Hvis parameter 84 = 03 vil den resettes til 00
  // Hvis parameter 87 = 03 vil den resettes til 00
  // Hvis parameter 88 = 03 vil den resettes til 00
	// Filter påført ecode7 fejl på Kostal potentiometerfejl og ecode8 fejl for Kostal fejl i forsyningsspænding. Er lavet.
  // Enkoderretning indlæres automatisk under parameter 14 ligesom på LCCV3D DSP. Parameter 11, 13 og 15 udgår da de ikke er nødvendige.
  // I dette program er altid valgt elektroniske endestop så DIL switch 10 anvendes ikke. E_limit sættes til 1 under powerup men sættes også i gen_ini som dobbelt sikring.	
	// Limits info vises i display ved power-up.
	// Foto i portramme preset med 3-dobbelt tryk er lavet og det vises i display 2 sek. når punkt indstilles.
	// 14-08-2019 STO funktion lavet som parametervalg på parameter 89:02 hvor AUX3 SS-relæ tilsluttes S100 SB og SC.
	// SC og SA forbindes med lus. Chk_weld er justeret for enkodertype. Hvis Inverter kører videre efter stop deaktiveres
	// AUX3 og der vises i display Inverter fail - Try new power-up - between door limits.
	// Der er ændret i main.c encoder.c interpreter.c display_keypad.c eprom.h globar_Var.c global_Var.h. mtimer.c
	// Checksum 0x0072530A
	//
	// 12-02-2020 LCC V3 INV_LSS-V01_11-12022020
	// Program til Dan-Doors ændres. 
	// Efter indlæring af endestop skal indlæres "curtain rep setpoint" i parameter 70.
	// Port kører dødmand ligesom når speed control indlæres, men under denne kørsel kan køres på 90% af opne endestopsvindue.
	// Ved tryk på stop indlæres et "curtain rep point". Ændringer i display_keypad.c, interpreter.c, encoder.c.
	// Hvis endestop grænser ændres skal foretages ny indlæring af "curtain rep setpoint" i parameter 70:01.
	//
	// 15-04-2020 V3 INV_LSS-V01_11UM-15042020
	// Programnavn lavet lidt kortere da software ikke kan håndtere et længere navn.
	// Program hvor test af om motor kører hvor den skulle have været slukket er frakoblet (ved inverterfejl)
	// Program er til test hos Rite Hite. 
	// 24-06-2020 Rettelse omkring fejlagtig inverterfejl visning så styring går i fejl.
	// Rettelser er i display_keypad.c og interpreter.c
	// Checksum 0x00732087
	//
	// 23-10-2020 V3 INV_LSS-V10_01UM-23102020
	// Special software for Dan-Door hvor Dan-Doors absolute enkoder kan tilsluttes (speedtech SCE absolute enkoder)
	// Denne enkoder sender hele tiden og skal ikke forespørges. Den sender 3 bytes, 6 bytes og 7 bytes. 
	// 17-12-2020 Der er ændringer i modbus.c, enoder.c, stm32f0xx_it.c learning.c og interpreter.c display_keypad.c. 
	// 05-01-2020 Flere nødvendige ændringer. Der laves også alarmering for lav batteri og stop ved encoder reset.
	// Hvis batteri er helt fladt eller enkoder er resat skal indlæres påny. Det vises på display.
	// Ved alarm for lavt batteri kan stadig køres men alarm vises skiftevis på lukke endestop.
	// 14-01-2021 com pakker fra enkoder laves i 3 særskilte arrays da der eller s periodisk går galt med synkroniseringen
  // fordi der kommer flere pakker indenfor et encoderrutine interval. se evt. konstruktionslog.  
	// Checksum  0x0074F634
	// 
	// 27-01-2021 V3 INV_LSS-V10_02UM-27012021
	// Indførelse af inaktiv kantliste de første 100mS efter hver start af kørsel. Dette for at gøre fejlaktiveringer
  // mindre ved brug af optisk kantliste på inverterstyringer. 
	// Checksum  0x0075157E
	//
  // 27-10-2021 V3 INV_LSM-V10_03UM-27102021
	// Med basis i V3 INV_LSS-V10_02UM-27012021 laves et program for M100 inverter til brug med en 2.2 kW motor.
	// 27-10-2021 Rettelser vedr. M100 setup som er anderledes end S100. Factory reset ændres ved Motor 1.5kW og 
	// Motor current 6.5A. Ændringer i display_keypad og start_ini.
	// Checksum 0x0075120B	
	//**********************************************************************************************************************************
	//**********************************************************************************************************************************
	//
	// V7E SRV2_V01-20102021
	// Første software til ny portstyring V7E SRV2.
	// Med basis i portstyringssoftware V3 INV_LSM-V10_03UM-27102021 videreudvikles dette program med flere funktioner fra bl.a. fra V7E SR protstyringen.
	// Der skal også bruges stumper fra LCC-V3D styringen omkring kontaktorstyring "control_motor_relays".
	// For at bibeholde historik inkluderes ovenstående basissoftware programlog.
	// 01-11-2021 Kommentarer til rettelser vedr. compilering.
	// 02-11-2021 Kommentarer til rettelser vedr. compilering.
	// 03-11-2021 Kommentarer til rettelser vedr. compilering.
	// 04-11-2021 Kommentarer til rettelser vedr. compilering.
	// 06-01-2022 Fejl rettet som skal rettes i andre programmer også. I display_keypad.c
	// 10-01-2022 Rettelse omkring kip(go func) i 35:03 som ikke virkede korrekt hvis 31:00 var valgt. I interpreter.c
	// 10-01-2022 Radio input lægges softwaremæssigt parallelt med Go func indtil videre. 
	// 11-01-2022 Ekstra test på ex_dw8k2 laves. Parameter 23:03 indføres som på V7E SR.
	// 12-01-2022 Rettelser omkring ekstra kantliste og indførelse af safety test på denne. Interpreter.c
	// 12-01-2022 Rettelse omkring 23:03 som ikke virkede korrekt. Interpreter.c
	// 24-01-2022 Parameter 11:00 ændres til 11:07 som ConDoor.
	// SCE encoder is not testet. Parameter 11:08 and 11:09.
	// Checksum 0x00C17AF2
	//
	// V4D-V01_00-10-03-2022
	// Første programversion til prototyper på LCCV4D.
	// 03-05-2022 rettelse i interpreter vedr. safety req som ikke skal køres i prog mode. 
	// Burde være nødvendig i V7E SRV2 også så det er lidt mystisk.
	// Rettelser i display_keypad.c og interpreter.
  // 06-05-2022 Rettelser omkring styring af display da colon ikke kører på digit 3 og 4 men digit 2 og 3 i stedet.
	// Factory reset ændres til 1:03 2:01 31:01. Parameter 31 ændres så kun 0, 1 og 4 er mulig da der kun er en foto.
	// 08-06-2022 Rettelse omkring factory clear som ikke virkede p.g.a. e27 testbit i keypad.c og interpreter.c
	// Tilføjelser/forbedring fra V7E SR til denne software også. I interpreter.c, display_keypad.c og read_ad_input.c
  // 09-06-2022 Rettelser i factory clear når inverter ikke er påsat. parameter 5 ekskluderes fra factory reset da det ikke kan
	// forenes med inverter factory reset. rettelser i main.c start_ini.c display_keypad.c og eprom.h
	// Rettelse omkring push_buttons på grafisk display hvis det ikke er forbundet i display_keypad.c
	// Mekaniske endestop mulighed fjernet i display_keypad.c da der ingen klemmer er for dette.
  // 10-06-2022 Systick justeres til korrekt 3.33mS gennemløb i stm32fxx_STLmain.c
  // Parameter valg 71:09 og 72:09 fjernes som valg da denne styring ikke har hardware til at køre med special trafic
  // light med 4 lamper.	
	// Checksum 0x00AA7C54
	//
	// V4D-V01_01-28-03-2023
	// 06-02-2023 Program udvides med sikkerhedsting ved inverterbrug som TÜV har krævet. (selvom denne måske ikke skal TÜV godkendes)
	// Dele taget fra V7E SRV2 programmet.
	// I stm32fxx_STLstartup.c er tilføjet en ekstra RE5 og RE6 relæ turn-off lidt efter kontaktor i tilfælde af FailSafePOR som køres ved ClassB fejl. 
	// Modbus genstartes hvis inverter har været slukket på grund af sikkerhedsting.
	// Chk_weld_value ændres fra 6 til 13 da den f.eks. i V7E SR er sat til 25 så 6 er nok for følsom. ecode3_tim reduceret i tid for hurtigere reaktion
	// Det skal muligvis justeres efter test i den virkelige verden.
	// Rettelser omkring EE_limits_check da en fejlagtig værdi kunne gøre at der køres low speed hele tiden. (værdi 0xFC).
	// Rettelser omkring åbnetryk ved power-up så port ikke kører op. ft_stop_pb derfor ændret til > ft_open_pb.
  // Der er ændret i main.c encoder.c stm32fxx_STLstartup.c start_ini.c global var.c og inverter_safety.c er tilføjet.
	// 11-04-2023 Rettelser omkring interlock der kunne aktiveres hvis photo er langsom ved sikkerhedstest på åbne endestop.
	// e5_photo extra clear at power up laves fordi photo er langsom til at tænde.	
	// Inverter tændes ved powerup hvis den er valgt. 
  // meas_dir_interval ændres så der kan indlæres langsommere kørsel med inverter. Envidere laves denne indlæring også ldt om
  // Så der først startes indlæring af enkoderretning 2 sek. efter at nedkørsel er startet. Dette fordi der er observeret at når
  // der er indlært åbne endestop og lukke endestop så skal indlæres, er der et slup i det mekaniske system med gear og enkoder
  // så der går 1.6 sek. førend enkoder faktisk flytter sig. (MFZ motor og inverter 10 Hz)
  // Fejl omkring mystisk stop under indlæring af endestop rettet. (omkring EEPROM gemning af værdier)	
	// Rettelser er i mtimer.c start_ini.c learning.c eprom.c.
	// Symbol for SAFETY CHAIN ændres og symbol for Emergency stop indføres på LED display. Der er ikke lavet specielt Emergency
	// stop symbol på grafisk display endnu. Her vises bare det alimindelige stop symbol endnu.
	// Checksum 0x00ABF307
	//
	// V4D-V02_01 06-11-2023
	// 30-10-2023 Program ændret til ny hardware.
	// USARTS3 kommunikation er ændret fra Extern half duplex til full-duplex på USART7 således at Option RX485 er ledig til andet formål.
	// Ændringer i display_keypad.c og meget i main.c omkring USART7 opsætning.
	// Symbol rettelse ved indlæring af parameter 14 der viste åbne symbol. Rettelse i display_keypad.c 
	// Display_keypad.c er flyttet i main.c for at få mere stabil multiplex på LED display
	// Tilføjelser i display_keypad.c for at få OLS symbol til at holde op med at blinke når åbne endestop nås.
	// Et lille blink mellem sidste 10 fejl visning indført så man kan se at man skifter hvis samme fejlkode. I display_keypad.c
	// 06-11-2023 Rettelse omkring special Go function der ikke fungerede hvis foto test er for langsom i kombination med wicket testen der
  // også testes ved stop. Rettelse er i interpreter.c
	// 09-11-2023 Parameter 5 sættes til 5:00 for LED display ved ny styring. Ændres ikke ved fakbriks reset. Ændring er i start_ini.c
	// 21-11-2023 Parameter 41 opsætning ændres så det kun er muligt at opsætte kostal til 41:00 og 41:04. Med ikke kostal er
	// det muligt at vælge 41:00 og 41:03 Rettelser er i display_keypad.c	(parameter 44 manglede i manual)
	// Rettelse omkring visning af sidste 10 fejl hvor der ikke blev vist en streg når der ingen fejl var men et tilfældigt
  // symbol. Rettelser i display_keypad.c. Rettelse omkring E:27 fejl som blev registreret i fejllog sat ved power-up når ingen
  // inverter var tilsluttet. Rettelser i main.c
	// 22-11-2023 parameter 11:07 gøres aktive for mekaniske endestop på FREE IN2 og FREE IN3. Rettelser i inout.c og display_keypad.c
	// og encoder.c Parameter 16:04 indføres hvor det er muligt at lukke fra halv åbne endestop. Rettelser er i interpreter.c display_keypad.c
	// inout.c and encoder.c Tilføjelser omkring interlock på halv åbne endestop. rettelser i interpreter.c. Safe_stop visning som
	// fast lys på folie LED tilføjes. Der er bl.a. Encoderfejl 1-9 samt svejst kontaktor.
	// 23-11-2023 Rettelse omkring parameter 71:55 og 72:55. Automatisk nulstilling af parameter 16:01 til 16:00 hvis encoder er 
	// valgt. Rettelser er i interpreter.c
	// Checksum 0x00AD5CA9
	//
	// V4D-V02-02 03-04-2024
	// Ny parameter 55 tilføjet hvor program kan sættes op, så reversering på kantliste kun reverserer 3 sek. Dette gælder også hvis der
	// køres dødmand ned. Programtilføjelsen er til Caljan. Fabriks reset på parameter 55:00 og parameter 21 ændres til 21:01.
  // efter fabriks reset skal Caljan ændre følgende parametre: 02:00 11:07, 55:01.
	// Rettelse i elektronisk tællervisning hvor de øverste 3 digit ikke blev vist.
	// Rettelser omkring fejlagtig kortvarig lukke endestopsvisning ved hvert stop. Rettelse omkring for stor filtertid på åbne og lukke endestop
	// ved valg af mekaniske endestop.
	// Der er rettet i interpreter.c display_keypad.c mtimer.c start_ini.c inout.c.
	// Checksum 0x00ADE273
	//
	// V4D-V02-03 29-05-2024
	// EEPROM ændres til større 32k type, men linier i eprom.c bibeholdes så der let kan skiftes tilbage til tidligere 8k type igen.
	// Rettelser er i eprom.c
	// Rettelse omkring curtain switch som skal være NC. Rettelse er i encoder.c
	// 17-06-2024 Rettelser omkring parameter 25 automatisk justering til 02 hvis parameter 16 > 0 eller relæ 7x:54 er valgt.
	// Rettelser omkring efterløb justering ved low speed kørsel ved curtain switch aktivering der kun skal virke med inverterdrift.
	// Rettelser. Ingen curtain function skal være mulig med mekaniske endestop.
	// Tilføjelser med automatisk instilling af parameter 26 og 27 ved mekaniske endestop valgt.
  // Rettelse omkring LED display ved indikering af photo disable point.
  // Rettelser omkring utilsigtet reset ved indlæring af "force control". Det skal rettes i bl. a. i V7E-SRV2 også.	
	// Der ændres i photo indlæring i portramme så 3 dobbelt tryk ikke virker mere (mtimer) og der indlæres ikke ved kørsel op mere.
	// Ved parameter 31:04 skal efter run vises blot manuelt køres hen til hvor man ønsker photo skal udkobles. Der bekræftes med 
	// PH - S for photo set.
	// Der er rettelser i display_keypad.c og interpreter.c og encoder.c og mtimer og req_photo_learn.c
	// Checksum 0x00AE6441
	//
	// V4D-V02-04 12-11-2024
	// Special function parameter 84:03 tilføjet hvor port skal stoppe hvis både åbne og lukketryk er aktiveret. Må ikke starte igen 
	// førend begge knapper har været sluppet, men det burde være i programmet. Ændringer er i interpreter.c og display_keypad.c
	// 19-11-2024 Rettelser omkring parameter 104 visning på LED display der viste 109. Rettelser omkring inverterfejl visning med
  // LED display idet der manglede visning af E:31.
	// 22-11-2024 Der ændres i Inverter safety som der ikke slukkes for inverter power førend efter 5 sek. efter power-up i tilfælde
  // af fejl. Dog hurtigt hvis der er moving fejl ecode3 (encoder flytter sig med burde være stoppet).
	// Inverter slukkes også kun for EE:01 fejl hvis endestop er indlært ecode2 == 0. ændringer i interter_safety.c primært.
  // AUX2 fabriksreset ændres fra 72:00 til 72:25. 
  // Automatisk setup af parameter 22 under indlæring af endestop gøres valgbar i define.h Automatisk setup af parameter 22 er 
  // frakoblet i dette standard program. ConDoor indlæring på gulv er også frakoblet i dette standard program.
	// 25-11-2024 Forbedringer omkring powerup efter fakbriksreset uden encoder før indlæring af endestop så inverter ikke slukker.
	// Reset af inverter fejl når denne læses uden fejl (inv_fail_active) blev før kun resat ved power-up. Der er rettelser i 
	// inverter_safety.c og display_keypad.c. 5 blink på folie LED indføres ved E31 fejl så kunde kan se den fejl udefra. Rettelse
	// i error_codes.c
	// Checksum 0x00AE5DBA
	//
	// V4D-V02-05 28-03-2025
	// Rettelse omkring relæ indstilling 40 da safety test fik relæ til at aktivere kortvarigt. 0.5 sek. delay for relæaktivering
	// er indført for at undgå dette. Skal rettes i V4D-V03-02 også og måske andre. Ændringer i interpreter.c og mtimer.c
	// Checksum 0x00AEAC1F
	//
	// V4D-V02-06 08-05-2025
	// Ny relæparameter xx:45 tilføjes til brug ved Caljan og andre. Functionen er ligesom xx:40 (ON by error) men blot inverteret. 
	// Derudover indføres flere fejl ting på både xx:40 og xx:45som skal aktivere for fejl. Det er afbrudt 8k2 kantliste,
  // Emergency Stop aktiveret, Safety chain aktiveret, begge endestop aktiveret og krystal fejl. De andre fejl der var der i forvejen var  
  // diverse E-koder samt enkoderfejl. Ændring indføres på alle relæer aux1, aux2, aux4 og aux5.	
	// Checksum 0x00B0BF64
	//
	// V4D-V02-07 05-12-2025
	// 05-12-2025 Rettelser omkring relæfunktioner i relæparameter xx:06 og xx:14
	// 08-12-2025 Rettelse på faktory clear på parameter 80 til 80:01.
	// Rettelse på relæparameter 20 så der ikke kommer impulse hvis det er en halv åbne komando fra åben position (16:03).
	// Rettelse relæparameter 21, Så der ikke tændes på halvåbne position og tid fryser.
  // 09-12-2025 Rettelse omkring forvarsling hvor der ikke kunne lukkes eller åbnes når halv åben endestop 16:03 og 16:04 var valgt også.	
	// Denne fejl må findes i software for V7E SR også.
	// Der er rettelser i mtimer.c og interpreter.c
	// Rettelse så man ikke kan køre ud af indlært område når man indlærer halv åbne endestop.
	// Der er rettelser i encoder.c og display_keypad.c
	// 09-12-2025 Rettelse i relæparameter 20 så der ikke kommer puls ved lukke på halv åbnetryk ved parameter 17:01
	// Rettelse omkring parameter 16:02 og 16:03 hvor autolukke ikke fungerede korrekt idet halv åbnetryk skal cleare autolukke og interlock
	// Fejlkode E:01 (kantlistefejl) flyttes op i prioitet.
	// Rettelser omkring KIP funktion som ikke virkede fra halv åben ved indstilling relæindstilling XX:54
	// 10-12-2025 Rettelse omkring 75/76/88:50 som fejlagtigt slukker relæ på halv åben når DW er valgt og efterløb er frakoblet. 
	// Ydermere aktiveres relæ hvis 75/76/88:50 aktiveres når port er placeret over lukke endestop
	// Rettelse så relæparameter xx:54 ikke resetter parameter 16 førend indlæring finder sted i xx:54.
	// Rettelse omkring enkoder test funktion tager fra V7E SR program.
	// Rettelse så der altid køres low speed ved indlæring i parameter 16
	// INDFØRELSE AF LOW SPEED FOR INVERTER VED HALV åBNE ENDESTOP ER IKKE LAVET ENDNU. DETTE FOREGåR UDELUKKENDE I ENKODER PROCESSOR 7E-IC4K71-5-SR
	// I V7E-SR STYRINGEN, DET ER OMFATTENDE AT LAVE NU Så DER VENTES LIDT MED DETTE.
	// 11-12-2025 Rettelse omkring Lamp-PCB så relæer ikke trækker kortvaring når der stoppes på halv åbne selvom det først skal trække på helt åben elle lukket.
  // Parameter 16 resettes til 16:00 ved ny indlæring af åbne eller lukke endestop.
	// Rettelse omkring 16:02 hvor Go function (KIP) kan åbne i små ryk når port står på halv åben.
	// Tilføjelse ved parameter 80 valg > 2 så afbrydelses tid startes ved valg. Rettelser i display_keypad.c
	// Rettelse omkring KIP da den ikke kunne lukke fra halv åben endestop.
	// Rettelse omkring parameter 71,72,74,75:52 så tiden er den samme selvom 2 eller 3 relæer er valgt til samme funktion.
	// 11-12-2025_b Rettelse omkring 71,72,74,75:50 sammen med forvarsling hvor der ikke kunne lukkes hvis der var power off når
  // port står mellem endestop. Rettelser i interpreter.c
	// 11-12-2025_c Rettelse omkring parameter 71,72,74,75:43 sammen med Foto1 indlært i portramme 31:04 så det også kan vises på vej op at foto er brudt.
	// Rettelser er i interpreter.c
	// 11-12-2025_d Rettelse omkring "before open limit" og "before close limit" aktiveringer ved relæparametre xx:30 og xx:31. 
	// Det virkede ikke førend parameter 85 og 86 blev ændret i værdi. Rettelser i encoder.c.
	// 11-12-2025_e Korte blink ved relæparameter xx:41 og xx:43 som kommer ved safety check ved hvert stop fjernes.
	// 11-12-2025_f Rettelse på relæparameter xx:52 hvor tiden 2 minutter ikke passede. Rettelser er i interpreter.c
	// 11-12-2025_g Rettelse af fejl omkring Dock leveler ved relæparameter xx:54. Rettelser i display_keypad.c
	// Checksum 0x00B313EE
  //
  // V4D-V02-08 13-02-2026
  // LCCV4D software udbygges med ting omkring low speed ved halv åbne endestop som findes i V7E SR programmet, men som ikke er ført over i LCCV4D programmet endnu. 
  // Det er ting som skal tages fra 7E-IC4K71-5-SR.
  // Ændringer i 7E-IC4K71-5-SR omkring E:09 fejl indføres ikke i dette program da det allerede er lavet på en anden måde i V4D programmet.
  // Rettelser fra 25-10-2022, 08-11-2023, 04-04-2025 i 7E-IC4K71-X-SR laves ikke da det vist ikke er relevant for V4D programmet.
	// 20-02-2026 Rettelser omkring lysgitter 21:06. Rettelser i main.c
	// 04-03-2026 Udvidelser af relæfunktioner med en relæudgang for lås med checksignal for åben lås på free input 2.
	// Styring venter med at køre indtil signal "åben lås" er observeret. Hvis ikke signal komer inden 2 sekunder vises E:33 fejl.
  // EE_par_70 fjernes i programmet flere stedder da der ikke findes AUX0 på V4D styringen	
	// Checksum 0x00B53706
	//
	// V4D-V02_10 30-03-2026
	// Program som V4D-V02-08, men beregnet for 32k EEPROM.
	// Checksum 0x00B53529
	//
	// V4D-V02_11 08-07-2026
	// Rettelse omkring autolukke som skal cleares ved tryk på lukke på GD-V1 displaymodul også.
	// Rettelser - alle steder med lukketryk og åbnetryk kontrolleres om lukketryk og åbnetryk display også er med.
	// Rettelse omkring warning som skal stoppe hvis auto-lukke reloades ved åbnetryk.
	// Rettelse er i mtimer.c og interpreter.c
	// Checksum 0x00B5507A
  // 
	//*****************************************************************************************************
  //	
	// *** Watchdog start *********************************************************************************	
	//	HAL_IWDG_Start(&hiwdg); // startes automatisk i nyere cubeversioner 4.18
  //  IWDG is 12 sec. at powerup to prevent timeout at modbus init 16-03-2017 
	//DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP; // watchdog downcounter is stopped by debug stop
	//aux2_on; // activate STO for S100 inverter by powerup 14-08-2019
	init_eeprom(); // moved from start_ini because we shall use EEPROM values now
	EE_read(EE_par_5); // Turn on inverter if selected  28-03-2023
  if (temp > 1)	
	 { // 21-11-2023
		door_up_on; // turn ON power to inverter.  28-03-2023
		timeout_modbus = 0; // 16-03-2017
	  e27_modbus = 0; // 16-03-2017
	  digit_1_on; // T9 OFF turn off LED display until right update is ready
	  digit_2_on; // T8 OFF
	  digit_3_on; // T7 OFF
	  digit_4_on; // T5 OFF
	  HAL_IWDG_Refresh(&hiwdg); //  refresh to prevent WDT timeout debug test 06-12-2021
	  for (int i=0; i<250; i++) // 2.5 sec. melko wait for inverter powerup. 1.9 sec. is measured as minimum
	                            // before the inverter is ready
     {
      HAL_Delay(10); //  10mS steps
		 }	
	  do
	   {
	    timeout_modbus++;	
	   }
	  while ((timeout_modbus < 3) && (initModbusParam() != HAL_OK)); // 16-03-2017  Max 2 init cycles
	  //	
	  if (timeout_modbus > 2) // more than 5.5 sec. with no answers
	   {
		  e27_modbus = 1; // check modbus connection + new powerup // 16-03-2017
	   }
    else	
	   {
		  inv_timeout = 1501; // prevent ecode1 fail stop after restart modbus	28-03-2023  
      // Configuring needed modbus Running Parameters	
      // Following modbus commands shall not be used for LSis. This is setup by PC and Driveview7 file which are downloaded				 
      //    do  // do added 13-06-2017		 
      //    status = getModbusState();
      //    while (status == HAL_BUSY);
      //    HAL_Delay(3); 
      //	  status = setModbusParam(F219_EEPROM_WRITE , 0); // must be 0 for save in EEPROM after power off. LSis ?
      //    do
      //    status = getModbusState();
      //    while (status == HAL_BUSY);
      //    HAL_Delay(3);
      //	  status = setModbusParam(F549_1ST_STG_DIR , 0);
      //    do
      //    status = getModbusState();
      //    while (status == HAL_BUSY);
      //    HAL_Delay(3);
      //		status = setModbusParam(F550_2ND_STG_DIR , 0);
      //    do
      //    status = getModbusState();
      //    while (status == HAL_BUSY);
      //		HAL_Delay(3);
      //    setModbusParam(F551_3RD_STG_DIR , 1);
      //    do
      //    status = getModbusState();
      //    while (status == HAL_BUSY);
      //		HAL_Delay(3);
      //    setModbusParam(F552_4TH_STG_DIR , 1);
      //    do
      //    status = getModbusState();
      //    while (status == HAL_BUSY);
	   }
   }	 // 21-11-2023
  //
	 
	//HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4); // used for safety check - clock check - 01-11-2021 ændret fra htim14 04-11-2021
	HAL_TIM_IC_Start_IT(&htim14, TIM_CHANNEL_1); // used for safety check - clock check 
  HAL_ADCEx_Calibration_Start(&hadc); // calibration of ADC for better accuracy typ. 5.9 uS only 
	 
	HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1); // start capture input for speed pulse	 
	//HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // start capture input for speed pulse - before timer 17 but now timer 2 01-11-2021	 
	//
  STL_InitRunTimeChecks(); // Routine moved on the other side of start_ini to prevent false EEPROM read when
	                         // tick timer is 1 mS. Tick timer is now adjusted i stm32fxx_STLmain.c line 98 + 100
                           // so it's not neccesary to adjust afterwards.	29-03-2017
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); // need to be there because STL_InitRunTimeChecks() make priority 192 for systick 
	                                          // 29-03-2017
	HAL_TIM_Base_Start(&htim6);	// Used for SCE encoder 04-11-2021
  __HAL_RCC_TIM6_CLK_ENABLE();
  //DBGMCU_APB1_FZ_DBG_TIM6_STOP; // 04-11-2021 ved ikke om lignende kan bruges
  __HAL_DBGMCU_FREEZE_TIM6();	 
	//
	RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;		
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP; // watchdog downcounter is stopped by debug stop
	//HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/600); // already adjusted in stm32fxx_STLmain.c
	//HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); // need to be there because line above make priority 3 for systick
  // 
	HAL_TIM_Base_Start(&htim3);	// timer for ADC DMA update 09-12-2021 
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_DBGMCU_FREEZE_TIM3();
  HAL_ADC_Start_DMA(&hadc, (uint32_t*) &AD_values,4); // 08-11-2021 starts ADC with DMA to fill AD array automatic. There should be be no need for using call_back is setup in cubemax is sample time set to 28 cycles and by 18MHz clock this is 1.5 uS. 
	// 
	start_ini(); // moved to just before while 02-05-2022
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	{
	 display_keypad(); // moved to top to get stable multiplex on LED display display	30-10-2023
	 serviceModbusHeartbeat(inverter_use);
	 if (READ_BIT(RCC->CSR, RCC_CSR_IWDGRSTF)) e28_wdt = 1; // Error code set by watchdog timeout 29-11-2016
   else e28_wdt = 0;                                      // only cleared by power off or open P/B, if set	
	 watchdog_on; // processor running monitor output
   timeTick1 = HAL_GetTick();
	 //**************************************
	 if (req_encoder == 0)
	  { 
		 //HAL_Delay(0); // debug wait 1.11 mS for stable LED display
     encoder(); // only runned half of main cycles cause of slow responds
		 encoder_TX();	
	  }
	 else 
	  {
		 HAL_Delay(0); // wait 1.11 mS for stable LED display and need to prevent UART_Receive timeout if display update i > 4mS
		}
	  req_encoder = !req_encoder; // toggle
	 //**************************************
	 EE_read(EE_check);	// 21-03-2017
	 if ((temp == 0) &&(dil_3 == 1) && (open_long_tim == 0) && (stop_disp_pb == 1) && (close_pb == 1)) // 22-03-2017
		 
	  {
	   EE_write(EE_check,255); // total clear with electronic counter	
	   open_long_tim = 1000; // not clear again next 3.3 sec 21-03-2017
		 ver_show = 450; // Show version  -there is no new complete new powerup 21-03-2017		
     start_ini();	
	  }
	 else	
	  { 
	   EE_read(EE_factory);	
	   if ((temp > 0) && (dil_3 == 1) && (open_long_tim == 0) && ((stop_pb == 1) | (stop_lid_pb == 1) | (stop_disp_pb == 1)))
      {
       EE_write(EE_overv,255);
       EE_write(EE_factory,0);
			 open_long_tim = 1000; // not clear again next 3.3 sec 21-03-2017	
			 ver_show = 450; // Show version and factory reset there is no new complete new powerup 21-03-2017	
       start_ini();
      }	
	  }
	 EE_read(EE_par_21); //get edge type selected
   if (temp == 5) e1_mon = 0; // no monitoring by wireless optical
   edgetype = temp;
	 if (edgetype == 5) edgetype = 1; // output from wireless optical edge is like PNE 
   if (edgetype == 6) edgetype = 3; // simulate fraba type when RAYLG is selected 20-02-2026 
   //
    gen_ini(); // reinitializing for additional double safety with use of BOD
		read_inout();
    adap_run_time();
    read_ad_inputs();            // read P1, dw/8k2 input and option input2 if analog is wanted
		mtimer();                    // opdatering af multitimerere
		error_codes();               // LED code as on V7R SR - LED on membrane keypad 29-12-2021 
		last_error_upd();            // last error updating routine
		EE_read(EE_par_22); // clear monitoring fail fast if parameter 22 is turn back to 0 
    if (temp == 0) e1_mon = 0;
    if ((edgetype == 2) || (edgetype == 3)) e1_mon = 0; // only monitoring fail by pne edge 
    //option_on; // debug		
	  //display_keypad(); // moved to top to get stable multiplex on LED display 30-10-2023
		//option_off; // debug
		push_code_req();
    req_photo_learn();
		learning(); // Check about programming is wanted 30-04-2019
		req_status_wanted();
    if ((EE_read(EE_factory) == 0) && (ver_show > 0))
    fac = 1;
    else
    fac = 0;
		//
	 if (disp_status == 1)
 	  {
	  } 
   else
	  {
		//option_off; // debug	
    if ((run_prog == 0) | (para_test == 1)) interpreter();
    else
     {
      if (((parameter == 43) || (parameter == 42)) && (p_value == 1)) 
       {
        interpreter(); 
       }
      else
       {
        if (run_prog == 1) // prog selected, no para_test active, no manual speed adjustment active
         {
          stop();
         }
       }
     }
    }
		
		
    //led_error_off;	  //debug
		control_motor_relays();
		EE_read(EE_par_5); // run inverter safety only when inverter is selected 
    if (temp > 1)	inverter_safety(); // 28-03-2023
		//
		//option_off; // debug
		//HAL_IWDG_Refresh(&hiwdg); // reload processor watchdog. 50mS to timeout by prescaler 4 and reload 500
		                          // deaktives ved debug
//		minVariabel = *(int *) (mitArray);   // debug test af hard fault
//		minVariabel = *(int *) (mitArray+1); // debug test af hard fault
		watchdog_off; // processor running monitor output to external watchdog. 
		if (something_saved == 1)
		  {
			 timeTick1 = HAL_GetTick(); // preset timeTick1, value is unknown cause of EEPROM saving have be running 30-01-2017
			 something_saved = 0;	
		  }
		//led_error_on;	  //debug	
		STL_DoRunTimeChecks(); // refresh of watchdog also 06-12-2021 approx. 12 x 1.66m = 20 mS 
		//option_on; // debug	
    timeTick2 = HAL_GetTick();	
    timetick_calc = timeTick2 - timeTick1;			
		while (HAL_GetTick() - timeTick1 < 3) // 
  	 {
		 } // Wait for next loop period of 3.33 ms
	  timeTick3 = HAL_GetTick();	
    timetick_calc2 = timeTick3 - timeTick1;
		//option_off; // debug 
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
   /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_CRCEx_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00901D23;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 1875;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 10;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  TIM_IC_InitTypeDef sConfigIC;

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV8;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim14, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIMEx_RemapConfig(&htim14, TIM_TIM14_RTC) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  TIM_IC_InitTypeDef sConfigIC;

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 99;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65000;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim17, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 192000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart3, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART4 init function */
static void MX_USART4_UART_Init(void)
{

  huart4.Instance = USART4;
  huart4.Init.BaudRate = 57600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart4, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART5 init function */
static void MX_USART5_UART_Init(void)
{

  huart5.Instance = USART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART7 init function */
static void MX_USART7_UART_Init(void)
{
 	__HAL_RCC_USART7_CLK_ENABLE(); // added by Melko E. must be started before init
//
  GPIO_InitTypeDef GPIO_InitStruct;

/* Configure USART7 TX pin (PF2) */
GPIO_InitStruct.Pin = GPIO_PIN_2;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
GPIO_InitStruct.Alternate = GPIO_AF1_USART7;
HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

/* Configure USART7 RX pin (PF3) */
GPIO_InitStruct.Pin = GPIO_PIN_3;
HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
//	
//	USART7 Initialization:
  huart7.Instance = USART7;
  huart7.Init.BaudRate = 192000;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart7.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
//USART7 Interrupt Priority (Optional, set to priority 3):
HAL_NVIC_SetPriority(USART3_8_IRQn, 3, 0);
HAL_NVIC_EnableIRQ(USART3_8_IRQn);
	
}

/* WWDG init function */
static void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 127;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Ch1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA0   ------> SharedAnalog_PA0
     PA5   ------> SharedAnalog_PA5
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DISPLAY_DATA_Pin|DISPLAY_CP_Pin|DISPLAY_STR_Pin|UP_RUNNING_Pin 
                          |DOOR_UP_Pin|DOOR_DOWN_Pin|AUX1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIGIT_1_Pin|DIGIT_2_Pin|DIGIT_3_Pin|DIGIT_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LAMP_CLK_Pin|LAMP_DATA_Pin|K1_CTRL_Pin|SPEED_STAGE_0_Pin 
                          |DOOR_RUN_Pin|SPEED_STAGE_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SAFETY_TEST_Pin|LED_LEDPAD_Pin|WIFI_GP2_Pin|WIFI_GP1_Pin 
                          |AUX2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DISPLAY_DATA_Pin DISPLAY_CP_Pin DISPLAY_STR_Pin UP_RUNNING_Pin 
                           DOOR_UP_Pin DOOR_DOWN_Pin AUX1_Pin */
  GPIO_InitStruct.Pin = DISPLAY_DATA_Pin|DISPLAY_CP_Pin|DISPLAY_STR_Pin|UP_RUNNING_Pin 
                          |DOOR_UP_Pin|DOOR_DOWN_Pin|AUX1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OPTICAL_EDGE_Pin */
  GPIO_InitStruct.Pin = OPTICAL_EDGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OPTICAL_EDGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SAFETY_PHOTO_Pin CLOSE_PB_Pin OPEN_PB_Pin GO_FUNCTION_Pin 
                           STOP_PB_Pin STOP_LID_Pin */
  GPIO_InitStruct.Pin = SAFETY_PHOTO_Pin|CLOSE_PB_Pin|OPEN_PB_Pin|GO_FUNCTION_Pin 
                          |STOP_PB_Pin|STOP_LID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGIT_1_Pin DIGIT_2_Pin DIGIT_3_Pin DIGIT_4_Pin */
  GPIO_InitStruct.Pin = DIGIT_1_Pin|DIGIT_2_Pin|DIGIT_3_Pin|DIGIT_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AN_PNE_EDGE_Pin AN_WICKET_Pin */
  GPIO_InitStruct.Pin = AN_PNE_EDGE_Pin|AN_WICKET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FREE_IN1_Pin */
  GPIO_InitStruct.Pin = FREE_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FREE_IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FREE_IN2_Pin FREE_IN3_Pin */
  GPIO_InitStruct.Pin = FREE_IN2_Pin|FREE_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LAMP_CLK_Pin LAMP_DATA_Pin K1_CTRL_Pin SPEED_STAGE_0_Pin 
                           DOOR_RUN_Pin SPEED_STAGE_1_Pin */
  GPIO_InitStruct.Pin = LAMP_CLK_Pin|LAMP_DATA_Pin|K1_CTRL_Pin|SPEED_STAGE_0_Pin 
                          |DOOR_RUN_Pin|SPEED_STAGE_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EX_AUTO_Pin EMER_STOP_Pin */
  GPIO_InitStruct.Pin = EX_AUTO_Pin|EMER_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SAFETY_TEST_Pin LED_LEDPAD_Pin WIFI_GP2_Pin WIFI_GP1_Pin 
                           AUX2_Pin */
  GPIO_InitStruct.Pin = SAFETY_TEST_Pin|LED_LEDPAD_Pin|WIFI_GP2_Pin|WIFI_GP1_Pin 
                          |AUX2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SAFETY_CHAIN_Pin MON_PH1_3_Pin MON_PH2_3_Pin WIFI_GP3_Pin */
  GPIO_InitStruct.Pin = SAFETY_CHAIN_Pin|MON_PH1_3_Pin|MON_PH2_3_Pin|WIFI_GP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DIL_SW1_Pin DIL_SW2_Pin DIL_SW3_Pin */
  GPIO_InitStruct.Pin = DIL_SW1_Pin|DIL_SW2_Pin|DIL_SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void StartUpClock_Config(void)
 {
	SystemClock_Config();
 }

void gen_ini() 
{
// void MX_GPIO_Init();               // tager for lang tid - ca 100 uS = 5000 clockcycles
// void MX_ADC_Init();                // tager nok også for lang tid
;
EE_read(EE_par_41);
if (temp == 2) motor_rpm_1500 = 0;    //
else motor_rpm_1500 = 1;              //	
//if (temp == 2) TIM14->PSC = 47;     // prescaler for - speed 2600 - 3500 selected
// TIM14->PSC = 95;               // prescaler for 1300 - 1750 RPM
relearn_ph1 = 0; // update ram from EE 06-10-2010
relearn_ph2 = 0; // update ram from EE 06-10-2010
EE_read(EE_relearn_ph1);
if (temp == 1) relearn_ph1 = 1; // update ram from EE 06-10-2010
EE_read(EE_relearn_ph2);
if (temp == 1) relearn_ph2 = 1; // update ram from EE 06-10-2010
EE_read(EE_par_36);
if (temp == 1) interlock = 0; // Hvis parameter 36 = 0 cleares interlock automatisk 16-05-2013	01-12-2021
// E_limit = 1; // be sure this is always 1 in this program 3001-04-2019	17-01-2022 mechanical limits must be possible
}
//
void HAL_RCC_CSSCallback(void)
{
 e29_xtal = 1;	                    // code for processor main X-tal failed 
 MX_USART3_UART_Init(); 	// ændret fra USART1 22-03-2022
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
