//#define  EEPROM_WRITE_MODE		//		1

// Define the Pins

///// Button Control /////
// " 74HC4051M " Mux chip for multi-Switch......
// Start , Water , Steam
// Large , Middle , Small ( Current,  Not Use..... )
// 수위센서1 -- (S357_1 = x6[2], 수위센서2 -- (S357_2 = X4[1]) (Water Position Sensor 2EA)
/*int pin_muxOnCH = A5;   // PC5(ADC5)
int pin_muxAddress_A = 4;   // PD4(XCK)
int pin_muxAddress_B = 3;   // PD3(INT1)
int pin_muxAddress_C = 2;   // PD2(INT0)
int pin_muxCE = 1;   // PD1(TXD) ---- Mux: INH pin
*/

//int powerButton = 0;
/// OP-LED /////
//int pinOp_LED = 0;      // PD0 - Red <=> Green

////////////////////////////////// - ship heater.....//////////////
/// OP-LED /////
int pinPower_LED = 6;			// PD6 - Blinking

int pinRun_LED = 9;			// PB1 - Blinking
int pinPtc_LED = 10;		// PB2 - Blinking

// Temperature Sensor ......
// " MAX31865 " chip multi-Drop connected.......
int pinTemps_DRDY_T = A4;		// PC4
int pinTempsCS_1T = A3;			// PC3
int pinTemps_SCK_T = A2;		// PC2
int pinTemps_SO_T = A1;			// PC1
int pinTemps_SI_T = A0;			// PC0
#include <SPI.h>

int pinShip_Heater = 5;		// PD5


int pinHeatTemp_Var = A6;		// ADC6
int pinLed_Dimmer = A7;			// ADC7

// 1차 데모에서,  실제 35도 ON, 40도 OFF된다고 함. 그래서 보정하기 위해....
// 3창 데모네서,  (5.0도 정도 보정/평균을 했는데도 불구하고) 35도에서 On,  37.8도에서 Off된다고 함.
/* 3차 데모 후, 메일 내용 ==>
  "OFF   PCB가 읽어들이는 온도 53도(1160옴)   /     실제온도 37.8도(1075옴)
   ON    PCB가 읽어들이는 온도 45도(1130옴)   /     실제온도 35도(1047옴)
     위에서 언급된 내용이 반복되고 있습니다." */
  
//#define  HEATER_TEMP_DIFFERENCE		5.0
//#define  HEATER_TEMP_DIFFERENCE		10.0	// 50, 55
//#define  HEATER_TEMP_DIFFERENCE		20.0	// 60, 65
#define  HEATER_TEMP_DIFFERENCE		20.0	// 60, 70

//#define  HEATER_MAX_TEMP_LEVEL		45.0+HEATER_TEMP_DIFFERENCE		// Registance : 1200 (45)  ohm
//--5th demo #define  HEATER_MAX_TEMP_LEVEL		50.0+HEATER_TEMP_DIFFERENCE		// Registance : 1200 (45)  ohm ==> 70도
//#define  HEATER_MAX_TEMP_LEVEL		70.0+HEATER_TEMP_DIFFERENCE		// Registance : 1200 (45)  ohm ==> 90도
//#define  HEATER_MAX_TEMP_LEVEL		50.0+HEATER_TEMP_DIFFERENCE		// Registance : 1200 (45)  ohm ==> 70도
// 2021.11.29
#define  HEATER_MAX_TEMP_LEVEL		65.0+HEATER_TEMP_DIFFERENCE		// Registance : 1500 (90)  ohm ==> 90도 ==> 1300도 이상 => Ptc Led On....
//#define  HEATER_MIN_TEMP_LEVEL		-70.0+HEATER_TEMP_DIFFERENCE	// Registance :  200 (-70) ohm ????
// 2021.12.03
// 200ohm -> -35도 정도로 추정함.
// 90도일때,  1300 ohm이므로,  테이블과 300 ohm정도 차이가 나서  이렇게 추정함.
//#define  HEATER_MIN_TEMP_LEVEL		-55.0+HEATER_TEMP_DIFFERENCE	// Registance :  200 (-35도) ohm ???? 
//#define  HEATER_MIN_TEMP_LEVEL		-80.0+HEATER_TEMP_DIFFERENCE
//실측데이터
// -40 => 790 ohm
// -50 => 755
// -60 => 715
// -185 => 200 ohm
// 그래서, 그냥  -60으로 하기로 했음.
#define  HEATER_MIN_TEMP_LEVEL		-80.0+HEATER_TEMP_DIFFERENCE	// Registance :  200 (-185도) ohm ????

#define  HEATER_ONN_TEMP_LEVEL		40+HEATER_TEMP_DIFFERENCE// 2022 06 24 저항 극성으로인한 이슈
//#define  HEATER_OFF_TEMP_LEVEL		45.0+HEATER_TEMP_DIFFERENCE
#define  HEATER_OFF_TEMP_LEVEL		50+HEATER_TEMP_DIFFERENCE



int brightNessVal = 0;
int brightNessVal_Pow = 0;
int brightNessVal_Run = 0;
int brightNessVal_Ptc = 0;
//int tempOffSetVal = 0;

//const int led_toggle_Interval = 2; // 3.0 sec.
//int       led_toggle_Count = 0;
//bool      led_Mode0 = false;
//bool      led_Mode1 = false;
//bool      led_Mode2 = false;

//bool	powerOn_firstMode = true;
//bool    heater_On = false;

bool  security_Verified = false;

//char epp_pdw[16] ={"P5Xq7vNb83PD!ySk"};//예제 비밀번호
//char epp_pdw_read[16];//Read pw from eeprom 비밀번호	
//uint8_t  epppw_length = 15;// 비밀번호 자릿수 - 1

////////////////////////////////// - ship heater.....//////////////
/*

// Heater 
int pinHeater_1 = A4;     // PC4(ADC4) -- Water Box
int pinHeater_2 = A3;     // PC3(ADC3) -- Steam Box
int pinHeater_3 = A2;     // PC2(ADC2) -- Recipe Box

// Ambient(Proximity) Sensor....
int pinAmbient_sens3 = A1;     // PC1(ADC1) -- 상
int pinAmbient_sens1 = A0;     // PC0(ADC0) -- 하
int pinAmbient_sens2 = A7;     // ADC7      -- 중

// Sol. Valve....
int pinSolvalv_1 = 8;     // PB0 -- Water Input
int pinSolvalv_2 = 9;     // PB1 -- Steam Input
//--Not operating int pinSolvalv_3 = 14;     // PB6 -- Recipe Water Input(육수 보충)
//--Not operating int pinSolvalv_4 = 15;     // PB7 -- Recipe Input(조리)

// Pressure Sensor  for Steam Box.....
int pinPress_sens = A6;     // ADC6 

// Temprature Sensor ......
// " MAX6675 " chip multi-Drop connected.......
int pinTempsCS_1 = 11;     // PB3 
int pinTempsCS_2 = 10;     // PB2 
int pinTemps_SO = 12;     // PB4 
int pinTemps_SCK = 13;     // PB5 

// MOtor 
int pin_MotorEF = 7;   // PD7(AIN1)     ---> Error Flag..... (open drain)
int pin_MotorPwm1 = 6;   // PD6(AIN0)   ---> Motor UP move
int pin_MotorPwm2 = 5;   // PD5(OCO0B) ---> Motor Down move

//-- const int motor_moveloop_delay = 10;
const int motor_moveloop_delay = 20;

// 스팀용기  압력 : 압력이  3k를 넘으면 터질 가능성이 잇어서  1.5~ 2.0k 유지 예정(안정성....)
// Input : (20mA....) 0V ~ 5V로 입력되는 것같은데,  확인해야 함.........
// 압력 센서를 읽어서,  물통의   히터를 가열/정지 결정.....
// test시 온도센서를 달아서,   110 ~ 125도 사이일 경우의  압력값을 확인 후,  가열/멈춤.....결정.....
//const int steamBox_Press_Margin_MIN = 250; // 
//const int steamBox_Press_Margin_MAX = 300; // 
//const int steamBox_Press_Margin_MIN = 100; //
//const int steamBox_Press_Margin_MAX = 200; //
//-- 1.5 bar ( ... 250 )  start......
//-- 2 bar ( ... 300 ) stop...
// 230 - 108도
// 260 - 115도
const int steamBox_Press_Margin_MIN = 245; //
const int steamBox_Press_Margin_MAX = 275; //


 // 물통 수위 sensor : Low / High
// NO use : const float waterBox_Tempe_MIN = 125; // 물통 : 125 ~ 150도 유지
// NO use : const float waterBox_Tempe_MAX = 150; // 

// 육수통 수위  sensor : Low / High
//const int recipeBox_Tempe_MIN = 90; // 육수통 : 90 ~ 95도 유지
const int recipeBox_Tempe_MIN = 86; // 육수통 : 86 ~ 95도 유지
const int recipeBox_Tempe_MIDD = 90; // 육수통 : 86 ~ 95도 유지
const int recipeBox_Tempe_MAX = 95; // 

//--20200915 const int steamBox_Tempe_MIN = 260; // 스팀 통 : 260 ~ 290도 유지
//--20200915 const int steamBox_Tempe_MAX = 290; //
//--20201105 const int steamBox_Tempe_MIN = 390; // 스팀 통 : 260 ~ 290도 유지
//--20201105const int steamBox_Tempe_MAX = 420; // 490 (140..)
//t1 const int steamBox_Tempe_MIN = 150; // 스팀 통 : 260 ~ 290도 유지
//t1 const int steamBox_Tempe_MAX = 170; // 490 (140..)
const int steamBox_Tempe_MIN = 110; // 스팀 통 : 260 ~ 290도 유지
const int steamBox_Tempe_MAX = 130; // 490 (140..)

// 모터위치확인 근접센서 : High / Low..

// Motor Control
// 250 = Stop speed,  0 = Max speed
const int motor_speed_MAX = 0;   // ?????????
const int motor_speed_Middle = 128;
const int motor_speed_MIN = 255;  // ????????

//-- Origin const int motor_moving_Speed = 128; 
//-- very slow....... const int motor_moving_Speed = 64; // ask for lhh 1/2 (128 ==> 64)
const int motor_moving_Speed = 128; // ask for lhh 1/2 (128 ==> 64)
const int motor_moving_ToBase_Speed = 30;

const int motor_move_Base = 0;

//--origin const int motor_updownmove_Count = 60;
//--2.5sec test const int motor_updownmove_Count = 250;
//--20200817 const int motor_updownmove_Count = 350;
//- test only const int motor_updownmove_Count = 1300;
//--const int motor_updownmove_Count = 350;

//--200911 const int motor_updownmove_Count = 750;
//--200911 const int motor_updownmove_Count_to_Top = 500;
//--1105 const int motor_updownmove_Count = 650;
//--1105 const int motor_updownmove_Count_to_Top = 400;
//t1 const int motor_updownmove_Count = 350;
//t1 const int motor_updownmove_Count_to_Top = 100;
//t2 const int motor_updownmove_Count = 450;
//t2 const int motor_updownmove_Count_to_Top = 200;
const int motor_updownmove_Count = 750;
const int motor_updownmove_Count_to_Top = 450;
const int motor_updownmove_Count_downMore = 350;

enum {
	nobutton_Normal_mode = 0,
	
	large_ButtonPressed_mode = 1,
	middle_ButtonPressed_mode = 2,
	small_ButtonPressed_mode = 3
	};

boolean waterBox_readyOk = false; 
boolean recipeBox_readyOk = false; 
boolean steamBox_readyOk = false; 

int jori_Mode = nobutton_Normal_mode;

//const int  mux_effective_value = 120;
//--const int  mux_effective_value = 320;

const int  mux_effective_value = 60;
const int  ambient_sensor_effective_value = 60;

//const int led_toggle_Interval = 2; // 3.0 sec.
//int       led_toggle_Count = 0; 
//bool      led_Mode = false;

bool	  start_btn_pressed = false;

int   recipewater_Input_count = 0;


int	  recipewater_Level_minCount = 0;
int	  recipewater_Level_maxCount = 0;

int	  wwater_Level_minCount = 0;
int	  wwater_Level_maxCount = 0;
*/



