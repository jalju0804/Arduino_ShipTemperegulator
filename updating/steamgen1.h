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
#define  HEATER_MAX_TEMP_LEVEL		60.0f	// Registance : 1500 (90)  ohm ==> 90도 ==> 1300도 이상 => Ptc Led On....
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
#define  HEATER_MIN_TEMP_LEVEL		-80.0f	// Registance :  200 (-185도) ohm ???? // 2022 08 27 -55까지로 설정해놈

#define  HEATER_ONN_TEMP_LEVEL		44.0f// before 45
//#define  HEATER_OFF_TEMP_LEVEL		45.0+HEATER_TEMP_DIFFERENCE
#define  HEATER_OFF_TEMP_LEVEL		47.0f // before 51



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

