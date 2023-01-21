#include <Adafruit_SPIDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <SPI.h>
#include <SPI.h>
//#define __MY_DEBUG_LOOP__
//#define __MY_DEBUG_LOG__

#if defined (__AVR__)
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#elif defined (__arm__)

#else
#error "Processor architecture is not supported."
#endif
#include "Arduino.h"


//#define  __MY_DEBUG_TRACE__
//#define  __MY_WATCHDOG__
//#define  __SLEEP_MODE_PWR_DOWN__

#include <EEPROM.h>
#include "steamgen1.h"

// Lib....
//#include "max6675.h"
#include "Adafruit_MAX31865.h"


#define READ_TEMP_MAXCOUNT		10	// 10번 온도를 읽어서,  가장 높은값과 낮은값을 뺀 평균값을 적용


/*******  PCT 센서 온도별 저항치....<<메일내용 참조 자료 ****
-55 C = 500 ohm
-50 C = 525 ohm
-40 C = 577 ohm
-30 C = 632 ohm
-20 C = 691 ohm
-10 C = 754 ohm

  0 C = 820 ohm ----->Nominal....

+10 C = 889 ohm
+20 C = 962 ohm
+25 C = 1000 ohm   --> ????
+30 C = 1039 ohm
+40 C = 1118 ohm
+50 C = 1202 ohm
+60 C = 1288 ohm
+70 C = 1379 ohm
+80 C = 1472 ohm
+90 C = 1569 ohm
+100 C = 1670 ohm
****************************************/
// 참고사항 : 2021.12.01 hhlee mail 내용
// 실측 데이터
// 60도 == 1190 옴
// 70도 == 1230 옴
// 90도 == 1300 옴
// 140도 == 1500 옴


float g_flReaddTempArray[11] = {0};
int	  g_ireadCount = 0;	
bool  g_breadTempFault = false;

int	  g_iDelayCount = 0;	
int serial_print = 0;
int temp_tmp = 0;

// Read Tempe. Sensor ( Water1 / Recipe2 Box )....
float read_TemperatureSensor()
{
	
	uint8_t tmp = 0;
	float Votmp = 0;
	
	float CurrentTemperature;
	while(tmp++ < 10) Votmp += analogRead(A0);
		
	float Vo = Votmp / 10.0f; // temper
	
	float R1 = 1000;
	float logR2, R2, T;
	float c1 = 26.07040123e-03, c2 = -41.74434822e-04, c3 = 185.6570531e-07;
	R2 = R1 * (1024.0 / (float)Vo - 1.0);
	logR2 = log(R2);
	T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));	
	CurrentTemperature = T - 273.15;
	
	if(Vo != 0) g_breadTempFault = false;
	else
	{
		Serial.println("Not detect sensor!!!!!");
		g_breadTempFault = true;
	}
	
	if(serial_print++ > 10)
	{
		serial_print = 0;
		Serial.print("analogRead(A0) : ");
		Serial.println(Vo);
	
		Serial.print("CurrentTemperatrue : ");
		Serial.println(CurrentTemperature);
		
		int tmp = analogRead(pinLed_Dimmer);
		Serial.print("Led_Dimmer : ");
		Serial.println(tmp);
	}

  return CurrentTemperature;    
}


void Led_OnandOff(int ledNum , bool onn) // 점등 - On...
{
	analogWrite(pinPower_LED, brightNessVal);		
	switch(ledNum)
	{
		case 1:		// Run Led
			onn ? brightNessVal_Run = brightNessVal : brightNessVal_Run = 0;
			break;
		case 2:		// PTc Led
			//digitalWrite(pinPtc_LED, led_Mode2 ? HIGH : LOW);
			onn ? brightNessVal_Ptc = brightNessVal : brightNessVal_Ptc = 0;
			break;
	}
	if (brightNessVal_Run == 0) 
		analogWrite(pinRun_LED, 0);		
	else
		analogWrite(pinRun_LED, brightNessVal);	
	if (brightNessVal_Ptc == 0)				
		analogWrite(pinPtc_LED, 0);	
	else
		analogWrite(pinPtc_LED, brightNessVal);				
}


void Error_PtcOn(bool  isFault_mode)
{
	Led_OnandOff(2, true);   // Ptc Led On....
	Led_OnandOff(1, false);   // Run Led Off....
	digitalWrite(pinShip_Heater, LOW);  //  Heater Off ...
}

/********* 8 MHz*************/
void setup() {
  
	// put your setup code here, to run once:
	Serial.begin(115200);
	delay(10);
	Serial.println("Setup Entering...!!!");	
	
	// LED pin define(0 pin)
	pinMode(pinPower_LED, OUTPUT);       
	pinMode(pinRun_LED, OUTPUT); 
	pinMode(pinPtc_LED, OUTPUT);   
	
	//pinMode(pinTemps_DRDY_T, INPUT); // 온도 센서
	pinMode(pinShip_Heater, OUTPUT);   
  
	pinMode(pinHeatTemp_Var, INPUT);
	pinMode(pinLed_Dimmer, INPUT);      
  
	//--Origin thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
	//thermo.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
  
 	Serial.print("Setup -- Dimmer Value/Bright : ");
 	int idimmerValue = analogRead(pinLed_Dimmer);
 	Serial.print(idimmerValue);
	brightNessVal = idimmerValue / 4; // 0 ~ 1023  => 0 ~ 255로 변경	
 	Serial.print(" / ");
 	Serial.println(brightNessVal);
	
	//brightNessVal = 255;	// No ref Dimmer val.....
	temp_tmp = 100;
	Led_OnandOff(0, true);   // Power Led On....
		
	// eeprom password write........
	uint8_t index_data = 0;	
	security_Verified = false;
#ifdef	EEPROM_PROTECT_MODE
#ifdef  EEPROM_WRITE_MODE
	for(index_data = 0; index_data <= epppw_length; index_data++)  {
		EEPROM.write(index_data,0);// Clear memory
		delay(10);
		EEPROM.write(index_data, epp_pdw[index_data]);// Save data at memory
		delay(10);
	}	
#else
	// eeprom password read & verify....	
	char rd_data = 0;
	for(index_data = 0; index_data <= epppw_length; index_data++)  {
		//epp_pdw_read[index_data] = EEPROM.read(index_data);
		rd_data = (char)EEPROM.read(index_data);
		if (rd_data != epp_pdw[index_data])
		{
			security_Verified = false;
			//break;		
		}
		else
			security_Verified = true;
		Serial.print("Index=");		Serial.print(index_data); Serial.print("("); Serial.print(epp_pdw[index_data]); Serial.print(")");
		Serial.print(",");
		Serial.print(" Value=");		Serial.println(rd_data); 
		//Serial.write(rd_data); 
		delay(50);
	}  
	if (security_Verified == true)
	{
		digitalWrite(pinShip_Heater, HIGH);  // Power on => Always  Heater.Onnnn..  	
		Serial.println("Heater ON - Setup.... init Power On !!");			
		Led_OnandOff(2, true);   // Ptc Led On.... 		
	}	
#endif	
#endif			
	//powerOn_firstMode = true;
	Serial.println("Setup End...!!!");	
}

void loop()
{
	
		// Led Bright Control....???????????????
		//Serial.print("Dimmer Value: ");
		int idimmerValue = analogRead(pinLed_Dimmer);
	/*	Serial.println(idimmerValue);*/
		//----Real 모드..... 
		brightNessVal = idimmerValue / 4; // 0 ~ 1023  => 0 ~ 255로 변경
	
		Led_OnandOff(3, true);   // Only for Pwm ....
		if(++temp_tmp > 100)
		{
			temp_tmp = 0;
			g_breadTempFault = false;
			float rd_temp = read_TemperatureSensor();
			if (g_breadTempFault == true)
			{
				g_ireadCount = 0;
		
				Error_PtcOn(true);
			}
			else
			{
				if (g_ireadCount++ >= READ_TEMP_MAXCOUNT)
				{
						g_ireadCount = 0;
						rd_temp = read_TemperatureSensor(); // 10번을 읽어서,  평균값을 적용해서 동작 시킨다
						
						//////////////////////////
						if (rd_temp > HEATER_MAX_TEMP_LEVEL)
						{
							// 2021.11.29  90도(1300ohm) 이상일 경우, ptc on....
							analogWrite(pinPtc_LED, brightNessVal);
							brightNessVal_Ptc = brightNessVal;
							digitalWrite(pinShip_Heater, LOW);  //  Heater Off ...(ptc on일 경우,  무조건 Heater off....)
							brightNessVal_Run = 0;
							Error_PtcOn(true);
							Serial.println("Temp. MAX!!");
						}
						else
						{
							if (rd_temp < HEATER_MIN_TEMP_LEVEL) // -- min linit ye :
							{
								// 2021.12.03  -35도(200ohm) 미만일 경우, ptc on....
								analogWrite(pinPtc_LED, brightNessVal);
								brightNessVal_Ptc = brightNessVal;
								digitalWrite(pinShip_Heater, LOW);  //  Heater Off ...(ptc on일 경우,  무조건 Heater off....)
								brightNessVal_Run = 0;
								Error_PtcOn(true);
								Serial.println("Temp. MIN!!");
							}
							else
							{
								//--211203 Led_OnandOff(2, false);   // Ptc Led Off....
								brightNessVal_Ptc = 0;   // Ptc Led Off....
								if (rd_temp < HEATER_ONN_TEMP_LEVEL)
								{
									//heater_On = true;
									digitalWrite(pinShip_Heater, HIGH);  //  Heater On ...
									Led_OnandOff(1, true);   // Run Led On....
									brightNessVal_Run = brightNessVal;
									//Serial.println("Heater ON-N!!");
								}
								else
								{
									if (rd_temp >= HEATER_OFF_TEMP_LEVEL)
									{
										Led_OnandOff(1, false);   // Run Led Off....
										brightNessVal_Run = 0;
										digitalWrite(pinShip_Heater, LOW);  //  Heater Off ...
										//Serial.println("Heater OF-f!!");
									}
								}
							}
						}
					}
			  }
		  }
	
	
}

void function_AlltestMode(int mode)
{
}

/* Motor PWM Ref...
 *  
 *  Fyi, the logic to get the TLE5205-2 to move is as follows:
OPEN
- IN1 = LOW
- IN2 = LOW

CLOSE
- IN1 = LOW
- IN2 = HIGH

BRAKE (slow close)
- IN1 = HIGH
- IN2 = LOW

OFF
- IN1 = HIGH
- IN2 = HIGH
 *  
 *  
 *  
void Motor_Pwm()
{  
 for (Speed = 150; Speed >= 1; Speed--)
 {
   analogWrite(Motortreiber1_IN1_Pin, Speed);
   digitalWrite(Motortreiber1_IN2_Pin, HIGH);  // Foward
   delay(200);
 }
 delay(1000);
 analogWrite(Motortreiber1_IN1_Pin, 255);   // Stop
 digitalWrite(Motortreiber1_IN2_Pin, HIGH);
 delay(2000);
 
 for (Speed = 150; Speed >= 1; Speed--)
 {
   digitalWrite(Motortreiber1_IN1_Pin, HIGH);
   analogWrite(Motortreiber1_IN2_Pin, Speed);  // Backward
   delay(200);
 }
 delay(1000);
 digitalWrite(Motortreiber1_IN1_Pin, HIGH);
 analogWrite(Motortreiber1_IN2_Pin, 255);    // Stop
 delay(2000);
}*/


/*********************
If you’re running an ATmega328 without an external oscillator, you might want to use the crystal pins (XTAL1 & XTAL2).

These pins are also known as PB6 and PB7. They are not supported by the Arduino IDE but you can control them with direct port manipulation on port B.

void setup() {
	// set as outputs
	DDRB |= (1 << DDB6) | (1 << DDB7);
}

void loop() {
	// set PB6 high
	PORTB |= (1 << PORTB6);
	delay(1000);
	// set PB6 low
	PORTB &= ~(1 << PORTB6);
	delay(1000);
	
	// both high
	PORTB |= (1 << PORTB6) | (1 << PORTB7);
	delay(1000);
	// both low
	PORTB &= ~((1 << PORTB6) | (1 << PORTB7));
	delay(1000);
}
*********************/