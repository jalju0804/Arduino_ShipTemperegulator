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

//int pinTemps_DRDY_T = A4;		// PC4
//int pinTempsCS_1T =   A3;		// PC3
//int pinTemps_SCK_T =  A2;		// PC2
//int pinTemps_SO_T =   A1;		// PC1
//int pinTemps_SI_T =   A0;		// PC0
// Use software SPI: CS, DI, DO, CLK
//--Origin Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
Adafruit_MAX31865 thermo = Adafruit_MAX31865(pinTempsCS_1T, pinTemps_SI_T, pinTemps_SO_T, pinTemps_SCK_T);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
// Use PT1000.....
#define RREF      4300.0
//#define RREF      4000.0

// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
// Use PT1000.....
#define RNOMINAL  1000.0 // --> +25-degrees-C resistance
#define RNOMINAL_SHIP  820.0 // --> +0-degrees-C resistance

// 1도 -> 3.9 도 차이 남.
#define RNOM_DIFF	(RNOMINAL - RNOMINAL_SHIP) / 3.9	// 180 / 3.9

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

//MAX6675 thermocouple (pinTemps_SCK, pinTempsCS_1, pinTemps_SO);
//MAX6675 thermocouple2(pinTemps_SCK, pinTempsCS_2, pinTemps_SO);


//const int wdt_timeout_count_MAX = 101; // w d t - time_out : 800 sec....(12 minutes)
//int wdt_timeout_count = 0; 
//volatile int iwdt_timeout_mode = 0;

//int mux_readValue = 1023;

//--200911 int g_iDelayTime = 1500;
//--210926 int g_iDelayTime = 2000;
//int g_iDelayTime = 1000;

//int g_iswPressed = -1;
//byte g_byteMuxValue = 0;

//bool g_fStartButtonOnePressed = false;
//bool g_fRecipeButtonOnePressed = false;
//bool g_fSteamButtonOnePressed = false;
float g_flReaddTempArray[11] = {0};
int	  g_ireadCount = 0;	
bool  g_breadTempFault = false;

int	  g_iDelayCount = 0;	

 ISR (PCINT1_vect)
{
#if defined (__MY_WATCHDOG__)	
	iwdt_timeout_mode = 0;
#endif
}

ISR(WDT_vect)
{
}

void watchdogOn() {
#if defined (__MY_WATCHDOG__)
	wdt_reset();

	// Clear the reset flag, the WDRF bit (bit 3) of MCUSR.
	MCUSR = MCUSR & B11110111;
	
	// Set the WDCE bit (bit 4) and the WDE bit (bit 3)
	// of WDTCSR. The WDCE bit must be set in order to
	// change WDE or the watchdog prescalers. Setting the
	// WDCE bit will allow updtaes to the prescalers and
	// WDE for 4 clock cycles then it will be reset by
	// hardware.
	WDTCSR = WDTCSR | B00011000;

	// Set the watchdog timeout prescaler value to 1024 K
	// which will yeild a time-out interval of about 8.0 s.
	WDTCSR = B00100001;

	// Enable the watchdog timer interupt.
	WDTCSR = WDTCSR | B01000000;
	//MCUSR = MCUSR & B11110111;
#endif	
}

ISR (TIMER1_COMPA_vect)
{
	TCNT1 = 0;			// manually reset Timer Count reset......
}
/*
int readMux_eachSensor (const byte which)
{
  // select correct MUX channel
  digitalWrite (pin_muxAddress_A, (which & 1) ? HIGH : LOW);  // low-order bit
  digitalWrite (pin_muxAddress_B, (which & 2) ? HIGH : LOW);
  digitalWrite (pin_muxAddress_C, (which & 4) ? HIGH : LOW);  // high-order bit
  // now read the sensor
  return analogRead (pin_muxOnCH);
}  // end of readSensor

byte readMux_AllChannel()
{
    // show all 8 sensor readings
  byte muxValue = 0; // 0x3ff - All released.....	
  
  //--no txd digitalWrite(pin_muxCE, LOW);    
  mux_readValue = 1023;
  for (byte i = 0; i < 8; i++)
    {
      Serial.print ("Mux Sensor(ch) ");
      Serial.print (i);
      Serial.print (" reads: ");
	  mux_readValue = readMux_eachSensor (i);
      Serial.println (mux_readValue);	
	    
	  if (mux_readValue < mux_effective_value)
			muxValue = muxValue | (B00000001 << i);
    }
	return muxValue;
}

byte readMux_SwChannel()
{
  //--no txd digitalWrite(pin_muxCE, LOW);  
    // show switch 3 sensor readings
  mux_readValue = 1023;
  byte swValue = 0; // 0x3ff - All released.....
  for (byte i = 0; i < 8; i++)
    {
		if ((i != 4) && (i != 6)) // Exclude 4, 6 : Water Level Sensor......
		{
#if defined (__MY_DEBUG_LOG__)
		  Serial.print ("Mux Sensor(switch-ch) ");
		  Serial.print (i);
		  Serial.print (" reads: ");
		  mux_readValue = readMux_eachSensor (i);
		  Serial.println (mux_readValue);     ///// ????????      
#else
		  mux_readValue = readMux_eachSensor (i);
#endif
		  if (mux_readValue < mux_effective_value)
		  {

			  // B0000 0000
			  // 0-1 : Start switch			  // 1-2 : Water switch
			  // 2-4 : Steam switch			  // 3-8 : Large switch
			  // 5-32 : Small switch		  // 7-128 : Medium switch
			  swValue = swValue | (B00000001 << i);			  
		  }
		}
    }
    return swValue;
} */

// Read Tempe. Sensor ( Water1 / Recipe2 Box )....
/*float read_TemperatureSensor(const byte which)
{
  float tempe = -1;
  return tempe;
    
  if (which == 1)
  {
      // Water Box....
#if defined (__MY_DEBUG_LOG__)      
      Serial.print("C 1= ");
      tempe = thermocouple.readCelsius();
      Serial.println(tempe);
#else
      tempe = thermocouple.readCelsius();
#endif      
  }
  else
  {
      // Recipe Box....
#if defined (__MY_DEBUG_LOG__)            
      Serial.print("C 2= ");
      tempe = thermocouple2.readCelsius();
      Serial.println(tempe);
#else
      tempe = thermocouple2.readCelsius();
#endif            
  } 
}*/
/*
int read_AmbientSensor(const byte pinNum)
{
  int ambientValue;
  //int ambientValue;  
#if defined (__MY_DEBUG_LOG__NO)      
  Serial.print("Ambient Sensor(pin-");  
  Serial.print(pinNum);      Serial.print("): ");  
#endif
  ambientValue = analogRead(pinNum);
#if defined (__MY_DEBUG_LOG__NO)      
  Serial.println(ambientValue); 
#endif
  if (ambientValue <= ambient_sensor_effective_value)
  {  
	  Serial.print("Ambient Sensor(pin-");
	  Serial.print(pinNum);      Serial.print("): ");
	  Serial.println(ambientValue); 
  }
  return ambientValue;  
}

void  SolValve34_Control(int valvNum, int ishigh)
{
	if (valvNum == 3) // Sol-Valve 3
	{
		if (ishigh == HIGH)
			// set PB6 high
			PORTB |= (1 << PORTB6);
		else
			// set PB6 low
			PORTB &= ~(1 << PORTB6);
	}
	else if (valvNum == 4)	// Sol-Valve 4
	{
		if (ishigh == HIGH)
			// set PB7 high
			PORTB |= (1 << PORTB7);
		else
			// set PB7 low
			PORTB &= ~(1 << PORTB7);
	}
}

void Delay_And_Check(int delay_Time, int chk_mode)
{
}

// Water Box Control.....
void control_WaterBox(int addMax_value)
{
	//--no txd digitalWrite(pin_muxCE, LOW);	  
	// check Water Level Sensor.....
	#if defined (__MY_DEBUG_LOG__)
	Serial.print("WaLevel-4= ");
	int waterLevel = readMux_eachSensor (4);
	Serial.println (waterLevel);
	#else
	int waterLevel = readMux_eachSensor (4);
	#endif
	if (waterLevel >= mux_effective_value)
	{
		waterBox_readyOk = false;

	    wwater_Level_maxCount = 0;
		if (wwater_Level_minCount++ > 6)
		{
			wwater_Level_minCount = 8; // no meaning...
			// 물 투입....
			digitalWrite(pinSolvalv_1, HIGH); // Open Sol. Valve....
		}		
		
		//20200915 When Level low, check tempe.
		int ipressValue = analogRead(pinPress_sens);
		if (ipressValue >= (steamBox_Press_Margin_MAX + addMax_value))
		{
			// Heater 멈춤.....
			digitalWrite(pinHeater_1, LOW);  // Off Heater....
		}
	}
	else
	{
		wwater_Level_minCount = 0;
		// 물 투입 정지....
		digitalWrite(pinSolvalv_1, LOW); // Close Sol. Valve....
					
		//if (wwater_Level_maxCount++ > 3)
		//{
			//wwater_Level_maxCount = 5; // no meaning...
			//
			//// 물 투입 정지....
			//digitalWrite(pinSolvalv_1, LOW); // Close Sol. Valve....
		//}
		// check Pressure  Sensor.....
		#if defined (__MY_DEBUG_LOG__)
		Serial.print("Pressure Sensor Value: ");
		int ipressValue = analogRead(pinPress_sens);
		Serial.println(ipressValue);
		#else
		int ipressValue = analogRead(pinPress_sens);
		#endif
		if (ipressValue < steamBox_Press_Margin_MIN)
		{
			waterBox_readyOk = false;
			
			// Heater 가열.....
			digitalWrite(pinHeater_1, HIGH);  // On Heater....
			
			Serial.println("Water Low Pressure(Heater On.!!");
		}
		else
		{
			if (ipressValue > (steamBox_Press_Margin_MAX + addMax_value))
			{
				// Heater 멈춤.....
				digitalWrite(pinHeater_1, LOW);  // Off Heater....
				Serial.println("Water Heater Off !!");
			}
			// Water Box OK.(Ready)....
			waterBox_readyOk = true;
			Serial.println("Water Box Ready Ok!!");
		}
	}
}

// Steam Box Control.....
void control_SteamBox(int addMax_value)
{
	// Check Temp. Sensor......
	float waterTempe = read_TemperatureSensor(1);
	
	//////////// for test only
	read_TemperatureSensor(2);	
	
	if (waterTempe <= steamBox_Tempe_MIN)
	{
		steamBox_readyOk = false;
		// Heater 가열.....
		digitalWrite(pinHeater_2, HIGH);  // On Heater....
	}
	else
	{	
		if (waterTempe >= (steamBox_Tempe_MAX + addMax_value))
		{
			// Heater 멈춤.....
			digitalWrite(pinHeater_2, LOW);  // Off Heater....
		}
		steamBox_readyOk = true;
		Serial.println("Steam Box Ready Ok!!");
	}
}


// Recipe Box Control.....
void control_RecipeBox(int addMax_value)
{
}

void motor_move(const int step)
{
}

// Large,middle,small 3 button 조리 시작.....>>>go! go! go!
void lms_start_ButtonPress(int joriMode)
{
}

// 조리 시작.....>>>go! go! go!
void start_ButtonPress(int joriMode)
{
}

// Recipe-Water Input Button.....>>>
void water_ButtonPress()
{
}

// Steam Input Button.....>>>
void steam_ButtonPress()
{
} */

// Read Tempe. Sensor ( Water1 / Recipe2 Box )....
float read_TemperatureSensor(byte which)
{
  float tempe = -1;
  
  uint16_t rtd = thermo.readRTD();

  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); 
  //float diffNorminal = RNOMINAL + RNOM_DIFF;
  tempe = thermo.temperature(RNOMINAL, RREF);
  Serial.println(tempe);

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
	  Serial.print("Fault 0x"); Serial.println(fault, HEX);
	  
	  g_breadTempFault = true;
	  
	  if (fault & MAX31865_FAULT_HIGHTHRESH) {
		  Serial.println("RTD High Threshold");
	  }
	  if (fault & MAX31865_FAULT_LOWTHRESH) {
		  Serial.println("RTD Low Threshold");
	  }
	  if (fault & MAX31865_FAULT_REFINLOW) {
		  Serial.println("REFIN- > 0.85 x Bias");
	  }
	  if (fault & MAX31865_FAULT_REFINHIGH) {
		  Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
	  }
	  if (fault & MAX31865_FAULT_RTDINLOW) {
		  Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
	  }
	  if (fault & MAX31865_FAULT_OVUV) {
		  Serial.println("Under/Over voltage");
	  }
	  thermo.clearFault();
  }  
  return tempe;    
}

void Led_Blinking(int ledNum) // 점멸 - Blinking...
{
	//switch(ledNum)
	//{
		//case 0:		// PO Led
			////digitalWrite(pinPower_LED, led_Mode0 ? HIGH : LOW);
			//led_Mode0 ? analogWrite(pinPower_LED, brightNessVal) : analogWrite(pinPower_LED, 0);
			//led_Mode0 = !led_Mode0;		
			//break;
		//case 1:		// Run Led
			////digitalWrite(pinRun_LED, led_Mode1 ? HIGH : LOW);
			//led_Mode1 ? analogWrite(pinRun_LED, brightNessVal) : analogWrite(pinRun_LED, 0);
			//led_Mode1 = !led_Mode1;		
			//break;
		//case 2:		// PTc Led			
			////digitalWrite(pinPtc_LED, led_Mode2 ? HIGH : LOW);
			//led_Mode2 ? analogWrite(pinPtc_LED, brightNessVal) : analogWrite(pinPtc_LED, 0);				
			//led_Mode2 = !led_Mode2;		
			//break;
	//}
}

void Led_OnandOff(int ledNum , bool onn) // 점등 - On...
{
	analogWrite(pinPower_LED, brightNessVal);		
	switch(ledNum)
	{
		//case 0:		// PO Led
			////digitalWrite(pinPower_LED, led_Mode0 ? HIGH : LOW);
			//onn ? brightNessVal_Pow = brightNessVal : brightNessVal_Pow = 0;
			//analogWrite(pinPower_LED, brightNessVal_Pow);
			////onn ? analogWrite(pinPower_LED, brightNessVal) : analogWrite(pinPower_LED, 0);
			//break;
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

/*void findMaxTempValue()
{
	float max = g_flReaddTempArray[0];
	for (int i = 0; i < READ_TEMP_MAXCOUNT; i++)
	{
		if (max < g_flReaddTempArray[i])
		{
			max = g_flReaddTempArray[i];
		}
	} 
}

void findMinTempValue()
{
	float min = g_flReaddTempArray[0];
	for (int i = 0; i < READ_TEMP_MAXCOUNT; i++)
	{
		if (min > g_flReaddTempArray[i])
		{
			min = g_flReaddTempArray[i];
		}
	}	
}*/

float calcAverageTempValue()
{
//	float max = g_flReaddTempArray[0];
//	float min = g_flReaddTempArray[0];
	float averageVal = g_flReaddTempArray[0];	
	float totalVal = 0;

	for (int i = 0; i < READ_TEMP_MAXCOUNT; i++)
	{
		//if (max < g_flReaddTempArray[i])
		//{
			//max = g_flReaddTempArray[i];
		//}
		//if (min > g_flReaddTempArray[i])
		//{
			//min = g_flReaddTempArray[i];
		//}
		totalVal += g_flReaddTempArray[i];
	}	
	//Serial.print("max/min Temp. Val = "); Serial.print(max); Serial.println(min);
	averageVal = totalVal / READ_TEMP_MAXCOUNT;
	Serial.print("aVerage Temp. Val = "); Serial.println(averageVal);	
	
	return averageVal;
}

void Error_PtcOn(bool  isFault_mode, float fValue)
{
	//powerOn_firstMode = false;
	Led_OnandOff(2, true);   // Ptc Led On....
	Led_OnandOff(1, false);   // Run Led Off....
	digitalWrite(pinShip_Heater, LOW);  //  Heater Off ...
	if (isFault_mode)
		Serial.print("Fault Occurred(Sensor Read Error)...(");	
	else
		Serial.print("Heater Of-f-Out Range...(");	
	Serial.print(fValue);	
	Serial.println(")");
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
  
	pinMode(pinShip_Heater, OUTPUT);   
  
	pinMode(pinHeatTemp_Var, INPUT);
	pinMode(pinLed_Dimmer, INPUT);      
  
	//--Origin thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
	thermo.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
  
	Serial.print("Setup -- Dimmer Value/Bright : ");
	int idimmerValue = analogRead(pinLed_Dimmer);
	Serial.print(idimmerValue);
	brightNessVal = idimmerValue / 4; // 0 ~ 1023  => 0 ~ 255로 변경	
	Serial.print(" / ");
	Serial.println(brightNessVal);
	
	//brightNessVal = 255;	// No ref Dimmer val.....
	
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
	
	float temp_tmp = read_TemperatureSensor(0);
	
	if(temp_tmp >= 42.0 && temp_tmp < 54.0 )EEPROM.write(23,1); //40 ~ 45 // 22 ~ 34
	else EEPROM.write(23,0);
}

void loop()
{
	if(EEPROM.read(23) == 0)
	{
		float temp_tmp = read_TemperatureSensor(0);
		Error_PtcOn(true, temp_tmp);
		if(temp_tmp >= 42.0 && temp_tmp < 54.0 ) EEPROM.write(23,1);
	}
	else
	{
		// Led Bright Control....???????????????
		//Serial.print("Dimmer Value: ");
		int idimmerValue = analogRead(pinLed_Dimmer);
		//Serial.println(idimmerValue);
		//----Real 모드..... 
		brightNessVal = idimmerValue / 4; // 0 ~ 1023  => 0 ~ 255로 변경
	
		Led_OnandOff(3, true);   // Only for Pwm ....
			
		if (g_iDelayCount++ >= 100)
		{
			g_iDelayCount = 0;
				
			Serial.print("Dimmer Value: ");
			Serial.println(idimmerValue);
		
			// Heater On-Off Temp. value Control..........
			Serial.print("Temp. Variable Offset Value: ");
			int itempVarValue = analogRead(pinHeatTemp_Var);
			Serial.println(itempVarValue);
			//tempOffSetVal = itempVarValue;
	
			//float rd_temp = read_TemperatureSensor(0);
			g_breadTempFault = false;
			//g_flReaddTempArray[g_ireadCount++] = read_TemperatureSensor(0);
			float frdd_temp = read_TemperatureSensor(0);
			if (g_breadTempFault == true)
			{
				g_ireadCount = 0;
		
				Error_PtcOn(true, frdd_temp);
				//powerOn_firstMode = false;
				//Led_OnandOff(2, true);   // Ptc Led On....
				//Led_OnandOff(1, false);   // Run Led Off....
				//digitalWrite(pinShip_Heater, LOW);  //  Heater Off ...
				//Serial.print("Fault Occurred(Sensor Read Error)...(");	Serial.print(frdd_temp);	Serial.println(")");
			}
			else
				g_flReaddTempArray[g_ireadCount++] = frdd_temp;
	
			if (g_ireadCount >= READ_TEMP_MAXCOUNT)
			{
				Serial.print("Temp. Read Count = "); Serial.println(g_ireadCount);
				g_ireadCount = 0;
				float rd_temp = calcAverageTempValue(); // 10번을 읽어서,  평균값을 적용해서 동작 시킨다
		
				//////////////////////////
				if (rd_temp > HEATER_MAX_TEMP_LEVEL)
				{
					// 2021.11.29  90도(1300ohm) 이상일 경우, ptc on....
					//--analogWrite(pinPtc_LED, brightNessVal);
					brightNessVal_Ptc = brightNessVal;
					digitalWrite(pinShip_Heater, LOW);  //  Heater Off ...(ptc on일 경우,  무조건 Heater off....)
					brightNessVal_Run = 0;
					Serial.println("Temp. MAX!!");
				}
				else
				{
					if (rd_temp < HEATER_MIN_TEMP_LEVEL) // -- min linit ye : 
					{
						// 2021.12.03  -35도(200ohm) 미만일 경우, ptc on....
						brightNessVal_Ptc = brightNessVal;
						digitalWrite(pinShip_Heater, LOW);  //  Heater Off ...(ptc on일 경우,  무조건 Heater off....)
						brightNessVal_Run = 0;
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
							//--211203 Led_OnandOff(1, true);   // Run Led On....
							brightNessVal_Run = brightNessVal;
							Serial.println("Heater ON-N!!");
						}
						else
						{
							if (rd_temp >= HEATER_OFF_TEMP_LEVEL)
							{
								//--211203 Led_OnandOff(1, false);   // Run Led Off....
								brightNessVal_Run = 0;
								digitalWrite(pinShip_Heater, LOW);  //  Heater Off ...
								Serial.println("Heater OF-f!!");
							}
						}
				   }
				}

			}		
		}

		//delay(g_iDelayTime);  // RealTime..(Urgent!!).......    
		delay(10);	
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
 
void dipswitchRead()
{
  /*
  int tmpDipValue = 0;

  // Read DIP Switch....
    Serial.print("DIP 1 Value:");
  //tmpDipValue = analogRead(pinDIP_1);
  //tmpDipValue = (digitalRead(pinDIP_1) == 1)?0:1;
  tmpDipValue = (digitalRead(pinDIP_1) == 1) ? 0 : 1;
  dipswitchValue = tmpDipValue;
  Serial.print(tmpDipValue);
    Serial.print(" \t");

    Serial.print("DIP 2 Value:");
    //tmpDipValue = analogRead(pinDIP_2);
  tmpDipValue = (digitalRead(pinDIP_2) == 1) ? 0 : 1;
  dipswitchValue += (tmpDipValue << 1);
    Serial.println(tmpDipValue);

  Serial.print("DIP 3 Value:");
  //tmpDipValue = analogRead(pinDIP_3);
  tmpDipValue = (digitalRead(pinDIP_3) == 1) ? 0 : 1;
  dipswitchValue += (tmpDipValue << 2);
  Serial.print(tmpDipValue);
  Serial.print(" \t");

    Serial.print("DIP 4 Value:");
    //tmpDipValue = analogRead(pinDIP_4);
  tmpDipValue = (digitalRead(pinDIP_4) == 1) ? 0 : 1;
  dipswitchValue += (tmpDipValue << 3);
    Serial.println(tmpDipValue);

    Serial.print("DIP Switch Value:");
    Serial.println(dipswitchValue); */
}


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