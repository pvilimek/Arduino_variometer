/*
Home made variometer code for Arduino
Feel free to use it as you wish.
(c) 2018 Petr Vilimek
email: spiderwolf@seznam.cz
*/

#include <Wire.h>
#include <MS5611.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>
#include <Adafruit_PCD8544.h>
#include <SimpleKalmanFilter.h>

// LCD PIN definition
#define RST 4
#define CE 5
#define DC 6
#define DIN 7
#define CLK 8

// LCD variable
Adafruit_PCD8544 display = Adafruit_PCD8544(RST, CE, DC, DIN, CLK);

// Kalman filter used for smoothing altitude values 
SimpleKalmanFilter pressureKalmanFilter(0.2, 0.2, 1.0);

/*
  MS5611 Barometric Pressure & Temperature Sensor. Output for MS5611_processing.pde
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/czujnik-cisnienia-i-temperatury-ms5611.html
  GIT: https://github.com/jarzebski/Arduino-MS5611
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
 */

// Barometer variable
MS5611 ms5611;

// Altitude 
double lastAlt = 0;
double newAlt = 0;
double maxAlt = 0;
double realAltitude = 0;
double estimated_altitude = 0;

// Pressure [Pa]
double realPressure = 0;

// Default sea level pressure [Pa]
double seaLevelPressure = 101325;

// Default LCD contrast
unsigned short lcdContrast = 40;

// Variometer
double vario = 0;

// Melody definition
unsigned int melodie = 0;
unsigned long noteDuration = 0;

// Delay
unsigned long pauseMillis = 0;

// The pushbutton pin definition
const int buttonMenuUp = 3;
const int buttonMenuDown = 12;
const int buttonUp = 10;    
const int buttonDown = 11;
const int buttonContrast = 2;

// Pushbutton states
int buttonUpState = 0;
int buttonDownState = 0;
int buttonMenuUpState = 0;
int buttonMenuDownState = 0;
int buttonContrastState = 0;

// Menu counter
int menuCounter = 1;

// Timer
unsigned long differenceTime = 0;
unsigned long varioPreviousTime = 0;
unsigned long previousTime = 0;
byte seconds;
byte minutes;
byte hours;

// Temperature
double realTemperature = 0;
double maxTemp = 0;
double minTemp = 50;
double newTemp = 0;

// Beep function
void beep(int melodie, unsigned long noteDuration, unsigned long pauseMillis){
tone(9, melodie, noteDuration);
delay(pauseMillis);
noTone(9);
}


// Voltage calculation
long readVcc() {
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
	#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
	ADMUX = _BV(MUX5) | _BV(MUX0);
	#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	ADMUX = _BV(MUX3) | _BV(MUX2);
	#else
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#endif

	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA,ADSC)); // measuring

	uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
	uint8_t high = ADCH; // unlocks both

	long result = (high<<8) | low;
	
	//scale_constant = internal1.1Ref * 1023 * 1000
	
	//internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)

	long scale_constant = 1111619L;
	//result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
	result = scale_constant / result;
	return result; // Vcc in millivolts
}


void setup() 
{
  // Initialize LCD 
  display.begin();
  display.clearDisplay();
  
  seconds = 00;
  minutes =  00;
  hours =  00;
  
  // Initialize MS5611 sensor
  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER
  while(!ms5611.begin(MS5611_ULTRA_HIGH_RES))
  {
    delay(500);
  }
}

void loop()
{

// LCD contrast tuning to adapt ambient light conditions
display.setContrast(lcdContrast);

// Timer conditions
if (millis() >= (previousTime))
{
	previousTime = previousTime + 1000;
	seconds = seconds +1;
	if (seconds == 60) {
		seconds = 0;
		minutes = minutes +1;
	}
	if (minutes == 60) {
		minutes = 0;
		hours = hours +1;
	}
}

// Button states
buttonUpState = digitalRead(buttonUp);
buttonDownState = digitalRead(buttonDown);
buttonMenuUpState = digitalRead(buttonMenuUp);
buttonMenuDownState = digitalRead(buttonMenuDown);
buttonContrastState = digitalRead(buttonContrast);

 // Reads true temperature & pressure (true = with compensation)
 realTemperature = ms5611.readTemperature(true);
 realPressure = ms5611.readPressure(true);

 // Calculates Max / Min temperature
 newTemp = realTemperature;
 if (newTemp > maxTemp)
 maxTemp = newTemp;
 if (newTemp < minTemp)
 minTemp = newTemp;
 
 // Calculates real altitude
 realAltitude = ms5611.getAltitude(realPressure, seaLevelPressure);
 
 // Calculates filtered altitude 
 estimated_altitude = pressureKalmanFilter.updateEstimate(realAltitude);
 
 // Calculates max altitude
 newAlt = realAltitude;
 if (newAlt > maxAlt)
 maxAlt = newAlt;

 // LCD contrast 
 if (buttonContrastState == HIGH) {
	 lcdContrast++;
 }

 if (lcdContrast == 51) {
 lcdContrast = 35;
 }

 // Calculates vario [m/s], there is multiplication const. base on formula: 1000 ms / avg. ms per one loop execution
  vario = (estimated_altitude - lastAlt)*(1000/differenceTime);

 // Beep tone frequency computed by function: y = 261.84*pow(M_E,0.3*x);
  melodie = 261.84*pow(M_E,0.3*vario);

 // Tone duration calculation: y = 1000/(x + 2)
  noteDuration = 1000/(vario+2);

 // Delay calculation: y = 1000/(x + 3)
  pauseMillis = 1000/(vario+3);

  // Vario no beep for values within the threshold 
  if ((vario <= 0.50) && (vario >= -2.00)) {
	  delay(pauseMillis);
	}

  // Vario beep above +0.5 m/s
   if (vario > 0.50) {
   beep(melodie, noteDuration, pauseMillis);
   }

  // Vario beep bellow -2.0 m/s
    if (vario < -2.00) {
    beep(165, 400, 100);
  }

  
  // Alt print  
  static char strAlt[10];
  dtostrf(estimated_altitude,5, 1, strAlt);
    
  // Seal Level print
  static char strSeaLevel[10];
  dtostrf(seaLevelPressure,6, 0, strSeaLevel);

    
  if (buttonUpState == HIGH){
	  seaLevelPressure = seaLevelPressure + 50;
  }

  if (buttonDownState == HIGH){
	  seaLevelPressure = seaLevelPressure - 50;
  }

  if (buttonMenuUpState == HIGH){
	  menuCounter++;
  }

  if (buttonMenuDownState == HIGH){
	  menuCounter--;
  }

  if (menuCounter == 9){
	  menuCounter = 1;
  }

  if (menuCounter == 0){
	  menuCounter = 8;
  }

  // Menu
  switch(menuCounter) {
	  case 1:
	  {
		  
		  // Current / Max altitude
		  static char strMaxAlt[10];
		  dtostrf(maxAlt,5, 1, strMaxAlt);
		  
		  display.drawLine(0, 0, display.width(), 0, BLACK);
		  display.drawLine(0, 8, display.width(), 8, BLACK);
		  display.setTextSize(1);
		  display.setTextColor(WHITE,BLACK);
		  display.setCursor(0,1);
		  display.println(" ALT         1");
		  display.setTextSize(2);
		  display.setTextColor(BLACK);
		  display.setCursor(0,17);
		  display.println(strAlt);
		  display.setTextSize(1);
		  display.setTextColor(BLACK);
		  display.setCursor(78,15);
		  display.println("m");
		  display.setCursor(0,40);
		  display.println("MAX:");
		  display.setCursor(30,40);
		  display.println(strMaxAlt);
		  display.setCursor(70,40);
		  display.println("m");
		  display.display();
		  display.clearDisplay();
	  }
	  break;
	  
	  case 2:
	  {
		  // Variometer
		  static char strVario[10];
		  dtostrf(vario,5, 2, strVario);

		  display.drawLine(0, 0, display.width(), 0, BLACK);
		  display.drawLine(0, 8, display.width(), 8, BLACK);
		  display.setTextSize(1);
		  display.setTextColor(WHITE,BLACK);
		  display.setCursor(0,1);
		  display.println(" VARIO       2");
		  display.setTextSize(2);
		  display.setTextColor(BLACK);
		  display.setCursor(0,17);
		  display.println(strVario);
		  display.setTextSize(1);
		  display.setTextColor(BLACK);
		  display.setCursor(65,16);
		  display.println("m/s");
		  display.setCursor(0,40);
		  display.println("ALT:");
		  display.setCursor(30,40);
		  display.println(strAlt);
		  display.setCursor(70,40);
		  display.println("m");
		  display.display();
		  display.clearDisplay();

	  }
	  break;
	  
	  case 3:
	  {
		  // Contrast
		  static char strContrast[10];
		  dtostrf(lcdContrast,5, 0, strContrast);

		  display.drawLine(0, 0, display.width(), 0, BLACK);
		  display.drawLine(0, 8, display.width(), 8, BLACK);
		  display.setTextSize(1);
		  display.setTextColor(WHITE,BLACK);
		  display.setCursor(0,1);
		  display.println(" CONTRAST    3");
		  display.setTextSize(2);
		  display.setTextColor(BLACK);
		  display.setCursor(0,17);
		  display.println(strContrast);
		  display.display();
		  display.clearDisplay();
	  }
	  break;
	  
	  case 4:
	  {
		  //Sea level adjustment
		  display.drawLine(0, 0, display.width(), 0, BLACK);
		  display.drawLine(0, 8, display.width(), 8, BLACK);
		  display.setTextSize(1);
		  display.setTextColor(WHITE,BLACK);
		  display.setCursor(0,1);
		  display.println(" SEA LEVEL   4");
		  display.setTextSize(1);
		  display.setTextColor(BLACK);
		  display.setCursor(0,15);
		  display.println("USE +- BUTTONS");
		  display.setCursor(15,25);
		  display.println(strSeaLevel);
		  display.setCursor(60,25);
		  display.println("Pa");
		  display.setCursor(0,40);
		  display.println("ALT:");
		  display.setCursor(30,40);
		  display.println(strAlt);
		  display.setCursor(70,40);
		  display.println("m");
		  display.display();
		  display.clearDisplay();
		  
 	  }
	  break;

	  case 5:
	  {
		  // Current pressure
		  static char strPressure[10];
		  dtostrf(realPressure,6, 0, strPressure);

		  display.drawLine(0, 0, display.width(), 0, BLACK);
		  display.drawLine(0, 8, display.width(), 8, BLACK);
		  display.setTextSize(1);
		  display.setTextColor(WHITE,BLACK);
		  display.setCursor(0,1);
		  display.println(" PRESSURE    5");
		  display.setTextColor(BLACK);
		  display.setCursor(10,20);
		  display.println(strPressure);
		  display.setCursor(65,20);
		  display.println("Pa");
		  display.setCursor(0,40);
		  display.println("ALT:");
		  display.setCursor(30,40);
		  display.println(strAlt);
		  display.setCursor(70,40);
		  display.println("m");
		  display.display();
		  display.clearDisplay();

	  }
	  break;
	  
	  case 6:
	  {   
	      // Temperature
		  static char strTemp[10];
		  dtostrf(realTemperature,5, 2, strTemp);	

		  // Max temperature print
		  static char strTempMax[10];
		  dtostrf(maxTemp,5, 2, strTempMax);

		  // Min temperature print
		  static char strTempMin[10];
		  dtostrf(minTemp,5, 2, strTempMin);

		  display.drawLine(0, 0, display.width(), 0, BLACK);
		  display.drawLine(0, 8, display.width(), 8, BLACK);
		  display.setTextSize(1);
		  display.setTextColor(WHITE,BLACK);
		  display.setCursor(0,1);
		  display.println(" TEMP        6");
		  display.setTextSize(2);
		  display.setTextColor(BLACK);
		  display.setCursor(0,12);
		  display.println(strTemp);
		  display.setTextSize(1);
		  display.setTextColor(BLACK);
		  display.setCursor(65,12);
		  display.println("C");
		  display.setCursor(0,30);
		  display.println("MAX:");
		  display.setCursor(25,30);
		  display.println(strTempMax);
		  display.setCursor(0,40);
		  display.println("MIN:");
		  display.setCursor(25,40);
		  display.println(strTempMin);
		  display.display();
		  display.clearDisplay();
	  }
	  break;
	  
	  case 7:
	  {
	      
		  // Voltage
		  float temp = (float)(readVcc());
		  float voltage = temp /1000;
		  
		  static char strVoltage[10];
	      dtostrf(voltage,3, 2, strVoltage);
		  
		  if (voltage > 3.00){
		  display.drawLine(0, 0, display.width(), 0, BLACK);
		  display.drawLine(0, 8, display.width(), 8, BLACK);
		  display.setTextSize(1);
		  display.setTextColor(WHITE,BLACK);
		  display.setCursor(0,1);
		  display.println(" VOLTAGE     7");
		  display.setTextSize(2);
		  display.setTextColor(BLACK);
		  display.setCursor(0,17);
		  display.println(strVoltage);
		  display.setTextSize(1);
		  display.setTextColor(BLACK);
		  display.setCursor(55,17);
		  display.println("V");
		  display.setCursor(0,40);
		  display.println("BATT: OK");
		  display.display();
		  display.clearDisplay();
		  }
		  else {
			  display.drawLine(0, 0, display.width(), 0, BLACK);
			  display.drawLine(0, 8, display.width(), 8, BLACK);
			  display.setTextSize(1);
			  display.setTextColor(WHITE,BLACK);
			  display.setCursor(0,1);
			  display.println(" VOLTAGE     8");
			  display.setTextSize(2);
			  display.setTextColor(BLACK);
			  display.setCursor(0,17);
			  display.println(strVoltage);
			  display.setTextSize(1);
			  display.setTextColor(BLACK);
			  display.setCursor(55,17);
			  display.println("V");
			  display.setCursor(0,40);
			  display.println("BATT:RECHARGE!");
			  display.display();
			  display.clearDisplay();
		  }
	  }
	  break;
	  
	  case 8:
	  {
	  // Timer
	  static char strCharHour[10];
	  static char strCharMin[10];
	  static char strCharSec[10];
	  static char zero = '0';
	  
	  dtostrf(hours,2,0,strCharHour);
	  dtostrf(minutes,2,0,strCharMin);
	  dtostrf(seconds,2,0,strCharSec);
	  
	  display.drawLine(0, 0, display.width(), 0, BLACK);
	  display.drawLine(0, 8, display.width(), 8, BLACK);
	  display.setTextSize(1);
	  display.setTextColor(WHITE,BLACK);
	  display.setCursor(0,1);
	  display.println(" TIMER       8");

	  // Hours
	  if (hours < 10) {
		  display.setTextSize(1);
		  display.setTextColor(BLACK);
		  display.setCursor(0,15);
		  display.println("HOUR:");
		  display.setCursor(40,15);
		  display.println(strCharHour);
		  display.setCursor(40,15);
		  display.println(zero);
	  }
	  else {
		  display.setTextSize(1);
		  display.setTextColor(BLACK);
		  display.setCursor(0,15);
		  display.println("HOUR:");
		  display.setCursor(40,15);
		  display.println(strCharHour);
	  }
	  
	  // Minutes
	  if (minutes < 10) {
		  display.setTextSize(1);
		  display.setTextColor(BLACK);
		  display.setCursor(0,25);
		  display.println("MIN:");
		  display.setCursor(40,25);
		  display.println(strCharMin);
		  display.setCursor(40,25);
		  display.println(zero);
	  }
	  else {
		  display.setTextSize(1);
		  display.setTextColor(BLACK);
		  display.setCursor(0,25);
		  display.println("MIN:");
		  display.setCursor(40,25);
		  display.println(strCharMin);
	  }
	  
	  //Seconds
	  if (seconds < 10) {
		  display.setTextSize(1);
		  display.setTextColor(BLACK);
		  display.setCursor(0,35);
		  display.println("SEC:");
		  display.setCursor(40,35);
		  display.println(strCharSec);
		  display.setCursor(40,35);
		  display.println(zero);
		  display.display();
		  display.clearDisplay();
	  }
	  else {
		  display.setTextSize(1);
		  display.setTextColor(BLACK);
		  display.setCursor(0,35);
		  display.println("SEC:");
		  display.setCursor(40,35);
		  display.println(strCharSec);
		  display.display();
		  display.clearDisplay(); 		  
	  }
	  break;
   }

} 

  // Set alt as last alt
  lastAlt = estimated_altitude;
  
  // Compute overall loop time
  differenceTime = millis() - varioPreviousTime;
  varioPreviousTime = differenceTime + varioPreviousTime;
}
