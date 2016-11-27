#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>      //comes with Arduino IDE (www.arduino.cc)
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SparkFunBME280.h>
#include <EEPROM.h>

//***********************************************************************************************************
// Sample sketch that achieves the lowest power on a Moteino of ~6.5uA
// Everything is put to sleep including the MCU, the radio (if any) and the FlashMem chip
//**** SETTINGS *********************************************************************************************
#define WITH_RFM69              //comment this line out if you don't have a RFM69 on your Moteino
//#define WITH_SPIFLASH           //comment this line out if you don't have the FLASH-MEM chip on your Moteino
#define IS_RFM69HW
//***********************************************************************************************************
//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
// Node ID will be set in EEPROM
byte NODEID;    //unique for each node on same network
// will be read from eeprom
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define IS_RFM69HW    //uncomment only for RFM69HW! Remove/comment if you have RFM69W!

// will be read from EEPROM
char ENCRYPTKEY[16]; //exactly the same 16 characters/bytes on all nodes!

#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -75
//*********************************************************************************************
#define ACK_TIME      30  // max # of ms to wait for an ack
#define LED     9  // Moteinos have LEDs on D9

#define SERIAL_EN             //comment this out when deploying to an installed Mote to save a few KB of sketch size
#define SERIAL_BAUD    115200

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#if defined(WITH_SPIFLASH)
    SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
#endif

char sendBuf[50];

// BME280 draws too much current only enabled when needed with D7
#define BME280_PWR_PIN 7
// default is 0x77 but I get one with 0x76
#define BME280_I2C_ADDRESS 0x76
BME280 mySensor;


#define BATT_MONITOR A7
// we use 4.7M and 1M 1% resistors = 1/(1000/(4700+1000)) see also here
// https://lowpowerlab.com/forum/low-power-techniques/battery-monitorsensing-ratio-calculation-on-motionmoteweathershield/
#define BATT_FORMULA(reading) reading * 0.0032 * 5.7
#define CYCLE_INTERVAL_SEC  600

void setup () {

	// Node ID and Encryption Key in EEPROM
	EEPROM.begin();
	NODEID = EEPROM.read(0);
	for(int i=0;i<16;i++)
	{
		ENCRYPTKEY[i] = EEPROM.read(i+1);
	}
	EEPROM.end();

#ifdef WITH_RFM69
  radio.sleep();
#endif

#ifdef WITH_SPIFLASH
  if (flash.initialize())
    flash.sleep();
#endif

  for (uint8_t i=0; i<=A5; i++)
  {
#ifdef WITH_RFM69
    if (i == RF69_SPI_CS) continue;
#endif
#ifdef WITH_SPIFLASH
    if (i == FLASH_SS) continue;
#endif
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
}

#define power_twi_enable()      (PRR &= (uint8_t)~(1 << PRTWI))
#define power_twi_disable()     (PRR |= (uint8_t)(1 << PRTWI))

bool battFlag = true;

void loop ()
{
  enableBME280(true);
  delay(20);
  setupBME280();
  float temp = mySensor.readTempC();
  float rh = mySensor.readFloatHumidity();
  mySensor.writeRegister(BME280_CTRL_MEAS_REG, 0x00); //sleep the BME280
  enableBME280(false);
  char str_temp[6];
  char str_rh[6];
  // 4 is mininum width, 2 is precision; float value is copied onto str_temp
  dtostrf(temp, 2, 1, str_temp);
  dtostrf(rh, 2, 0, str_rh);
  sprintf(sendBuf, "{\"i\":%d,\"s\":1,\"t\":%s,\"h\":%s}",NODEID,str_temp, str_rh);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.encrypt(ENCRYPTKEY);
  #ifdef IS_RFM69HW
   	  radio.setHighPower(true); //uncomment only for RFM69HW!
  #endif
  sendMessage(sendBuf);

  // report Battery state every other
  if(battFlag)
  {
	  char BATstr[10];
	  float volts = getBatteryVoltage();
	  dtostrf(volts, 3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
	  sprintf(sendBuf, "{\"i\":%d,\"s\":2,\"bat\":%s}",NODEID,BATstr);
	  sendMessage(sendBuf);
	  battFlag = false;
  }
  else
  {
	  battFlag = true;
  }
  radio.sleep();
  //sleep TWI
  TWCR &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWEA));
  // saves ~ 160uAmps, Wire keeps the internal pullups running!!!
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);
  power_twi_disable();
  int counter = 0;
  while(counter < CYCLE_INTERVAL_SEC)
  {
	  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	  counter+=8;
  }
  power_twi_enable();
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}

bool sendMessage(const char* buff)
{
	bool ret = false;
	byte sendLen = strlen(buff);

	if (radio.sendWithRetry(GATEWAYID, buff, sendLen))
	{
	    ret = true;
	}
	return ret;
}

void enableBME280(const bool enable)
{
	if(enable)
	{
		pinMode(BME280_PWR_PIN, OUTPUT);
		digitalWrite(BME280_PWR_PIN, HIGH);
	}
	else
	{
		pinMode(BME280_PWR_PIN, OUTPUT);
		digitalWrite(BME280_PWR_PIN, LOW);
	}
}

void setupBME280()
{
		//***Driver settings********************************//
	//commInterface can be I2C_MODE or SPI_MODE
	//specify chipSelectPin using arduino pin names
	//specify I2C address.  Can be 0x77(default) or 0x76

	//For I2C, enable the following and disable the SPI section
	mySensor.settings.commInterface = I2C_MODE;
	mySensor.settings.I2CAddress = BME280_I2C_ADDRESS;

	//For SPI enable the following and dissable the I2C section
	//mySensor.settings.commInterface = SPI_MODE;
	//mySensor.settings.chipSelectPin = 10;


	//***Operation settings*****************************//

	//renMode can be:
	//  0, Sleep mode
	//  1 or 2, Forced mode
	//  3, Normal mode
	mySensor.settings.runMode = 3; //Normal mode

	//tStandby can be:
	//  0, 0.5ms
	//  1, 62.5ms
	//  2, 125ms
	//  3, 250ms
	//  4, 500ms
	//  5, 1000ms
	//  6, 10ms
	//  7, 20ms
	mySensor.settings.tStandby = 0;

	//filter can be off or number of FIR coefficients to use:
	//  0, filter off
	//  1, coefficients = 2
	//  2, coefficients = 4
	//  3, coefficients = 8
	//  4, coefficients = 16
	mySensor.settings.filter = 0;

	//tempOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.tempOverSample = 1;

	//pressOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.pressOverSample = 1;

	//humidOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.humidOverSample = 1;
	//Calling .begin() causes the settings to be loaded
	delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
	mySensor.begin();
	delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
}

float getBatteryVoltage()
{
	float batteryVolts = 0.0;
	unsigned long readings=0;
	for (byte i=0; i<10; i++) //take 10 samples, and average
	{
		unsigned int sensorValue = analogRead(BATT_MONITOR);
		delay(1);
		readings += sensorValue;
	}
	batteryVolts = BATT_FORMULA(readings / 10.0);
	return batteryVolts;
}
