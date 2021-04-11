#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>      //comes with Arduino IDE (www.arduino.cc)
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
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

// Motion Logic
#define MOTION_PIN     3  // D3
#define MOTION_IRQ 1 // hardware interrupt 1 (D3) - where motion sensors OUTput is connected, this will generate an interrupt every time there is MOTION
volatile boolean motionDetected=false;
#define DUPLICATE_INTERVAL 90000 //avoid duplicates

#define BATT_INTERVAL 3600000 // read and report battery voltage every this many ms (approx)

// each Message has additional monotonic session Key
uint32_t sessionKey = 0;
char sendBuf[50];
#define BATT_MONITOR A7
// we use 4.7M and 1M 1% resistors = 1/(1000/(4700+1000)) see also here
// https://lowpowerlab.com/forum/low-power-techniques/battery-monitorsensing-ratio-calculation-on-motionmoteweathershield/
#define BATT_FORMULA(reading) reading * 0.0032 * 5.7

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

  pinMode(MOTION_PIN, INPUT);
  attachInterrupt(MOTION_IRQ, motionIRQ, RISING);
  pinMode(LED, OUTPUT);
}


void motionIRQ()
{
  motionDetected=true;
}

bool battFlag = true;

uint16_t batteryReportCycles=0;
uint32_t time=0, now=0, MLO=0, BLO=0;
byte motionRecentlyCycles=0;

void loop ()
{
	//Serial.begin(SERIAL_BAUD);
	now = millis();
	if (motionDetected && (time-MLO > DUPLICATE_INTERVAL))
	{
		radio.initialize(FREQUENCY,NODEID,NETWORKID);
		radio.encrypt(ENCRYPTKEY);
		#ifdef IS_RFM69HW
			radio.setHighPower(true); //uncomment only for RFM69HW!
		    //https://lowpowerlab.com/forum/rf-range-antennas-rfm69-library/is-there-a-table-showing-tx-power-(in-dbm)-for-each-setpowerlevel(-)-value/
		 	radio.setPowerLevel(20);
		#endif
		MLO = time; //save timestamp of event
	    incrementSessionKey();
	    sprintf(sendBuf, "{\"i\":%d,\"s\":3,\"m\":1,\"sk\":%lu}",NODEID,sessionKey);
	    radio.setPowerLevel(16);
	    sendMessage(sendBuf);
	    radio.sleep();
	}
	else if (time-BLO > BATT_INTERVAL)
	{
		BLO = time;
	    batteryReportCycles=0;
	    radio.initialize(FREQUENCY,NODEID,NETWORKID);
	    radio.encrypt(ENCRYPTKEY);
	    #ifdef IS_RFM69HW
	    	  radio.setHighPower(true); //uncomment only for RFM69HW!
	       	  //https://lowpowerlab.com/forum/rf-range-antennas-rfm69-library/is-there-a-table-showing-tx-power-(in-dbm)-for-each-setpowerlevel(-)-value/
	       	  radio.setPowerLevel(16);
	    #endif
	    char BATstr[10];
	    float volts = getBatteryVoltage();
	    dtostrf(volts, 3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
	    sprintf(sendBuf, "{\"i\":%d,\"s\":2,\"bat\":%s}",NODEID,BATstr);
	    sendMessage(sendBuf);
	    radio.sleep();
	}
	  //while motion recently happened sleep for small slots of time to better approximate last motion event
	  //this helps with debouncing a "MOTION" event more accurately for sensors that fire the IRQ very rapidly (ie panasonic sensors)
	  if (motionDetected ||motionRecentlyCycles>0)
	  {
	    if (motionDetected) motionRecentlyCycles=8;
	    else motionRecentlyCycles--;
	    motionDetected=false; //do NOT move this after the SLEEP line below or motion will never be detected
	    time = time + 250 + millis()-now;
	    radio.sleep();
	    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
	  }
	  else
	  {
	    time = time + 8000 + millis()-now;
	    radio.sleep();
	    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	  }
	batteryReportCycles++;
	Serial.flush();
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


uint32_t incrementSessionKey()
{
	union
	{
		byte b[4];
		uint16_t low;
		uint32_t key;
	} data;

	uint16_t lowPart = sessionKey & 0xffff;
	// increase EEPROM after reset or if lower part overflows
	if(sessionKey == 0 || lowPart == 0xffff )
	{
		data.key = 0;
		data.b[0] = EEPROM.read(17);
		data.b[1] = EEPROM.read(18);
		data.low++;
		EEPROM.put(17,data.low);
		sessionKey = 0;
		sessionKey |= data.key;
		sessionKey <<= 16;
	}
	else
	{
		sessionKey+=1;
	}
	return sessionKey;
}


