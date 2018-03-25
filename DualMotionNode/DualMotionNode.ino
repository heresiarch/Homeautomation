#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>      //comes with Arduino IDE (www.arduino.cc)
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <EEPROM.h>

#include <PinChangeInterrupt.h>



#define WITH_RFM69              //comment this line out if you don't have a RFM69 on your Moteino
//#define WITH_SPIFLASH           //comment this line out if you don't have the FLASH-MEM chip on your Moteino
#define IS_RFM69HW

#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
//*********************************************************************************************
#define ACK_TIME      30  // max # of ms to wait for an ack

#define SERIAL_EN             //comment this out when deploying to an installed Mote to save a few KB of sketch size
#define SERIAL_BAUD    55700

#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -75

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

#define BATT_INTERVAL 3600000 // read and report battery voltage every this many ms (approx)
// each Message has additional monotonic sequence key
#define BATT_MONITOR A7
// we use 4.7M and 1M 1% resistors = 1/(1000/(4700+1000)) see also here
// https://lowpowerlab.com/forum/low-power-techniques/battery-monitorsensing-ratio-calculation-on-motionmoteweathershield/
#define BATT_FORMULA(reading) reading * 0.0032 * 5.7

#define MOTION_PIN  4
#define TROUBLE_PIN 5

#define DUPLICATE_INTERVAL 1000 //avoid duplicates

// Node ID will be set in EEPROM
// will be read from eeprom
byte NODEID;    //unique for each node on same network
// will be read from EEPROM
char ENCRYPTKEY[16]; //exactly the same 16 characters/bytes on all nodes!

uint32_t sessionKey = 0;
char sendBuf[50];


volatile uint8_t motion = 0;
//The setup function is called once at startup of the sketch
void setup()
{
	// Node ID and Encryption Key in EEPROM
	EEPROM.begin();
	NODEID = EEPROM.read(0);
	for(int i=0;i<16;i++)
	{
		ENCRYPTKEY[i] = EEPROM.read(i+1);
	}
	EEPROM.end();
	#ifdef WITH_RFM69
		radio.initialize(FREQUENCY,NODEID,NETWORKID);
		radio.encrypt(ENCRYPTKEY);
		#ifdef IS_RFM69HW
			radio.setHighPower(true); //uncomment only for RFM69HW!
			//https://lowpowerlab.com/forum/rf-range-antennas-rfm69-library/is-there-a-table-showing-tx-power-(in-dbm)-for-each-setpowerlevel(-)-value/
		#endif
		radio.setPowerLevel(16);
		radio.sleep();
	#endif
	for (uint8_t i=0; i<=A5; i++)
	{
		#ifdef WITH_RFM69
			if (i == RF69_SPI_CS) continue;
		#endif
		pinMode(i, OUTPUT);
		digitalWrite(i, LOW);
	}
	// enable PINs as Input and internal Pullups

	pinMode(MOTION_PIN, INPUT);
	digitalWrite(MOTION_PIN, HIGH);       // turn on pullup resistors

	pinMode(TROUBLE_PIN, INPUT);
	digitalWrite(TROUBLE_PIN, HIGH);       // turn on pullup resistors
	// enable LED
	pinMode(LED, OUTPUT);
}

uint32_t time=0, now=0, MLO=0, BLO=0;
// The loop function is called in an endless loop
void loop()
{
	Serial.begin(57600);
	now = millis();
	uint8_t motion = detectMotion();
	if(motion > 0 && (time-MLO > DUPLICATE_INTERVAL))
	{
		Serial.println	("Motion or Trouble");
		Serial.println(motion);
		incrementSessionKey();
		sprintf(sendBuf, "{\"i\":%d,\"s\":3,\"m\":%d,\"sk\":%lu}",NODEID,motion,sessionKey);
		radio.initialize(FREQUENCY,NODEID,NETWORKID);
		radio.encrypt(ENCRYPTKEY);
		#ifdef IS_RFM69HW
			radio.setHighPower(true); //uncomment only for RFM69HW!
			//https://lowpowerlab.com/forum/rf-range-antennas-rfm69-library/is-there-a-table-showing-tx-power-(in-dbm)-for-each-setpowerlevel(-)-value/
		#endif
		radio.setPowerLevel(16);
		sendMessage(sendBuf);
		radio.sleep();
		MLO = time; //save timestamp of event
		motion = 0;
	}
	else if (time-BLO > BATT_INTERVAL)
	{
		char BATstr[10];
		float volts = getBatteryVoltage();
		dtostrf(volts, 3,2, BATstr); //update the BATStr which gets sent every BATT_CYCLES or along with the MOTION message
		sprintf(sendBuf, "{\"i\":%d,\"s\":2,\"bat\":%s}",NODEID,BATstr);
		radio.initialize(FREQUENCY,NODEID,NETWORKID);
	    radio.encrypt(ENCRYPTKEY);
	    #ifdef IS_RFM69HW
    	  radio.setHighPower(true); //uncomment only for RFM69HW!
       	  //https://lowpowerlab.com/forum/rf-range-antennas-rfm69-library/is-there-a-table-showing-tx-power-(in-dbm)-for-each-setpowerlevel(-)-value/
	    #endif
	    radio.setPowerLevel(16);
	    sendMessage(sendBuf);
	    radio.sleep();
	    BLO = time;
	    Serial.println("Battery");
	    Serial.println(volts);
	}
	time = time + 1000 + millis() - now;
	radio.sleep();
	Serial.flush();
	LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
	Serial.flush();
}

byte detectMotion()
{
	byte pinState = B00000000;
	// Motion = 1 Trouble = 2 both = 3
	bitWrite(pinState, 0, !digitalRead(MOTION_PIN));
	bitWrite(pinState, 1, !digitalRead(TROUBLE_PIN));
	return pinState;
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

void blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS/2);
  digitalWrite(PIN,LOW);
  delay(DELAY_MS/2);
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
	sessionKey+=1;
	return sessionKey;
}

