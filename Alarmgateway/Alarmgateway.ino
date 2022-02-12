// Sample RFM69 receiver/gateway sketch, with ACK and optional encryption, and Automatic Transmission Control
// Passes through any wireless received messages to the serial port & responds to ACKs
// It also looks for an onboard FLASH chip, if present
// RFM69 library and sample code by Felix Rusu - http://LowPowerLab.com/contact
// Copyright Felix Rusu (2015)

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>      //comes with Arduino IDE (www.arduino.cc)
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash
#include <EEPROM.h>
#include <ArduinoJson.h>


//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
// read from EEPROM later
byte NODEID = 1;    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
// read from EEPROM later
char ENCRYPTKEY[16];
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//*********************************************************************************************

#define SERIAL_BAUD   115200

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

// Broadcast burst message with sessionKey
#define REMOTE_ADDRESS 255

// the secure session key
unsigned long sessionKey = 0;

void setup() {
   // Node ID and Encryption Key in EEPROM
   EEPROM.begin();
   NODEID = EEPROM.read(0);
   for(int i=0;i<16;i++)
   {
	 ENCRYPTKEY[i] = EEPROM.read(i+1);
   }
   EEPROM.end();

  createSessionKey();
  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  radio.spyMode(true);
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  if (flash.initialize())
  {
    Serial.print("SPI Flash Init OK. Unique MAC = [");
    flash.readUniqueId();
    for (byte i=0;i<8;i++)
    {
      Serial.print(flash.UNIQUEID[i], HEX);
      if (i!=8) Serial.print(':');
    }
    Serial.println(']');
  }
  else
    Serial.println("SPI Flash MEM not found (is chip soldered?)...");

#ifdef ENABLE_ATC
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)");
#endif
}

byte ackCount=0;
uint32_t packetCount = 0;


struct NodeMsg{
	byte cmd;
	unsigned long secKey;
} nodeMsg;

char sendBuf[50];

void loop() {

    //process any serial input
  if (Serial.available() > 0)
  {
    String cmd = Serial.readString();
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(cmd.c_str());
    // Test if parsing succeeds.
    if (!root.success()) {
        Serial.println("Invalid JSON Command");
        Serial.println(cmd);


        //return;
    }
    //{"c":"wake","id":"255"}
    //{"c":"msg","id":"100","m":"0"}
    //{"c":"msg","id":"100","m":"1"}
    String theCmd = root["c"];
    // wake sleeping nodes 255 for all
    if(theCmd.equals("wake"))
    {
    	uint8_t id = root["id"];
    	Serial.println(id);
    	// increment session key;
    	sessionKey++;
    	// send Key to Gateway
    	sprintf(sendBuf, "{\"i\":\"%d\",\"sessionKey\":\"%lu\"}",NODEID,sessionKey);
    	Serial.println(sendBuf);
    	// wake all low power sleep nodes
    	nodeMsg.cmd = 'w';
    	nodeMsg.secKey = sessionKey;
        radio.listenModeSendBurst(id, (char*)&nodeMsg, sizeof(NodeMsg));
        delay(250);
        radio.initialize(FREQUENCY,NODEID,NETWORKID);
        #ifdef IS_RFM69HW
          radio.setHighPower(); //only for RFM69HW!
        #endif
        radio.encrypt(ENCRYPTKEY);
        radio.promiscuous(promiscuousMode);
    }
    // all other messages over here, when nodes awake
    else if(theCmd.equals("msg"))
    {
    	uint8_t id = root["id"];
    	String msg = root["m"];
    	nodeMsg.cmd = msg.charAt(0);
    	nodeMsg.secKey = sessionKey;
    	if (radio.sendWithRetry(id, (char*)&nodeMsg, sizeof(NodeMsg), 3))
    	{
    		sprintf(sendBuf, "{\"trans\":\"ok\",\"node\":\"%d\"}",id);
    	}
    	else
    	{
    		sprintf(sendBuf, "{\"trans\":\"fail\",\"node\":\"%d\"}",id);
    	}
    	Serial.println(sendBuf);
    }
  }

  if (radio.receiveDone())
  {
    Serial.print("#[");
    Serial.print(++packetCount);
    Serial.print(']');
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    if (promiscuousMode)
    {
      Serial.print("to [");Serial.print(radio.TARGETID, DEC);Serial.print("] ");
    }
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");

    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.print(" - ACK sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      if (ackCount++%3==0)
      {
        Serial.print(" Pinging node ");
        Serial.print(theNodeID);
        Serial.print(" - ACK...");
        delay(3); //need this when sending right after reception .. ?
        if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
          Serial.print("ok!");
        else Serial.print("nothing");
      }
    }
    Serial.println();
    Blink(LED,3);
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
// on each reboot increment EEPROM value by 1 of the higher 2 bytes of our 4 bytes session key
void createSessionKey()
{
	sessionKey=1024;
	/*
	sessionKey=0;
	union
	{
		byte b[2];
		unsigned int high;
	} data;
	for(int i = 0; i < 2; i++)
	{
		data.b[i] = EEPROM.read(i);
	}
    data.high = data.high + 1;
    for(int i = 0; i < 2; i++)
    {
    	EEPROM.write(i,data.b[i]);
    }
    sessionKey |= data.high;
    sessionKey <<= 16;
    Serial.println(sessionKey);*/
}



