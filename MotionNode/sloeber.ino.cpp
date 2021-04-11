#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2021-04-11 16:31:40

#include "Arduino.h"
#include <RFM69.h>
#include <RFM69_ATC.h>
#include <SPI.h>
#include <LowPower.h>
#include <SPIFlash.h>
#include <EEPROM.h>

void setup () ;
void motionIRQ() ;
void loop () ;
bool sendMessage(const char* buff) ;
float getBatteryVoltage() ;
uint32_t incrementSessionKey() ;

#include "MotionNode.ino"


#endif
