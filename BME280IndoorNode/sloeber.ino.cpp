#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2018-02-09 11:50:40

#include "Arduino.h"
#include <RFM69.h>
#include <RFM69_ATC.h>
#include <SPI.h>
#include <LowPower.h>
#include <SPIFlash.h>
#include <SparkFunBME280.h>
#include <EEPROM.h>

void setup () ;
void loop () ;
bool sendMessage(const char* buff) ;
void enableBME280(const bool enable) ;
void setupBME280() ;
float getBatteryVoltage() ;

#include "BME280IndoorNode.ino"


#endif
