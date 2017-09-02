/*
 * DS1307.cpp
 *
 *  Created on: 22 févr. 2015
 *      Author: Fabien
 */

#include "DS1307.h"
#include "Misc.h"
#include "Wire.h"

  // RTC::RTC(void){
  //
  // }

  // Convert normal decimal numbers to binary coded decimal
  byte decToBcd(byte val)
  {
  	return ( (val/10*16) + (val%10) );
  }
  // Convert binary coded decimal to normal decimal numbers
  byte bcdToDec(byte val)
  {
  	return ( (val/16*10) + (val%16) );

  }

void DS1307::init(uint8_t address){
  m_address=address;
}

// Function to set the current time, change the second&minute&hour to the right time
void DS1307::setDateDs1307(StructRTC RTC){
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(decToBcd(0));
  Wire.write(decToBcd(RTC.second));    // 0 to bit 7 starts the clock
  Wire.write(decToBcd(RTC.minute));
  Wire.write(decToBcd(RTC.hour));      // If you want 12 hour am/pm you need to set
                                            // bit 6 (also need to change readDateDs1307)
  Wire.write(decToBcd(RTC.day_of_week));
  Wire.write(decToBcd(RTC.day_of_month));
  Wire.write(decToBcd(RTC.month));
  Wire.write(decToBcd(RTC.year));
  Wire.endTransmission();
}

// Function to gets the date and time from the ds1307 and prints result
void DS1307::getDateDs1307(StructRTC *RTC){
  // Reset the register pointer
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(decToBcd(0));
  Wire.endTransmission();
  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);
  RTC->second     = bcdToDec(Wire.read() & 0x7f);
  RTC->minute     = bcdToDec(Wire.read());
  RTC->hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
  RTC->day_of_week  = bcdToDec(Wire.read());
  RTC->day_of_month = bcdToDec(Wire.read());
  RTC->month      = bcdToDec(Wire.read());
  RTC->year       = bcdToDec(Wire.read());
}

uint8_t DS1307::getAddress(){
  return m_address;
}
