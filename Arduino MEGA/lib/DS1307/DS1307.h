/*
 * DS1307.h
 *
 *  Created on: 22 févr. 2015
 *      Author: Fabien
 */

#ifndef DS1307_H_
#define DS1307_H_
#include "Arduino.h"
#include "stdint.h"
#define DS1307_I2C_ADDRESS 0x68  // Define the I²C address of the Tiny RTC module

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val);
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val);

typedef struct StructRTC
{
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day_of_week;
    uint8_t day_of_month;
    uint8_t month;
    uint8_t year;
} StruRTC;

class DS1307
{
public:

  // typedef struct StructRTC
  // {
  //     uint8_t second;
  //     uint8_t minute;
  //     uint8_t hour;
  //     uint8_t day_of_week;
  //     uint8_t day_of_month;
  //     uint8_t month;
  //     uint8_t year;
  // } StruRTC;

  // RTC(void);                        //!< Constructeur
  void init(uint8_t address);      //!< Init the RTC
  void setDateDs1307(StructRTC);   //!< Set the current time, change the second&minute&hour to the right time
  void getDateDs1307(StruRTC *RTC);            //!< Gets the date and time from the ds1307 and prints result
  uint8_t getAddress();
  //StruRTC			TimeRTC;
private:
  uint8_t m_address;


};
#endif /* DS1307_H_ */
