//
//  DS1631.cpp
//  Functions to write and read to the DS1631
//  temperature sensor over I2C (two-wire)
//  interface. Requires the Wire library.
//
//  Created by Luke Miller on 12/30/12.
//
//	Modified by Charles Galant on 06/26/15
//
//
//  Released into the public domain.
//  http://github.com/millerlp/DS1631

/*
 * DS131.cpp
 *
 *  Created on: 22 févr. 2015
 *      Author: Fabien
 */

#include "DS1631.h"
#include <Wire.h>
// The constructor.
DS1631::DS1631(){
	MSByte=0;
	LSByte=0;
    _address=DS1631_ADDR;  // Ground Addr 72 for DS1631
}

//*****************************************************
// Public Methods

// Tell the DS1631 to stop making temperature readings.
void DS1631::stopConversion(){
    Wire.beginTransmission(_address);
    Wire.write(0x22); // Stop conversion
    Wire.endTransmission();
}

// Tell the DS1631 to begin making temperature readings
// If 1SHOT = True, only one temperature reading is taken.
// If 1SHOT = False, the DS1631 is in continuous mode and
// will keep making temperature readings until told to
// stop.
void DS1631::startConversion(){
    Wire.beginTransmission(_address);
    Wire.write(0x51); // Start conversion
    Wire.endTransmission();

}

// Write to the configuration registers
// You'll primarily use this to set the
// conversion resolution (which affects
// the time taken for a reading) and whether
// the DS1631 takes continuous readings or
// goes into a low-power idle state after
// taking 1 reading (1-shot mode).
// 13   = 12 bit, 1-shot mode
// 9    = 11 bit, 1-shot mode
// 5    = 10 bit, 1-shot mode
// 1    = 9  bit, 1-shot mode

// 12   = 12 bit, continuous mode
// 8    = 11 bit, continuous mode
// 4    = 10 bit, continuous mode
// 0    = 9  bit, continuous mode
void DS1631::writeConfig(uint8_t _data){
    stopConversion();
    Wire.beginTransmission(_address);
    Wire.write(0xAC);        // AC : Access Config
    Wire.write(_data);
    Wire.endTransmission();
    startConversion();
}

// Read the configuration registers
uint8_t DS1631::readConfig(){
    uint8_t _data;
    Wire.beginTransmission(_address);
    Wire.write(0xAC);        // AC : Access Config
    Wire.endTransmission();
    Wire.requestFrom(_address, 1);
    if(Wire.available()) {
        _data = Wire.read();
    }
    return _data;
}

// Request a temperature reading from the DS1631
// The high and low bytes are saved
void DS1631::readT(){
    Wire.beginTransmission(_address);
    Wire.write(0xAA); // AA : Request Temperature
    Wire.endTransmission();
    Wire.requestFrom(_address,2); // READ 2 bytes
    Wire.available(); // 1st byte
    MSByte = Wire.read(); // read a byte
    Wire.available(); // 2nd byte
    LSByte = Wire.read(); // read a byte
}

// Read the temperature and return a floating point
// temperature value, in degrees Celsius
float DS1631::readTempF(){
    int T_dec;
    double T;
    readT();
    // T° processing. Shift the LSByte right 4 positions
    // The resulting binary value, converted to base-10
    // and multiplied by 0.0625, is the decimal part of
    // the temperature.
    LSByte = LSByte>>4;

    // The MSByte (8 bits), converted to base-10,
    // represents the whole number portion of the
    // temperature. When the left-most bit is 1,
    // this represents the special case of a
    // negative temperature value, so you must
    // subtract off 256 to get the whole number
    // value.
	// Negative temperature fix contributed by Jürgen Thierry
	if(MSByte>=0x80)
    { //if sign bit is set
        float negMSByte = MSByte - 256;
        T = (float)negMSByte + (float)LSByte*0.0625;
        return T;
    }
    // Combine the whole number and fractional
    // parts of the temperature
    T = (float) MSByte + (float) LSByte*0.0625;

    return T;
}

// Read the temperature in 1-shot mode, with a
// wait built in so that the temperature
// has time to update. This isn't necessary when
// using the continuous measuring mode.
float DS1631::readTempOneShot(){
    long lastMillis = millis();
    float T;
    // Send command to start taking temperature reading
    startConversion();
    // Now wait for the configuration register's
    // most significant bit to be returned as 1,
    // indicating that the reading is done. A 12-bit
    // reading can take up to 750 milliseconds
    while ( !conversionDone() ) {
        // Wait a little while before checking
        // the configuration register again
        long elapsed = 0;
		while (elapsed < 50) {
			elapsed = millis() - lastMillis;
		}
		if(elapsed > 1000){
			// Abort polling if conversion takes more
			// than 1000ms. Instead return integer minimum.
			return -32768;
		}
    }
    // Once the temperature conversion is done,
    // read the value from the DS1631
    T = readTempF();
    // After reading the temperature, put the
    // DS1631 back into low-power idle state.
    stopConversion();
    return T;
}

// Read the temperature in 1-shot mode and
// return an integer composed of the MSByte
// and LSByte returned from the DS1631, stored
// in two's complement format. The integer
// would be the most compact way of storing the
// temperature data, since it can be stored in
// two bytes.
uint16_t DS1631::readTempOneShotInt(){
    long lastMillis = millis();
    uint16_t T;
    startConversion();
    while ( !conversionDone() ){
		// Wait a little while before checking
        // the configuration register again
        long elapsed = 0;
		while (elapsed < 50) {
			elapsed = millis() - lastMillis;
		}
		if(elapsed > 1000){
			// Abort polling if conversion takes more
			// than 1000ms. Instead return integer minimum.
			return -32768;
		}
    }
    readT(); // Get MSByte and LSByte

    T = word(MSByte,LSByte);
    return T;
    // If you take the integer and split it
    // back into its highByte and lowByte using
    // the highByte() and lowByte() functions of
    // Arduino, you can quickly calculate the
    // temperature using the rules outlined above
    // in the readTempF() function.
}

// Read the temperature and return a double value
// If you divide the returned double value by 16,
// you get the temperature value in °C
int32_t DS1631::readTempD(){ // 1/16°C = 12 Bit accuracy 0.0625°C
    int T_dec;
    int32_t T;
    readT();

    T=((int32_t)MSByte << 8) + LSByte;
    // T° processing
    if(T >= 0x8000){   // If sign bit is set, then temp is negative
        T = T - 0xFFFF;
    }
    T = T >>4;
    return T;
}

// Check if the temperature reading (Conversion) is
// finished. 12-bit readings take up to 750ms.
bool DS1631::conversionDone(){  // if Conversion Done = Boolean = True
    uint8_t _data = readConfig();
    // This OR's the _data value with 127 (b01111111)
    // If the most significant bit is 1, the result
    // is 255 (b11111111)
    if ((_data | 127)==255)
        return true;
    else
        return false;
}


//void initDS131(){
//
//	 // Opening I2C interface as Master
//	  Wire.begin();
//	  // Opening the serial interface
////	  mySerial.begin(115200);
//
//	  // First, stop the conversion (if configured in continuous
//	  // conversion mode) to access the configuration register.
//	  Wire.beginTransmission(DS1631_ADDR);
//	  Wire.write(0x22); // Stop conversion
//	  Wire.endTransmission();
//
//	  //Write the TH parameter (2 bytes) (25 degree)
//	    Wire.beginTransmission(DS1631_ADDR);
//	    Wire.write(0xA1); // Access TH
//	    Wire.write(0x19);
//	    Wire.write(0x00);
//	    Wire.endTransmission();
//
//	  //Write the TL parameter (2 bytes) (24 degree)
//	    Wire.beginTransmission(DS1631_ADDR);
//	    Wire.write(0xA2); // Access TL
//	    Wire.write(0x18);
//	    Wire.write(0x00);
//	    Wire.endTransmission();
//
//	  // Configuration of DS1631 (see Datasheet)
//	  Wire.beginTransmission(DS1631_ADDR);
//	  Wire.write(0xAC); // Access Configuration
//	  Wire.write(DS1631_CFG);
//	  Wire.endTransmission();
//
//	#if DS1631_CFG_1SHOT == 0
//	  // If continuous mode is set, start continuous conversion
//	  Wire.beginTransmission(DS1631_ADDR);
//	  Wire.send(0x51);
//	  Wire.endTransmission();
//	  delay(750); // delay to be sure that the first value is correct
//	#endif
//}
//
//float readTempOne(boolean oneShotMode){
//  float temp = 0.0;
//
//  if (oneShotMode)
//  {
//    // Temperature conversion request
//    Wire.beginTransmission(DS1631_ADDR);
//    Wire.write(0x51);
//    Wire.endTransmission();
//
//    delay(750); // delay for 12 bits precision
//
//    // Now we wait the end of the conversion
//    boolean tempAvailable = false;
//    while (!tempAvailable)
//    {
//      // See if the conversion is finished
//      // in the configuration byte
//      Wire.beginTransmission(DS1631_ADDR);
//      Wire.write(0xAC);
//      Wire.endTransmission();
//      Wire.requestFrom(DS1631_ADDR,1);
//      Wire.available();
//      tempAvailable = Wire.read() & DS1631_DONE_MASK;
//      delay(10);
//    }
//  }
//
//  // read result
//  Wire.beginTransmission(DS1631_ADDR);
//  Wire.write(0xAA); // read last conversion
//  Wire.endTransmission();
//  Wire.requestFrom(DS1631_ADDR,2);
//  Wire.available();
//  int tempMS = (int) Wire.read();
//  Wire.available();
//  int tempLS = (int) Wire.read();
//
//  tempLS >>= 4; // last 4 bits are always 0
//
//  if (tempMS & 0x80) // if the signed bit = 1
//    tempMS -= 256;
//
//  // Float conversion
//  temp = (float) (tempMS + tempLS*0.0625);
//
//  return temp;
//}
