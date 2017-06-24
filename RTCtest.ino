// DS3231_Serial_Easy
// Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved
// web: http://www.RinkyDinkElectronics.com/
//
// A quick demo of how to use my DS3231-library to 
// quickly send time and date information over a serial link
//
// To use the hardware I2C (TWI) interface of the Arduino you must connect
// the pins as follows:
//
// Arduino Uno/2009:
// ----------------------
// DS3231:  SDA pin   -> Arduino Analog 4 or the dedicated SDA pin
//          SCL pin   -> Arduino Analog 5 or the dedicated SCL pin
//
//(1) + --- connects to RPi pin #1 3V3
//(2) D --- connects to RPi pin #3 SDA
//(3) C --- connects to RPi pin #5 SCL
//(4) N/C
//(5) - --- connects to RPi pin #9 GND
//
// The internal pull-up resistors will be activated when using the 
// hardware I2C interfaces.
//
// You can connect the DS3231 to any available pin but if you use any
// other than what is described above the library will fall back to
// a software-based, TWI-like protocol which will require exclusive access 
// to the pins used, and you will also have to use appropriate, external
// pull-up resistors on the data and clock signals.
//

#ifndef DS3231_h
#define DS3231_h

#if defined(__AVR__)
  #include "Arduino.h"
  #include "HW_AVR_defines.h"
#elif defined(__PIC32MX__)
  #include "WProgram.h"
  #include "HW_PIC32_defines.h"
#elif defined(__arm__)
  #include "Arduino.h"
  #include "HW_ARM_defines.h"
#endif

#define DS3231_ADDR_R 0xD1
#define DS3231_ADDR_W 0xD0
#define DS3231_ADDR   0x68

#define FORMAT_SHORT  1
#define FORMAT_LONG   2

#define FORMAT_LITTLEENDIAN 1
#define FORMAT_BIGENDIAN  2
#define FORMAT_MIDDLEENDIAN 3

#define MONDAY    1
#define TUESDAY   2
#define WEDNESDAY 3
#define THURSDAY  4
#define FRIDAY    5
#define SATURDAY  6
#define SUNDAY    7

#define SQW_RATE_1    0
#define SQW_RATE_1K   1
#define SQW_RATE_4K   2
#define SQW_RATE_8K   3

#define OUTPUT_SQW    0
#define OUTPUT_INT    1

class Time
{
public:
  uint8_t   hour;
  uint8_t   min;
  uint8_t   sec;
  uint8_t   date;
  uint8_t   mon;
  uint16_t  year;
  uint8_t   dow;

  Time();
};

class DS3231
{
  public:
    DS3231(uint8_t data_pin, uint8_t sclk_pin);
    void  begin();
    Time  getTime();
    void  setTime(uint8_t hour, uint8_t min, uint8_t sec);
    void  setDate(uint8_t date, uint8_t mon, uint16_t year);
    void  setDOW();
    void  setDOW(uint8_t dow);

    char  *getTimeStr(uint8_t format=FORMAT_LONG);
    char  *getDateStr(uint8_t slformat=FORMAT_LONG, uint8_t eformat=FORMAT_LITTLEENDIAN, char divider='.');
    char  *getDOWStr(uint8_t format=FORMAT_LONG);
    char  *getMonthStr(uint8_t format=FORMAT_LONG);
    long  getUnixTime(Time t);

    void  enable32KHz(bool enable);
    void  setOutput(byte enable);
    void  setSQWRate(int rate);
    float getTemp();

  private:
    uint8_t _scl_pin;
    uint8_t _sda_pin;
    uint8_t _burstArray[7];
    boolean _use_hw;

    void  _sendStart(byte addr);
    void  _sendStop();
    void  _sendAck();
    void  _sendNack();
    void  _waitForAck();
    uint8_t _readByte();
    void  _writeByte(uint8_t value);
    void  _burstRead();
    uint8_t _readRegister(uint8_t reg);
    void  _writeRegister(uint8_t reg, uint8_t value);
    uint8_t _decode(uint8_t value);
    uint8_t _decodeH(uint8_t value);
    uint8_t _decodeY(uint8_t value);
    uint8_t _encode(uint8_t vaule);
#if defined(__arm__)
    Twi   *twi;
#endif
};
#endif

// Init the DS3231 using the hardware interface
DS3231  rtc(SDA, SCL);

void setup()
{
  // Setup Serial connection
  Serial.begin(115200);
  // Uncomment the next line if you are using an Arduino Leonardo
  //while (!Serial) {}
  
  // Initialize the rtc object
  rtc.begin();
  
  // The following lines can be uncommented to set the date and time
  //rtc.setDOW(WEDNESDAY);     // Set Day-of-Week to SUNDAY
  //rtc.setTime(17, 27, 0);     // Set the time to 12:00:00 (24hr format)
  //rtc.setDate(24, 6, 2017);   // Set the date to January 1st, 2014
}

void loop()
{
  
  // Send date
  Serial.print(rtc.getDateStr());
  Serial.print(" -- ");

  // Send time
  Serial.println(rtc.getTimeStr());
  
  // Wait one second before repeating :)
  delay (1000);
}
