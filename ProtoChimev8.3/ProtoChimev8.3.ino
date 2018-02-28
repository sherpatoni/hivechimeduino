
#include <SPI.h>
#include <SD.h>
File hivechimelog;

#define DEBUG true  // turn debug message on or off in serial
#include <SoftwareSerial.h>
SoftwareSerial ESPSerial(2, 3); // RX, TX
int Sensorstate = 10000;



// Connect left board connector (C1) to Adruino 5V
// Connect second from left board connector (C2) to Adruino GND
// Board connector (C3) is left open 
int clockPin        = 5; // connects to Clock pin of shift registers : (C4) next to empty/middle connector on sensor board
int ploadPin        = 6; // connects to Parallel load aka latch pin  : (C5) middle connector on sensor board
int dataPin         = 7; // connects to the Q7 aka data pin          : (C6) outermost connector on sensor board

String ChimeID  = "HC0001"; // to keep track of origin on UDP packets
/*  Connections for RTC clock chip::
(1) + --- connects to 3V3
(2) D --- connects to SDA  -> Arduino Analog 4
(3) C --- connects to SCL  -> Arduino Analog 5
(4) Not used
(5) - --- connects to GND
*/

#define NUMBER_SENSOR_BOARDS   5 // adjust to number of boards plugged in.
#define SENSOR_PER_BOARDS   8
#define DELAY_MSEC   100  // Delay between sensor reads.
#define BOARD_READING int // type used to write the state of the 8 sensors of a board. convert to binary to decimal to read the states

int ChimeState[NUMBER_SENSOR_BOARDS]={}; // array used to record the current state of all the sensors on all the boards. Each element of the array corresponds to a board
int oldChimeState[NUMBER_SENSOR_BOARDS]={}; // array used to find changes in state of all sensors on all boards
bool Change_Sensors=false; // boolean to indicate that a sensor has changed state
String UDP_StateString; //This is used to build the string to send to UDP and display state of sensors

/* This function is a shift-in routine reading the serial Data from one shift register chip. 
 *  Not that you need a latch signal on the ploadpin before calling this function
*/
BOARD_READING read_shift_regs()
{
    long bitVal;
    BOARD_READING bytesVal = 0;

    for(int i = 0; i < SENSOR_PER_BOARDS; i++)
    {
        bitVal = digitalRead(dataPin);
   //      Serial.print(bitVal); // uncomment to debug and get readings directly in serial monitor
        bytesVal |= (bitVal << ((SENSOR_PER_BOARDS-1) - i)); // Set the corresponding bit in bytesVal.
        digitalWrite(clockPin, HIGH); // pulse data to get shift
        digitalWrite(clockPin, LOW);
    }
    return(bytesVal);
}


// the following section is straight from the clock chip online stuff. needs to be cut down to essentials.
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


void setup() {
  ESPSerial.begin(9600);
  Serial.begin(9600); 
  delay(2000);

  sendData("AT+RST\r\n",2000,DEBUG);
  sendData("AT+CIPMUX=1\r\n",2000,DEBUG);
  sendData("AT+CIPSTART=0,\"UDP\",\"34.216.40.20\",9095\r\n",2000,DEBUG);
  delay(2000);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  hivechimelog = SD.open("datalog.txt", FILE_WRITE);

  // if the file opened successfully, write start of new log:
  if (hivechimelog) {
    Serial.print("Writing to datalog.txt...");
    hivechimelog.println("Starting new log.");
    hivechimelog.close();     // close the file:
    Serial.println("done.");
  } else {
    Serial.println("error opening datalog.txt");    // Error message if file did not open correctly:
  }
  
    pinMode(ploadPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, INPUT);

    digitalWrite(clockPin, LOW); // initialize the state of the clock pin
    digitalWrite(ploadPin, HIGH); // intitialize the state of the load pin

    digitalWrite(ploadPin, LOW);
    digitalWrite(ploadPin, HIGH);
    
    for(int k = 0; k < NUMBER_SENSOR_BOARDS; k++)
    { oldChimeState[k]  = ChimeState[k]  = read_shift_regs();  }

    rtc.begin();  // Initialize the rtc object
  
/* The following lines can be uncommented to set the date and time 
 */
//  rtc.setDOW(SUNDAY);         // Set Day-of-Week to SUNDAY
//  rtc.setTime(13, 31, 0);     // Set the time to 12:00:00 (24hr format)
//  rtc.setDate(4, 9, 2017);   // Set the date to January 1st, 2014



}


// This the main loop with repeated sensing and writing
void loop() {

    Change_Sensors=false;  //Reset the indicator of change in sensors to false
    digitalWrite(ploadPin, LOW); // Latch the data from sensors to shift registers 
    digitalWrite(ploadPin, HIGH);

    for(int k = 0; k < NUMBER_SENSOR_BOARDS; k++) // read the boards one by one
    {
        ChimeState[k]  = read_shift_regs(); // read state of sensors one board at a time 
        Change_Sensors =  ((ChimeState[k] != oldChimeState[k]) || Change_Sensors); // track change in sensors 
    }
    
    
    if(Change_Sensors)  // write change with time stamp on SD if there change == true
    {   UDP_StateString = ChimeID;
        hivechimelog = SD.open("datalog.txt", FILE_WRITE); // write the time stamp
        hivechimelog.print(rtc.getDateStr());
        hivechimelog.print("T");
        hivechimelog.print(rtc.getTimeStr());  
        hivechimelog.print("Z"); 
        
        for(int n = 0; n < NUMBER_SENSOR_BOARDS; n++) // write the state of sensors one board at a time
          { hivechimelog.print(ChimeState[n]) ;
            hivechimelog.print("B"); // board separator
            Serial.print(ChimeState[n]);
            Serial.print("B");
            UDP_StateString = UDP_StateString +"B" + ChimeState[n];
          } 
            UDP_StateString = UDP_StateString + "0000000000";
            sendData("AT+CIPSEND=0,26\r\n",10,DEBUG);    
            sendData(UDP_StateString,10,DEBUG);  
        Serial.println("");   
        hivechimelog.println("");    
        hivechimelog.close();


    for(int k = 0; k < NUMBER_SENSOR_BOARDS; k++){ oldChimeState[k]  = ChimeState[k]; } // set the oldChimeState to new 
    }
    delay(DELAY_MSEC);
}

/*
* Name: sendData
* Description: Function used to send data to ESP8266.
* Params: command - the data/command to send; timeout - the time to wait for a response; debug - print to Serial window?(true = yes, false = no)
* Returns: The response from the esp8266 (if there is a reponse)
*/
     String sendData(String command, const int timeout, boolean debug)
{
String response = "";

ESPSerial.print(command); // send the read character to the esp8266

long int time = millis();

while( (time+timeout) > millis())
{
while(ESPSerial.available())
{

// The esp has data so display its output to the serial window
char c = ESPSerial.read(); // read the next character.
response+=c;
}
}

if(debug)
{
Serial.print(response);
}

return response;
}

