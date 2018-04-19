/* Reference: http://arduino.cc/en/Tutorial/WebClientRepeating  */
/* Should be a signifiant re-write*/

// Update 20180418: Start changes to test out thingspeak
// Updates 20180314: try to sync update times within the second, and adjust the times to the RTC. Will sync the RTC more than at startup too later
//
// Updates 20170916
// Changed naming for temperature probe buses, tested out second client, tried sending post bits individually, rather than building the string (didn't work)

// Key to performance was adding " HTTP/1.1" to the string sent to dweet.io 
// which then keeps the connection open so there doesn't have to be the slow reconnect for every update


// Working again. Now doesn't wait for temperature conversion to take the delay off the main loop time
// Now web update is ~ 67mS and LCD update is ~55mS (201703 data)

// Work items:
// 1. Update the day tickover time using the RTC
// 2. Move the hour and day rollover checking to the main loop, instead of when the string to send to dweet.io is built (it's pretty fast anyway, but cleaner)
// 3. Sort out passing the reference to the one_wire buss instance (class) to the functions
// 4. Bunch of error checking to the main functions
// 5. Add the watchdog timer function in
// 6. Log data to an SD card
// 7. Add menus back in to see other data if wanted
// 8. Check whether the shortened ethernet timeout is still needed. Don't think it is any more. It's a hack to have edited the global file anyway, and it looks like it would get replaced with an update to a new version of the Arduino tools. (Which it was)


#include <SPI.h>
#include <Ethernet.h> // Note: had problems in the past with timeout in void EthernetClient::stop(), changed timeout to 300mS (from 1000mS) in /Applications/Arduino.app/Contents/Java/libraries/Ethernet/src/EthernetClient.cpp
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include "Wire.h" // for RTC
#include <EthernetUdp.h> // for NTP

// Define which Arduino pin is used for each on-wire sensor bus: labels are out of date now.
// #define ONE_WIRE_BUS_PIN2 2    // All the sensors connected to pin 2 (currently all 4 using ethernet cables 20170916
#define ONE_WIRE_BUS_PINA1 A1  // Sensors connected to pin A1 ('analog' pin can be used for digital IO) (currently probe sensor only
//#define ONE_WIRE_BUS_GREY 3   // Left in for historical reasons. Not connected or used right now

#define DS18B20_RESOLUTION 9 //set the temperature probe resolution (e.g to 9, 10, 11 bits)

// Temperature Sensor Addresses. If first byte is 0x28 then DS18B20MODEL
DeviceAddress insideThermometer = { 0x28, 0x89, 0x80, 0x7C, 0x2, 0x0, 0x0, 0x5C }; //2889807C0200005C blue sheath, blue sensor wire (inside)
DeviceAddress heaterThermometer = { 0x28, 0x3B, 0x9D, 0x7C, 0x02, 0x00, 0x00, 0xA5 }; //283B9D7C020000A5 grey wire sensor, blue sensor wire (heater)
DeviceAddress basementThermometer = { 0x28, 0x22, 0xB0, 0x7C, 0x02, 0x00, 0x00, 0x7C}; //2822B07C0200007C white wire sensor (basement)
DeviceAddress outsideThermometer = { 0x28, 0xFF, 0xF4, 0x8C, 0x45, 0x16, 0x03, 0x9A }; //28FFF48C4516039A probe, blue sheath, green sensor wire (outside)
// DeviceAddress probe1Thermometer = { 0x28, 0xFF, 0xC9, 0xBD, 0x50, 0x16, 0x4, 0xC9 }; //28FFC9BD501604C9

//Forward Declarations
void query_temperature_probes(); //function prototype. Should have these for all the functions


unsigned long loopCounter;
uint8_t scratchPad[9];
char tBuf[6]; //buffer to hold acsii representation of temperature when used with .5 degree resolution
uint8_t lcdDisplayMode = 0;
uint8_t lcdSubDisplayMode = 0;

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

char server[] = "www.dweet.io";
char dweetString[250]; //the command buffer sent to dweet.io
char stringRTCtime[] = "00/00/00_--:--:--";
char stringDayTickTime[] = "--/--/--_--:--:--";
char stringHourTickTime[] = "--/--/--_--:--:--";
char stringUpdateTickTime[] = "--/--/--_--:--:--";


#define DS1307_ADDRESS 0x68 //RTC from example and address found by scanner
byte zero = 0x00; //workaround for issue #527 (needed for RTC)

unsigned long tempTimestamp;

unsigned long lastConnectionTime;             // last time you connected to the server, in milliseconds
unsigned long currentMillisTime;
unsigned long currentInterval;
//unsigned long idleTicks;  //counter of the number of times you get to the bottom of the main loop without doing something

#define DAY_END 0 //at what hour does the day rollover. 0 for midnight

const unsigned long ticksHour = 3600L * 1000L;  // set to 3600 seconds for the hour
const unsigned long ticksDay = 24L * ticksHour; // proper time intervals
const unsigned long ticksSecond = 1000L; // constant to convert seconds to milliseconds. unsigned long to force math to happen properly for timers
const int timerAdjustmentJog = 300; // the amount to adjust all the timing intervals to get back into sync
int timerAdjustment;

const unsigned long postingInterval = 10000L; // delay between web updates, in milliseconds. Update to 10 seconds (now tested down to 5)
                                               
const unsigned long lcdPostingInterval = 1000L;  // change to update the LCD every second,

unsigned long lastLCDconnectionTime;
unsigned long currentLCDinterval;

//unsigned long lastTimeCheckTime;

unsigned long lastHourTime; //snapshot of millis() timer when last hour event occurred.
unsigned long lastDayTime; //snapshot of millis() timer when last day event occured.
unsigned long hourTimer; //how many milliseconds into the current hour we are right now
unsigned long dayTimer; //how many milliseconds into the day we are right now.

int lastUpdateSecond; // used to check for RTC second changing during the update window

//const unsigned long ticksHour = 120L * 1000L; //speed up hour time for testing (set to two minutes)
//const unsigned long ticksDay = 5L * ticksHour; // speed up the day time for testing (5 hours)


#define TEMP 0 // Array index for current temperature
#define MAX 1  // Array index for maximum temperature current hour
#define MIN 2  // Array index for minimum temperature current hour
#define MAXH 3 // Array index for maximum temperature past hour
#define MINH 4 // Array index for minimum temperature past hour
#define MAXCD 5 // Array index for maximum temperature current day
#define MINCD 6 // Array index for minimum temperature current day
#define MAXD 7 // Array index for maximum temperature past day
#define MIND 8 // Array index for minimum temperature past day

#define DAY_TIMER_EEPROM 10 // write to address 10 for the dayTimer

// Temperatures are stored as a single signed byte but doubled so that there can be 0.5 degree resolution
int8_t tOutside[9], tInside[9], tBasement[9], tHeater[3]; //Arrays to store temperatures. Indexes as per definitions above

LiquidCrystal lcd(8, 9, A2, 5, 6, 7); // updated value ... re-route pin 4 to A2 to avoid conflict with SD card

IPAddress ip(192, 168, 21, 69); //picked a hard coded ethernet address for the shield

// UDP Declarations for NTP
unsigned int localPort = 8888;       // local port to listen for UDP packets
char timeServer[] = "time.nist.gov"; // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWireMain(ONE_WIRE_BUS_PINA1); //Change to using this for the temperature bus, external pullup resistor for UNO
//OneWire oneWireMain(ONE_WIRE_BUS_PIN2);  //classes start with upper case letter, variables start with lower case
 
// Pass our oneWire reference to Dallas Temperature.
// +++ Question to Kevin: is this a declaration, function call, executable statement ?
// the "variables" "blue_sensors, white_sensors, and grey_sensors" are "defined" in these calls
//
DallasTemperature main_sensors(&oneWireMain); //**was blue sensors

// initialize the library instance:
// ++ QfK is this intializing anything, checking for a connection, or just a definition?
//
EthernetClient client_1;
EthernetClient client_2; // try a second client at the same time
EthernetUDP UdpNTP; // A UDP instance to let us send and receive packets over UDP


void setup() {
  
  // start serial port:
  Serial.begin(115200);
  Serial.println(F("\r\n\n--- Start ---")); 

  Wire.begin(); //for RTC

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Main Menu");

// disable the SD card by switching pin 4 high
// not using the SD card in this program, but if an SD card is left in the socket,
// it may cause a problem with accessing the Ethernet chip, unless disabled
// from https://startingelectronics.org/tutorials/arduino/ethernet-shield-web-server-tutorial/basic-web-server/
   pinMode(4, OUTPUT);
   digitalWrite(4, HIGH);
  
  // give the ethernet module time to boot up:
  delay(ticksSecond);
  
  // start the Ethernet connection using a fixed IP address and DNS server:
  Ethernet.begin(mac, ip);
  
  // print the Ethernet board/shield's IP address:
  Serial.print(F("My IP address: "));
  Serial.println(Ethernet.localIP());

  int udpStatus = UdpNTP.begin(localPort);
  Serial.print(F("UDP init status: ")); Serial.println(udpStatus);
  
  lcd.setCursor(0,1);
  lcd.print(Ethernet.localIP());

 // Start up the one-wire library for temperature sensors  
 // +++ QfK (Need to check that all three need to be done. grey_sensors was missing in previous versions and things seemed to work)
 //
  main_sensors.begin();
//  secondary_sensors.begin();
//  grey_sensors.begin();

// set the resolution of all the temperature sensors
  main_sensors.setResolution(DS18B20_RESOLUTION); // adds 452 bytes to the code size! 
//  secondary_sensors.setResolution(DS18B20_RESOLUTION);// second call is only 10 bytes

  main_sensors.setWaitForConversion(true); // this is the default, but be explicit for first pass reads. Block waiting for the temperature conversion.

  //** New Call to get the status of the devices on the bus
  // ++ QFK should I be defining these "functions" (methods, whatever) up earlier. Seems that I should be
  //
  query_temperature_probes();

Serial.print(F("start updateTempSensors"));
  updateTempSensors(); //do an initial temperature reading and update the global variables
Serial.println(F("...done"));

// Initialize the min and max variables
  tOutside[MAX] = tOutside[TEMP];
  tOutside[MIN] = tOutside[TEMP];
  
  tOutside[MAXH] = EEPROM.read(MAXH); // retrieve this from EEPROM on restart
  tOutside[MINH] = EEPROM.read(MINH); // retrieve this from EEPROM on restart
  tOutside[MAXCD] = EEPROM.read(MAXCD); // retrieve this from EEPROM on restart
  tOutside[MINCD] = EEPROM.read(MINCD); // retrieve this from EEPROM on restart
  tOutside[MAXD] = EEPROM.read(MAXD); // retrieve this from EEPROM on restart
  tOutside[MIND] = EEPROM.read(MIND); // retrieve this from EEPROM on restart

  EEPROM.get(DAY_TIMER_EEPROM, dayTimer);
  Serial.print(F("DayTimer Value Recovered> ")); Serial.println(dayTimer);

  currentMillisTime=millis(); // initialize to get proper timing for initial request

  updateRTC();

/*
  sendNTPpacket(timeServer); // send an NTP packet to a time server
  //wait for the reply
  while (!UdpNTP.parsePacket()) { // wait for the UPD reply to be there
    Serial.print(".");
    delay(10);
  }
  Serial.print(F("msecs for NTP reply: ")); Serial.println(millis()-currentMillisTime);
  parseNTPreply1();
*/
  
  Serial.print(F("Initial RTC Time: ")); Serial.println(stringRTCtime);
  
  httpRequest(); //Send the first data without waiting
    
  main_sensors.setWaitForConversion(false); // change so that from here on there's no wait for the temperature conversion.
                                            // so no busy waiting on the critical path  

  setRTCfromDweet(); // get the time from the network, update the RTC, and initialize/synchronize the update, LCD, hour and day periods
  updateRTC();

  Serial.print(F("RTC Time after setting from network: ")); Serial.println(stringRTCtime); //updated stringRTCtime

  int udpTIME = getNTPtime();
  Serial.println (udpTIME);



  Serial.println(F("\r\nNew Setup Done\r\n"));
} // end setup()

//********************************************************************************************************
//
// The Main Loop
//
// 
//********************************************************************************************************

void loop() {
  // first priority is to see if the web information needs to be updated
  // if the posting inteval seconds have passed since your last connection,
  // then connect again and send data:

  currentMillisTime=millis(); //get the current time tick and save it so all things can have same timestamp 
  
  currentInterval = currentMillisTime - lastConnectionTime;  
  if (currentInterval >= postingInterval) {  //then time to update
    if (currentInterval > (postingInterval+2)) { // weren't just busy waiting, add in 2msec for loop overheads
      Serial.print(F("Web Update Interval Exceeded: ")); Serial.println(currentInterval); // so log the exception
    }

    lastUpdateSecond = updateRTC(); // this is fast, seems to only take about a millisecond.

    httpRequest(); //connect to the sensors and send the data ***this needs a return code to check.
    
    lastConnectionTime += postingInterval; //set timestamp for next web update
    
    lastUpdateSecond = lastUpdateSecond % 10; //just check the last digit 
    if (lastUpdateSecond != 0 ) { // then update is not synced to an even 10 seconds so adjust connection time
      if (lastUpdateSecond == 1) { // took too long, so make the next update interval a bit shorter
        timerAdjustment = -1 * timerAdjustmentJog;
      } else if (lastUpdateSecond == 9) { //update too soon so make the next update interval a bit longer
        timerAdjustment = timerAdjustmentJog;
      } else { // way out, log an error, need to have better handling here
        Serial.print(F("\r\nUpdate Second out of bounds: "));Serial.println(lastUpdateSecond);
      }
      lastLCDconnectionTime += timerAdjustment;
      lastConnectionTime += timerAdjustment;
      lastHourTime += timerAdjustment;
      lastDayTime += timerAdjustment;
      Serial.print(F("\r\nTiming Update of: "));Serial.println(timerAdjustment);
      delay(timerAdjustmentJog); // wait for adjustment time is past in case you adjusting forward to a time that hasn't happened yet.
      Serial.print(F("Web Update Total Time: ")); Serial.println(millis()-currentMillisTime);
      Serial.println(stringRTCtime); 
    }
// Serial.print(currentMillisTime); Serial.print(F(" ")); Serial.println(millis()); //Print millis() timestamp for analysis  
  } // end of things done during a temperature sensor web update.

// Second Priority is check if Hour or Day has elapsed and update stats as required
  
  updateHourDayStats();  
 
  
//Next (third priority) check to see if it's time to update the LCD information
//
  currentMillisTime=millis(); //get the current time tick and save it so all things can have same timestamp
  currentLCDinterval = currentMillisTime - lastLCDconnectionTime; 
  if (currentLCDinterval >= lcdPostingInterval) {  //then time to update the screen
    if (currentLCDinterval > (lcdPostingInterval+2)) { // weren't just busy waiting, add 2 ms just to ignore cases where loop overhead was the cause.
                                                      // I have only seen +1ms cases in the logs
      Serial.print(F("\rLCD Update Interval Exceeded: ")); Serial.println(currentLCDinterval); // so log the exception
 //     Serial.print(F("Last Connection Times. LCD: "));Serial.print(lastLCDconnectionTime);
 //     Serial.print(F(" Server: "));Serial.print(lastConnectionTime);
 //     Serial.print(F(" HB: "));Serial.println(loopCounter);
    }
    lastLCDconnectionTime += lcdPostingInterval; // keep the updates synced


    updateRTC(); // this is fast, seems to only take about a millisecond.
    
    lcd.clear();
    if (lcdDisplayMode < 4) { // Normal Display Mode
      lcd.print(F("IT:"));
      lcd.print(asciiTemp( getByteTemp(insideThermometer), tBuf)); 
      lcd.setCursor(8,0);
      lcd.print(F("OT:"));
      lcd.print(asciiTemp( getByteTemp(outsideThermometer), tBuf));
      lcd.setCursor(0,1);
      lcd.print(loopCounter %100); // last two digits of the loopCounter, just to check things are updating
      lcd.setCursor(3,1);
      lcd.print(F("Time"));
      lcd.setCursor(8,1);
      lcd.print(&stringRTCtime[9]); //reach into the Date/Time string to get just the Current time part

 //     lcdDisplayMode++;
    } else { //Heater, Basement, Hour, Day Timestamps
      lcd.print(F("HT:"));
      lcd.print(asciiTemp( getByteTemp(heaterThermometer), tBuf)); 
      lcd.setCursor(8,0);
      lcd.print(F("BT:"));
      lcd.print(asciiTemp( getByteTemp(basementThermometer), tBuf));
      lcd.setCursor(3,1);
      lcd.print(F("HrT"));
      lcd.setCursor(0,1); //beginning of second line
      if (lcdSubDisplayMode == 0) { // cycle through timestamps
        lcd.print(F("H "));
        lcd.print(&stringHourTickTime[3]); 
      } else if (lcdSubDisplayMode == 1) {
        lcd.print(F("D "));
        lcd.print(&stringDayTickTime[3]);
      } else if (lcdSubDisplayMode ==2) {
        lcd.print(F("U "));
        lcd.print(&stringUpdateTickTime[3]);
        
      } else if (lcdSubDisplayMode == 3) {
        lcd.print(F("HS"));
        lcd.print(hourTimer);
      } else { // lcdSubDisplayMode should be 4
        lcd.print(F("DS"));
        lcd.print(dayTimer);
      }
      
      // end of sub display update
 
    } // end second display update
    
    if (lcdDisplayMode > 6) {
      lcdDisplayMode = 0;
      if (lcdSubDisplayMode >= 2) lcdSubDisplayMode = 0; //set to 4 to enable the final two LCD Sub Display options.
      else lcdSubDisplayMode ++;
    }
     else lcdDisplayMode++;

//    lcd.setCursor(0,1);
//    lcd.print(loopCounter %100);
//    lcd.setCursor(3,1);
//    lcd.print(asciiTemp( getByteTemp(outsideThermometer), tBuf));
//    lcd.setCursor(8,1);
//    lcd.print(&stringRTCtime[9]); //reach into the Date/Time string to get just the time part

    main_sensors.requestTemperatures(); // Send the command to get temperatures, with resolution set to 9, this takes about 97 mS.
                                        // should be done converting by the next update cycle, don't wait for the conversion to take place

    Serial.print(millis()-currentMillisTime); Serial.print(' '); //print out time to do the LCD update
//Serial.println(currentMillisTime);  //Print millis() timestamp for analysis  

  } // end lcd update section

  // Final Priority
  // if there's incoming data from the net connection send it out the serial port.  This is for debugging purposes only:
  // (data is coming back one character at a time here)
  
  if (client_1.available()) {
    char c = client_1.read();
    Serial.write(c);
  } 

  
  } // End loop()
  
//********************************************************************************************************
//
//   End of Main Loop.
//
//********************************************************************************************************

// this method makes a HTTP connection to the server:
void httpRequest() {
  char tempBuff[10]; //temporary buffer to store the Temperature value converted to a string.

  if (!client_1.connected()) {
    Serial.println(F("not connected, so connect to server"));
    client_1.stop(); //This seems to be needed to clean up before a new connect command will succeed. Although it times out.
    Serial.print(F("Stopped old client in: ")); Serial.println(millis()-currentMillisTime);
    tempTimestamp = millis();
    client_1.connect(server, 80); //connect if needed **need to check a return code here
    Serial.print(F("\r\nConnect Time: ")); Serial.println(millis()-tempTimestamp);
  }

  updateTempSensorsNoUpdate(); // Get the temperature readings and update the global variables

  
// Process and check for min and maximum hourly outside temperatures    
// This could also be moved outside this update check loop.
// Maybe into the calculate and archive average data when/if I add it.

    if(tOutside[TEMP] > tOutside[MAX]) {
      tOutside[MAX] = tOutside[TEMP];
    }
    
    if(tOutside[TEMP] < tOutside[MIN]) {
      tOutside[MIN] = tOutside[TEMP];
    }
    
// Check if new Daily min or max
    if(tOutside[TEMP] > tOutside[MAXCD]) {
      tOutside[MAXCD] = tOutside[TEMP];
    }
    
    if(tOutside[TEMP] < tOutside[MINCD]) {
      tOutside[MINCD] = tOutside[TEMP];
    }
  
  // if there's a successful connection to the "www.dweet.io" server (as defined in the server variable):
  if (client_1.connected()) {  //if there's a connection, use it
 // Serial.println(F("Connection still open, so use it"));
    
  // Build the HTTP request:
    
    strcpy_P(dweetString,(PGM_P) F("POST /dweet/quietly/for/CWF828312?&InT=")); // omit the /quietly part of URL for long connection response
    strcat(dweetString, asciiTemp( tInside[TEMP],tempBuff));
    strcat_P(dweetString,(PGM_P) F("&OutT="));
    strcat(dweetString, asciiTemp( tOutside[TEMP],tempBuff));
    strcat_P(dweetString,(PGM_P) F("&BasT="));
    strcat(dweetString, asciiTemp( tBasement[TEMP],tempBuff));
    strcat_P(dweetString,(PGM_P) F("&HiT="));
    strcat(dweetString, asciiTemp( tHeater[TEMP],tempBuff));
    strcat_P(dweetString,(PGM_P) F("&HB="));
    ltoa(loopCounter, tempBuff, 10);
    strcat(dweetString, tempBuff); 
    strcat_P(dweetString,(PGM_P) F("&TOD="));
    strcat(dweetString, stringRTCtime); //use the time string from just before httpUpdate() was called
    strcpy(stringUpdateTickTime,stringRTCtime);  // preserve the timestamp for later reporting through the UI
    strcat_P(dweetString,(PGM_P) F("&MinOutT_LH="));
    
    strcat(dweetString, asciiTemp( tOutside[MINH],tempBuff));
    strcat_P(dweetString,(PGM_P) F("&MaxOutT_LH="));
    strcat(dweetString, asciiTemp( tOutside[MAXH],tempBuff));
    strcat_P(dweetString,(PGM_P) F("&MinOutT_CH="));
    strcat(dweetString, asciiTemp( tOutside[MIN],tempBuff));
    strcat_P(dweetString,(PGM_P) F("&MaxOutT_CH="));
    strcat(dweetString, asciiTemp( tOutside[MAX],tempBuff));
     
    strcat_P(dweetString,(PGM_P) F("&MinOutT_CD="));
    strcat(dweetString, asciiTemp( tOutside[MINCD],tempBuff));
    strcat_P(dweetString,(PGM_P) F("&MaxOutT_CD="));
    strcat(dweetString, asciiTemp( tOutside[MAXCD],tempBuff));
    strcat_P(dweetString,(PGM_P) F("&MinOutT_YD="));
    strcat(dweetString, asciiTemp( tOutside[MIND],tempBuff));
    strcat_P(dweetString,(PGM_P) F("&MaxOutT_YD="));
    strcat(dweetString, asciiTemp( tOutside[MAXD],tempBuff));

    strcat_P(dweetString,(PGM_P) F(" HTTP/1.1"));  //try adding the protocol stuff in case it makes a difference
                                                   //which is does ... adding this makes part of the response string 
                                                   // "Connection: keep-alive" rather than "Connection: close", 
                                                   // and the subsequent client.stop command doesn't time out.
//  Serial.print(F("dweet strnlen: "));Serial.println (strlen (dweetString)); // print out length of string to make sure buffer is large enough
    client_1.println(dweetString);

    client_1.println(F("Host: www.dweet.io")); // Every http request needs to have the host part (apparently ... but works without this too
//    client_1.println(F("Connection: close"));  // seems to work whether this line is included or not
                                               // if this is added then dweet.io will close the connection after the request, but arduino
                                               // will keep things (sort of) open. So a client.stop is needed before a reconnect will work
                                               // although it will timeout
                                               // it seems to be fine to leave the connection open anyway, and save all this overhead
    
    client_1.println();                          // blank line signals end of http request
    
    loopCounter++;
  }
  else {
    // if you couldn't make a connection:
    Serial.print(F("\r\nconnection failed in httpRequest: "));
  }
  Serial.print(F("\r\n\nFinished httpRequest at: ")); Serial.print(stringRTCtime);
  Serial.print(F(" in: ")); Serial.print(millis()-currentMillisTime); Serial.println(F(" msec\r\n"));
  
} // end httpRequest()

void updateTempSensors() {

   // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus

  // ***** work item: I guess I should be passing in the one-wire bus instances that are used. 
  // Right now "main_sensors, secondary_sensors, & grey_sensors" are global variables
  

Serial.print(F("Requesting temperatures..."));
  main_sensors.requestTemperatures(); // Send the command to get temperatures
//  secondary_sensors.requestTemperatures(); // Send the command to get temperatures
//  grey_sensors.requestTemperatures(); // Send the command to get temperatures


  updateTempSensorsNoUpdate(); //read the sensors and update the global variables

} // end updateTempSensors

void updateTempSensorsNoUpdate() {

  tInside[TEMP] = getByteTemp(insideThermometer);
  tOutside[TEMP] = getByteTemp(outsideThermometer); 
  tBasement[TEMP] = getByteTemp(basementThermometer); 
  tHeater[TEMP] = getByteTemp(heaterThermometer);
} // end updateTempSensorsNoUpdate


// read directly from the DS18B20 sensor. Assumes that resolution is set to 9, and that main_sensors bus is in use.
// returns the temperature as a signed byte with a value that is twice the actual temperature reading, so there is no decimal part.
// This needs to have error checking added, and it would be nice to be able to pass in the reference to the right sensor onewire bus
//
int8_t getByteTemp (uint8_t *address){
  uint8_t scratchPad[9];
  main_sensors.readScratchPad(address, scratchPad);
  return ((scratchPad[TEMP_MSB] << 5) | (scratchPad[TEMP_LSB] >> 3));
} //end getByteTemp


// Takes an input temperature of a signed byte with actual temperature doubled, to give .5 degree resolution over a two digit temperature range
char* asciiTemp (int8_t storedTemp, char* buf) { //better check whether my pointer operators are in the right place
  byte i=0; //changing this to an int grows code by 12 bytes?
  int8_t actualTemp;  //this is the signed 8 bit type, apparently safer than char.

  actualTemp = storedTemp;
  if (actualTemp < 0) { //negative temperature
    buf[i++] = '-';
    actualTemp = actualTemp * -1; //convert to a positive number
  }
  
  if (actualTemp > 19) { //two digit number
    buf[i++] = (actualTemp / 20) + '0';  //tens digit
  }
  buf[i++] = (actualTemp % 20)/2 + '0';  // ones digit
  buf[i++] = '.';
  if ((actualTemp % 2) == 1) { //odd, so add .5
    buf[i++] = '5';
  } else {
    buf[i++] = '0';
  }
  buf[i] = 0; // add the null to terminate the string
  return(buf);
} //end asciiTemp()


void query_temperature_probes() {

// Query all the temperature sensors on the three one-wire busses   

  // ***** work item: I guess I should be passing in the one-wire bus instances that are used. 
  // Right now "sensors, white_sensors, & grey_sensors" are global variables

  uint8_t number_of_sensors, sensor_index;
  // arrays to hold device addresses (array of 8 bytes)
  DeviceAddress ds18b20_address;
  
  Serial.println(F("Start Temperature Sensor Status Report"));
  main_sensors.requestTemperatures(); // Send the command to get temperatures, which can take a while while the conversion takes place
//  secondary_sensors.requestTemperatures(); // Send the command to get temperatures
//  grey_sensors.requestTemperatures(); // Send the command to get temperatures

  Serial.println(F("Sensors on the Pin 2 Bus"));
  number_of_sensors = main_sensors.getDeviceCount();
  Serial.print(F("Number of devices: "));
  Serial.println(number_of_sensors);
   for (sensor_index = 0; sensor_index < number_of_sensors; sensor_index ++) {
    // finds an address at a given index on the bus
    main_sensors.getAddress(ds18b20_address, sensor_index);
    Serial.print(F("Device "));
    Serial.print(sensor_index);
    Serial.print(F(": "));
    printAddress(ds18b20_address);
    float tempC = main_sensors.getTempC(ds18b20_address);
    Serial.print(F(" Temp C: "));
    Serial.print(tempC);
    Serial.print(F(" Resolution: "));
    Serial.print(main_sensors.getResolution(ds18b20_address));
    Serial.println();
  }
  Serial.println();  
  Serial.println();Serial.println(F("End of Temperature Sensor Report"));Serial.println();

} // end query_temperature_probes()


// function to print a device address (taken from the "Multiple" example)
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function to print a device's resolution (taken from the "Multiple" example)
// +++ QFK how do I pass in the "sensors" instance as a reference?
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print(F("Resolution: "));
  Serial.print(main_sensors.getResolution(deviceAddress));
  Serial.println();    
}

byte bcdToDec(byte val)  {
// Convert binary coded decimal to normal decimal numbers
  return ( (val/16*10) + (val%16) );
}

byte decToBcd(byte val){
// Convert normal decimal numbers to binary coded decimal
  return ( (val/10*16) + (val%10) );
}

int updateRTC(){

  // Reset the register pointer
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write((uint8_t)0x00);  //casting the number to avoid method ambiguity (was issue #527)
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 7);

  byte second = bcdToDec(Wire.read());
  byte minute = bcdToDec(Wire.read());
  byte hour = bcdToDec(Wire.read() & 0b111111); //24 hour time
  byte weekDay = bcdToDec(Wire.read()); //0-6 -> sunday - Saturday
  byte monthDay = bcdToDec(Wire.read());
  byte month = bcdToDec(Wire.read());
  byte year = bcdToDec(Wire.read());

  stringRTCtime[0] = (year / 10) + '0';
  stringRTCtime[1] = (year % 10) + '0';
  stringRTCtime[3] = (month / 10) + '0';
  stringRTCtime[4] = (month % 10) + '0';
  stringRTCtime[6] = (monthDay / 10) + '0';
  stringRTCtime[7] = (monthDay % 10) + '0';
  stringRTCtime[9] = (hour / 10) + '0'; //hour MSD
  stringRTCtime[10] = (hour % 10) + '0'; //hour LSD
  stringRTCtime[12] = (minute / 10) + '0';
  stringRTCtime[13] = (minute % 10) + '0';
  stringRTCtime[15] = (second / 10) + '0';
  stringRTCtime[16] = (second % 10) + '0';

  return second; //return the current second value
} //end updateRTC()



//Used to get a server response to parse time date from
//
unsigned long httpShortRequest() {
  unsigned long requestTimestamp;
  unsigned long request2_start;
  
  request2_start = millis();

  if (!client_2.connected()) {
    Serial.println(F("Client 2 not connected, so connect to server"));
    client_2.stop(); //This seems to be needed to clean up before a new connect command will succeed. Although it times out.
    Serial.print(F("Stopped old client 2 in: ")); Serial.println(millis()-request2_start);
    tempTimestamp = millis();
    client_2.connect(server, 80); //connect if needed **need to check a return code here
    Serial.print(F("\r\nConnect 2 Time: ")); Serial.println(millis()-tempTimestamp);
  }
  
  // if there's a successful connection to the "www.dweet.io" server (as defined in the server variable):
  if (client_2.connected()) {  //if there's a connection, use it
 // Serial.println(F("Connection still open, so use it"));
    
  // Build the HTTP request string:
    strcpy_P(dweetString,(PGM_P) F("POST /dweet/quietly/for/CWF828313?Timestamp=")); // omit the /quietly part of URL for long connection response
    strcat(dweetString, stringRTCtime);
    strcat_P(dweetString,(PGM_P) F(" HTTP/1.1"));  //try adding the protocol stuff important to keep connection in keep-alive state
                                                   
    client_2.println(dweetString);

    client_2.println(F("Host: www.dweet.io")); // Every http request needs to have the host part (apparently ... but works without this too
                                               // Adding this to the string before the println above doesn't seem to work. Not sure why.
                                               
    client_2.println();                          // blank line signals end of http request
    requestTimestamp=millis();
    
    Serial.print(F("\r\nFinished httpShortRequest in: "));
  } // end of if client2 connected
  else {
    // if you couldn't make a connection:
    Serial.print(F("\r\nconnection failed in httpRequest2: "));
  }
  Serial.println(millis()-request2_start); Serial.println();
  return requestTimestamp;
} // end httpShortRequest()

void setRTCfromDweet() {
  
  int x =0;
  char cbuf[32];
  int tmonth;
  int flag =0;
  unsigned long timestamp;
  int secondsAdjustment; // estimate of time needed to adjust seconds for response delay
  
  byte second; //0-59
  byte minute; //0-59
  byte hour; //0-23
  byte weekDay; //1-7 Sunday=1
  byte monthDay; //1-31
  byte month; //1-12
  byte year; //0-99
  
// this function will send a request to DWEET.io to get the timestamp response and use it to set the RTC clock

  timestamp = httpShortRequest();
  
  Serial.println(F("Waiting for RTC Request Reply"));
  delay(3000); //wait 3 seconds, should be long enough to let the reply finish coming back from the server, adjust time at end for the delay & processing
  
    while (client_2.available()) {
    char c = client_2.read();
    if (c == 'D') flag = 1; //then start of the Date string
    if (c == 'G') flag = 0; //end of the date string with the G from GMT
    if (flag == 1) {
      cbuf[x++] = c;
    }
    }
    cbuf[x-1] = 0; //null terminate the response string for printing
    Serial.print(F("Time string from dweet.io response: ")); Serial.println(&cbuf[6]);
    // first get the weekday
    switch (cbuf[7]) { // use second character of the day string
      case 'o': 
        weekDay = 2; //Monday
        break;
      case 'e' :
        weekDay = 4; //Wednesday
        break;
      case 'h':
        weekDay = 5; //Thursday
        break;
      case 'r':
        weekDay = 6; //Friday
        break;
      case 'a':
        weekDay = 7; //Saturday
        break;
      case 'u':
        if (cbuf[5] == 'S') weekDay = 1; //Sunday
        if (cbuf[5] == 'T') weekDay = 3; //Tuesday
        break;      
    }
    
    tmonth = (cbuf[15] << 8) + cbuf[16]; // get the last two characters of the month string, and convert to an int to use for switch statment comparison 
                                         // More reliable and explicit than trying to do it with cast and pointers
                                         
    switch ( tmonth ) {
      case 0x616e:
        month = 1;
        break;
      case 0x6562:
        month = 2;
        break;
      case 0x6172:
        month = 3;
        break;
      case 0x7072:
        month = 4;
        break;
      case 0x6179:
        month = 5;
        break;
      case 0x756e:
        month = 6;
        break;
      case 0x756c:
        month = 7;
        break;
      case 0x7567:
        month = 8;
        break;
      case 0x6570:
        month = 9;
        break;
      case 0x6374:
        month = 10;
        break;
      case 0x6f76:
        month = 11;
        break;
      case 0x6563:
        month = 12;
        break;
      default:
        month = 0; // no match
        Serial.println( tmonth, HEX );  
    }
    year = 10*(cbuf[20] - '0') + cbuf[21] - '0';
    monthDay =  10*(cbuf[11]-'0') +cbuf[12] - '0';
    hour = 10*(cbuf[23] - '0')+cbuf[24] - '0';
 
 // The following is for EDT
    if (hour < 4) { // make the conversion from GMT to EST (not worrying about year rollover)
      hour = 20 + hour;
      monthDay--; // will make a month of 0 for month end rollover, and I don't want to mess with all the different month lengths
      if (weekDay == 1) weekDay =7;
      else weekDay --;
    } else hour = hour - 4;

    /* The following is for EST
    if (hour < 5) { // make the conversion from GMT to EST (not worrying about year rollover)
      hour = 19 + hour;
      monthDay--; // will make a month of 0 for month end rollover, and I don't want to mess with all the different month lengths
      if (weekDay == 1) weekDay =7;
      else weekDay --;
    } else hour = hour - 5;
    */
    minute = 10*(cbuf[26] - '0')+cbuf[27] - '0';
    second = 10*(cbuf[29]- '0')+cbuf[30] -'0';
    
    secondsAdjustment = (millis() - timestamp + 100)/ticksSecond; //convert from ms to seconds, and adjust for rounding/latency (a bit)
    Serial.print(F("Seconds Adjustment: "));Serial.println(secondsAdjustment);

    second = second + secondsAdjustment;
    if (second >=60) {
      second = second - 60;
      minute++;
/*      
      if (minute ==60) {
        minute = 0;
        hour++;
        if (hour == 24) {
          hour = 0;
          monthDay ++;
          if (monthDay > 28) { //there is a risk of a month end rollover
            switch (month) {
              case 2: //Feb. Doesn't take leap years into account.
                month ++;
                monthDay = 1;
                break;
             case 4: //April
             case 6: //June
             case 9: //September
             case 11: //November
               if (monthDay == 31) {
                month ++;
                monthDay = 1;
                break;
               }
             case 12: //Dec
             if (monthDay == 32) {
              month = 1; //Set to January
              monthDay = 1;
              year ++; //new year!
              break;
             }
             default:
               if (monthDay == 32) {
                month ++;
                monthDay = 1;
                break; 
               }
              } // switch
            } // monthDay >> 28
          } // hour
        } // minute = 60

*/        
      } // second >= 60


//now set the RTC to the updated server time that was returned
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(zero); //stop Oscillator

  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));
  Wire.write(decToBcd(weekDay));
  Wire.write(decToBcd(monthDay));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));

  Wire.write(zero); //start 

  Wire.endTransmission();
  
// POLL FORWARD UNTIL THE RTC second ticks over and then I could set all the event offsets relative to the beginning
// of the RTC Second
Serial.print(F("Wait for second rollover"));
byte currentSecond = updateRTC(); // get the value of the current second
Serial.println(currentSecond);
while (currentSecond == updateRTC() ) {
  Serial.print(".");
  delay(10);
}
second = updateRTC(); //get the new second value
Serial.println(second);

  timestamp = millis(); // this is used as the baseline to set all the event timestamps

  //**********
  // ** I Need to get all these timestamp offsets right.
  // Can't be adding some, and subtracting others
  // E.G. The lastConnectionTime can be in the future if second ends in zero (because of the +400) which causes all the timing loops to go haywire
  //
  //**************
  
  lastConnectionTime = timestamp -  ((second % 10) * ticksSecond) + 400L; // sync the connection time to even 10 second time within a minute
                                                                   // add 400 msec to try to get web update to fall in the middle of the one second timing window

  lastLCDconnectionTime = timestamp + 200L; // start updating LCD just before first post, and sync to 200 msec before web update
  
  long hourAdjustment = (ticksSecond * (second + (minute * 60))); 
  lastHourTime = timestamp - hourAdjustment + 300L; // try to make the hour change at the right time, and 300 msec into the update window
//
// set the hour for the day rollover
//  
  int dayEndHour = DAY_END; // even hour you want the day to end at.
  
  unsigned long dayAdjustment;
  if (hour >= dayEndHour) {  
    dayAdjustment = (hour - dayEndHour) * ticksHour;
    Serial.print(F("hour > dayEndHour "));
  } else {
    dayAdjustment = ( 24L + hour - dayEndHour) * ticksHour;
    Serial.print(F("hour < dayEndHour "));
  }
  lastDayTime = lastHourTime - dayAdjustment; // note that lastHourTime already has the 300msec timing adjustment

  Serial.print(F("DayAdjustment: ")); Serial.println(dayAdjustment);
  Serial.print(hourAdjustment);Serial.print(F(" <- hour adjustment. millis -> ")); Serial.println(millis());

  Serial.print(F("millis           now is:  ")); Serial.println(millis());
  Serial.print(F("millis timestamp now is:  ")); Serial.println(timestamp);
  Serial.print(F("set lastLCDconnectionTime:")); Serial.println(lastLCDconnectionTime);
  Serial.print(F("set lastConnectionTime:   ")); Serial.println(lastConnectionTime);
  Serial.print(F("set lastHourTime to:      ")); Serial.println(lastHourTime);
  Serial.print(F("set lastDayTime to:       ")); Serial.println(lastDayTime);
  Serial.print(F("next hour tick at:        ")); Serial.println(timestamp - lastHourTime);
  
    
    Serial.println(weekDay);
    Serial.println(year);
    Serial.println(month);
    Serial.println(monthDay);
    Serial.println(hour);
    Serial.println(minute);
    Serial.println(second);

    delay(400); // wait to make sure none of the times are in the future.

    Serial.println(F("Finished set RTC from network"));
  } //end setRTCfromDweet()

void updateHourDayStats() {

// If an day has elapsed then copy the day data and reset the min/max to current values
//
    currentMillisTime = millis();
    updateRTC();
    
    dayTimer = currentMillisTime - lastDayTime; //snapshot the dayTimer for use in later reporting
    if (dayTimer >= ticksDay) {
      lastDayTime += ticksDay;
      tOutside[MAXD] = tOutside[MAXCD];
      tOutside[MAXCD] = tOutside[TEMP];
      tOutside[MIND] = tOutside[MINCD];
      tOutside[MINCD] = tOutside[TEMP];

      strcpy(stringDayTickTime,stringRTCtime); // preserve the timestamp for later reporting
      
      Serial.print(F("\r\nDay Timer Kicked, value: ")); Serial.print(dayTimer); 
      Serial.print(F(" RTC Time: ")); Serial.print(stringRTCtime);
      Serial.print(F(" Day Update Duration: "));Serial.println(millis()-currentMillisTime);
    }


// If an hour has elapsed then close interval & copy the data and reset the min/max to current values
    
    hourTimer = currentMillisTime - lastHourTime; //snapshot the hourTimer for use in later reporting
    if (hourTimer >= ticksHour) {
      lastHourTime += ticksHour;
      
      tOutside[MAXH] = tOutside[MAX];
      tOutside[MAX] = tOutside[TEMP];
      tOutside[MINH] = tOutside[MIN];
      tOutside[MIN] = tOutside[TEMP];

//    And archive some informtion to EEPROM for startup after a reboot
      EEPROM.update(MAXH,tOutside[MAXH]);
      EEPROM.update(MINH,tOutside[MINH]);
      EEPROM.update(MAXCD,tOutside[MAXCD]);
      EEPROM.update(MINCD,tOutside[MINCD]);
      EEPROM.update(MAXD,tOutside[MAXD]);
      EEPROM.update(MIND,tOutside[MIND]);

      EEPROM.put(DAY_TIMER_EEPROM, dayTimer); //store how far into the day we were

      strcpy(stringHourTickTime,stringRTCtime);  // preserve the timestamp for later reporting via UI
      
      Serial.print(F("\r\nHour Timer Kicked. HourTimer: ")); Serial.print(hourTimer);
      Serial.print(F(" RTC Time: ")); Serial.print(stringRTCtime);
      //Serial.print(F(" NTP UTC Time: ")); 
      Serial.print(getNTPtime());
      Serial.print(F("  Hour Update Duration: "));Serial.println(millis()-currentMillisTime);

    }
} // end updateHourDayStats()


void sendNTPpacket(char* address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  UdpNTP.beginPacket(address, 123); //NTP requests are to port 123
  UdpNTP.write(packetBuffer, NTP_PACKET_SIZE);
  UdpNTP.endPacket();
}

/*
void parseNTPreply() {

 if (UdpNTP.parsePacket()) {
    // We've received a packet, read the data from it
    UdpNTP.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);


    // print the hour, minute and second:
    Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if (((epoch % 3600) / 60) < 10) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ((epoch % 60) < 10) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
  }
  Ethernet.maintain();
} // END parseNTPreply

*/

int getNTPtime() {
  int timestampUDP;

  timestampUDP = millis();
  sendNTPpacket(timeServer); // send an NTP packet to a time server

    while (!UdpNTP.parsePacket()) { // wait for the UPD reply to be there
  //  Serial.print(".");
    delay(10);
    if ((millis() - timestampUDP) > 2000) { // give up waiting for a reply if it's been longer than two seconds
      return 0;
    }
    } // got a reply so process it
    
    timestampUDP = millis() - timestampUDP; // round trip time

    // We've received a packet, read the data from it
    UdpNTP.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
//    Serial.print("Seconds since Jan 1 1900 = ");
//    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
//    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
//   Serial.println(epoch);


    // print the hour, minute and second:
    Serial.print(F(" The UTC time is "));       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if (((epoch % 3600) / 60) < 10) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ((epoch % 60) < 10) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
  
  Ethernet.maintain();
  return timestampUDP;
  
} // getNTPtime()
