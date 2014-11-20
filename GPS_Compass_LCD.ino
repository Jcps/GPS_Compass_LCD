/* 
 Built on an Arduino Uno
 
 This program uses a Adafruit Ultimate GPS Breakout V3 + Adafruit's HMC5883L triple-axis magnometer + an LCD shield. 
 
 This uses code from the example code in Adafruit's libraries:
 https://github.com/adafruit/Adafruit_HMC5883_Unified
 https://github.com/adafruit/Adafruit-GPS-Library
 
 It also uses the Adafruit Sensor Library:
 https://github.com/adafruit/Adafruit_Sensor
 
*/

#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// set up the LCD shield
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // these pins match the OSSEP LCD shield I have
// set up the magnometer
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
// set up the GPS module
SoftwareSerial softSerialGPS(3, 2);
Adafruit_GPS GPS(&softSerialGPS);
#define GPSECHO false

// variables for the display
int locationNumber = 0;
int displayPageNumber = 0;

int isLogging = false;

// data for the locations, I'm sure there's a more elegant way to do this
const int numPlaces = 3;
String placeNames [numPlaces] = {
  "Indianapolis",
  "Julian",
};
double placeLoc [numPlaces][2] = {
  {
    39.768422, -86.158070  }
  ,
  {
    39.638679, -86.862767  }
};

boolean usingInterrupt = false;
void useInterrupt(boolean);

// degrees symbol for the LCD
byte degSymbol[8] = {
  0b01110,
  0b01010,
  0b01110,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};
byte recordingSymbol[8] = {
  0b00100,
  0b01110,
  0b00100,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

// buttons
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

int read_LCD_buttons(){
  adc_key_in = analogRead(0);
  if (adc_key_in > 1000) return btnNONE;
  if (adc_key_in < 50)   return btnRIGHT; 
  if (adc_key_in < 195)  return btnUP;
  if (adc_key_in < 380)  return btnDOWN;
  if (adc_key_in < 555)  return btnLEFT;
  if (adc_key_in < 790)  return btnSELECT;  
  return btnNONE;
}

void setup()  
{
  Serial.begin(115200);
  Serial.println("\nSampson, John \n GPS+Compass+LCD v0.5 ");
  /* GPS stuff, see documentation to figure out what this does */
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  delay(1000);
  softSerialGPS.println(PMTK_Q_RELEASE);

  // LCD setup, create the degree symbol and setup the LCD dimensions 
  lcd.createChar(0, degSymbol);
  lcd.createChar(1, recordingSymbol);
  lcd.begin(16, 2);

  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no magsensor detected ... Check your wiring!");
    while(1);
  }
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
  // writing direct to UDR0 is much much faster than Serial.print 
  // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

// the loop, where the fun stuff gets done
uint32_t timer = millis();
void loop()                     // run over and over again
{
  lcd_key = read_LCD_buttons();
  switch (lcd_key){
    case btnRIGHT: // change the page
      if(displayPageNumber<2) displayPageNumber++; 
      delay(500);
      break;
    case btnLEFT: // change the page
      if(displayPageNumber>0) displayPageNumber--;
      delay(500);
      break;
    case btnUP: // change the location
      if(locationNumber < numPlaces-1) locationNumber++;
      delay(500);
      break;
    case btnDOWN: // change the location
      if(locationNumber > 0) locationNumber--;
      delay(500);
      break;
    case btnSELECT: // start logging
      if(isLogging){
        Serial.print("STOPPING LOGGING....");
        if (GPS.LOCUS_StopLogger()){ 
          Serial.println(" STOPPED!"); 
          isLogging = false;
        } 
        else { 
          Serial.println(" no response :(");
        }
      } else {
        Serial.print("STARTING LOGGING....");
        if (GPS.LOCUS_StartLogger()){ 
          Serial.println(" STARTED!"); 
          isLogging = true;
        } 
        else { 
          Serial.println(" no response :(");
        }
      }
      delay(1000);
      break;
    case btnNONE:
      break;       
  }

  // GPS stuff
  sensors_event_t event; 
  mag.getEvent(&event);
  if (! usingInterrupt) {
    char c = GPS.read();
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {  
    timer = millis(); // reset the timer
    
    if (GPS.fix) {
      // calculate the current location in radians
      double currentLat_rads = GPS.latitudeDegrees * M_PI / 180; 
      double currentLon_rads = GPS.longitudeDegrees * M_PI / 180;
      
      // get the destiation location in rads and deg
      double destinationLat = placeLoc[locationNumber][0];
      double destinationLon = placeLoc[locationNumber][1];
      double destinationLat_rads = destinationLat * M_PI / 180; 
      double destinationLon_rads = destinationLon * M_PI / 180;

      // get the bearing
      double y = sin(destinationLon_rads-currentLon_rads) * cos(destinationLat_rads);
      double x = cos(currentLat_rads)*sin(destinationLat_rads) - sin(currentLat_rads)*cos(destinationLat_rads)*cos(destinationLon_rads-currentLon_rads);
      double bearingR =  atan2(y, x);
      double bearing = bearingR * 180/M_PI;
      bearing = fmod((bearing + 360), 360); 

      // get the distance using the haversine formula
      double r = 6372.8;
      double deltaLat = destinationLat_rads - currentLat_rads;
      double deltaLon = destinationLon_rads - currentLon_rads;
      double a = sin(deltaLat / 2) * sin(deltaLat / 2) + sin(deltaLon / 2) * sin(deltaLon / 2) * cos(currentLat_rads) * cos(destinationLat_rads);
      double c = 2 * asin(sqrt(a));
      double distance = r * c;
      distance = distance*0.621371;

      // HEADING       
      float heading = atan2(event.magnetic.y, event.magnetic.x);
      float declinationAngle = -0.0698131701;
      heading += declinationAngle;
      if(heading < 0) heading += 2*PI;
      if(heading > 2*PI) heading -= 2*PI;
      float headingDegrees = heading * 180/M_PI;

      // delta of heading and bearing 
      float deltaAngle = fmod((bearing - headingDegrees),360);


      // LCD output
      lcd.clear();
      // write the recording symbol if the GPS is logging
      lcd.setCursor(14, 0);
      if(isLogging) lcd.write(byte(1));
      
      lcd.setCursor(15, 0); 
      lcd.print(displayPageNumber);
      
      switch(displayPageNumber){
        case 0: // location name + distance + angle
          lcd.setCursor(0,0); 
          lcd.print(locationNumber+1); 
          lcd.setCursor(2, 0); 
          lcd.print(placeNames[locationNumber]);
          lcd.setCursor(0, 1); 
          lcd.print(distance,1); 
          lcd.print(" m");
          lcd.setCursor(9, 1); 
          lcd.print(deltaAngle); 
          lcd.write(byte(0));
          break;
        case 1: // location name + location coordinates 
          lcd.setCursor(0, 0); 
          lcd.print(locationNumber+1); 
          lcd.setCursor(2, 0); 
          lcd.print(placeNames[locationNumber] + " loc");
          lcd.setCursor(0, 1); 
          lcd.print(destinationLat, 4); 
          lcd.print(","); 
          lcd.print(destinationLon, 4);
          break;
        case 2: // current location coordinates
          lcd.setCursor(0, 0); 
          lcd.print("Current loc");
          lcd.setCursor(0, 1); 
          lcd.print(GPS.latitudeDegrees, 4); 
          lcd.print(","); 
          lcd.print(GPS.longitudeDegrees, 4);
          break;
      }  
    } 
    else { // if the GPS doesn't have a fix
      
      lcd.clear();
      
      lcd.setCursor(0, 0);
      lcd.print("Fix: "); 
      lcd.print((int)GPS.fix); 
      
      lcd.setCursor(0, 1);
      lcd.print("Quality: "); 
      lcd.print((int)GPS.fixquality);   
    }
  }
}

