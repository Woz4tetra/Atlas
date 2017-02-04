#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3,2);
Adafruit_GPS GPS(&mySerial);

#define DEFAULT_RATE 115200
#define GPS_RATE 9600
#define WHOIAM "gps"
#define LED13 13
#define GPS_UPDATE_RATE_HZ 5 // 1, 5, 10


uint32_t timer = millis();
boolean usingInterrupt = false;
bool paused = false;

void setup()
{

  Serial.begin(115200);

  // 9600 default ,also 4800
  GPS.begin(GPS_RATE);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  if (GPS_UPDATE_RATE_HZ == 1) {
        GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
        GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
    }
    else if (GPS_UPDATE_RATE_HZ == 5) {
        GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
        GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    }
    else if (GPS_UPDATE_RATE_HZ == 10) {
        GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
        GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
        GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    }

}


void check_gps()
{
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", ");
      Serial.println(GPS.longitudeDegrees, 4);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
}
  
void readSerial()
{
    while (Serial.available())
    {
        GPS.pause(true); 
        String command = Serial.readStringUntil('\n');
        Serial.println(command);
        GPS.pause(false);
    }
}
void loop()
{

//    if (!paused)
//    {
//        
//      char c = GPS.read();
//      if (GPS.newNMEAreceived()) 
//      {
//    
//        if (!GPS.parse(GPS.lastNMEA()))   
//          return;
//
//        // approximately every 2 seconds or so, print out the current stats
//        //if (millis() - timer > 2000) {
//        //timer = millis(); // reset the timer
//        //check_gps();
//        //}
//     else
//      {
//        delay(100);
//      }
    readSerial();
   // }
  //}
}

