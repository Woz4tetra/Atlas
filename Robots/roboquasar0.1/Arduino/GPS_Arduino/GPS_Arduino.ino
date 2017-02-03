#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define DEFAULT_RATE 9600 
// baud rate must be 9600 for this. the different baud rate is what causes shit
#define WHOIAM "gps"
#define LED13 13
#define GPSECHO true
#define GPS_UPDATE_RATE_HZ 5

boolean usingInterrupt = false;
unsigned long gps_update_delay = (unsigned long)(1.0 / ((float)(GPS_UPDATE_RATE_HZ)) * 1000);

SoftwareSerial gpsSerial(3, 2);
Adafruit_GPS GPS(&gpsSerial);

bool led_state = false;
bool paused = true;
uint32_t timer = millis();


void writeWhoiam()
{
    Serial.print("iam");
    Serial.print(WHOIAM);
    Serial.print('\n');
}

void writeInit()
{
    Serial.print("init:");
    Serial.print("delay:");
    Serial.print(gps_update_delay);
    Serial.print('\n');
}

void setLed(bool state)
{
    led_state = state;
    digitalWrite(LED13, led_state);
}

void pause()
{
    Serial.print("stopping\n");
    paused = true;
}

void unpause() {
    paused = false;
}

void readSerial()
{
    while (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        Serial.println(command);

        if (command.equals("whoareyou")) {
            writeWhoiam();
        }
        else if (command.equals("init?")) {
            writeInit();
        }
        else if (command.equals("start"))
        {
            setLed(HIGH);
            unpause();
        }
        else if (command.equals("stop"))
        {
            setLed(LOW);
            pause();
        }
    }
}

void updateGPS()
{
    if (!usingInterrupt)
    {
        // read data from the GPS in the 'main loop'
        char c = GPS.read();
        // if you want to debug, this is a good time to do it!
        if (GPSECHO)
        if (c) Serial.print(c);
    }
}
//// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
//SIGNAL(TIMER0_COMPA_vect) {
//    char c = GPS.read();
//    // if you want to debug, this is a good time to do it!
//}


void setup()
{
    Serial.begin(DEFAULT_RATE);
    GPS.begin(9600);

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

void loop()
{
    // put your main code here, to run repeatedly:
    if (!paused)
    {
        updateGPS();
    }
    else
    {
        delay(100);
    }
    readSerial();
}
