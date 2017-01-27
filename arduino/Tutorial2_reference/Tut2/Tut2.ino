
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define DEFAULT_RATE 115200
#define WHOIAM "gps"  // define whoiam ID here (unique to each robot object)
#define LED13 13

String command = "";
bool led_state = false;
bool paused = true;

#define GPS_BAUD_RATE 9600
#define GPS_UPDATE_RATE_HZ 1
unsigned long gps_update_delay = (unsigned long)(1.0 / ((float)(GPS_UPDATE_RATE_HZ)) * 1000);

SoftwareSerial gpsSerial(3, 2);

Adafruit_GPS GPS(&gpsSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

uint32_t timer = millis();

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
    } else {
        // do not call the interrupt function COMPA anymore
        TIMSK0 &= ~_BV(OCIE0A);
        usingInterrupt = false;
    }
}

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
        command = Serial.readStringUntil('\n');

        // if (character == '\n')
        // {
        // Serial.println(command);
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
    // in case you are not using the interrupt above, you'll
    // need to 'hand query' the GPS, not suggested :(
    if (!usingInterrupt) {
        // read data from the GPS in the 'main loop'
        char c = GPS.read();
        // if you want to debug, this is a good time to do it!
        if (GPSECHO)
        if (c) Serial.print(c);
    }

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

        if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }

    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();

    // approximately every (gps_update_delay) seconds or so, print out the current stats
    if (millis() - timer > gps_update_delay)
    {
        timer = millis(); // reset the timer

        Serial.print("Time: ");
        Serial.print(GPS.hour, DEC); Serial.print(':');
        Serial.print(GPS.minute, DEC); Serial.print(':');
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        Serial.print(GPS.milliseconds);
        Serial.print("\tDate: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.print(GPS.year, DEC);
        Serial.print("\tFix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.print((int)GPS.fixquality);
        if (GPS.fix) {
            Serial.print("\tLocation: ");
            Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
            Serial.print(", ");
            Serial.print(GPS.longitude, 4); Serial.print(GPS.lon);
            Serial.print("\tLocation degrees: ");
            Serial.print(GPS.latitudeDegrees, 4);
            Serial.print(", ");
            Serial.print(GPS.longitudeDegrees, 4);

            Serial.print("\tSpeed (knots): "); Serial.print(GPS.speed);
            Serial.print("\tAngle: "); Serial.print(GPS.angle);
            Serial.print("\tAltitude: "); Serial.print(GPS.altitude);
            Serial.print("\tSatellites: "); Serial.print((int)GPS.satellites);
        }
        Serial.print('\n');
    }
}

void setup() {
    // connect at DEFAULT_RATE so we can read the GPS fast enough and echo without dropping chars
    // also spit it out
    Serial.begin(DEFAULT_RATE);
    // Serial.println("Adafruit GPS library basic test!");

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(GPS_BAUD_RATE);

    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time

    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz

    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    // the nice thing about this code is you can have a timer0 interrupt go off
    // every 1 millisecond, and read data from the GPS for you. that makes the
    // loop code a heck of a lot easier!
    useInterrupt(true);

    delay(1000);
    // Ask for firmware version
    gpsSerial.println(PMTK_Q_RELEASE);

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

void loop() {
    if (!paused) {
        updateGPS();
    }
    else {
        delay(100);  // Minimize activity when not in use
    }

    readSerial();
}
