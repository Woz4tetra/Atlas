//
//  MyGPS.cpp
//  NewSerialTest
//
//  Created by Benjamin Warwick on 11/18/15.
//
//

#include "MyGPS.h"

#define mySerial Serial1

Adafruit_GPS GPS(&mySerial);

const int gps_float_array_len = 4;
float gps_float_array[gps_float_array_len];

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

#ifdef __AVR__
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

void useInterrupt(boolean v)
{
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
#endif


void gps_setup()
{
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    mySerial.begin(9600);
    
    // Enable RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);
    
    
#ifdef __arm__
    usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
    useInterrupt(true);
#endif
    
    delay(1000);
}

void gps_update(uint8_t *input_array)
{
    // in case you are not using the interrupt above, you'll
    // need to 'hand query' the GPS, not suggested :(
    if (!usingInterrupt) {
        // read data from the GPS in the 'main loop'
        GPS.read();
    }
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences! 
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
        
//        if (!GPS.parse(GPS.lastNMEA())) {  // this also sets the newNMEAreceived() flag to false
//            return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    
    gps_float_array[0] = GPS.latitude;
    gps_float_array[1] = GPS.longitude;
    gps_float_array[2] = GPS.speed;
    gps_float_array[3] = GPS.angle;
    
    input_array[16] = (uint8_t)GPS.satellites;
    input_array[17] = (uint8_t)GPS.fixquality;
    
    for (uint8_t float_index = 0; float_index < gps_float_array_len; float_index++) {
        to_hex(gps_float_array[float_index], input_array, float_index);
    }
}




