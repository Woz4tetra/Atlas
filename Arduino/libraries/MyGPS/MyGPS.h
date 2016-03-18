//
//  MyGPS.hpp
//  NewSerialTest
//
//  Created by Benjamin Warwick on 11/18/15.
//
//

#include <Adafruit_GPS.h>
#include <Utilities.h>

#ifndef MyGPS_h
#define MyGPS_h

const int gps_array_len = 18;
// = 4 * 8 / 2, 4 float numbers, 2 hex digits for every uint8
// + 2 for fix and satelittes


void gps_setup();
void gps_update(uint8_t *input_array);

#endif /* MyGPS_h */
