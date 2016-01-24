//
//  Utilities.cpp
//  RoboQuasar1.0_Arduino
//
//  Created by Benjamin Warwick on 11/27/15.
//  Copyright © 2015 Benjamin Warwick. All rights reserved.
//

#include "Utilities.h"


void to_hex(float input, uint8_t *array, int start)
{
    byte* bytearray = (byte*) &input;
    short float_length = 4;
    start *= float_length;
    
    for (int index = float_length - 1; index >= 0; index--) {
        array[((float_length - 1) - index) + start] = bytearray[index];
    }
}



