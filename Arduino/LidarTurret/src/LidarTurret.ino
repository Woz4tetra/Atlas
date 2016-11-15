// #define DEBUG_LIDAR_TURRET
// #define USE_SOFTSERIAL

#include <Lidar.h>

Lidar lidar;
unsigned long t0 = micros();

void setup()
{
    #ifdef USE_SOFTSERIAL
    Serial.begin(9600);
    #endif
    lidar.begin();
}

void loop()
{
    lidar.update();
    lidar.checkSerial();

    if ((micros() - t0) > 1000) {
        lidar.writeDistance();
        t0 = micros();
    }
}
