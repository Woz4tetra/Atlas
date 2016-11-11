#include <Lidar.h>

Lidar lidar;
unsigned long t0 = micros();

void setup()
{
    Serial.begin(9600);
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
