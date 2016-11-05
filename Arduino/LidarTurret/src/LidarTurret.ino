#include <Lidar.h>

Lidar lidar;

void setup()
{
    Serial.begin(9600);
    lidar.begin();
}

void loop()
{
    lidar.update();
    lidar.checkSerial();
    // lidar.writeDistance();
}
