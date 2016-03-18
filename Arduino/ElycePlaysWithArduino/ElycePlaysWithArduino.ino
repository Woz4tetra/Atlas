#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Servo servo_thing;
Adafruit_BNO055 bno = Adafruit_BNO055();
char incoming = '\0';

// Serial.available()
// char incoming = '\0';
// incoming = Serial.read();

void setup()
{
    // put your setup code here, to run once:
    servo_thing.attach(9);
    Serial.begin(9600);
    if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
}

void loop()
{
    // put your main code here, to run repeatedly:
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    // int thing = 180;
    // servo_thing.write(thing);
    if (Serial.available() > 0) {
        incoming = Serial.read();
        if (incoming == 'i') {
            servo_thing.write(0);
        }
        else if (incoming == 'o') {
            servo_thing.write(180);
        }
    }


    Serial.println(map(analogRead(A0), 109, 472, 0, 180));
    Serial.print("X: ");
    Serial.print(euler.x());
    Serial.print(" Y: ");
    Serial.print(euler.y());
    Serial.print(" Z: ");
    Serial.print(euler.z());
    Serial.print("\t\t");
    // thing = 0;
    //
    // servo_thing.write(thing);
    // delay(1000);
    // Serial.println(map(analogRead(A0), 109, 472, 0, 180));
    // Serial.print("X: ");
    // Serial.print(euler.x());
    // Serial.print(" Y: ");
    // Serial.print(euler.y());
    // Serial.print(" Z: ");
    // Serial.print(euler.z());
    // Serial.print("\t\t");

}
