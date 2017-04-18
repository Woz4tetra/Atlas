/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->	http://www.adafruit.com/products/1438

This sketch creates a fun motor party on your desk *whiirrr*
Connect a unipolar/bipolar stepper to M3/M4
Connect a DC motor to M1
Connect a hobby servo to SERVO1
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);


// And connect a DC motor to port M1
Adafruit_DCMotor *motor_1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor_2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor_3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor_4 = AFMS.getMotor(4);

// We'll also test out the built in Arduino Servo library
Servo servo1;
Servo servo2;


void setup() {
    Serial.begin(9600);           // set up Serial library at 9600 bps
    Serial.println("MMMMotor party!");

    AFMS.begin();  // create with the default frequency 1.6KHz
    //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

    // Attach a servo to pin #10
    servo1.attach(10);
    servo2.attach(9);

    // turn on motor M1
    motor_1->setSpeed(200);
    motor_1->run(RELEASE);

    motor_2->setSpeed(200);
    motor_2->run(RELEASE);

    motor_3->setSpeed(200);
    motor_3->run(RELEASE);

    motor_4->setSpeed(200);
    motor_4->run(RELEASE);
}

int i;
void loop() {
    motor_1->run(FORWARD);
    motor_2->run(FORWARD);
    motor_3->run(FORWARD);
    motor_4->run(FORWARD);
    Serial.println("Forward");
    for (i=0; i<255; i++) {
        servo1.write(map(i, 0, 255, 0, 180));
        servo2.write(map(i, 0, 255, 0, 180));
        motor_1->setSpeed(i);
        motor_2->setSpeed(i);
        motor_3->setSpeed(i);
        motor_4->setSpeed(i);
        delay(3);
    }

    for (i=255; i!=0; i--) {
        servo1.write(map(i, 0, 255, 0, 180));
        servo2.write(map(i, 0, 255, 0, 180));
        motor_1->setSpeed(i);
        motor_2->setSpeed(i);
        motor_3->setSpeed(i);
        motor_4->setSpeed(i);
        delay(3);
    }

    motor_1->run(BACKWARD);
    motor_2->run(BACKWARD);
    motor_3->run(BACKWARD);
    motor_4->run(BACKWARD);
    Serial.println("Backward");
    for (i=0; i<255; i++) {
        servo1.write(map(i, 0, 255, 0, 180));
        servo2.write(map(i, 0, 255, 0, 180));
        motor_1->setSpeed(i);
        motor_2->setSpeed(i);
        motor_3->setSpeed(i);
        motor_4->setSpeed(i);
        delay(3);
    }

    for (i=255; i!=0; i--) {
        servo1.write(map(i, 0, 255, 0, 180));
        servo2.write(map(i, 0, 255, 0, 180));
        motor_1->setSpeed(i);
        motor_2->setSpeed(i);
        motor_3->setSpeed(i);
        motor_4->setSpeed(i);
        delay(3);
    }
}
