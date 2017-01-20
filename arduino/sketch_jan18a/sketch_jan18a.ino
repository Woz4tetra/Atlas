#include <Wire.h>
#include <LIDARLite.h>

#define MOTOR_PIN 3

#define ENCODER_PIN_1 2
#define ENCODER_PIN_2 4
#define MACRO_ENCODER_PIN A0

byte encoderPin1Last;
int numPulses;
boolean direction;

LIDARLite myLidarLite;

char character = '\0';
String command = "";
int motor_speed = 0;

void encoderInit()
{
    direction = true;  //default -> Forward
    pinMode(ENCODER_PIN_2, INPUT);
    pinMode(MACRO_ENCODER_PIN, INPUT);
    attachInterrupt(0, wheelSpeed, CHANGE);//int.0
}

void wheelSpeed()
{
    int state = digitalRead(ENCODER_PIN_1);
    if ((encoderPin1Last == LOW) && state == HIGH)
    {
        int val = digitalRead(ENCODER_PIN_2);
        if (val == LOW && direction) {
            direction = false;  // Reverse
        }
        else if (val == HIGH && !direction)
        {
            direction = true;  // Forward
        }
    }
    encoderPin1Last = state;

    if (!direction)  numPulses++;
    else  numPulses--;
}

void setup() {
    Serial.begin(115200);

    myLidarLite.begin(0, true);
    myLidarLite.configure(0);

    encoderInit();

    pinMode(MOTOR_PIN, OUTPUT);
}

void loop() {
    Serial.print(myLidarLite.distance());
    Serial.print('\t');
    Serial.print(analogRead(MACRO_ENCODER_PIN));
    Serial.print('\t');
    Serial.println(numPulses);
    numPulses = 0;
    // myLidarLite.distance();
    while (Serial.available() && character != '\n')
    {
        character = Serial.read();
        if (character != '\n') {
            command += character;
        }
    }

    if (character == '\n') {
        motor_speed = command.toInt();
        Serial.println(motor_speed);
        analogWrite(MOTOR_PIN, motor_speed);
        command = "";
        character = '\0';
    }
    delay(5);
}
