#include <Wire.h>
#include <LIDARLite.h>
#include <Atlasbuggy.h>

#define MOTOR_PIN 3

#define ENCODER_PIN_1 2
#define ENCODER_PIN_2 4

#define BREAKBEAM_PIN 6
unsigned long numRotations = 0;
int beamState = 0, lastBeamState = 0;

byte encoderPin1Last;
int numPulses = 0;
boolean direction;
long encCounts = 0;

unsigned long pulseWidth;
#define TRIGGER 8
#define MONITOR 9

Atlasbuggy buggy("lidar");

void encoderInit()
{
    direction = true;  //default -> Forward
    pinMode(ENCODER_PIN_2, INPUT);
    attachInterrupt(0, wheelSpeed, CHANGE);//int.0
}

bool checkMacroEnc()
{
    beamState = digitalRead(BREAKBEAM_PIN);
    if (beamState && !lastBeamState) {
        lastBeamState = beamState;
        buggy.setLed(HIGH);
        return true;
    }
    if (!beamState && lastBeamState) {
        buggy.setLed(LOW);
    }
    lastBeamState = beamState;
    return false;
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
        else if (val == HIGH && !direction) {
            direction = true;  // Forward
        }
    }
    encoderPin1Last = state;

    if (!direction)  numPulses++;
    else  numPulses--;
}

void startMotor() {
    analogWrite(MOTOR_PIN, 255);
}

void stopMotor() {
    analogWrite(MOTOR_PIN, 0);
}

void writeSerial()
{
    if (checkMacroEnc()) {
        Serial.print('r');
        numRotations++;
        Serial.print(numRotations);
        Serial.print('\n');
        encCounts = 0;
    }

    pulseWidth = pulseIn(MONITOR, HIGH);
    if (pulseWidth != 0)
    {
        // ticks, distance (tab seperated)
        Serial.print('l');
        noInterrupts();
        Serial.print(numPulses);
        encCounts += numPulses;
        numPulses = 0;
        interrupts();

        Serial.print('\t');
        Serial.print(pulseWidth);  // distance in CM

        Serial.print('\n');
    }
}


void setup()
{
    buggy.begin();

    pinMode(TRIGGER, OUTPUT); // Set pin TRIGGER as trigger pin
    digitalWrite(MONITOR, LOW); // Set trigger LOW for continuous read

    pinMode(MONITOR, INPUT); // Set pin MONITOR as monitor pin

    delay(50);

    encoderInit();

    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(BREAKBEAM_PIN, INPUT);
    digitalWrite(BREAKBEAM_PIN, HIGH);  // turn on the pullup
}

void loop()
{
    while (buggy.available()) {
        int status = buggy.readSerial();
        if (status == 2) {
            startMotor();
        }
        else if (status == 1) {
            stopMotor();
        }
    }

    if (!buggy.isPaused()) {
        writeSerial();
    }
}
