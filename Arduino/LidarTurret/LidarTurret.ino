
#include <Wire.h>
#include <LIDARLite.h>
#include <SoftwareSerial.h>
#include <TimerOne.h>

bool paused = false;

SoftwareSerial softSerial(10, 11);  // Rx, Tx
volatile bool shouldCheckSerial = false;
char character = '\0';
bool typeFound = false;
String commandType = "";
String command = "";

LIDARLite myLidarLite;
int distance = 0;

#define RED_PIN 11
#define GREEN_PIN 10
#define BLUE_PIN 9

int red[3] = {255, 0, 0};
int green[3] = {0, 255, 0};
int blue[3] = {0, 0, 255};

int orange[3] = {255, 128, 0};
int limeGreen[3] = {128, 255, 0};
int aqua[3] = {128, 255, 0};
int skyBlue[3] = {128, 255, 255};
int slateBlue[3] = {128, 128, 255};
int seafoam[3] = {0, 255, 128};
int banana[3] = {255, 255, 128};
int salmon[3] = {255, 102, 102};

#define ENCODER_PIN A3
#define ENCODER_HIGH_VALUE 800
#define ENCODER_LOW_VALUE 300
unsigned long longThreshold = 15000;  // ~7300 for small tick, ~21000 for large

bool encoderLow = false;
unsigned long encoderCounts = 0;
unsigned long encoderRotations = 0;
unsigned long encPrevTime = micros();
unsigned long encCurrTime = micros();
unsigned long dt = 10000000;

#define out_A_PWM 4
#define out_A_IN1 5
#define out_A_IN2 6
int motorSpeed = 255;
bool motorDirection = false;

void setColor(int r, int g, int b)
{
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
}

void setColor(int *rgb) {
    setColor(rgb[0], rgb[1], rgb[2]);
}

bool checkEncoder()
{
    if (analogRead(ENCODER_PIN) < ENCODER_LOW_VALUE && encoderLow)  // count only when leaving an unblocked region
    {
        distance = myLidarLite.distance();

        encoderLow = false;
        encCurrTime = micros();
        dt = encCurrTime - encPrevTime;

        encPrevTime = encCurrTime;

        if (dt > longThreshold) {  // 1 full rotation event
            encoderCounts = 0;
            encoderRotations++;

            // Serial.print('\t');
            // Serial.print(encoderRotations);
            // Serial.print('\t');
            // Serial.println(dt);
        }
        else {  // regular count
            encoderCounts++;
            // Serial.print(encoderCounts);
            // Serial.print('\t');
            // Serial.println(dt);
        }

        writeData();

        return true;
    }
    else if (analogRead(ENCODER_PIN) > ENCODER_HIGH_VALUE) {
        encoderLow = true;
        return false;
    }
}

unsigned int find_enc_long_thresh()
{
    setMotorSpeed(false, 255);
    delay(100);

    checkEncoder();

    unsigned long min_dt = dt;
    unsigned long max_dt = 0;

    unsigned long start_time = millis();

    while ((millis() - start_time) < 100) {
        checkEncoder();
    }

    while ((millis() - start_time) < 3000)
    {
        if (checkEncoder())
        {
            if (dt < min_dt) {
                min_dt = dt;
            }

            if (dt > max_dt) {
                max_dt = dt;
            }
        }
    }

    longThreshold = (max_dt + min_dt) / 2;
    encoderCounts = 0;
    encoderRotations = 0;
}

void setMotorSpeed(boolean direction, int speed)
{ //speed from 0 to 255
    if (direction) {
        digitalWrite(out_A_IN1,HIGH);
        digitalWrite(out_A_IN2,LOW);
    }
    else {
        digitalWrite(out_A_IN1,LOW);
        digitalWrite(out_A_IN2,HIGH);
    }
    analogWrite(out_A_PWM, speed);
}

void readCommand()
{
    if (softSerial.available())
    {
        typeFound = false;
        commandType = "";
        command = "";
        character = '\0';

        while (character != '\n' && softSerial.available()) {
            character = softSerial.read();
            Serial.println(character);
            Serial.println(character, DEC);
            if (character == '\t') {
                typeFound = true;
                continue;
            }

            if (!typeFound) {
                commandType += character;
            }
            else {
                command += character;
            }
        }

        if (character == '\n') {
            if (commandType.equals("M"))
            {
                motorDirection = (bool)(command.substring(0, 1).toInt());
                motorSpeed = command.substring(1).toInt();

                if (0 <= motorSpeed && motorSpeed <= 255) {
                    setMotorSpeed(motorDirection, motorSpeed);
                }
            }
            else if (commandType.equals("P")) {
                paused = (bool)(command.toInt());
                if (paused) {
                    setMotorSpeed(motorDirection, 0);
                }
                else {
                    setMotorSpeed(motorDirection, motorSpeed);
                }
            }
        }
    }
}

void shouldRead() {
    shouldCheckSerial = true;
}

void initSerial()
{
    // check serial every 0.25 seconds
    Timer1.initialize(250000);
    Timer1.attachInterrupt(shouldRead);

    softSerial.begin(9600);
}


void writeData()
{
    softSerial.print(encoderCounts);
    softSerial.print('\t');
    softSerial.print(encoderRotations);
    softSerial.print('\t');
    softSerial.print(distance);
    softSerial.print('\n');
}

void setup()
{
    Serial.begin(115200);
    myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
    myLidarLite.configure(0);  // default configuration

    // pinMode(RED_PIN, OUTPUT);
    // pinMode(GREEN_PIN, OUTPUT);
    // pinMode(BLUE_PIN, OUTPUT);

    pinMode(ENCODER_PIN, INPUT);

    pinMode(out_A_PWM,OUTPUT);
    pinMode(out_A_IN1,OUTPUT);
    pinMode(out_A_IN2,OUTPUT);

    encoderLow = analogRead(ENCODER_PIN) < ENCODER_HIGH_VALUE;

    initSerial();

    find_enc_long_thresh();

    setMotorSpeed(motorDirection, motorSpeed);
}

unsigned long time0 = millis();

void loop()
{
    if (!paused) {
        checkEncoder();
    }

    if (shouldCheckSerial) {
        shouldCheckSerial = false;
        readCommand();
    }
}
