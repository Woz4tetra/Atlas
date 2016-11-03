#include <Arduino.h>

#include <Wire.h>
#include <LIDARLite.h>
#include <SoftwareSerial.h>

SoftwareSerial softSerial(11, 12);  // Rx, Tx
bool paused = false;
char commandType = '\0';
String command = "";
char character = '\0';
unsigned long checkTime = millis();

LIDARLite myLidarLite;
int distance = 0;

#define RED_PIN 9
#define GREEN_PIN 8
#define BLUE_PIN 7

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
#define TICKS_PER_ROTATION 38

bool encoderLow = false;
unsigned long encoderCounts = 0;
unsigned long encoderRotations = 0;
unsigned long enc_t0 = micros();
unsigned long enc_t1 = micros();
unsigned long enc_dt = 0;

#define out_A_PWM 4
#define out_A_IN1 5
#define out_A_IN2 6

int motorSpeed = 10;
bool motorDirection = true;

void setColor(int r, int g, int b)
{
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
}

void setColor(int *rgb) {
    setColor(rgb[0], rgb[1], rgb[2]);
}

void writeData()
{
    // Write current reading from encoder and LIDAR to serial
    softSerial.print(encoderCounts);
    softSerial.print('\t');
    softSerial.print(encoderRotations);
    softSerial.print('\t');
    softSerial.print(distance);
    softSerial.print('\n');
}

bool checkEncoder()
{
    /*
     * Check if the encoder went from a low to high value. This indicates if
     * the encoder just passed through an encoder slit. Poll the LIDAR and
     * send the data over serial.
     *
     * Return true if an encoder tick is encountered.
     */
    if (analogRead(ENCODER_PIN) < ENCODER_LOW_VALUE && encoderLow)
    {  // count only when leaving an unblocked region
        enc_t1 = micros();
        enc_dt = enc_t1 - enc_t0;
        enc_t0 = enc_t1;
        distance = myLidarLite.distance();

        encoderLow = false;

        if (encoderCounts >= TICKS_PER_ROTATION) {  // 1 full rotation event
            encoderCounts = 0;
            encoderRotations++;
        }
        else {  // regular count
            encoderCounts++;
        }

        writeData();

        return true;
    }
    else if (analogRead(ENCODER_PIN) > ENCODER_HIGH_VALUE) {
        encoderLow = true;
    }
    return false;
}

void stopMotor() {
    analogWrite(out_A_PWM, 0);
}

void setMotorSpeed(boolean direction, int speed)
{ // speed = 0...100
    if (direction) {
        digitalWrite(out_A_IN1, HIGH);
        digitalWrite(out_A_IN2, LOW);
    }
    else {
        digitalWrite(out_A_IN1, LOW);
        digitalWrite(out_A_IN2, HIGH);
    }
    if (speed > 0) {
        analogWrite(out_A_PWM, map(speed, 1, 100, 127, 255));
    }
    else {
        stopMotor();
    }
}

void checkSerial()
{
    while (softSerial.available() && character != '\n')
    {
        character = softSerial.read();
        if (character != '\n') {
            command += character;
        }
    }

    if (character == '\n')
    {
        Serial.println(command);
        character = '\0';

        commandType = command.charAt(0);

        if (commandType == 'B') {  // start command
            paused = false;
            if (command.length() > 1) {
                motorSpeed = command.substring(1).toInt();
            }
            calibrate();
        }
        else if (commandType == 'E') {  // stop command
            paused = true;
            stopMotor();
        }
        else if (commandType == 'D') {  // change motor direction
            motorDirection = (bool)(command.substring(1).toInt());
            setMotorSpeed(motorDirection, motorSpeed);
        }
        else if (commandType == 'M') {  // change motor speed
            motorSpeed = command.substring(1).toInt();
            setMotorSpeed(motorDirection, motorSpeed);
        }
    }
}

void calibrate()
{
    int index = 0;
    unsigned long max_dt = 0;
    int max_index = 0;

    setMotorSpeed(motorDirection, motorSpeed);
    unsigned long time0 = millis();

    while ((millis() - time0) < 1000) {
        checkEncoder();
    }

    setColor(banana);

    while (index < TICKS_PER_ROTATION)
    {
        if (checkEncoder()) {
            if (enc_dt > max_dt) {
                max_dt = enc_dt;
                max_index = index;
            }
            index++;
        }
    }
    Serial.println(max_dt);
    Serial.println(max_index);
    setColor(red);

    index = 0;
    while (index < max_index) {
        if (checkEncoder()) {
            index++;
        }
    }

    setColor(green);

    encoderCounts = 0;
    encoderRotations = 0;

    stopMotor();
}

void setup()
{
    Serial.begin(115200);
    softSerial.begin(9600);

    myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
    myLidarLite.configure(0);  // default configuration

    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);

    pinMode(ENCODER_PIN, INPUT);

    pinMode(out_A_PWM,OUTPUT);
    pinMode(out_A_IN1,OUTPUT);
    pinMode(out_A_IN2,OUTPUT);

    encoderLow = analogRead(ENCODER_PIN) < ENCODER_HIGH_VALUE;

    setColor(seafoam);

    calibrate();
}

void loop()
{
    // if (!paused)
    // {
    //     checkEncoder();
    //
    //     if ((millis() - checkTime) > 1000)
    //     {
    //         // checkSerial();
    //         checkTime = millis();
    //     }
    // }
}
