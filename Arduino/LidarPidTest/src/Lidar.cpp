
#include "Lidar.h"

#define ENCODER_PIN A3
#define ENCODER_HIGH_VALUE 800
#define ENCODER_LOW_VALUE 300
#define TICKS_PER_ROTATION 38

#define RED_PIN 9
#define GREEN_PIN 8
#define BLUE_PIN 7

#define out_A_PWM 10
#define out_A_IN1 6
#define out_A_IN2 5 

#define SOFTSERIAL_RX 11
#define SOFTSERIAL_TX 12



Lidar::Lidar()
{
    pinMode(ENCODER_PIN, INPUT);

    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);

    pinMode(out_A_PWM, OUTPUT);
    pinMode(out_A_IN1, OUTPUT);
    pinMode(out_A_IN2, OUTPUT);

    _softSerial = new SoftwareSerial(SOFTSERIAL_RX, SOFTSERIAL_TX);
    _lidarLite = new LIDARLite();

    _encoderCounts = 0;
    _encoderRotations = 0;
    _distance = 0;

    _paused = false;
    _commandType = '\0';
    _command = "";
    _character = '\0';

    initColors();
}

void Lidar::initColors()
{
    red[0] = 255;         red[1] = 0;           red[2] = 0;
    green[0] = 0;         green[1] = 255;       green[2] = 0;
    blue[0] = 0;          blue[1] = 0;          blue[2] = 255;
    orange[0] = 255;      orange[1] = 128;      orange[2] = 0;
    limeGreen[0] = 128;   limeGreen[1] = 255;   limeGreen[2] = 0;
    aqua[0] = 128;        aqua[1] = 255;        aqua[2] = 0;
    skyBlue[0] = 128;     skyBlue[1] = 255;     skyBlue[2] = 255;
    slateBlue[0] = 128;   slateBlue[1] = 128;   slateBlue[2] = 255;
    seafoam[0] = 0;       seafoam[1] = 255;     seafoam[2] = 128;
    banana[0] = 255;      banana[1] = 255;      banana[2] = 128;
    salmon[0] = 255;      salmon[1] = 102;      salmon[2] = 102;
}

void Lidar::begin()
{
    _lidarLite->begin(0, true);
    _lidarLite->configure(0);

    _encoderLow = analogRead(ENCODER_PIN) < ENCODER_HIGH_VALUE;

    _enc_t0 = micros();
    _enc_t1 = micros();
    _enc_dt = _enc_t1 - _enc_t0;

    calibrate();

    setMotorSpeed(100);
    setMotorSpeed(-100);
    setMotorSpeed(0);
}

unsigned int Lidar::getEncoderCounts() {
    return _encoderCounts;
}

unsigned int Lidar::getEncoderRotations() {
    return _encoderRotations;
}

void Lidar::setColor(int r, int g, int b)
{
    analogWrite(RED_PIN, r);
    analogWrite(GREEN_PIN, g);
    analogWrite(BLUE_PIN, b);
}

void Lidar::setColor(int *rgb) {
    setColor(rgb[0], rgb[1], rgb[2]);
}

void Lidar::writeToSerial()
{
    _softSerial->print(_encoderCounts);
    _softSerial->print('\t');
    _softSerial->print(_encoderRotations);
    _softSerial->print('\t');
    _softSerial->print(_distance);
    _softSerial->print('\n');
}

bool Lidar::update()
{
    /*
     * Check if the encoder went from a low to high value. This indicates if
     * the encoder just passed through an encoder slit. Poll the LIDAR and
     * send the data over serial.
     *
     * Return true if an encoder tick is encountered.
     */
    if (analogRead(ENCODER_PIN) < ENCODER_LOW_VALUE && _encoderLow)
    {
        // count only when leaving an unblocked region
        _enc_t1 = micros();
        _enc_dt = _enc_t1 - _enc_t0;
        _enc_t0 = _enc_t1;
        _distance = _lidarLite->distance();

        _encoderLow = false;

        if (_encoderCounts >= TICKS_PER_ROTATION) {  // 1 full rotation event
            _encoderCounts = 0;
            _encoderRotations++;
        }
        else {  // regular count
            _encoderCounts++;
        }

        writeToSerial();

        return true;
    }
    else if (analogRead(ENCODER_PIN) > ENCODER_HIGH_VALUE) {
        _encoderLow = true;
    }
    return false;
}


void Lidar::checkSerial()
{
    while (_softSerial->available() && _character != '\n')
    {
        _character = _softSerial->read();
        if (_character != '\n') {
            _command += _character;
        }
    }

    if (_character == '\n')
    {
        #ifdef DEBUG_LIDAR_TURRET
        Serial.println(_command);
        #endif
        _character = '\0';

        _commandType = _command.charAt(0);

        if (_commandType == 'B') {  // start command
            _paused = false;
            if (_command.length() > 1) {
                _motorSpeed = _command.substring(1).toInt();
            }
            calibrate();
        }
        else if (_commandType == 'E') {  // stop command
            _paused = true;
            setMotorSpeed(0);
        }
        else if (_commandType == 'D') {  // change motor direction
            if (_command.substring(1).toInt()) {
                _motorSpeed = -abs(_motorSpeed);
            }
            else {
                _motorSpeed = abs(_motorSpeed);
            }
            setMotorSpeed(_motorSpeed);
        }
        else if (_commandType == 'M') {  // change motor speed
            _motorSpeed = _command.substring(1).toInt();
            setMotorSpeed(_motorSpeed);
        }
    }
}

void Lidar::setMotorSpeed(int speed)
{ // speed = 0...100
    if (speed > 0) {
        digitalWrite(out_A_IN1, HIGH);
        digitalWrite(out_A_IN2, LOW);
    }
    else {
        digitalWrite(out_A_IN1, LOW);
        digitalWrite(out_A_IN2, HIGH);
    }

    analogWrite(out_A_PWM, map(abs(speed), 0, 100, 0, 255));
}

void Lidar::calibrate()
{

}
