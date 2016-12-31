
#include "Lidar.h"

#define ENCODER_HIGH_VALUE 800
#define ENCODER_LOW_VALUE 300
#define TICKS_PER_ROTATION 38

#define RED_PIN 11
#define GREEN_PIN 8
#define BLUE_PIN 7

#define ENCODER_PIN A3

#define out_A_PWM 13
#define out_A_IN1 6
#define out_A_IN2 5

#define SOFTSERIAL_RX 9
#define SOFTSERIAL_TX 10

#define DEFAULT_MOTOR_SPEED 55

#define MIN_MOTOR_SPEED 35
#define MAX_MOTOR_SPEED 100

#define LIDAR_WHO_I_AM "lidar"

#define SAFE_ENC_DT 7750

// Extremely fast string to int function
char _int2str[7];
char* int2str(register int i) {
    register unsigned char L = 1;
    register char c;
    register boolean m = false;
    register char b;  // lower-byte of i
    // negative
    if ( i < 0 ) {
        _int2str[ 0 ] = '-';
        i = -i;
    }
    else L = 0;
    // ten-thousands
    if( i > 9999 ) {
        c = i < 20000 ? 1
        : i < 30000 ? 2
        : 3;
        _int2str[ L++ ] = c + 48;
        i -= c * 10000;
        m = true;
    }
    // thousands
    if( i > 999 ) {
        c = i < 5000
        ? ( i < 3000
            ? ( i < 2000 ? 1 : 2 )
            :   i < 4000 ? 3 : 4
        )
        : i < 8000
        ? ( i < 6000
            ? 5
            : i < 7000 ? 6 : 7
        )
        : i < 9000 ? 8 : 9;
        _int2str[ L++ ] = c + 48;
        i -= c * 1000;
        m = true;
    }
    else if( m ) _int2str[ L++ ] = '0';
    // hundreds
    if( i > 99 ) {
        c = i < 500
        ? ( i < 300
            ? ( i < 200 ? 1 : 2 )
            :   i < 400 ? 3 : 4
        )
        : i < 800
        ? ( i < 600
            ? 5
            : i < 700 ? 6 : 7
        )
        : i < 900 ? 8 : 9;
        _int2str[ L++ ] = c + 48;
        i -= c * 100;
        m = true;
    }
    else if( m ) _int2str[ L++ ] = '0';
    // decades (check on lower byte to optimize code)
    b = char( i );
    if( b > 9 ) {
        c = b < 50
        ? ( b < 30
            ? ( b < 20 ? 1 : 2 )
            :   b < 40 ? 3 : 4
        )
        : b < 80
        ? ( i < 60
            ? 5
            : i < 70 ? 6 : 7
        )
        : i < 90 ? 8 : 9;
        _int2str[ L++ ] = c + 48;
        b -= c * 10;
        m = true;
    }
    else if( m ) _int2str[ L++ ] = '0';
    // last digit
    _int2str[ L++ ] = b + 48;
    // null terminator
    _int2str[ L ] = 0;
    return _int2str;
}

Lidar::Lidar()
{
    pinMode(ENCODER_PIN, INPUT);

    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);

    pinMode(out_A_PWM, OUTPUT);
    pinMode(out_A_IN1, OUTPUT);
    pinMode(out_A_IN2, OUTPUT);

    #ifdef USE_SOFTSERIAL
    _serial = new SoftwareSerial(SOFTSERIAL_RX, SOFTSERIAL_TX);
    #endif

    _lidarLite = new LIDARLite();

    _encoderCounts = 0;
    _encoderRotations = 0;
    _distance = 0;

    _motorSpeed = 0;
    _motorDirection = true;

    _paused = false;
    _commandType = '\0';
    _command = "";
    _character = '\0';

    _kp = 1.0;
    _kd = 15.0;
    _ki = 0.0;

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
    #ifdef USE_SOFTSERIAL
    _serial->begin(115200);
    #endif

    #ifdef USE_SOFTSERIAL
    Serial.begin(115200);
    #endif

    _lidarLite->begin(1, true);

    _encoderLow = analogRead(ENCODER_PIN) < ENCODER_HIGH_VALUE;

    _enc_t0 = micros();
    _enc_t1 = micros();
    _enc_dt = _enc_t1 - _enc_t0;

    calibrate();

    _serial_t0 = millis();
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

void Lidar::writeEncoder()
{
    #ifdef USE_SOFTSERIAL
    _serial->print(int2str(_encoderCounts));
    _serial->print('\t');
    _serial->print(int2str(_encoderRotations));
    _serial->print('\n');
    #endif

    #ifndef USE_SOFTSERIAL
    Serial.print(int2str(_encoderCounts));
    Serial.print('\t');
    Serial.print(int2str(_encoderRotations));
    Serial.print('\n');
    #endif
}

void Lidar::writeDistance()
{
    _distance = _lidarLite->distance();

    #ifdef USE_SOFTSERIAL
    _serial->printprint(int2str(_distance));
    _serial->printprint('\n');
    #endif

    #ifndef USE_SOFTSERIAL
    Serial.print(int2str(_distance));
    Serial.print('\n');
    #endif
}

void Lidar::updateEncoder()
{
    // count only when leaving an unblocked region
    _enc_t1 = micros();
    _enc_dt = _enc_t1 - _enc_t0;
    _enc_t0 = _enc_t1;

    if (_motorDirection)
    {
        if (_encoderCounts >= TICKS_PER_ROTATION - 1) {  // 1 full rotation event
            _encoderCounts = 0;
            _encoderRotations++;
        }
        else {  // regular count
            _encoderCounts++;
        }
    }
    else
    {
        if (_encoderCounts <= 0) {  // 1 full rotation event in reverse
            _encoderCounts = TICKS_PER_ROTATION - 1;
            _encoderRotations--;
        }
        else {  // regular count in reverse
            _encoderCounts--;
        }
    }

    int error = (int)(_enc_dt) - SAFE_ENC_DT;
    int speed = (int)(0.02 * error);

    setMotorSpeed(speed + DEFAULT_MOTOR_SPEED);
}

bool Lidar::checkEncoder(int encoderValue)
{
    if (!_paused)
    {
        // int encoderValue = analogRead(ENCODER_PIN);
        if (encoderValue < ENCODER_LOW_VALUE && _encoderLow)
        {
            _encoderLow = false;
            return true;
        }
        else if (encoderValue > ENCODER_HIGH_VALUE) {
            _encoderLow = true;
        }
    }
    return false;
}

bool Lidar::update(int encoderValue)
{
    /*
     * Check if the encoder went from a low to high value. This indicates if
     * the encoder just passed through an encoder slit. Poll the LIDAR and
     * send the data over serial.
     *
     * Return true if an encoder tick is encountered.
     */

    if (checkEncoder(encoderValue)) {
        updateEncoder();

        writeDistance();
        writeEncoder();

        return true;
    }
    return false;
}

bool Lidar::updateNoSerial()
{
    if (checkEncoder(analogRead(ENCODER_PIN))) {
        updateEncoder();

        return true;
    }
    return false;
}

bool Lidar::update() {
    return update(analogRead(ENCODER_PIN));
}

void Lidar::checkSerial()
{
    #ifdef USE_SOFTSERIAL
    if ((millis() - _serial_t0) > 1000 && _serial->available())
    #endif

    #ifndef USE_SOFTSERIAL
    if ((millis() - _serial_t0) > 1000 && Serial.available())
    #endif
    {
        #ifdef USE_SOFTSERIAL
        while (_serial->available() && _character != '\n')
        #endif

        #ifndef USE_SOFTSERIAL
        while (Serial.available() && _character != '\n')
        #endif
        {
            #ifdef USE_SOFTSERIAL
            _character = _serial->read();
            #endif

            #ifndef USE_SOFTSERIAL
            _character = Serial.read();
            #endif

            if (_character != '\n') {
                _command += _character;
            }
        }

        if (_character == '\n')
        {
            _character = '\0';

            _commandType = _command.charAt(0);

            if (_command.equals("whoareyou")) {
                writeWhoIAm();
            }
            else if (_command.equals("ready?")) {
                Serial.print("ready!\n");
                if (_paused) {
                    _paused = false;
                    calibrate();
                }
            }
            else if (_command.equals("stop")) {
                if (!_paused) {
                    setColor(orange);
                    _paused = true;
                    stopMotor();
                }
            }

            // if (_commandType == 'D') {  // change motor direction
            //     setMotorDirection((bool)(_command.substring(1).toInt()));
            // }
            // else if (_commandType == 'M') {  // change motor speed
            //     setMotorSpeed(_command.substring(1).toInt());
            // }
        }

        _command = "";

        _serial_t0 = millis();
    }
}

void Lidar::writeWhoIAm()
{
    #ifdef USE_SOFTSERIAL
    _serial->print("iam");
    _serial->print(LIDAR_WHO_I_AM);
    _serial->print('\n');
    #endif

    #ifndef USE_SOFTSERIAL
    Serial.print("iam");
    Serial.print(LIDAR_WHO_I_AM);
    Serial.print('\n');
    #endif

}

void Lidar::setMotorDirection(bool direction)
{ // speed = 0...100
    _motorDirection = direction;
    if (_motorDirection) {
        digitalWrite(out_A_IN1, LOW);
        digitalWrite(out_A_IN2, HIGH);
    }
    else {
        digitalWrite(out_A_IN1, HIGH);
        digitalWrite(out_A_IN2, LOW);
    }
}

void Lidar::setMotorSpeed(int speed)
{ // speed = 0...100
    setMotorDirection(speed > 0);

    speed = abs(speed);
    if (speed < MIN_MOTOR_SPEED && speed != 0) {
        speed = MIN_MOTOR_SPEED;
    }
    if (speed > MAX_MOTOR_SPEED) {
        speed = MAX_MOTOR_SPEED;
    }

    speed = map(speed, 0, 100, 0, 255);
    analogWrite(out_A_PWM, speed);

    _motorSpeed = speed;
}

void Lidar::stopMotor()
{
    _motorSpeed = 0;
    analogWrite(out_A_PWM, _motorSpeed);
}

void Lidar::goToTick(int goal)
{
    int error = 0;
    int speed = 0;
    int prev_error = 0;
    int sum_error = 0;

    while (_encoderCounts != goal)
    {
        if (updateNoSerial())
        {
            // if (counts >= TICKS_PER_ROTATION - 1) {
            //     counts = 0;
            // }
            // goal += counts * TICKS_PER_ROTATION;
            error = goal - _encoderCounts;
            speed = (int)(_kp * error + _kd * (error - prev_error) + _ki * sum_error);

            Serial.print(_encoderCounts); Serial.print(',');
            Serial.print(error); Serial.print(',');
            Serial.print(speed); Serial.print('\n');

            setMotorSpeed(speed);
            prev_error = error;
            sum_error += error;
        }
    }

    stopMotor();
}

void Lidar::calibrate()
{
    setColor(red);/*
    delay(2000);

    // setMotorSpeed(MAX_MOTOR_SPEED);
    // delay(125);
    // setMotorSpeed(DEFAULT_MOTOR_SPEED);
    // delay(500);
    //
    // unsigned long max_dt = 0;
    // unsigned long min_dt = 100000;
    // // int max_tick = 0;
    //
    // while (!update()) {  }  // Wait for first encoder tick
    //
    // // setColor(blue);
    //
    // _encoderCounts = 0;
    // _encoderRotations = 0;
    // while (_encoderRotations < 1)
    // {
    //     if (update())
    //     {
    //         Serial.print("thing: "); Serial.print(_encoderCounts); Serial.print(',');
    //         Serial.println(_enc_dt);
    //         if (_enc_dt > max_dt) {
    //             max_dt = _enc_dt;
    //             // max_tick = _encoderCounts;
    //         }
    //         if (_enc_dt < min_dt) {
    //             min_dt = _enc_dt;
    //         }
    //     }
    // }
    //
    // unsigned long avg_dt = (max_dt + max_dt + min_dt) / 3;
    //
    // while (_enc_dt < avg_dt) {
    //     update();
    // }
    // setColor(green);

    setMotorSpeed(DEFAULT_MOTOR_SPEED);
    goToTick(TICKS_PER_ROTATION);

    setColor(blue);

    stopMotor();
    unsigned long t0 = millis();
    while ((millis() - t0) < 1000) {
        updateNoSerial();
    }

    goToTick(0);
    setColor(green);

    while (true) {  }

    // setColor(orange);

    // goToTick(_encoderCounts, TICKS_PER_ROTATION - 5);


    _encoderCounts = 0;
    _encoderRotations = 0;*/

    setMotorSpeed(DEFAULT_MOTOR_SPEED);

    setColor(green);
}
