#include <Wire.h>
#include <LIDARLite.h>

#define DEFAULT_RATE 115200
#define WHOIAM "lidar"  // define whoiam ID here (unique to each robot object)
#define LED13 13
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

bool paused = true;
bool led_state = false;

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
        setLed(HIGH);
        return true;
    }
    if (!beamState && lastBeamState) {
        setLed(LOW);
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

void writeWhoiam()
{
    Serial.print("iam");
    Serial.print(WHOIAM);
    Serial.print('\n');
}

void setLed(bool state)
{
    led_state = state;
    digitalWrite(LED13, led_state);
}

void pause()
{
    analogWrite(MOTOR_PIN, 0);

    Serial.print("stopping\n");

    paused = true;
}

void unpause()
{
    analogWrite(MOTOR_PIN, 255);

    paused = false;
}

void readSerial()
{
    while (Serial.available())
    {
        String command = Serial.readStringUntil('\n');

        if (command.equals("whoareyou")) {
            writeWhoiam();
        }
        else if (command.equals("init?")) {
            writeInit();
        }
        else if (command.equals("start"))
        {
            setLed(HIGH);
            unpause();
        }
        else if (command.equals("stop"))
        {
            setLed(LOW);
            pause();
        }
    }

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

void writeInit()
{
    Serial.print("init:");
    Serial.print(12.5); // view size, using optical aperature, in mm
    Serial.print('\t');
    Serial.print(200); //  scan rate,  1–500Hz
    Serial.print('\t');
    Serial.print(15); // detection angle in degrees, no fucking cloo
    Serial.print('\t');
    Serial.print(400000); //  distance no detection, 40 m
    Serial.print('\n');
}

void setup()
{
    Serial.begin(DEFAULT_RATE);

    pinMode(TRIGGER, OUTPUT); // Set pin TRIGGER as trigger pin
    digitalWrite(MONITOR, LOW); // Set trigger LOW for continuous read

    pinMode(MONITOR, INPUT); // Set pin MONITOR as monitor pin

    delay(50);

    encoderInit();

    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(LED13, OUTPUT);
    pinMode(BREAKBEAM_PIN, INPUT);
    digitalWrite(BREAKBEAM_PIN, HIGH);  // turn on the pullup
}

void loop()
{
    if (!paused) {
        writeSerial();
    }
    else {
        delay(100);  // Minimize activity when not in use
    }

    readSerial();
}
