#include <Wire.h>
#include <LIDARLite.h>

#define DEFAULT_RATE 115200
#define WHOIAM "lidarsensor"  // define whoiam ID here (unique to each robot object)
#define LED13 13

LIDARLite myLidarLite;

unsigned long prev_time = 0;
unsigned long curr_time = 0;
unsigned long delta_time = 0;

char character = '\0';
char command_type = '\0';
String command = "";

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
    digitalWrite(LED13, LOW);
    Serial.print("stopping\n");

    paused = true;
}

void unpause()
{
    curr_time = micros();
    prev_time = curr_time;
    digitalWrite(LED13, HIGH);
    paused = false;
}

unsigned long dt()
{
    curr_time = micros();
    if (prev_time < curr_time) {
        delta_time = (0xffffffffUL - prev_time) + curr_time;
    }
    else {
        delta_time = curr_time - prev_time;
    }
    prev_time = curr_time;

    return delta_time;
}

void readSerial()
{
    while (Serial.available() && character != '\n')
    {
        character = Serial.read();
        if (character != '\n') {
            command += character;
        }
    }

    if (character == '\n')
    {
        command_type = command.charAt(0);
        if (command.equals("whoareyou")) {
            writeWhoiam();
        }
        else if (command.equals("init?")) {
            writeInit();
        }
        else if (command.equals("start")) {
            unpause();
        }
        else if (command.equals("stop")) {
            pause();
        }

        character = '\0';
        command = "";
    }
}

void writeSerial()
{
    Serial.print(int2str(myLidarLite.distance()));  // distance in CM
    // Serial.print('\t');
    // Serial.print(dt());
    Serial.print('\n');
}

void writeInit()
{
    Serial.print("init:");
    Serial.print('\n');
}

void setup()
{
    Serial.begin(DEFAULT_RATE);

    myLidarLite.begin(0, true);
    myLidarLite.configure(0);

    pinMode(LED13, OUTPUT);
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
