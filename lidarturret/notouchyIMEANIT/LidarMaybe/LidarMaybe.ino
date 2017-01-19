#include <Wire.h>
#include <LIDARLite.h>

#define DEFAULT_RATE 115200
#define WHOIAM "no_fuck_YOU"  // define whoiam ID here (unique to each robot object)
#define LED13 13
#define MOTOR_PIN 3

#define ENCODER_PIN_1 2
#define ENCODER_PIN_2 4

byte encoderPin1Last;
int numPulses;
boolean direction;

char character = '\0';
char command_type = '\0';
String command = "";

bool paused = true;

unsigned long long t0_us = 0;
unsigned long long t0_ms = 0;
bool led_state = false;

void encoderInit()
{
    direction = true;  //default -> Forward
    pinMode(ENCODER_PIN_2,INPUT);
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

void write_whoiam()
{
    Serial.print("iam");
    Serial.print(WHOIAM);
    Serial.print('\n');
}

void set_led(bool state) 
{
    led_state = state;
    digitalWrite(LED13, led_state);
}

void readSerial()
{
    while (Serial.available() && character != '\n')
    {
        character = Serial.read();
        if (character != '\n') 
        {
            command += character;
        }
    }

    if (character == '\n')
    {
        // command_type = command.charAt(0);
        if (command.equals("whoareyou")) 
        {
            write_whoiam();
        }
        else if (command.equals("init?"))
        {
            digitalWrite(LED13, HIGH);
            Serial.print("init:");

            writeInit();

            Serial.print("\n");
        }
        else if (command.equals("stop"))
        {
            digitalWrite(LED13, LOW);
            paused = true;
        }
        else if (command.equals("start"))
        {
            digitalWrite(LED13, HIGH);
            paused = false;

            t0_us = micros();
            t0_ms = millis();
        }
       /* {
            Serial.print(command);
            Serial.print(" is not a valid command");
            Serial.print('\n');
        }
        // Put other commands here
       /* else if (command_type == 'l')
        {
            switch(command.substring(1).toInt()) {
                case 0: set_led(LOW); break;
                case 1: set_led(HIGH); break;
                case 2: set_led(!led_state); break;
            }
        }*/

        character = '\0';
        command = "";
    }
}

void writeSerial()
{
   // ticks, distance (tab seperated)
    Serial.print((int)(numPulses));
    Serial.print('\t');
    Serial.print((int)(myLidarLite.distance());
    Serial.print('\n');
    numPulses = 0;
 }

void writeInit()
{
    Serial.print((int)(12.5); // view size, using optical aperature, in mm
    Serial.print('\t');
    Serial.print((int)(200); //  scan rate,  1–500Hz
    Serial.print('\t');
    Serial.print((int)(15); // detection angle in degrees, no fucking cloo
    Serial.print('\t');
    Serial.print((int)(400000); //  distance no detection, 40 m
    Serial.print('\n');
}

void setup()
{
    Serial.begin(DEFAULT_RATE);

    myLidarLite.begin(0, true);
    myLidarLite.configure(0);

    encoderInit();

    
    pinMode(LED13, OUTPUT);
    // Initialize arduino peripherals
}

void loop()
{
    if (!paused) 
    {
        writeSerial();
        if (character == '\n') 
        {
            motor_speed = command.toInt();
            Serial.println(motor_speed);
            analogWrite(MOTOR_PIN, motor_speed);
            command = "";
            character = '\0';
        }
    }
    else {
        delay(5);  // Minimize activity when not in use
    }

    readSerial();
}

    
