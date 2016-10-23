
#include <Wire.h>
#include <LIDARLite.h>

LIDARLite myLidarLite;

#define MOTOR_PIN 2
bool motor_value = false;

long long time0 = millis();

#define RED_PIN 11
#define GREEN_PIN 10
#define BLUE_PIN 9

int counter = 0;
int red_counter = 0;
int green_counter = 0;
int blue_counter = 0;

#define ENCODER_PIN A3

#define out_A_PWM 3
#define out_A_IN1 5
#define out_A_IN2 6

void setColor(int red, int green, int blue)
{
    analogWrite(RED_PIN, red);
    analogWrite(GREEN_PIN, green);
    analogWrite(BLUE_PIN, blue);
}

void setup()
{
    Serial.begin(115200); // Initialize serial connection to display distance readings
    myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
    myLidarLite.configure(0); // Change this number to try out alternate configurations

    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, motor_value);

    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);

    pinMode(ENCODER_PIN, INPUT);

    pinMode(out_A_PWM,OUTPUT);
    pinMode(out_A_IN1,OUTPUT);
    pinMode(out_A_IN2,OUTPUT);
}

void motor_speed(boolean direction, int speed)
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

void loop()
{
    // Take a measurement with receiver bias correction and print to serial terminal
    Serial.print(myLidarLite.distance());
    Serial.print(", ");
    Serial.print(analogRead(ENCODER_PIN));

    if ((millis() - time0) > 1000)
    {
        motor_value = !motor_value;
        digitalWrite(MOTOR_PIN, motor_value);
    }

    setColor(red_counter, green_counter, blue_counter);
    if (counter == 0) {
        red_counter += 1;
        green_counter += 1;
        if (red_counter > 255) {
            counter += 1;
            red_counter = 0;
        }
        motor_speed(true, red_counter);
    }
    else if (counter == 1) {
        green_counter += 1;
        blue_counter += 1;
        if (green_counter > 255) {
            counter += 1;
            green_counter = 0;
        }
        motor_speed(false, green_counter);
    }
    else if (counter == 2) {
        blue_counter += 1;
        red_counter += 1;
        if (blue_counter > 255) {
            counter = 0;
            blue_counter = 0;
        }
    }

    Serial.print('\n');
}
