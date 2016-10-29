
#include <Wire.h>
#include <LIDARLite.h>

LIDARLite myLidarLite;

#define RED_PIN 11
#define GREEN_PIN 10
#define BLUE_PIN 9

int red[3] = {255, 0, 0};
int green[3] = {0, 255, 0};
int blue[3] = {0, 0, 255};

int orange[3] = {255, 128, 0};
int lime_green[3] = {128, 255, 0};
int aqua[3] = {128, 255, 0};
int sky_blue[3] = {128, 255, 255};
int slate_blue[3] = {128, 128, 255};
int seafoam[3] = {0, 255, 128};
int banana[3] = {255, 255, 128};
int salmon[3] = {255, 102, 102};

#define ENCODER_PIN A3
#define ENCODER_HIGH_VALUE 900
#define ENCODER_LOW_VALUE 100
#define ENC_LONG_COUNT_THRESH 100

bool encoder_low = false;
unsigned long encoder_counts = 0;
unsigned long encoder_rotations = 0;
unsigned long  enc_prev_time = micros();
unsigned long  enc_curr_time = micros();

#define out_A_PWM 3
#define out_A_IN1 5
#define out_A_IN2 6

void setColor(int red, int green, int blue)
{
    analogWrite(RED_PIN, red);
    analogWrite(GREEN_PIN, green);
    analogWrite(BLUE_PIN, blue);
}

void setColor(int *rgb) {
    setColor(rgb[0], rgb[1], rgb[2]);
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

void setup()
{
    Serial.begin(115200); // Initialize serial connection to display distance readings
    myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
    myLidarLite.configure(0); // Change this number to try out alternate configurations

    // pinMode(RED_PIN, OUTPUT);
    // pinMode(GREEN_PIN, OUTPUT);
    // pinMode(BLUE_PIN, OUTPUT);

    pinMode(ENCODER_PIN, INPUT);

    // pinMode(out_A_PWM,OUTPUT);
    // pinMode(out_A_IN1,OUTPUT);
    // pinMode(out_A_IN2,OUTPUT);

    // encoder_low = analogRead(ENCODER_PIN) < ENCODER_HIGH_VALUE;
    //
    // motor_speed(false, 255);
}

void loop()
{
    Serial.println(myLidarLite.distance());
    // if (analogRead(ENCODER_PIN) < ENCODER_LOW_VALUE && encoder_low)  // count only when leaving an unblocked region
    // {
    //     encoder_low = false;
    //
    //     // Take a measurement with receiver bias correction and print to serial terminal
    //     Serial.print(myLidarLite.distance());
    //     Serial.print(", ");
    //
    //     Serial.print(encoder_counts);
    //     Serial.print(", ");
    //
    //     enc_curr_time = micros();
    //     Serial.print((int)(enc_curr_time - enc_prev_time));
    //     Serial.print('\n');
    //
    //     enc_prev_time = enc_curr_time;
    //
    //     if ((enc_curr_time - enc_prev_time) > ENC_LONG_COUNT_THRESH) {  // 1 full rotation event
    //         encoder_counts += 3;
    //         encoder_rotations++;
    //     }
    //     else {  // regular count
    //         encoder_counts++;
    //     }
    // }
    // else if (analogRead(ENCODER_PIN) > ENCODER_HIGH_VALUE) {
    //     encoder_low = true;
    // }
}
