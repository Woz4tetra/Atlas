
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
#define ENCODER_HIGH_VALUE 800
#define ENCODER_LOW_VALUE 300
unsigned long enc_long_count_thresh = 15000;  // ~7300 for small tick, ~21000 for large

bool encoder_low = false;
unsigned long encoder_counts = 0;
unsigned long encoder_rotations = 0;
unsigned long enc_prev_time = micros();
unsigned long enc_curr_time = micros();
unsigned long dt = 10000000;

#define out_A_PWM 4
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

bool check_encoder()
{
    if (analogRead(ENCODER_PIN) < ENCODER_LOW_VALUE && encoder_low)  // count only when leaving an unblocked region
    {
        encoder_low = false;
        enc_curr_time = micros();
        dt = enc_curr_time - enc_prev_time;

        enc_prev_time = enc_curr_time;

        if (dt > enc_long_count_thresh) {  // 1 full rotation event
            encoder_counts += 3;
            encoder_rotations++;
        }
        else {  // regular count
            encoder_counts++;
        }

        return true;
    }
    else if (analogRead(ENCODER_PIN) > ENCODER_HIGH_VALUE) {
        encoder_low = true;
        return false;
    }
}

unsigned int find_enc_long_thresh()
{
    motor_speed(false, 255);
    delay(100);

    check_encoder();

    unsigned long min_dt = dt;
    unsigned long max_dt = 0;

    unsigned long start_time = millis();

    while ((millis() - start_time) < 100) {
        check_encoder();
    }

    while ((millis() - start_time) < 3000)
    {
        if (check_encoder())
        {
            if (dt < min_dt) {
                min_dt = dt;
            }

            if (dt > max_dt) {
                max_dt = dt;
            }
        }
    }

    enc_long_count_thresh = (max_dt + min_dt) / 2;
    encoder_counts = 0;
    encoder_rotations = 0;
}

void motor_speed(boolean direction, int speed)
{ //speed from 0 to 255
    if (direction) {
        digitalWrite(out_A_IN1, HIGH);
        digitalWrite(out_A_IN2, LOW);
    }
    else {
        digitalWrite(out_A_IN1, LOW);
        digitalWrite(out_A_IN2, HIGH);
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

    pinMode(out_A_PWM,OUTPUT);
    pinMode(out_A_IN1,OUTPUT);
    pinMode(out_A_IN2,OUTPUT);

    encoder_low = analogRead(ENCODER_PIN) < ENCODER_HIGH_VALUE;

    find_enc_long_thresh();

    motor_speed(false, 255);
}

unsigned long time0 = millis();

void loop()
{
    if (check_encoder() && (millis() - time0) > 500)
    {
        time0 = millis();

        Serial.print(myLidarLite.distance());
        Serial.print("\t");

        Serial.print(encoder_counts);
        Serial.print("\t");

        Serial.print(encoder_rotations);
        Serial.print("\t");

        Serial.print((int)(dt));
        Serial.print('\n');
    }
}
