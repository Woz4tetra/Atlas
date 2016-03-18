#include <Servo.h>

Servo myservo;
String in_string = "";

int analog_min = 0;
int analog_max = 0;

int servo_min = 0;
int servo_max = 180;

unsigned int readout_counter = 0;

void calibrate(Servo servo, int analog_pin, int min_pos, int max_pos)
{
    Serial.println("calibrating...");
    servo.write(min_pos);
    delay(1000);
    analog_min = analogRead(analog_pin);
    
    servo.write(max_pos);
    delay(1000);
    analog_max = analogRead(analog_pin);
    
    myservo.write(min_pos);
    Serial.println("done!");
}

void setup()
{
    Serial.begin(9600);
    myservo.attach(9);  // attaches the servo on pin 9 to the servo object
    
    pinMode(A0, INPUT);
    calibrate(myservo, A0, servo_min, servo_max);
}

void loop()
{
    if (Serial.available()) {
        int in_char = Serial.read();
        if (isDigit(in_char)) {
            in_string += (char)in_char;
        }
        if (in_char == '\n')
        {
            Serial.print("value: ");
            Serial.println(in_string.toInt());
            myservo.write(in_string.toInt());
            in_string = "";
            readout_counter = 75;
        }
    }
    if (readout_counter > 0) {
        readout_counter -= 1;
    }
    if (readout_counter == 1) {
        Serial.print("analog: ");
        Serial.println(map(analogRead(A0), analog_min, analog_max,
                           servo_min, servo_max));
    }
    delay(15);
}