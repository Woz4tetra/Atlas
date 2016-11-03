#include <Lidar.h>

 Lidar lidar;

void motorSpinTest()
{
    for (int speed = 0; speed <= 255; speed++) {
        lidar.setMotorSpeed(speed);
        Serial.println(speed);
        delay(5);
    }

    for (int speed = 255; speed >= 0; speed--) {
        lidar.setMotorSpeed(speed);
        Serial.println(speed);
        delay(5);
    }

    for (int speed = 0; speed <= 255; speed++) {
        lidar.setMotorSpeed(-speed);
        Serial.println(speed);
        delay(5);
    }

    for (int speed = 255; speed >= 0; speed--) {
        lidar.setMotorSpeed(-speed);
        Serial.println(speed);
        delay(5);
    }
}

int minSpeed = 0;
int maxSpeed = 100;
int motorSpeed = 0;
float kp = 25.0;
float kd = 0.0;
float ki = 0.0;

float prev_error = 0.0;
float sum_error = 0.0;

void reset_pid()
{
    prev_error = 0;
    sum_error = 0;
}

bool motorPid(int encoder, int goal)
{
    int error = goal - encoder;
    motorSpeed = (int)(kp * error + kd * (error - prev_error) + ki * sum_error);
    prev_error = error;
    sum_error += error;

    if (abs(motorSpeed) < minSpeed) {
        motorSpeed = (int)(minSpeed * (motorSpeed > 0) - (motorSpeed < 0));
    }
    else if (abs(motorSpeed) > maxSpeed) {
        motorSpeed = (int)(maxSpeed * (motorSpeed > 0) - (motorSpeed < 0));
    }

    lidar.setMotorSpeed(motorSpeed);

    return error == 0;
}

void setup()
{
    Serial.begin(9600);
    lidar.begin();
    delay(3000);

    motorSpinTest();

    lidar.setMotorSpeed(0);
    lidar.setColor(lidar.red);
}

void loop()
{
    // lidar.update();
    // if (motorPid(lidar.getEncoderCounts(), 20)) {
    //     lidar.setColor(lidar.limeGreen);
    // }
}
