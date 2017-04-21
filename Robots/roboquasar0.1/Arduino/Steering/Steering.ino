#include <AccelStepper.h>
#include <Atlasbuggy.h>

#define DELIMITER_PIN 6

unsigned long prev_time = 0;
unsigned long curr_time = 0;
unsigned long delta_time = 0;

#define MAX_SPEED 200
#define RIGHT_LIMIT -150
#define LEFT_LIMIT 150
#define POSITION_ZERO 120
//135
#define CENTER_OFFSET 50

int position = 0;
int speedCommand = 0;
bool prevIsRunning = false;

#define STEPPER_PIN_1 3
#define STEPPER_PIN_2 2
#define STEPPER_PIN_3 4
#define STEPPER_PIN_4 5

AccelStepper stepper(AccelStepper::FULL4WIRE,
    STEPPER_PIN_1,
    STEPPER_PIN_2,
    STEPPER_PIN_3,
    STEPPER_PIN_4
);

Atlasbuggy buggy("steering");

unsigned long dt()
{
    curr_time = millis();
    if (prev_time < curr_time) {
        delta_time = (0xffffffffUL - prev_time) + curr_time;
    }
    else {
        delta_time = curr_time - prev_time;
    }

    return delta_time;
}

void setTime() {
    prev_time = curr_time;
}

void disengageStepper()
{
    digitalWrite(STEPPER_PIN_1, LOW);
    digitalWrite(STEPPER_PIN_2, LOW);
    digitalWrite(STEPPER_PIN_3, LOW);
    digitalWrite(STEPPER_PIN_4, LOW);
}

void setPosition(int p) {
    if (stepper.distanceToGo() == 0)
    {
        if (p > LEFT_LIMIT) {
            p = LEFT_LIMIT;
        }
        else if (p < RIGHT_LIMIT) {
            p = RIGHT_LIMIT;
        }
        stepper.moveTo(p);
        Serial.print('g');
        Serial.print(p);
        Serial.print('\n');
        speedCommand = 0;
    }
}

void setVelocity(int v)
{
    if (v == 0) {
        stepper.stop();
        stepper.setSpeed(MAX_SPEED);
    }
    else {
        stepper.setSpeed(v);
    }
    speedCommand = v;
}

bool approachSwitch(int speed)
{
    stepper.setCurrentPosition(0);
    stepper.setSpeed(speed);
    int goalPosition = -1000;
    while (digitalRead(DELIMITER_PIN))
    {
        stepper.run();
        if (stepper.distanceToGo() == 0)
        {
            goalPosition -= 1000;
            stepper.moveTo(goalPosition);
        }
        while (buggy.available())
        {
            int status = buggy.readSerial();
            if (status == 1) {  // stop event
                return false;
            }
        }
    }
    stepper.stop();
    delay(10);
    stepper.setCurrentPosition(0);

    return true;
}

void calibrate()
{
    Serial.print("calibrating\n");
    if (!digitalRead(DELIMITER_PIN)) {
        stepper.setSpeed(-MAX_SPEED);
        stepper.runToNewPosition(50);
        stepper.setCurrentPosition(0);
    }
    bool status = approachSwitch(-MAX_SPEED);
    if (!status)
    {
        disengageStepper();
        return;
    }
    stepper.runToNewPosition(CENTER_OFFSET);

    status = approachSwitch(-MAX_SPEED / 8);
    if (!status)
    {
        disengageStepper();
        return;
    }
    stepper.runToNewPosition(POSITION_ZERO);
    stepper.setSpeed(MAX_SPEED);
    stepper.setCurrentPosition(0);

    Serial.print("done!\n");
}

void setup()
{
    buggy.begin();

    pinMode(DELIMITER_PIN, INPUT);

    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(2000.0);

    String leftLimitStr = String(RIGHT_LIMIT);
    String rightLimitStr = String(LEFT_LIMIT);
    String maxSpeedStr = String(MAX_SPEED);

    buggy.setInitData(maxSpeedStr + "\t" + leftLimitStr + "\t" + rightLimitStr);
}

void loop()
{
    while (buggy.available())
    {
        int status = buggy.readSerial();
        if (status == 2) {  // start event
           calibrate();
        }
        else if (status == 1) {  // stop event
            disengageStepper();
            prev_time = millis();
            curr_time = millis();
            delta_time = millis();
        }
        if (status == 0) {  // user command
            String command = buggy.getCommand();
            if (command.substring(0, 1).equals("p")) {
                setPosition(command.substring(1).toInt());
            }
            else if (command.substring(0, 1).equals("v")) {
                setVelocity(command.substring(1).toInt());
            }
            else if (command.equals("c")) {
                calibrate();
            }
        }
        // else if (status == -1)  // no command
    }

    if (!buggy.isPaused())
    {
        if (speedCommand != 0) {
            stepper.runSpeed();

            // if (stepper.currentPosition() > LEFT_LIMIT) {
            //     setPosition(LEFT_LIMIT);
            //     speedCommand = 0;
            // }
            // else if (stepper.currentPosition() < RIGHT_LIMIT) {
            //     setPosition(RIGHT_LIMIT);
            //     speedCommand = 0;
            // }
        }
        // else if (stepper.distanceToGo() != 0) {
        else {
            stepper.run();
        }

        if (stepper.isRunning() && dt() > 50)
        {
            Serial.print('p');
            Serial.print(stepper.currentPosition());
            Serial.print('\n');
            setTime();
        }
        else if (!stepper.isRunning() && prevIsRunning != stepper.isRunning())
        {
            Serial.print('d');
            Serial.print(stepper.currentPosition());
            Serial.print('\n');
            // setVelocity(0);
        }
        prevIsRunning = stepper.isRunning();
    }
    else {
        delay(100);
    }
}
