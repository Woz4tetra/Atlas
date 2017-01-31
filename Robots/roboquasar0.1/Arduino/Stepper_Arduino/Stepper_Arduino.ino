#include <AccelStepper.h>

#define DEFAULT_RATE 115200
#define WHOIAM "steering"
#define LED13 13
#define DELIMITER_PIN 6

bool led_state = false;
bool paused = true;

unsigned long prev_time = 0;
unsigned long curr_time = 0;
unsigned long delta_time = 0;

#define MAX_SPEED 200
#define LEFT_LIMIT -150
#define RIGHT_LIMIT 150

int position = 0;
int speedCommand = 0;
bool prevIsRunning = false;

#define STEPPER_PIN_1 2
#define STEPPER_PIN_2 3
#define STEPPER_PIN_3 4
#define STEPPER_PIN_4 5

AccelStepper stepper(AccelStepper::FULL4WIRE,
    STEPPER_PIN_1,
    STEPPER_PIN_2,
    STEPPER_PIN_3,
    STEPPER_PIN_4
);

void writeWhoiam()
{
    Serial.print("iam");
    Serial.print(WHOIAM);
    Serial.print('\n');
}

void writeInit()
{
    Serial.print("init:");
    Serial.print(MAX_SPEED);
    Serial.print('\t');
    Serial.print(LEFT_LIMIT);
    Serial.print('\t');
    Serial.print(RIGHT_LIMIT);
    Serial.print('\n');
}

void setLed(bool state)
{
    led_state = state;
    digitalWrite(LED13, led_state);
}

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

void pause()
{
    Serial.print("stopping\n");
    paused = true;
    disengageStepper();

    prev_time = millis();
    curr_time = millis();
    delta_time = millis();
}

void unpause() {
    paused = false;
    calibrate();
}

void setPosition(int p) {
    stepper.setSpeed(MAX_SPEED);
    if (p > RIGHT_LIMIT) {
        p = RIGHT_LIMIT;
    }
    else if (p < LEFT_LIMIT) {
        p = LEFT_LIMIT;
    }
    stepper.moveTo(p);
    speedCommand = 0;
}

void setVelocity(int v)
{
    stepper.setSpeed(v);
    speedCommand = v;
}

void approachSwitch(int speed)
{
    stepper.setCurrentPosition(0);
    stepper.setSpeed(speed);
    while (digitalRead(DELIMITER_PIN)) {
        stepper.runSpeed();
    }
    stepper.setCurrentPosition(0);
}

void calibrate()
{
    Serial.print("calibrating\n");
    if (!digitalRead(DELIMITER_PIN)) {
        stepper.setSpeed(200);
        stepper.runToNewPosition(-50);
        stepper.setCurrentPosition(0);
    }
    approachSwitch(200);
    stepper.runToNewPosition(-20);

    approachSwitch(15);
    stepper.runToNewPosition(-150);
    stepper.setSpeed(200);
    stepper.setCurrentPosition(0);

    Serial.print("done!\n");
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
        else if (command.substring(0,1).equals("p"))
        {
            setPosition(command.substring(1).toInt());
        }
        else if (command.substring(0,1).equals("v"))
        {
            setVelocity(command.substring(1).toInt());
        }
    }
}

void setup()
{
    Serial.begin(DEFAULT_RATE);

    pinMode(DELIMITER_PIN, INPUT);

    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(1000.0);
}

void loop() {
    if (!paused)
    {
        if (speedCommand != 0) {
            stepper.runSpeed();

            if (stepper.currentPosition() > RIGHT_LIMIT) {
                setPosition(RIGHT_LIMIT);
                speedCommand = 0;
            }
            else if (stepper.currentPosition() < LEFT_LIMIT) {
                setPosition(LEFT_LIMIT);
                speedCommand = 0;
            }
        }
        else if (stepper.distanceToGo() != 0) {
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
            setVelocity(0);
        }
        prevIsRunning = stepper.isRunning();
    }
    else {
        delay(100);
    }

    readSerial();
}
