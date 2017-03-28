// to change the position it stops at, change variables goalPosition1 and goalPosition2
// the difference of 200 is approximately the length necessary
// press b in serial monitor to extend the actuator
// press n in serial monitor to retract the actuator
// depending on which direction you mount the actuator, exchange the value of the two variables
#include <Atlasbuggy.h>

#define forwardPin 6
#define backwardPin 5
#define potHighPin 7
#define potReadPin A0

#define BRAKE_POS 90
#define RELEASE_POS 120

#define MAX_SPEED 255

bool braking = false;
bool moving = true;
bool pause_ping = true;
int goal = RELEASE_POS;

uint32_t timer = millis();
uint32_t pingTimer = millis();
int prevPosition = 0;

Atlasbuggy buggy("brakes");

void setup() {
    buggy.begin();

    pinMode(potReadPin, INPUT);

    pinMode(forwardPin, OUTPUT);
    pinMode(backwardPin, OUTPUT);
    pinMode(potHighPin, OUTPUT);

    stopMotor();

    digitalWrite(potHighPin, HIGH);

    prevPosition = currentPosition();

    String brake_pos = String(BRAKE_POS);
    String release_pos = String(RELEASE_POS);
    buggy.setInitData(brake_pos + "\t" + release_pos);
}

int currentPosition() {
    return analogRead(potReadPin);
}

void motorForward() {
    analogWrite(backwardPin, 0);
    analogWrite(forwardPin, MAX_SPEED);
}

void motorBackward() {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, MAX_SPEED);
}

void motorForward(int speed) {
    analogWrite(backwardPin, 0);
    analogWrite(forwardPin, abs(speed));
}

void motorBackward(int speed) {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, abs(speed));
}

void stopMotor() {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, 0);
}

void loop() {
    while (buggy.available()) {
        int status = buggy.readSerial();
        if (status == 2) {  // start event
            pingTimer = millis();
            Serial.print('s');
            Serial.print(currentPosition());
            Serial.print('\n');

            timer = millis();
            prevPosition = currentPosition();
            stopMotor();
            moving = false;
            delay(1500);
            pingTimer = millis();
        }
        if (status == 0) {  // user command
            if (buggy.getCommand().equals("b")) {
                goal = BRAKE_POS;
                braking = true;
                moving = true;
            }
            else if (buggy.getCommand().equals("r")) {
                goal = RELEASE_POS;
                braking = false;
                moving = true;
            }
            else if (buggy.getCommand().equals("t")) {
                pingTimer = millis();
            }
            else if (buggy.getCommand().equals("p")) {
                pause_ping = true;
            }
            else if (buggy.getCommand().equals("u")) {
                pause_ping = false;
                pingTimer = millis();
            }
            else {
                stopMotor();
                moving = false;
            }
        }
        // else if (status == -1)  // no command
    }

    if (!buggy.isPaused() && !pause_ping) {
        // if millis() or timer wraps around, we'll just reset it
        if (pingTimer > millis())  pingTimer = millis();

        if ((millis() - pingTimer) > 1000) {
            goal = BRAKE_POS;
            braking = true;
            moving = true;
            buggy.pause();
        }
    }

    int current = currentPosition();
    if (abs(current - prevPosition) > 20) {
        moving = true;
        current = currentPosition();
    }
    if (moving && ((braking && current < BRAKE_POS) || (!braking && current > RELEASE_POS))) {
        stopMotor();
        moving = false;
        Serial.print('s');
        Serial.print(current);
        Serial.print('\n');
    }

    if (moving) {
       if (goal < currentPosition()) {
           motorBackward();
       }
       else {
           motorForward();
       }
    }


    if (timer > millis())  timer = millis();

    if (millis() - timer > 100) {
        timer = millis(); // reset the timer

        if (abs(current - prevPosition) > 2) {
            Serial.print('m');
            Serial.print(current);
            Serial.print('\n');
            prevPosition = current;
        }
    }
}
