// to change the position it stops at, change variables goalPosition1 and goalPosition2
// the difference of 200 is approximately the length necessary
// press b in serial monitor to extend the actuator
// press n in serial monitor to retract the actuator
// depending on which direction you mount the actuator, exchange the value of the two variables
#include <Atlasbuggy.h>
#include <PID_v1.h>

#define POS_ERROR 2
#define POT_OFFSET 28.0
#define MIN_OUTPUT 200

#define relay1Pin 7
#define relay2Pin 8
#define potHighPin 12
#define potReadPin 0
#define pwmPin 6

#define BRAKE_POS 175
#define RELEASE_POS 225

double goalPosition;
double currentPosition;
double pidOutput;

double kp = 2.0;
double kd = 1.0;
double ki = 0.0;

bool extending = false;
bool retracting = false;

bool commandIssued = false;
bool innerPause = false;
bool prevPause = false;
uint32_t timer = millis();

PID linearAc(&currentPosition, &pidOutput, &goalPosition, kp, kd, ki, DIRECT);

Atlasbuggy buggy("brakes");

void setup() {
    buggy.begin();

    pinMode(potReadPin, INPUT);

    pinMode(relay1Pin, OUTPUT);
    pinMode(relay2Pin, OUTPUT);
    pinMode(potHighPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);

    digitalWrite(relay1Pin, LOW);
    digitalWrite(relay2Pin, LOW);

    digitalWrite(potHighPin, HIGH);

    currentPosition = analogRead(potReadPin) - POT_OFFSET;
    goalPosition = currentPosition;

    linearAc.SetMode(AUTOMATIC);

    String pos_error = String(POS_ERROR);
    String brake_pos = String(BRAKE_POS);
    String release_pos = String(RELEASE_POS);
    buggy.setInitData(pos_error + "\t" + brake_pos + "\t" + release_pos);
}

void control() {

    // read the value from the sensor:
    currentPosition = (double)(analogRead(potReadPin)) - POT_OFFSET;

    if (abs(goalPosition - currentPosition) < POS_ERROR) {
        innerPause = true;
    }
    else{
        innerPause = false;
    }
    if (!prevPause && innerPause) {
        commandIssued = false;
        Serial.print((int)(currentPosition));
        Serial.print('\n');
    }
    prevPause = innerPause;

    if (commandIssued) {
        linearAc.Compute();
        if (abs(pidOutput) < MIN_OUTPUT) {
            pidOutput = MIN_OUTPUT * (pidOutput > 0) - (pidOutput < 0);
        }

        if (goalPosition < currentPosition) {
            retracting = true;
            extending = false;
            digitalWrite(relay1Pin, LOW);
            digitalWrite(relay2Pin, HIGH);
            analogWrite(pwmPin, 255 - pidOutput);
            // Serial.println(255 - pidOutput);
        }
        else if (goalPosition > currentPosition) {
            retracting = false;
            extending = true;
            digitalWrite(relay1Pin, HIGH);
            digitalWrite(relay2Pin, LOW);
            analogWrite(pwmPin, pidOutput);
            // Serial.println(pidOutput);
        }

        if (timer > millis())  timer = millis();

        if (millis() - timer > 100) {
            timer = millis(); // reset the timer
            // Serial.print(pidOutput);
            // Serial.print('\t');
            // Serial.print((int)(goalPosition));
            // Serial.print('\t');
            Serial.print((int)(currentPosition));
            Serial.print('\n');
        }
    }

    if (extending && goalPosition - currentPosition < POS_ERROR) {
        //we have reached our goal, shut the relay off
        digitalWrite(relay1Pin, LOW);
        extending = false;
    }

    if (retracting && currentPosition - goalPosition < POS_ERROR) {
        //we have reached our goal, shut the relay off
        digitalWrite(relay2Pin, LOW);
        retracting = false;
    }
}

void loop() {
    while (buggy.available()) {
        int status = buggy.readSerial();
        if (status == 2) {  // start event
            Serial.print((int)(currentPosition));
            Serial.print('\n');
        }
        // else if (status == 1) {  // stop event
        //
        // }
        if (status == 0) {  // user command
            commandIssued = true;

            String command = buggy.getCommand();
            if (command.substring(0, 1).equals("b")) {
                goalPosition = BRAKE_POS;
            }
            else if (command.substring(0, 1).equals("r")) {
                goalPosition = RELEASE_POS;
            }
            else {
                goalPosition = (double)(command.toInt());
                // Serial.print("goalPosition: ");
                // Serial.println(goalPosition);
            }

            if (goalPosition < BRAKE_POS) {
                goalPosition = BRAKE_POS;
            }
            // else if (goalPosition > RELEASE_POS) {
            //    goalPosition = RELEASE_POS;
            // }
        }
        // else if (status == -1)  // no command
    }

    if (!buggy.isPaused()) {
        control();
    }
    else {
        delay(100);
    }
}
