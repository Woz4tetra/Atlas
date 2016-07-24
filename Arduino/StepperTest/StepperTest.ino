#include <Stepper.h>

const int stepsPerRevolution = 513;  // change this to fit the number of steps per revolution
// for your motor

Stepper myStepper(stepsPerRevolution, 4, 5, 6, 7);

bool forward = true;
long time0 = millis();
int step_count = 0;

void setup() {
  // initialize the serial port:
  Serial.begin(9600);

  myStepper.setSpeed(100);
}

void loop() {
  // step one step:
  // myStepper.step(1);
  // Serial.print("steps:");
  // Serial.println(stepCount);
  // stepCount++;
  // Serial.println()
  // if (millis() - time0 > 1000) {
  //     forward = !forward;
  //     time0 = millis();
  // }
  if (abs(step_count) > stepsPerRevolution || step_count < 0) {
      forward = !forward;
  }
  Serial.println(step_count);
  if (forward) {
      myStepper.step(1);
      step_count++;
  }
  else {
      myStepper.step(-1);
      step_count--;
  }
  // delay(100);
}
