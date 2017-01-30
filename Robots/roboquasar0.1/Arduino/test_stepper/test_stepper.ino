#include <AccelStepper.h>

AccelStepper stepper(AccelStepper::FULL4WIRE,2,3,4,5);

void setup() {
  stepper.setMaxSpeed(200.0);
  stepper.setAcceleration(100.0);
  stepper.moveTo(24);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (stepper.distanceToGo() == 0)
  {
    stepper.moveTo(-stepper.currentPosition());
  }
  stepper.run();
}
