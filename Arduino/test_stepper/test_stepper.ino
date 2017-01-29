#include <AccelStepper.h>

AccelStepper stepper(AccelStepper::FULL4WIRE,0,1,2,3);

void setup() {
  stepper.setMaxSpeed(200.0);
  stepper.setAcceleration(100.0);
  stepper.moveTo(24);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (stepper.distanceToGo() == 0)
  {
    stepper.moveTo(-stepper.currentPosition();
  }
  stepper.run();
}
