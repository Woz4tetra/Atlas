#include <AccelStepper.h>

#define DEFAULT_RATE 115200
#define WHOIAM "steering"
#define LED13 13

bool led_state = false;
bool paused = true;

AccelStepper stepper(AccelStepper::FULL4WIRE,2,3,4,5);

{
    Serial.print("iam");
    Serial.print(WHOIAM);
    Serial.print('\n');
}

void writeInit()
{
    Serial.print("init:");
    Serial.print("delay:");
    Serial.print(BNO055_SAMPLERATE_DELAY_MS);
    Serial.print('\n');
}

void setLed(bool state)
{
    led_state = state;
    digitalWrite(LED13, led_state);
}

void pause()
{
    Serial.print("stopping\n");
    paused = true;
}

void unpause() {
    paused = false;
}

void readSerial()
{
    while (Serial.available())
    {
        String command = Serial.readStringUntil('\n');

        // if (character == '\n')
        // {
        // Serial.println(command);
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
          setPosition(command.substring(1,command.length()).toInt());
        }
        else if (command.substring(0,1).equals("s"))
        {
          setSpeed(command.substring(1,command.length()).toInt());
        }
    }
}

void setPosition(int p) {
  stepper.moveTo(p);
}

void setSpeed(int s) {
  stepper.setSpeed(s);
}

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
