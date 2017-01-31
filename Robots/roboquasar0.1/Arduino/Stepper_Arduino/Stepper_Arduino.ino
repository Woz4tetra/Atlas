#include <AccelStepper.h>

#define DEFAULT_RATE 115200
#define WHOIAM "steering"
#define LED13 13
#define DELIMITER_PIN 6

bool led_state = false;
bool paused = true;
String command = "";
int position = 0;
int speed = 0;

AccelStepper stepper(AccelStepper::FULL4WIRE,2,3,4,5);

void writeWhoiam()
{
    Serial.print("iam");
    Serial.print(WHOIAM);
    Serial.print('\n');
}

void writeInit()
{
    Serial.print("init:");
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
        command = Serial.readStringUntil('\n');

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
          setPosition(command.substring(1).toInt());
        }
        else if (command.substring(0,1).equals("v"))
        {
          setVelocity(command.substring(1).toInt());
        }
    }
}

void setPosition(int p) {
  stepper.moveTo(p);
}

void setVelocity(int v) {
  stepper.setSpeed(v);
}

void calibrate() {
  Serial.print("Calibrating stepper... \n");
  while !(digitalRead(DELIMITER_PIN))
  {
    stepper.step(-5);
  }
  stepper.step(150);
  Serial.print("done!\n");
}

void setup() {
  Serial.begin(DEFAULT_RATE);

  pinMode(DELIMITER_PIN, INPUT);

  stepper.setMaxSpeed(200.0);
  stepper.setAcceleration(100.0);

  calibrate();
}

void loop() {}

  if (stepper.distanceToGo() != 0)
  {
    stepper.runSpeed();
  }

  if (stepper.isRunning())
  {
    delay(10);
    Serial.print(stepper.currentPosition())
  }

  readSerial();
}
