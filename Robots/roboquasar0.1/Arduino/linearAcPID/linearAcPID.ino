// to change the position it stops at, change variables goalPosition1 and goalPosition2
// the difference of 200 is approximately the length necessary
// press b in serial monitor to extend the actuator
// press n in serial monitor to retract the actuator
// depending on which direction you mount the actuator, exchange the value of the two variables
#include <PID_v1.h>
#define DEFAULT_RATE 115200
#define WHOIAM "linearAc"
#define LED13 13

#define relay1Pin 7
#define relay2Pin 8
#define potHighPin 12
#define potReadPin 0
#define powerPin 2
#define PWM 6

int potWiperValue = 0;
int goalPosition = 50;
int goalPosition1 = 50;
int goalPosition2 = 150;

double GoalPosition, CurrentPosition, Output;

char sig = 'i';
boolean Extending = false;
boolean Retracting = false;

bool led_state = false;
bool paused = true;
bool innerPause = false;

PID linearAc(&CurrentPosition, &Output, &GoalPosition, 2, 5, 1, DIRECT);

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
    else if (command.equals("brake"))
    {
      sig = 'b';
    }
    else if (command.equals("normal"))
    {
      sig = 'n';
    }
  }
}

void setup() {
  Serial.begin(DEFAULT_RATE);

  pinMode(potReadPin, INPUT);

  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(potHighPin, OUTPUT);
  pinMode(powerPin, OUTPUT);
  pinMode(PWM, OUTPUT);

  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay2Pin, LOW);
  digitalWrite(powerPin, HIGH);

  digitalWrite(potHighPin, HIGH);

  CurrentPosition = analogRead(potReadPin);
  GoalPosition = goalPosition2;

  linearAc.SetMode(AUTOMATIC);
}

void control() {

  // read the value from the sensor:
  CurrentPosition = analogRead(potReadPin);

  // press b in serial monitor to extend the actuator
  if (sig == 'b') {
    GoalPosition = goalPosition1;
  }
  // press n in serial monitor to retract the actuator
  if (sig == 'n') {
    GoalPosition = goalPosition2;
  }

  if (abs(GoalPosition - CurrentPosition) <= 3) {
    innerPause = true;
  }else{
    innerPause = false;
  }

  if (not innerPause) {
    linearAc.Compute();

    if (GoalPosition <= CurrentPosition) {
      Retracting = true;
      Extending = false;
      digitalWrite(relay1Pin, LOW);
      digitalWrite(relay2Pin, HIGH);
      analogWrite(PWM, 255 - Output);
      Serial.println(255 - Output);
    }
    else if (GoalPosition >= CurrentPosition) {
      Retracting = false;
      Extending = true;
      digitalWrite(relay1Pin, HIGH);
      digitalWrite(relay2Pin, LOW);
      analogWrite(PWM, Output);
      Serial.println(Output);
    }
  }

  if (Extending = true && GoalPosition - CurrentPosition < 2) {
    //we have reached our goal, shut the relay off
    digitalWrite(relay1Pin, LOW);
    boolean Extending = false;
  }

  if (Retracting = true && CurrentPosition - GoalPosition < 2) {
    //we have reached our goal, shut the relay off
    digitalWrite(relay2Pin, LOW);
    boolean Retracting = false;
  }
}

void loop() {
  if (!paused) {
    control();
  }
  else {
    delay(100);
  }
  readSerial();
}

