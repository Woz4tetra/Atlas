// to change the position it stops at, change variables goalPosition1 and goalPosition2
// the difference of 200 is approximately the length necessary
// press b in serial monitor to extend the actuator
// press n in serial monitor to retract the actuator
// depending on which direction you mount the actuator, exchange the value of the two variables

const int relay1Pin =  7;      
const int relay2Pin =  8;      
const int potHighPin = 12;    
const int potLowPin = 13;    
const int potReadPin = 0;    
const int power = 2;

int potWiperValue = 0;
int goalPosition = 0;
int goalPosition1 = 50;
int goalPosition2 = 250;
int CurrentPosition = 0;
char sig = 'i';
boolean Extending = false;
boolean Retracting = false;


void setup() {


  Serial.begin(9600);

  pinMode(potReadPin, INPUT);
  
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(potHighPin, OUTPUT);
  pinMode(potLowPin, OUTPUT);
  pinMode(power, OUTPUT);

  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay2Pin, LOW);
  digitalWrite(power, HIGH);

  digitalWrite(potHighPin, HIGH);
  digitalWrite(potLowPin, LOW);


}

void loop() {

  // read the value from the sensor:
  CurrentPosition = analogRead(potReadPin);
    
  if (Serial.available() > 0) {
    sig = Serial.read();
    // press b in serial monitor to extend the actuator
    if (sig == 'b') {
      Serial.println("braking");
      goalPosition = goalPosition2;
    }
    // press n in serial monitor to retract the actuator
    if (sig == 'n') {
      Serial.println("normal");
      
      goalPosition = goalPosition1;
    }
  }
  
  if (goalPosition < CurrentPosition-20) {
    Retracting = true;
    Extending = false;
    digitalWrite(relay1Pin, LOW);
    digitalWrite(relay2Pin, HIGH);
  }
  else if (goalPosition > CurrentPosition+20) {
    Retracting = false;
    Extending = true;
    digitalWrite(relay1Pin, HIGH);
    digitalWrite(relay2Pin, LOW);
  }

  if (Extending = true && CurrentPosition - goalPosition>5) {
    //we have reached our goal, shut the relay off
    digitalWrite(relay1Pin, LOW);
    boolean Extending = false;
  }

  if (Retracting = true && goalPosition - CurrentPosition>5) {
    //we have reached our goal, shut the relay off
    digitalWrite(relay2Pin, LOW);
    boolean Retracting = false;
  }
}

