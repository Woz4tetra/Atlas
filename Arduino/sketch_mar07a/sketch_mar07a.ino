#define STEPPER_PIN_1 3
#define STEPPER_PIN_2 2
#define STEPPER_PIN_3 4
#define STEPPER_PIN_4 5

bool pin_1 = false;
bool pin_2 = false;
bool pin_3 = false;
bool pin_4 = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print(c); Serial.print('\t');
    if (c == '1') {
      pin_1 = !pin_1;
      digitalWrite(STEPPER_PIN_1, pin_1);
      Serial.println(pin_1);
    }
    else if (c == '2') {
      pin_2 = !pin_2;
      digitalWrite(STEPPER_PIN_2, pin_2);
      Serial.println(pin_2);
    }
    else if (c == '3') {
      pin_3 = !pin_3;
      digitalWrite(STEPPER_PIN_3, pin_3);
      Serial.println(pin_3);
    }
    else if (c == '4') {
      pin_4 = !pin_4;
      digitalWrite(STEPPER_PIN_4, pin_4);
      Serial.println(pin_4);
    }
  }
}
