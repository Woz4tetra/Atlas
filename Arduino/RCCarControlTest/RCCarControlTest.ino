
String in_string = "";
int val;    // variable to read the value from the analog pin
int motor_backward_pin = 9;
int motor_forward_pin = 10;

void setup() {
  Serial.begin(9600);
  pinMode(motor_forward_pin, OUTPUT);
  pinMode(motor_backward_pin, OUTPUT);
}

void loop() {
  if (Serial.available()) {
        char in_char = Serial.read();
        if (isDigit(in_char) || in_char == '-') {
            in_string += (char)in_char;
        }
        if (in_char == '\n')
        {
            val = in_string.toInt();
            Serial.print("value: ");
            Serial.println(val);
            if (val > 0) {
              analogWrite(motor_backward_pin, 0);
              analogWrite(motor_forward_pin, abs(val));
            }
            else if (val == 0) {
              analogWrite(motor_forward_pin, 0); // TODO: braking. You can stop suddenly by applying the previous value (divided by 1.5?) in the opposite direction for a short period of time
              analogWrite(motor_backward_pin, 0);
            }
            else {
              analogWrite(motor_forward_pin, 0);
              analogWrite(motor_backward_pin, abs(val));
            }
            in_string = "";
        }
    }
  delay(15);
}

