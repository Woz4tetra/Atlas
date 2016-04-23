
String in_string = "";
int val;    // variable to read the value from the analog pin
int left_pin = 6;
int right_pin = 5;

void setup() {
  Serial.begin(9600);
  pinMode(right_pin, OUTPUT);
  pinMode(left_pin, OUTPUT);
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
              analogWrite(left_pin, 0);
              analogWrite(right_pin, abs(val));
            }
            else if (val == 0) {
              analogWrite(right_pin, 0); // TODO: braking. You can stop suddenly by applying the previous value (divided by 1.5?) in the opposite direction for a short period of time
              analogWrite(left_pin, 0);
            }
            else {
              analogWrite(right_pin, 0);
              analogWrite(left_pin, abs(val));
            }
            in_string = "";
        }
    }
  delay(15);
}
