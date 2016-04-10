
#define REFLECT_PIN1 9
#define ONOFF_PIN 13

void setup()
{
    Serial.begin(9600);
    pinMode(REFLECT_PIN1, INPUT);
    pinMode(ONOFF_PIN, OUTPUT);
    digitalWrite(ONOFF_PIN, HIGH);
}

void loop()
{
    Serial.println(digitalRead(REFLECT_PIN1));
    delay(50);
}
