
char character = '\0';
String command = "";

void setup() {
    Serial.begin(115200);


}

void loop() {
    Serial.println("something");
    delay(1000);

    while (Serial.available() && character != '\n')
    {
        character = Serial.read();
        if (character != '\n') {
            command += character;
        }
    }
    if (character == '\n')
    {
        Serial.println(command);
        if (command.equals("something")) {
            // do a thing
        }

        character = '\0';
        command = "";
    }
}
