#define DEFAULT_RATE 115200
#define WHOIAM ""  // define whoiam ID here (unique to each robot object)
#define LED13 13

char character = '\0';
// char command_type = '\0';
String command = "";

void write_whoiam()
{
    Serial.print("iam");
    Serial.print(WHOIAM);
    Serial.print('\n');
}

void readSerial()
{
    while (Serial.available() && character != '\n')
    {
        character = Serial.read();
        if (character != '\n') {
            command += character;
        }
    }

    if (character == '\n')
    {
        // command_type = command.charAt(0);


        if (command.equals("whoareyou")) {
            write_whoiam();
        }
        else if (command.equals("init?")) {
            Serial.print("init:");
            // Put other initialization data here

            Serial.print("\n");
        }
        else if (command.equals("stop")) {
            digitalWrite(LED13, HIGH);
            Serial.print("stopping\n");
        }

        /* Put other commands here */
        character = '\0';
        command = "";
    }
}

void setup()
{
    Serial.begin(DEFAULT_RATE);

    pinMode(LED13, OUTPUT);


}

void loop()
{
    readSerial();


}
