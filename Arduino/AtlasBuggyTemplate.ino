#define DEFAULT_RATE 115200
#define WHOIAM ""  // define whoiam ID here (unique to each robot object)
#define LED13 13

char character = '\0';
// char command_type = '\0';
String command = "";

bool paused = true;

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
        else if (command.equals("init?"))
        {
            digitalWrite(LED13, HIGH);
            paused = false;

            Serial.print("init:");
            // Put other initialization data here

            Serial.print("\n");
        }
        else if (command.equals("stop"))
        {
            digitalWrite(LED13, LOW);
            paused = true;
        }
        // Put other commands here
        /*
        else if (command.equals("do something cool"))
        {
            buildMeARobot("cheese");
            flyToTheMoon(299792457, "m/s");
        }
        */

        character = '\0';
        command = "";
    }
}

void writeSerial()
{
    // Write sensor data to serial here
}

void setup()
{
    Serial.begin(DEFAULT_RATE);

    pinMode(LED13, OUTPUT);

    // Initialize arduino peripherals
}

void loop()
{
    if (!paused) {
        writeSerial();
    }
    else {
        delay(100);  // Minimize activity when not in use
    }

    readSerial();
}
