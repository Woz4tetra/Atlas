#define DEFAULT_RATE 115200
#define WHOIAM ""  // define whoiam ID here (unique to each robot object)
#define LED13 13

bool paused = true;

void write_whoiam()
{
    Serial.print("iam");
    Serial.print(WHOIAM);
    Serial.print('\n');
}

void readSerial()
{
    while (Serial.available())
    {
        String command = Serial.readStringUntil('\n');

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
    }
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
        // Write sensor data to serial here
    }
    else {
        delay(100);  // Minimize activity when not in use
    }

    readSerial();
}
