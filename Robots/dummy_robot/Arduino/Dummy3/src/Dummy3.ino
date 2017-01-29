#define DEFAULT_RATE 115200
#define WHOIAM "dummy3"  // define whoiam ID here (unique to each robot object)
#define LED13 13

char character = '\0';
char command_type = '\0';
String command = "";

bool paused = true;

unsigned long long t0_us = 0;
unsigned long long t0_ms = 0;
bool led_state = false;

void write_whoiam()
{
    Serial.print("iam");
    Serial.print(WHOIAM);
    Serial.print('\n');
}

void set_led(bool state) {
    led_state = state;
    digitalWrite(LED13, led_state);
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
        command_type = command.charAt(0);
        if (command.equals("whoareyou")) {
            write_whoiam();
        }
        else if (command.equals("init?"))
        {
            Serial.print("init:");
            // Put other initialization data here

            Serial.print("\n");
        }
        else if (command.equals("start"))
        {
            set_led(HIGH);
            paused = false;

            t0_us = micros();
            t0_ms = millis();
        }
        else if (command.equals("stop"))
        {
            set_led(LOW);
            paused = true;
        }
        // Put other commands here
        else if (command_type == 'l')
        {
            switch(command.substring(1).toInt()) {
                case 0: set_led(LOW); break;
                case 1: set_led(HIGH); break;
                case 2: set_led(!led_state); break;
            }
        }

        character = '\0';
        command = "";
    }
}

void writeSerial()
{
    // Write sensor data to serial here
    if ((micros() - t0_us) > 5000) {
        Serial.print((int)(millis() - t0_ms));
        Serial.print('\t');
        Serial.print((int)(micros() - t0_us));
        Serial.print('\n');
        t0_us = micros();
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
        writeSerial();
    }
    else {
        delay(100);  // Minimize activity when not in use
    }

    readSerial();
}
