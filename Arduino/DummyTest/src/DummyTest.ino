#define DEFAULT_RATE 115200
#define WHO_I_AM 1

char character = '\0';
String command = "";

void write_who_i_am()
{
    Serial.print("iam");
    Serial.print(WHO_I_AM);
    Serial.print('\n');
}

void read_serial()
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
        character = '\0';

        if (command.equals("whoareyou")) {
            write_who_i_am();
        }
    }
}

void setup() {
    Serial.begin(DEFAULT_RATE);
}

void loop()
{
    read_serial();
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(micros());
    Serial.print('\n');
    delay(500);
}
