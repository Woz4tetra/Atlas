#define DEFAULT_RATE 115200
#define WHO_I_AM ""  // put ID here
#define LED13 13

char character = '\0';
// char command_type = '\0';
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
        // command_type = command.charAt(0);


        if (command.equals("whoareyou")) {
            write_who_i_am();
        }
        else if (command.equals("ready?")) {
            Serial.print("ready!\n");
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
    Serial.print("ready!\n");

    pinMode(LED13, OUTPUT);


}

void loop()
{
    read_serial();


}
