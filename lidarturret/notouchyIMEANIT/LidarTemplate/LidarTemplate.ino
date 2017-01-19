#define DEFAULT_RATE 115200
#define WHOIAM "no_fuck_YOU"  // define whoiam ID here (unique to each robot object)
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

void set_led(bool state) 
{
    led_state = state;
    digitalWrite(LED13, led_state);
}

void readSerial()
{
    while (Serial.available() && character != '\n')
    {
        character = Serial.read();
        if (character != '\n') 
        {
            command += character;
        }
    }

    if (character == '\n')
    {
        // command_type = command.charAt(0);
        if (command.equals("whoareyou")) 
        {
            write_whoiam();
        }
        else if (command.equals("init?"))
        {
            digitalWrite(LED13, HIGH);
            

            Serial.print("init:");
            // Put other initialization data here

            Serial.print("\n");
        }
        else if (command.equals("stop"))
        {
            digitalWrite(LED13, LOW);
            paused = true;
        }
        else if (command.equals("start"))
        {
            digitalWrite(LED13, HIGH);
            paused = false;

            t0_us = micros();
            t0_ms = millis();
        }
       /* {
            Serial.print(command);
            Serial.print(" is not a valid command");
            Serial.print('\n');
        }
        // Put other commands here
       /* else if (command_type == 'l')
        {
            switch(command.substring(1).toInt()) {
                case 0: set_led(LOW); break;
                case 1: set_led(HIGH); break;
                case 2: set_led(!led_state); break;
            }
        }*/

        character = '\0';
        command = "";
    }
}

void writeSerial()
{
   // ticks, distance (tab seperated)
    Serial.print((int)(tick));
    Serial.print('\t');
    Serial.print((int)(distance);
    Serial.print('\n');
 }

void writeInit()
{
    Serial.print((int)();
    Serial.print('\t');
    Serial.print((int)();
    Serial.print('\n');

 // in calibration, find below and send in init
 // scan size, rate, detection angle (degrees), distance no detection
}
   /* // Write sensor data to serial here
    // Serial.print(adc_value);
    // Serial.print("\t");
    // Serial.print(micros());
    // Serial.print('\n');
    delay(500);*/
}

void setup()
{
    Serial.begin(DEFAULT_RATE);
    pinMode(LED13, OUTPUT);

    // Initialize arduino peripherals
}

void loop()
{
    if (!paused) 
    {
        writeSerial();
    }
    else {
        delay(100);  // Minimize activity when not in use
    }

    readSerial();
}

/* motor shit to keep in mind 
 transistor for speed control on motor(possibly directional)
 pwm to control of one ardino pin (analog write)
 total three pins ( two encoder, one pwm)
 look up quadrature encoder for more on how the encoder works

what kind of control are we looking for
 pid sort of thing for the calibrate, then just fine tuning for the best cloud
 tied together
 */
