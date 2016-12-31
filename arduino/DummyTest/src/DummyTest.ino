#define DEFAULT_RATE 115200
#define WHO_I_AM "dummy"
#define LED13 13

volatile byte adc_value = 0;

char character = '\0';
char command_type = '\0';
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
        command_type = command.charAt(0);


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

        if (command_type == 'l') {
            digitalWrite(LED13, (bool)(command.substring(1).toInt()));
        }

        command = "";
    }
}

void setup()
{
    Serial.begin(DEFAULT_RATE);
    Serial.print("ready!\n");

    ADCSRA = 0;             // clear ADCSRA register
    ADCSRB = 0;             // clear ADCSRB register
    ADMUX |= (3 & 0x07);    // set A3 analog input pin
    ADMUX |= (1 << REFS0);  // set reference voltage
    ADMUX |= (1 << ADLAR);  // left align ADC value to 8 bits from ADCH register

    // sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
    // for Arduino Uno ADC clock is 16 MHz and a conversion takes 13 clock cycles
    //ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz
    ADCSRA |= (1 << ADPS2);                     // 16 prescaler for 76.9 KHz
    //ADCSRA |= (1 << ADPS1) | (1 << ADPS0);    // 8 prescaler for 153.8 KHz

    ADCSRA |= (1 << ADATE); // enable auto trigger
    ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
    ADCSRA |= (1 << ADEN);  // enable ADC
    ADCSRA |= (1 << ADSC);  // start ADC measurements

    pinMode(LED13, OUTPUT);
}

ISR(ADC_vect) {
    adc_value = ADCH;  // read 8 bit value from ADC
}

void loop()
{
    read_serial();
    // Serial.print(adc_value);
    // Serial.print("\t");
    // Serial.print(micros());
    // Serial.print('\n');
    delay(500);
}
