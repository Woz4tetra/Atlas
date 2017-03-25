void setup()
{
    pinMode(13, OUTPUT);

    noInterrupts();           // disable all interrupts

    UBRR0H = 0x00;
    UBRR0L = 207;

    UCSR0B = 0x98;             // Serial interrupt
    UCSR0C = 0x06;             // Serial interrupt

    interrupts();             // enable all interrupts
}

void loop()
{

}

ISR(USART0_RXC)          // timer compare interrupt service routine
{
    // char inChar = (char)Serial.read();

    char inChar = UDR0;
    delay(5);

    Serial.print(inChar);
    delay(20);

    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(25);               // wait for a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(25);
}
