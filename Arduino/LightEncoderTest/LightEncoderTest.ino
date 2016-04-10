//timer interrupts
//by Amanda Ghassaei
//June 2012
//http://www.instructables.com/id/Arduino-Timer-Interrupts/

/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3 of the License, or
* (at your option) any later version.
*
*/


volatile bool is_on_whiteA, is_on_whiteB;
volatile bool phase_A, phase_B;
volatile long encoder_pos;
volatile int light_valueA, light_valueB;
int maxA, maxB;
int minA = 1000, minB = 1000;

volatile long encoder_posA;
volatile long encoder_posB;

#define HYST_TRIG_HIGH_A 460
#define HYST_TRIG_LOW_A 450

#define HYST_TRIG_HIGH_B 460
#define HYST_TRIG_LOW_B 450

void setup()
{
    Serial.begin(9600);

    cli();//stop interrupts

    ////set timer0 interrupt at 2kHz
    //  TCCR0A = 0;// set entire TCCR2A register to 0
    //  TCCR0B = 0;// same for TCCR2B
    //  TCNT0  = 0;//initialize counter value to 0
    //  // set compare match register for 2khz increments
    //  OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
    //  // turn on CTC mode
    //  TCCR0A |= (1 << WGM01);
    //  // Set CS01 and CS00 bits for 64 prescaler
    //  TCCR0B |= (1 << CS01) | (1 << CS00);
    //  // enable timer compare interrupt
    //  TIMSK0 |= (1 << OCIE0A);

    //set timer1 interrupt
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 1hz increments
    OCR1A = 100;//15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12 and CS10 bits for 1024 prescaler
    TCCR1B |= (1 << CS12) | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    sei();//allow interrupts

}//end setup

ISR(TIMER1_COMPA_vect)
{
    light_valueA = analogRead(A0);
    if (!is_on_whiteA && (light_valueA >= HYST_TRIG_HIGH_A))
    {
        encoder_posA++;
        phase_A = !phase_A;
        if (phase_A && phase_B) encoder_pos --;
        if (phase_A && !phase_B) encoder_pos ++;
        if (!phase_A && phase_B) encoder_pos ++;
        if (!phase_A && !phase_B) encoder_pos --;
        is_on_whiteA = true;
    }
    else if (is_on_whiteA && (light_valueA <= HYST_TRIG_LOW_A))
    {
        encoder_posA++;
        phase_A = !phase_A;
        if (phase_A && phase_B) encoder_pos --;
        if (phase_A && !phase_B) encoder_pos ++;
        if (!phase_A && phase_B) encoder_pos ++;
        if (!phase_A && !phase_B) encoder_pos --;
        is_on_whiteA = false;
    }

    light_valueB = analogRead(A1);
    if (!is_on_whiteB && (light_valueB >= HYST_TRIG_HIGH_B))
    {
        encoder_posB++;
        phase_B = !phase_B;
        if (phase_A && phase_B) encoder_pos ++;
        if (phase_A && !phase_B) encoder_pos --;
        if (!phase_A && phase_B) encoder_pos --;
        if (!phase_A && !phase_B) encoder_pos ++;
        is_on_whiteB = true;
    }
    else if (is_on_whiteB && (light_valueB <= HYST_TRIG_LOW_B)) {
        encoder_posB++;
        phase_B = !phase_B;
        if (phase_A && phase_B) encoder_pos ++;
        if (phase_A && !phase_B) encoder_pos --;
        if (!phase_A && phase_B) encoder_pos --;
        if (!phase_A && !phase_B) encoder_pos ++;
        is_on_whiteB = false;
    }
}


void loop()
{
    if (maxA < light_valueA) {
        maxA = light_valueA;
    }
    if (maxB < light_valueB) {
        maxB = light_valueB;
    }
    if (minA > light_valueA && light_valueA > 0) {
        minA = light_valueA;
    }
    if (minB > light_valueB && light_valueB > 0) {
        minB = light_valueB;
    }
    Serial.print(encoder_pos); Serial.print(", ");
    Serial.print(encoder_posA); Serial.print(", ");
    Serial.print(encoder_posB); Serial.print(", ");
    Serial.print(light_valueA); Serial.print(", ");
    Serial.print(light_valueB); Serial.print('\n');
    // Serial.print(maxA); Serial.print(", ");
    // Serial.print(minA); Serial.print(", ");
    // Serial.print(maxB); Serial.print(", ");
    // Serial.print(minB); Serial.print('\n');
    delay(10);
}
