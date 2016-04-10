// hold flashlight ~1in away. Put encoder disk right in front of the sensor

volatile bool is_on_white;
volatile long enc_distance;
#define HYST_TRIG_HIGH 505
#define HYST_TRIG_LOW 490

void setup()
{
  Serial.begin(9600);

  cli();//stop interrupts

//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 124;//15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

sei();//allow interrupts

}//end setup


ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  int light_value = analogRead(A0);

  if (!is_on_white && (light_value >= HYST_TRIG_HIGH))
  {
    enc_distance += 1; //WHEEL_CIR;
    is_on_white = true;
  }
  else if (is_on_white && (light_value <= HYST_TRIG_LOW)) {
    enc_distance += 1; //WHEEL_CIR;
    is_on_white = false;
  }
}

void loop() {
  //do other things here
  Serial.print(enc_distance); Serial.print(", ");
  Serial.print(analogRead(A0)); Serial.print('\n');
}
