#define encoderPinA 2
#define encoderPinB 3

volatile bool phase_A, phase_B;
volatile long encoder_pos;
void setup() {
    pinMode(encoderPinA, INPUT); 
    pinMode(encoderPinB, INPUT); 
    // encoder pin on interrupt 0 (pin 2)
    
    attachInterrupt(digitalPinToInterrupt(2), doEncoderA, CHANGE);
    // encoder pin on interrupt 1 (pin 3)
    
    attachInterrupt(digitalPinToInterrupt(3), doEncoderB, CHANGE);  
    Serial.begin (115200);
    phase_A = digitalRead(encoderPinA);
    phase_B = digitalRead(encoderPinB);
    encoder_pos = 0;
}

void loop(){ 
    char buf[100];
    sprintf(buf, "ticks: %ld\n", encoder_pos);
    Serial.print(buf);
    delay(100);
}

void doEncoderA(){
    phase_A = !phase_A;
    if (phase_A && phase_B) encoder_pos --;
    if (phase_A && !phase_B) encoder_pos ++;
    if (!phase_A && phase_B) encoder_pos ++;
    if (!phase_A && !phase_B) encoder_pos --;
}

void doEncoderB(){
    phase_B = !phase_B;
    if (phase_A && phase_B) encoder_pos ++;
    if (phase_A && !phase_B) encoder_pos --;
    if (!phase_A && phase_B) encoder_pos --;
    if (!phase_A && !phase_B) encoder_pos ++;
}
