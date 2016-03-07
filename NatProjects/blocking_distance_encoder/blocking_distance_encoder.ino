/*  Written by Nat Jeffries
    Distnac Encoder for Self Driving Buggy Project
    Version 1.0
*/

#define CIR_MM 479
#define CIR 1074 //mm 2.237 (for 6 inch wheel)
#define MAG A0

long total_distance;

void disp1(int num){
  int val = (num == 0) ? B01011111 : 
  (num == 1) ? B00000110 : 
  (num == 2) ? B00111101 : 
  (num == 3) ? B00101111 : 
  (num == 4) ? B01100110 : 
  (num == 5) ? B01101011 : 
  (num == 6) ? B01111011 : 
  (num == 7) ? B00000111 : 
  (num == 8) ? B01111111 : 
  (num == 9) ? B01100111 : 0;
  for (int i=7; i<=13; i++){
    digitalWrite(i, val & 1);
    val = val>> 1;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(MAG, INPUT);
  pinMode(LED, OUTPUT);
  //DDRD |= 0xFC;
  for (int i=6; i<=13; i++) pinMode(i, OUTPUT);
  disp1(0);
  count = 0;
  pinstate = LOW;
  total_distance = 0;
}

void wait_for_signal(){
  while ((analogRead(A0) > 450) && (analogRead(A0) < 550));
}

int num = 0;

void loop() {
  wait_for_signal();
  total_distance += CIR_MM;
  Serial.println(total_distance);
  while ((analogRead(A0) < 510) || (analogRead(A0) > 515));
  int a = analogRead(A0);
}
