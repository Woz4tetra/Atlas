/*  Written by Nat Jeffries
    Distnac Encoder for Self Driving Buggy Project
    Version 1.0
*/
#define THRESHOLD_HIGH 900
#define THRESHOLD_LOW 800
#define RAD_M 0.279
#define CIR_M (RAD_M*PI)
#define MAG A0

double total_distance;

void setup() {
  Serial.begin(9600);
  pinMode(MAG, INPUT);
  total_distance = 0;
}

void wait_for_signal(){
  while (analogRead(A0) < THRESHOLD_HIGH); 
  //while ((analogRead(A0) > THRESHOLD_LOW) && (analogRead(A0) < THRESHOLD_HIGH));
}

int num = 0;

void loop() {
  wait_for_signal();
  total_distance += (float)CIR_M;
  Serial.println(total_distance);
  while(analogRead(A0) > THRESHOLD_LOW);
  //while ((analogRead(A0) < THRESHOLD_LOW) || (analogRead(A0) > THRESHOLD_HIGH));
}
