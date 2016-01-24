/*  Nat Jeffries Fall 2015
    CIA buggy speedometer project
    Version 1.1
*/

#define NUM_SLOTS 50
#define CIR_MM 479//mm ( for a 6 inch wheel)
/*** !!THE FOLLOWING IS FOR MY BIKE!!! ***/
#define CIR 2413//1074 //diameter to calculate MPH (cir (mm) * (mph/(m/s)))
#define MS_PER_BUCKET 50 //ms
#define POLL_RATE 2 //ms between polling

/* global variables */
int time_buckets[NUM_SLOTS]; //number of 
unsigned long start_time;
int prev_magnet;
int loop_count; // loop counter - determine when to increment bucket
int trigger_count; //number of sensor edges seen in current bucket
long total_distance_travelled; //distance covered in mm

/* I/O helpers */
void show_mph(){
  int total = _sum_buckets();
  //Serial.println(total);
  int mph = 0;
  if (total > 0)
    mph = (CIR * total)/(MS_PER_BUCKET * NUM_SLOTS);
  //Serial.println(mph);
  disp(1, mph%10);
  disp(0, mph/10);
}

void disp(int dispnum, int num){
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
  for (int i=7*dispnum; i<=6+7*dispnum; i++){
    if (i == 0)
      digitalWrite(A5, val & 1);
    else
      digitalWrite(i, val & 1);
    val = val>> 1;
  }
}

/* internal helpers */
void _insert_bucket(int val){
  for (int i=NUM_SLOTS-1; i>0; i--){
    time_buckets[i] = time_buckets[i-1];
  }
  time_buckets[0] = val;
}

int _magnet(){
  int mag_val = analogRead(A0);
  return ((mag_val > 550) || (mag_val < 450));
}
  

int _check_sensor(){
  int mag = _magnet();
  int ret = (mag && !prev_magnet); //only register if new magnet pass detected
  prev_magnet = mag;
  if (ret){
    total_distance_travelled += CIR_MM;
  }
  return ret;
}

int _sum_buckets(){
  int total = 0;
  for (int i=0; i<NUM_SLOTS; i++){
    total += time_buckets[i];
  }
  return total;
}
  
/* setup and main loop */
void setup(){
  Serial.begin(9600);
  int i;
  /* set port directions */
  for (i=0; i<=13; i++){
    pinMode(i, OUTPUT);
  }
  pinMode(A0, INPUT);
  pinMode(A5, OUTPUT);
  for(i=0; i<NUM_SLOTS; i++){
    time_buckets[i] = 0;
  }
  prev_magnet = 0;
  loop_count = 0;
  trigger_count = 0;
  total_distance_travelled = 0;
}

void loop(){
  start_time = micros();
  if (loop_count++ > 25){
    _insert_bucket(trigger_count);
    show_mph();
    Serial.println(total_distance_travelled);
    loop_count = 0;
    trigger_count = 0;
  }
  trigger_count = (_check_sensor()) ? trigger_count + 1 : trigger_count;
  delayMicroseconds(2000 - (micros() - start_time));
}
