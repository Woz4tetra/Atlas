 
// inslude the SPI library:
#include <SPI.h>

char buf[100];
volatile byte pos;
volatile boolean process_it;

void setup(){
  Serial.begin(9600);
  pinMode(3, INPUT); 
  //enable slave mode 
  pinMode(MISO,OUTPUT);
  SPCR |= _BV(SPE);
  
  //initialize buffer
  pos = 0;
  
  SPI.attachInterrupt();
  Serial.println("SPI Testing");
}

ISR(SPI_STC_vect)
{
  byte c = SPDR;
  if(pos<sizeof buf)
  {
    buf[pos++]=c;  
  }
  //echo incoming byte back out
  SPDR = c;
}

void loop(){
//  SPDR=0;
  if (digitalRead(3)) {
    Serial.println("!");
  }
  if(pos==8){
    for(int i=0;i<8;i++){
      Serial.print(buf[i],HEX);
      Serial.print(" ");
    }
    Serial.println();
    pos=0;
    SPDR=0;
  }
}
