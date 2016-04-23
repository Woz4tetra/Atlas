#include "HallEncoder.h"

void setup()
{
    encoder_setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println((int)encoder_distance());
  delay(100);
}
