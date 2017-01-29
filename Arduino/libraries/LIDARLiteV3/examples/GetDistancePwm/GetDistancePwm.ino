/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  GetDistancePwm

  This example shows how to read distance from a LIDAR-Lite connected over the
  PWM interface.

  Connections:
  LIDAR-Lite 5 Vdc (red) to Arduino 5v
  LIDAR-Lite Ground (black) to Arduino GND
  LIDAR-Lite Mode control (yellow) to Arduino digital input (pin 3)
  LIDAR-Lite Mode control (yellow) to 1 kOhm resistor lead 1
  1 kOhm resistor lead 2 to Arduino digital output (pin 2)

  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5v
  680uF capacitor (-) to Arduino GND

  See the Operation Manual for wiring diagrams and more information:
  http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf

------------------------------------------------------------------------------*/

unsigned long pulseWidth;
#define TRIGGER 8
#define MONITOR 9

void setup()
{
  Serial.begin(115200); // Start serial communications

  pinMode(TRIGGER, OUTPUT); // Set pin TRIGGER as trigger pin
  digitalWrite(MONITOR, LOW); // Set trigger LOW for continuous read

  pinMode(MONITOR, INPUT); // Set pin MONITOR as monitor pin
}

void loop()
{
  pulseWidth = pulseIn(MONITOR, HIGH); // Count how long the pulse is high in microseconds

  // If we get a reading that isn't zero, let's print it
  if(pulseWidth != 0)
  {
    // pulseWidth = pulseWidth / 10; // 10usec = 1 cm of distance
    Serial.println(pulseWidth); // Print the distance
  }
}
