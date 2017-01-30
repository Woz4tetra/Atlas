#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
Adafruit_GPS GPS(&Serial1);

#define DEFAULT_RATE 115200
#define WHOIAM "GPS"
#define LED13 13
#define GPSECHO false

boolean usingInterrpt = false;

Adafruit_GPS GPS(&Serial1);
HardwareSerial mySerial = Serial1;
bool led_state = false;
bool paused = true;

void writeWhoiam()
{
    Serial.print("iam");
    Serial.print(WHOIAM);
    Serial.print('\n');
}

void writeInit()
{
    Serial.print("init:");
    //maybe?
    Serial.print('\n');
}

void setLed(bool state)
{
    led_state = state;
    digitalWrite(LED13, led_state);
}

void pause()
{
    Serial.print("stopping\n");
    paused = true;
}

void unpause() {
    paused = false;
}

void readSerial()
{
    while (Serial.available())
    {
        String command = Serial.readStringUntil('\n');

        if (command.equals("whoareyou")) {
            writeWhoiam();
        }
        else if (command.equals("init?")) {
            writeInit();
        }
        else if (command.equals("start"))
        {
            setLed(HIGH);
            unpause();
        }
        else if (command.equals("stop"))
        {
            setLed(LOW);
            pause();
        }
    }
}

void updateGPS()
{
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  if (GPS.newNMEAreceived()) 
  {
    if (!GPS.parse(GPS.lastNMEA()))
    {
      return;  
    } 
    
    if (GPS.fix) {
      Serial.print(GPS.hour, DEC); Serial.print('\t');
      Serial.print(GPS.minute, DEC); Serial.print('\t');
      Serial.print(GPS.seconds, DEC); Serial.print('\t');
      Serial.print(GPS.milliseconds); Serial.print('\t');
      
      Serial.print(GPS.day, DEC); Serial.print('\t');
      Serial.print(GPS.month, DEC); Serial.print('\t');
      Serial.print(GPS.year, DEC); Serial.print('\t'); 
      Serial.print((int)GPS.fixquality); Serial.print('\t')
      
      Serial.print(GPS.latitude, 4); Serial.print('\t')
      Serial.print(GPS.lat); Serial.print('\t')
      Serial.print(GPS.longitude, 4); Serial.print('\t')
      Serial.print(GPS.lon); Serial.print('\t')
      Serial.print(GPS.latitudeDegrees, 4); Serial.print('\t')
      Serial.print(GPS.longitudeDegrees, 4); Serial.print('\t')
  
      Serial.print(GPS.speed); Serial.print('\t')
      Serial.print(GPS.angle); Serial.print('\t')
      Serial.print(GPS.altitude); Serial.print('\t')
      Serial.print((int)GPS.satellites); Serial.print('\t')
  
      Serial.print('\n')
  }
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);  
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void setup() 
{
  Serial.begin(DEFAULT_RATE);
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  useInterrupt(true);

  /* Initialise the sensor */
  if(!GPS.LOCUS_StartLogger())
  {
    Serial.print("No GPS detected");
  }

  useInterrupt(true);
}

void loop() 
{
  // put your main code here, to run repeatedly:
  if (!paused) 
  {
      updateGPS();
  }
  else 
  {
      delay(100);
  }
  readSerial();
}
