#include <Wire.h>
//#include <MPU9250.h>
#include <Adafruit_GPS.h>
#include <Servo.h>
#include <Arduino.h>
#include <DueTimer.h>

Adafruit_GPS GPS(&Serial1);
//MPU9250 accelgyro;

Servo servo1;
int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
float lat, lon;
uint16_t gps_ms;
unsigned  long enc_value = 0;
int servo_pos;

bool is_in_range; //if true, trigger when we see out-of-range value
int last_rising_edge; //ms
volatile unsigned int enc_distance = 0; // counts of the encoder

#define HYST_TRIG_HIGH 950 //TODO: Tune these based on OBSERVED values
#define HYST_TRIG_LOW 850
//#define WHEEL_CIR (10.0 * PI)
#define ADC_POLLING_PERIOD_US 1000 //1KHz polling rate
/*
 The following back-of-the-envelope calcultaions indicate max speed:
 Assume we must poll at least 50x per revolution in order to 
 ALWAYS detect the magnet (this is conservative).
 Inches per Revolution = 31.415
 1KHz sample rate => 20 revolutions per second at 50 samples/rev
 628.3 inches per second => (approx) 37mph
 */

/* handler for timer interrupt during which ADC is polled */
void handler()
{
    int hall_value = analogRead(A0);
    
    if (is_in_range && (hall_value > HYST_TRIG_HIGH))
    {
        enc_distance += 1; //WHEEL_CIR;
        is_in_range = false;
    }
    else if (!is_in_range && (hall_value <= HYST_TRIG_LOW)) {
        is_in_range = true;
    }
}

void encoder_setup()
{
    noInterrupts();
    
    pinMode(A0, INPUT);
    
    is_in_range = false; //make sure we don't start at > 0 distance
    enc_distance = 0;
    
    Timer1.attachInterrupt(handler); //handler is a function pointer
    Timer1.start(ADC_POLLING_PERIOD_US);
    interrupts();
}

void setup()
{
	//noInterrupts();
	Serial.begin(115200);
  Serial.println("initialized serial");

	/* init GPS */
	GPS.begin(9600);
	Serial1.begin(9600);

	// Enable RMC
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

	// Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

	// Request updates on antenna status, comment out to keep quiet
    //GPS.sendCommand(PGCMD_ANTENNA);
  Serial.println("initialized GPS");
    pinMode(13, OUTPUT);

    servo1.attach(3);
	servo_pos = 0;
    servo1.write(servo_pos);
	Serial.println("initialized servo");
	/* init IMU */
	Wire.begin();
	//accelgyro.initialize();
  Serial.println("initialized IMU");
	encoder_setup();
	//interrupts();
}

void loop()
{
	char serial_buf[50];
	//accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	char *GPS_resp;
	if (GPS.newNMEAreceived()){
		GPS.read();
		lat = GPS.latitude;
		lon = GPS.longitude;
		gps_ms = GPS.milliseconds;
		//GPS.parseNMEA(GPS.lastNMEA());
	}
  char ch = 0;
  if (Serial.available())
    ch = Serial.read();
	if (ch == 'r'){
		sprintf(serial_buf, ">%ld,%ld,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			micros(), gps_ms, lat, lon, ax, ay, az, gx, gy, gz, mx, my, mz, enc_distance);
		Serial.write(serial_buf);
		Serial.flush();
	}
  if (ch == 's'){
    Serial.println("entered servo handler");
    char c;
    int i = 0;
    char buf[10];
    while (Serial.available() > 0){
      c = Serial.read();
      buf[i++] = c;
    }
    buf[i] = '\0';
    Serial.println(buf);
    servo_pos = atoi(buf);
    Serial.print("set servo to "); Serial.println(servo_pos);
    servo1.write(servo_pos);
    Serial.flush();
  }
	delay(10);
}
