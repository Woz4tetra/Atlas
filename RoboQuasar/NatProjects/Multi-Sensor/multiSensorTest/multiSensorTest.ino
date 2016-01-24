/** @file multiSensorTest.ino
 *  
 *  Self Driving Buggy Sensor Board Drivers
 *  
 *  @author Nat Jeffries
 *  Add yourselves to authors field upon contribution!
 *  
 *  In order to run this code you need the following libraries:
 *  https://github.com/ivanseidel/DueTimer
 *  MPU6050 drivers (built-in ?)
 *  Wire.h libary (built-in)
 *  
 *  Known Iusses:
 *  Sometimes the board issues a high-pitched buzz --weird
 */
//#include <Servo.h> --> currently causes naming conflict
//TODO: modify source code to temporarily remove Timer0 -2
//from the DueTimer source code 
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "DueTimer.h"

/* Driver-wide definitions */
#define DEBUG

/* IMU Globals */
MPU6050 accelgyro;
int pos;
//Servo servo1;
int16_t ax, ay, az;
int16_t gx, gy, gz;

/* Distance encoder Globals */
bool is_in_range; //if true, trigger when we see out-of-range value
int last_rising_edge; //ms
volatile int curr_distance;//inches... subject to group consensus
#define HYST_TRIG_HIGH 950 //TODO: Tune these based on OBSERVED values
#define HYST_TRIG_LOW 850
#define WHEEL_CIR (10.0 * PI)
#define ADC_POLLING_PERIOD_US 1000 //1KHz polling rate
/*The following back-of-the-envelope calcultaions indicate max speed:
 Assume we must poll at least 50x per revolution in order to 
 ALWAYS detect the magnet (this is conservative).
 Inches per Revolution = 31.415
 1KHz sample rate => 20 revolutions per second at 50 samples/rev
 628.3 inches per second => (approx) 37mph
 */
/* handler for timer interrupt during which ADC is polled */
void handler(){
    int hall_value = analogRead(A0);
    if (is_in_range && (hall_value > HYST_TRIG_HIGH)){
        curr_distance += WHEEL_CIR;
        is_in_range = false;
    }
    else if (!is_in_range && (hall_value <= HYST_TRIG_LOW)){
        is_in_range = true;
    }
}

void setup() {
    /*disable interrupts to ensure we won't receive the first timer 
     or I2C interrupt before we are ready*/
    noInterrupts();
    pinMode(A0, INPUT);
    //servo1.attach(2);
    Serial.begin(115200); //high rate necessary for some debug
    Wire.begin(); //I2C library
    accelgyro.initialize(); //IMU library
    is_in_range = false; //make sure we don't start at > 0 distance
    curr_distance = 0;
    Timer3.attachInterrupt(handler); //handler is a function pointer
    Timer3.start(ADC_POLLING_PERIOD_US);
    interrupts();
}

void loop() {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
#ifdef DEBUG
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print("\n");
    Serial.print(curr_distance); Serial.print("\n");
    delay(100);
#endif
}
