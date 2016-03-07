#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055
 
 Connections
 ===========
 Connect SCL to analog 5
 Connect SDA to analog 4
 Connect VDD to 3.3V DC
 Connect GROUND to common ground
 
 History
 =======
 2015/MAR/03  - First release (KTOWN)
 */

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

/**************************************************************************/
/*
 Arduino setup function (automatically called at startup)
 */
/**************************************************************************/
void setup(void)
{
    Serial.begin(9600);
    Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
    
    /* Initialise the sensor */
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    
    delay(1000);
    
    /* Display the current temperature */
    int8_t temp = bno.getTemp();
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");
    
    bno.setExtCrystalUse(true);
    
    Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/**************************************************************************/
/*
 Arduino loop function, called once 'setup' is complete (your own code
 should go here)
 */
/**************************************************************************/
void loop(void)
{
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    
    /* Display the floating point data */
    Serial.print("X: ");
    Serial.print(accel.x());
    Serial.print(" Y: ");
    Serial.print(accel.y());
    Serial.print(" Z: ");
    Serial.print(accel.z());
    Serial.print("\t\t");
    
    Serial.print("X: ");
    Serial.print(gyro.x());
    Serial.print(" Y: ");
    Serial.print(gyro.y());
    Serial.print(" Z: ");
    Serial.print(gyro.z());
    Serial.print("\t\t");
    
    Serial.print("X: ");
    Serial.print(mag.x());
    Serial.print(" Y: ");
    Serial.print(mag.y());
    Serial.print(" Z: ");
    Serial.print(mag.z());
    Serial.print("\n");
    
    delay(BNO055_SAMPLERATE_DELAY_MS);
}
