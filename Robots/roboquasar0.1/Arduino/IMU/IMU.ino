#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <EEPROM.h>
#include <Atlasbuggy.h>

/* Set the delay between fresh samples */
#define INCLUDE_FILTERED_DATA

#ifdef INCLUDE_FILTERED_DATA
int sample_rate_delay_ms = 10;
#else
int sample_rate_delay_ms = 100;
#endif

Atlasbuggy buggy("imu");

// default (same as OPERATION_MODE_NDOF)
Adafruit_BNO055 bno = Adafruit_BNO055();

// Accelerometer & gyroscope only for getting relative orientation, subject to gyro drift
// Adafruit_BNO055 bno = Adafruit_BNO055(0x08); // OPERATION_MODE_IMUPLUS

// Accelerometer & magnetometer only for getting relative orientation
// Adafruit_BNO055 bno = Adafruit_BNO055(0x0a);  // OPERATION_MODE_M4G

// Gets heading only from compass
// Adafruit_BNO055 bno = Adafruit_BNO055(0x09); // OPERATION_MODE_COMPASS

// OPERATION_MODE_NDOF without fast magnetometer calibration
// Adafruit_BNO055 bno = Adafruit_BNO055(OPERATION_MODE_NDOF_FMC_OFF);

void updateIMU() {
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2

    #ifdef INCLUDE_FILTERED_DATA
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    // xyz is yaw pitch roll for some reason... switching roll pitch yaw
    Serial.print("ex");
    Serial.print(euler.z(), 4);
    Serial.print("\tey");
    Serial.print(euler.y(), 4);
    Serial.print("\tez");
    Serial.print(euler.x(), 4);
    #endif

    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    #ifdef INCLUDE_FILTERED_DATA
    Serial.print("\tmx");
    #else
    Serial.print("mx");
    #endif
    Serial.print(mag.x(), 4);
    Serial.print("\tmy");
    Serial.print(mag.y(), 4);
    Serial.print("\tmz");
    Serial.print(mag.z(), 4);

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    Serial.print("\tgx");
    Serial.print(gyro.x(), 4);
    Serial.print("\tgy");
    Serial.print(gyro.y(), 4);
    Serial.print("\tgz");
    Serial.print(gyro.z(), 4);

    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    Serial.print("\tax");
    Serial.print(accel.x(), 4);
    Serial.print("\tay");
    Serial.print(accel.y(), 4);
    Serial.print("\taz");
    Serial.print(accel.z(), 4);

    imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    Serial.print("\tlx");
    Serial.print(linaccel.x(), 4);
    Serial.print("\tly");
    Serial.print(linaccel.y(), 4);
    Serial.print("\tlz");
    Serial.print(linaccel.z(), 4);

    #ifdef INCLUDE_FILTERED_DATA
    // Quaternion data
    imu::Quaternion quat = bno.getQuat();
    Serial.print("\tqw");
    Serial.print(quat.w(), 4);
    Serial.print("\tqx");
    Serial.print(quat.x(), 4);
    Serial.print("\tqy");
    Serial.print(quat.y(), 4);
    Serial.print("\tqz");
    Serial.print(quat.z(), 4);


    /* Display calibration status for each sensor. */
    uint8_t sys_stat, gyro_stat, accel_stat, mag_stat = 0;
    bno.getCalibration(&sys_stat, &gyro_stat, &accel_stat, &mag_stat);
    Serial.print("\tss");
    Serial.print(sys_stat, DEC);
    Serial.print("\tsg");
    Serial.print(gyro_stat, DEC);
    Serial.print("\tsa");
    Serial.print(accel_stat, DEC);
    Serial.print("\tsm");
    Serial.print(mag_stat, DEC);
    #endif

    Serial.print('\n');

    delay(sample_rate_delay_ms);
}

void setup() {
  buggy.begin();

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  // int8_t temp = bno.getTemp();
  // Serial.print("Current Temperature: ");
  // Serial.print(temp);
  // Serial.println(" C");
  // Serial.println("");

  bno.setExtCrystalUse(true);

}

void loop() {

    while (buggy.available()) {
        buggy.readSerial();
        // int status = buggy.readSerial();
        // if (status == 2) {  // start event
        //
        // }
        // else if (status == 1) {  // stop event
        //
        // }
        // else if (status == 0) {  // user command
        //
        // }
    }


    if (!buggy.isPaused()) {
        updateIMU();
    }
}
