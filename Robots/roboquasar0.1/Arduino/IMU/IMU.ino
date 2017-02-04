#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Atlasbuggy.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS 100

Atlasbuggy buggy("imu");

Adafruit_BNO055 bno = Adafruit_BNO055();

void updateIMU() {
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    Serial.print("ex");
    Serial.print(euler.x(), 4);
    Serial.print('\t');
    Serial.print("ey");
    Serial.print(euler.y(), 4);
    Serial.print('\t');
    Serial.print("ez");
    Serial.print(euler.z(), 4);
    Serial.print('\t');

    imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    Serial.print("mx");
    Serial.print(magnetometer.x(), 4);
    Serial.print('\t');
    Serial.print("my");
    Serial.print(magnetometer.y(), 4);
    Serial.print('\t');
    Serial.print("mz");
    Serial.print(magnetometer.z(), 4);
    Serial.print('\t');

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    Serial.print("gx");
    Serial.print(gyro.x(), 4);
    Serial.print('\t');
    Serial.print("gy");
    Serial.print(gyro.y(), 4);
    Serial.print('\t');
    Serial.print("gz");
    Serial.print(gyro.z(), 4);
    Serial.print('\t');

    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    Serial.print("ax");
    Serial.print(accel.x(), 4);
    Serial.print('\t');
    Serial.print("ay");
    Serial.print(accel.y(), 4);
    Serial.print('\t');
    Serial.print("az");
    Serial.print(accel.z(), 4);
    Serial.print('\t');

    imu::Vector<3> accel_wg = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    Serial.print("agx");
    Serial.print(accel_wg.x(), 4);
    Serial.print('\t');
    Serial.print("agy");
    Serial.print(accel_wg.y(), 4);
    Serial.print('\t');
    Serial.print("agz");
    Serial.print(accel_wg.z(), 4);
    Serial.print('\n');

    /*
    // Quaternion data
    imu::Quaternion quat = bno.getQuat();
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");
    /*

    /* Display calibration status for each sensor. */
    // uint8_t system, gyro, accel, mag = 0;
    // bno.getCalibration(&system, &gyro, &accel, &mag);
    // Serial.print("CALIBRATION: Sys=");
    // Serial.print(system, DEC);
    // Serial.print(" Gyro=");
    // Serial.print(gyro, DEC);
    // Serial.print(" Accel=");
    // Serial.print(accel, DEC);
    // Serial.print(" Mag=");
    // Serial.print(mag, DEC);
    // Serial.print('\n');

    delay(BNO055_SAMPLERATE_DELAY_MS);
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
  int8_t temp = bno.getTemp();
  // Serial.print("Current Temperature: ");
  // Serial.print(temp);
  // Serial.println(" C");
  // Serial.println("");

  bno.setExtCrystalUse(true);

  pinMode(LED13, OUTPUT);

  // Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

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
    }


    if (!buggy.isPaused()) {
        updateIMU();
    }
}
