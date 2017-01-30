#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define DEFAULT_RATE 115200
#define WHOIAM "imu"
#define LED13 13

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS 10

Adafruit_BNO055 bno = Adafruit_BNO055();

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
    Serial.print("delay:");
    Serial.print(BNO055_SAMPLERATE_DELAY_MS);
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

        // if (character == '\n')
        // {
        // Serial.println(command);
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

void updateIMU() {
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    /* Display the floating point data */
    // Serial.print("X: ");
    Serial.print(euler.x(), 4);
    Serial.print('\t');
    Serial.print(euler.y(), 4);
    Serial.print('\t');
    Serial.print(euler.z(), 4);
    Serial.print('\t');

    imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    Serial.print(magnetometer.x(), 4);
    Serial.print('\t');
    Serial.print(magnetometer.y(), 4);
    Serial.print('\t');
    Serial.print(magnetometer.z(), 4);
    Serial.print('\t');

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    Serial.print(gyro.x(), 4);
    Serial.print('\t');
    Serial.print(gyro.y(), 4);
    Serial.print('\t');
    Serial.print(gyro.z(), 4);
    Serial.print('\t');

    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    Serial.print(accel.x(), 4);
    Serial.print('\t');
    Serial.print(accel.y(), 4);
    Serial.print('\t');
    Serial.print(accel.z(), 4);
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
  Serial.begin(DEFAULT_RATE);

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

  // Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

}

void loop() {
  if (!paused) {
      updateIMU();
  }
  else {
      delay(100);
  }
  readSerial();
}
