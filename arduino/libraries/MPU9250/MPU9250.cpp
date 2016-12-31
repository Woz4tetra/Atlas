//
//  MPU9250.cpp
//  
//
//  Created by Benjamin Warwick on 11/17/15.
//
//

#include <Wire.h>
#include <Arduino.h>
#include <DueTimer.h>
#include "MPU9250.h"

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission();
    
    // Read Nbytes
    Wire.requestFrom(Address, Nbytes); 
    uint8_t index=0;
    while (Wire.available())
        Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}

volatile bool intFlag = false;

void callback() {
    intFlag=true;
}

void mpu_setup()
{
    // Arduino initializations
    Wire.begin();
    
    //    Serial.println("type c to print data");
    
    // Set accelerometers low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS,29,0x06);
    // Set gyroscope low pass filter at 5Hz
    I2CwriteByte(MPU9250_ADDRESS,26,0x06);
    
    
    // Configure gyroscope range
    I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
    // Configure accelerometers range
    I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
    // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
    
    // Request continuous magnetometer measurements in 16 bits
    I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
    
    pinMode(13, OUTPUT);
    Timer7.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
    Timer7.start(10000);         // initialize timer1, and set a 1/2 second period
    
    Serial.println("MPU initialized");
}

void mpu_update(uint8_t* accelgyro_buf, uint8_t* magnet_buf)
{
    while (!intFlag);
    intFlag = false;
    
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, accelgyro_buf);
    
    uint8_t ST1;
    do {
        I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
//        if (!(ST1 & 0x01)) {
//            Serial.println("Magnet read failure...");
//        }
    }
    while (!(ST1 & 0x01));
    
    // Read magnetometer data    
    I2Cread(MAG_ADDRESS, 0x03, 7, magnet_buf);
}
