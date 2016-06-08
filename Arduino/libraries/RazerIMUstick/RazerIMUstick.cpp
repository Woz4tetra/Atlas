#include <Wire.h>
#include <RazerIMUstick.h>

RazerIMU::RazerIMU()
{
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    exInt = 0.0;
    eyInt = 0.0;
    ezInt = 0.0;
    twoKp = twoKpDef;
    twoKi = twoKiDef;
    integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
    
    for (int i = 0; i < 3; i++) {
        _accel_data[i] = _magnet_data[i] = _gyro_data[i] = 0;
    }
}

void RazerIMU::begin()
{
    Wire.begin();
    
    init_adxl345();
    init_hmc5843();
    init_itg3200();
}

void RazerIMU::update(int &ax, int &ay, int &az,
                      int &gx, int &gy, int &gz,
                      int &mx, int &my, int &mz)
{
    read_adxl345();
    read_hmc5843();
    read_itg3200();
    
    ax = _accel_data[0]; ay = _accel_data[1]; az = _accel_data[2];
    gx = _gyro_data[0]; gy = _gyro_data[1]; gz = _gyro_data[2];
    mx = _magnet_data[0]; my = _magnet_data[1]; mz = _magnet_data[2];
}

void RazerIMU::filter(float qw, float qx, float qy, float qz)
{
    now = micros();
    sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
    lastUpdate = now;
    
    int ax, ay, az;
    int gx, gy, gz;
    int mx, my, mz;
    
    update(ax, ay, az,
           gx, gy, gz,
           mx, my, mz);
    
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    float qa, qb, qc;
    
    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
    
    // Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
    if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
        float hx, hy, bx, bz;
        float halfwx, halfwy, halfwz;
        
        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
        
        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        
        // Estimated direction of magnetic field
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
        
        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (my * halfwz - mz * halfwy);
        halfey = (mz * halfwx - mx * halfwz);
        halfez = (mx * halfwy - my * halfwx);
    }
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if ((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
        float halfvx, halfvy, halfvz;
        
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Estimated direction of gravity
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        
        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex += (ay * halfvz - az * halfvy);
        halfey += (az * halfvx - ax * halfvz);
        halfez += (ax * halfvy - ay * halfvx);
    }
    
    // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
    if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f)
    {
        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }
        
        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    
    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}


void RazerIMU::i2c_write(int address, byte reg, byte data)
{
    // Send output register address
    Wire.beginTransmission(address);
    Wire.write(reg);
    // Connect to device and send byte
    Wire.write(data); // low byte
    Wire.endTransmission();
}

void RazerIMU::i2c_read(int address, byte reg, int count, byte* data)
{
    int i = 0;

    // Send input register address
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    // Connect to device and request bytes
    Wire.beginTransmission(address);
    Wire.requestFrom(address,count);
    while(Wire.available()) // slave may send less than requested
    {
        char c = Wire.read(); // receive a byte as character
        data[i] = c;
        i++;
    }
    Wire.endTransmission();
}

void RazerIMU::init_adxl345()
{
    byte data = 0;

    i2c_write(ADXL345_ADDRESS, ADXL_REGISTER_PWRCTL, ADXL_PWRCTL_MEASURE);

    //Check to see if it worked!
    i2c_read(ADXL345_ADDRESS, ADXL_REGISTER_PWRCTL, 1, &data);
    Serial.println((unsigned int)data);
}

void RazerIMU::read_adxl345()
{
    byte bytes[6];
    memset(bytes,0,6);

    //read 6 bytes from the ADXL345
    i2c_read(ADXL345_ADDRESS, ADXL345_REGISTER_XLSB, 6, bytes);

    //now unpack the bytes
    for (int i=0;i<3;++i) {
        _accel_data[i] = (int)bytes[2*i] + (((int)bytes[2*i + 1]) << 8);
    }
}

void RazerIMU::init_itg3200()
{
    byte data = 0;

    //Set DLPF to 42 Hz (change it if you want) and
    //set the scale to "Full Scale"
    i2c_write(ITG3200_ADDRESS, ITG3200_REGISTER_DLPF_FS, ITG3200_FULLSCALE | ITG3200_42HZ);

    //Sanity check! Make sure the register value is correct.
    i2c_read(ITG3200_ADDRESS, ITG3200_REGISTER_DLPF_FS, 1, &data);

    Serial.println((unsigned int)data);
}

void RazerIMU::read_itg3200()
{
    byte bytes[6];
    memset(bytes,0,6);

    //read 6 bytes from the ITG3200
    i2c_read(ITG3200_ADDRESS, ITG3200_REGISTER_XMSB, 6, bytes);  //now unpack the bytes
    for (int i=0;i<3;++i) {
        _gyro_data[i] = (int)bytes[2*i + 1] + (((int)bytes[2*i]) << 8);
    }
}

void RazerIMU::init_hmc5843()
{
    byte data = 0;
    //set up continuous measurement
    i2c_write(HMC5843_ADDRESS, HMC5843_REGISTER_MEASMODE, HMC5843_MEASMODE_CONT);

    //Sanity check, make sure the register value is correct.
    i2c_read(HMC5843_ADDRESS, HMC5843_REGISTER_MEASMODE, 1, &data);
    Serial.println((unsigned int)data);
}

void RazerIMU::read_hmc5843()
{
    byte bytes[6];
    memset(bytes,0,6);

    //read 6 bytes from the HMC5843
    i2c_read(HMC5843_ADDRESS, HMC5843_REGISTER_XMSB, 6, bytes);

    //now unpack the bytes
    for (int i=0;i<3;++i) {
        _magnet_data[i] = (int)bytes[2*i + 1] + (((int)bytes[2*i]) << 8);
    }
}

float invSqrt(float number) {
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;
    
    x = number * 0.5F;
    y = number;
    i = * ( long * ) &y;
    i = 0x5f375a86 - ( i >> 1 );
    y = * ( float * ) &i;
    y = y * ( f - ( x * y * y ) );
    return y;
}
