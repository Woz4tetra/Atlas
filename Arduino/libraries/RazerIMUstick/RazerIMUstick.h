
#include "Arduino.h"
#include <Wire.h>

#define  ADXL345_ADDRESS (0xA6 >> 1)
//There are 6 data registers, they are sequential starting
//with the LSB of X.  We'll read all 6 in a burst and won't
//address them individually
#define ADXL345_REGISTER_XLSB (0x32)

//Need to set power control bit to wake up the adxl345
#define ADXL_REGISTER_PWRCTL (0x2D)
#define ADXL_PWRCTL_MEASURE (1 << 3)

#define ITG3200_ADDRESS (0xD0 >> 1)
//request burst of 6 bytes from this address
#define ITG3200_REGISTER_XMSB (0x1D)
#define ITG3200_REGISTER_DLPF_FS (0x16)
#define ITG3200_FULLSCALE (0x03 << 3)
#define ITG3200_42HZ (0x03)

#define HMC5843_ADDRESS (0x3C >> 1)
//First data address of 6 is XMSB.  Also need to set a configuration register for
//continuous measurement
#define HMC5843_REGISTER_XMSB (0x03)
#define HMC5843_REGISTER_MEASMODE (0x02)
#define HMC5843_MEASMODE_CONT (0x00)

#define Kp 2.0f			// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f		// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.5f		// half the sample period

#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.1f) // 2 * integral gain

class RazerIMU {
public:
    RazerIMU();
    void begin();
    void update(int &ax, int &ay, int &az,
                int &gx, int &gy, int &gz,
                int &mx, int &my, int &mz);
    void filter(float qw, float qx, float qy, float qz);
    
private:
    float iq0, iq1, iq2, iq3;
    float exInt, eyInt, ezInt;  // scaled integral error
    volatile float twoKp;      // 2 * proportional gain (Kp)
    volatile float twoKi;      // 2 * integral gain (Ki)
    volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
    volatile float integralFBx,  integralFBy, integralFBz;
    float sampleFreq;
    unsigned long lastUpdate, now;
    
    void i2c_write(int address, byte reg, byte data);
    void i2c_read(int address, byte reg, int count, byte* data);
    
    void init_adxl345();
    void init_itg3200();
    void init_hmc5843();
    
    void read_adxl345();
    void read_itg3200();
    void read_hmc5843();
    
    int _accel_data[3];
    int _gyro_data[3];
    int _magnet_data[3];
};

float invSqrt(float number);