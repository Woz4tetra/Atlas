#include <Arduino.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <SoftwareSerial.h>


#define DEBUG_LIDAR_TURRET

class Lidar {
public:
    Lidar();
    void begin();
    bool update();
    void checkSerial();
    void setMotorSpeed(int speed);
    void stopMotor();
    void setMotorDirection(bool direction);
    void setColor(int r, int g, int b);
    void setColor(int *rgb);
    unsigned int getEncoderCounts();
    unsigned int getEncoderRotations();
    void writeDistance();

    int red[3];
    int green[3];
    int blue[3];

    int orange[3];
    int limeGreen[3];
    int aqua[3];
    int skyBlue[3];
    int slateBlue[3];
    int seafoam[3];
    int banana[3];
    int salmon[3];
private:
    bool _encoderLow;
    SoftwareSerial *_softSerial;
    LIDARLite *_lidarLite;

    int _encoderCounts;
    int _encoderRotations;
    int _distance;
    unsigned long _enc_t1, _enc_dt, _enc_t0;
    unsigned long _serial_t0;
    unsigned long _distance_t0;

    bool _paused;
    char _commandType;
    String _command;
    char _character;

    int _motorSpeed;
    bool _motorDirection;

    float _kp, _kd, _ki;

    float _prev_error, _sum_error;

    void initColors();
    void writeEncoder();
    void calibrate();

    bool goToTick(int goal);
};
