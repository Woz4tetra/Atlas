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
    void setColor(int r, int g, int b);
    void setColor(int *rgb);
    unsigned int getEncoderCounts();
    unsigned int getEncoderRotations();

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

    unsigned int _encoderCounts;
    unsigned int _encoderRotations;
    int _distance;
    unsigned long _enc_t1;
    unsigned long _enc_dt;
    unsigned long _enc_t0;

    bool _paused;
    char _commandType;
    String _command;
    char _character;

    int _motorSpeed;

    void initColors();
    void writeToSerial();
    void calibrate();
};
