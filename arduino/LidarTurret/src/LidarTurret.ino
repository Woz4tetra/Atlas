
// #define USE_SOFTSERIAL
#define VALUE_ARRAY_LEN 255

#include <Lidar.h>

Lidar lidar;
unsigned long t0 = micros();
volatile int encoderValues[VALUE_ARRAY_LEN];
volatile int encoderValue = 0;
volatile int valueIndex = 0;
int currentIndex = 0;

void setup()
{
    #ifdef USE_SOFTSERIAL
    Serial.begin(9600);
    #endif

    lidar.begin();

    // Timer1.initialize(200);
    // Timer1.attachInterrupt(readAnalog);
}

// void readAnalog()
// {
//     encoderValue = analogRead(ENCODER_PIN);
//     if (encoderValues[valueIndex] != encoderValue)
//     {
//         valueIndex++;
//         if (valueIndex >= VALUE_ARRAY_LEN) {
//             valueIndex = 0;
//         }
//     }
//     encoderValues[valueIndex] = encoderValue;
// }

void loop()
{
    // lidar.setColor(lidar.orange);

    // noInterrupts();
    lidar.update();
    // currentIndex++;
    // if (currentIndex >= VALUE_ARRAY_LEN) {
    //     currentIndex = 0;
    // }
    // interrupts();
    //
    // lidar.setColor(lidar.green);

    lidar.checkSerial();

    // if ((micros() - t0) > 1000) {
    //     lidar.writeDistance();
    //     t0 = micros();
    // }
}
