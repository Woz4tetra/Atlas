
#include <RazerIMUstick.h>

RazerIMU imu;

int ax, ay, az;
int gx, gy, gz;
int mx, my, mz;

void setup()
{
    Serial.begin(9600);

    Serial.print("initializing...");
    imu.begin();
    Serial.println("done!");
}


void loop()
{
    imu.update(
        ax, ay, az,
        gx, gy, gz,
        mx, my, mz
    );

    Serial.print("A:");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\n");

    Serial.print("G:");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.print(gz);
    Serial.print("\n");

    Serial.print("M:");
    Serial.print(mx);
    Serial.print("\t");
    Serial.print(my);
    Serial.print("\t");
    Serial.print(mz);
    Serial.print("\n");

    //Sample at 10Hz
    delay(100);
}
