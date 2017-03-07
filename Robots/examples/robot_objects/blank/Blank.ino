
// Make sure the Atlasbuggy Arduino library is in your Arduino's 'libraries' folder
#include <Atlasbuggy.h>


Atlasbuggy buggy("blank");  // replace "blank" with the whoiam ID you want

int property_1 = 64;
#define PROPERTY_2 255;

void setup()
{
    buggy.begin();  // initializes hardware serial to 115200 baud

    String prop1 = String(property_1);
    String prop2 = String(PROPERTY_2);

    // create initialization data. These should be constants of the system
    buggy.setInitData(prop1 + "\t" + prop2);

    // Initialize sensors or actuators
}

void loop()
{
    while (buggy.available()) {
        int status = buggy.readSerial();
        if (status == 2) {  // start event

        }
        else if (status == 1) {  // stop event

        }
        // if serial isn't being read properly, status 5 is if an 's' character comes through
        // stop any intensive processes here
        else if (status == 5) {

        }
    }

    if (!buggy.isPaused()) {
        // unpaused behavior. Avoid putting intensive code outside this block unless you need it
    }
}
