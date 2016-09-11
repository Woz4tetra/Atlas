#include <SPI.h>
#include <TFT.h>
#define RESET  7
#define DC   8

#define BROWN 9
#define RED 10
#define ORANGE 11
#define YELLOW 12
#define PHASE_A BROWN
#define PHASE_A_ ORANGE
#define PHASE_B RED
#define PHASE_B_ YELLOW
void feedForward(int steps);
void feedReverse(int steps);
uint8_t stepper_pins;//8 low bits written to stepper pins
uint8_t stepper_sequence[] = {
  0x08, 0x0C, 0x04, 0x06, 0x02, 0x03, 0x01, 0x09};
void setup()
{
	stepper_pins = 1;
	pinMode(PHASE_A, OUTPUT);
	pinMode(PHASE_A_, OUTPUT);
	pinMode(PHASE_B, OUTPUT);
	pinMode(PHASE_B_, OUTPUT);
  // myScreen.begin();
  // myScreen.background(0, 0, 240);
  // myScreen.println(F("Arduino TFT Bitmap Example"));
}
void loop()
{while(true){
        int i;
        for(i = 0; i < 2000; i++) {
	if(stepper_pins & 0x1)
		digitalWrite(PHASE_A, HIGH);
	else
		digitalWrite(PHASE_A, LOW);
	if(stepper_pins & 0x2)
		digitalWrite(PHASE_B, HIGH);
	else
		digitalWrite(PHASE_B, LOW);
	if(stepper_pins & 0x4)
		digitalWrite(PHASE_A_, HIGH);
	else
		digitalWrite(PHASE_A_, LOW);
	if(stepper_pins & 0x8)
		digitalWrite(PHASE_B_, HIGH);
	else
		digitalWrite(PHASE_B_, LOW);
        stepper_pins = stepper_sequence[i % 8];
	//stepper_pins = (stepper_pins << 1);
        //stepper_pins = (stepper_pins | (stepper_pins >> 4)) & 0xF;
        delay(50);  // stepper speed
        if(millis() % 1000 < 500)
                digitalWrite(13, HIGH);
        else
                digitalWrite(13, LOW);
        }}
        //while(true) {}
}
