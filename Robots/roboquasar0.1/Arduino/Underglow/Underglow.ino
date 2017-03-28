#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#include <Atlasbuggy.h>

#define PIN 6

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

#define LED_NUM 30

Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_NUM, PIN, NEO_GRB + NEO_KHZ800);
Atlasbuggy buggy("underglow");

#define SIGNAL_DELAY 1
#define SIGNAL_INCREMENT 3
#define SIGNAL_CYCLES 2

int signal_r, signal_g, signal_b = 0;
uint32_t time0 = millis();
uint32_t time1 = millis();

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and maximize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void setup() {
    buggy.begin();
    // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
    // #if defined (__AVR_ATtiny85__)
    //   if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
    // #endif
    // End of trinket special code

    strip.begin();
    strip.show();

    // flashColors(strip.Color(0, 255, 0), strip.Color(255, 255, 255), 4, 100);
    fadeColors(0, 0, 0, 255, 255, 255, 1, SIGNAL_DELAY, SIGNAL_INCREMENT);
    fadeColors(255, 255, 255, 0, 255, 0, SIGNAL_CYCLES, SIGNAL_DELAY, SIGNAL_INCREMENT);
    fadeColors(255, 255, 255, 0, 0, 0, 1, SIGNAL_DELAY, SIGNAL_INCREMENT);

     buggy.setInitData(String(LED_NUM));
}

uint16_t i, j;
bool pauseCycle = false;
void loop() {
    while (buggy.available()) {
        int status = buggy.readSerial();

        if (status == 2) {  // start event
            fadeColors(255, 255, 255, 1, SIGNAL_DELAY, SIGNAL_INCREMENT);
            fadeColors(255, 255, 255, 0, 0, 255, SIGNAL_CYCLES, SIGNAL_DELAY, SIGNAL_INCREMENT);
        }
        else if (status == 1) {  // stop event
            fadeColors(255, 0, 0, SIGNAL_CYCLES * 2, SIGNAL_DELAY, SIGNAL_INCREMENT);
            // fadeColors(255, 255, 255, 255, 0, 0, SIGNAL_CYCLES, SIGNAL_DELAY, SIGNAL_INCREMENT);
            fadeColors(255, 255, 255, 0, 0, 0, 1, SIGNAL_DELAY, SIGNAL_INCREMENT);
            // colorWipe(strip.Color(255, 255, 255), 20);
        }
        else if (status == 5) {  // Received the letter 's'
            pauseCycle = true;
        }
        else if (status == 0)
        {
            String command = buggy.getCommand();
            if (command.charAt(0) == 'l')
            {
                pauseCycle = true;
                int led_num = command.substring(1, 4).toInt();
                int r = command.substring(4, 7).toInt();
                int g = command.substring(7, 10).toInt();
                int b = command.substring(10, 13).toInt();
                if (command.length() > 13)
                {
                    int stop_num = command.substring(13, 16).toInt();
                    for (int index = led_num; index < stop_num; index++) {
                        strip.setPixelColor(index, strip.Color(r, g, b));
                    }
                }
                else {
                    strip.setPixelColor(led_num, strip.Color(r, g, b));
                }
            }
            else if (command.charAt(0) == 'd') {
                strip.show();
            }
            else if (command.charAt(0) == 'r') {
                pauseCycle = false;
            }
            else if (command.length() > 1 && command.substring(0, 2).equals("fr")) {
                pauseCycle = true;
                int cycle_num = command.substring(2, 3).toInt();
                fadeColors(random(0, 255), random(0, 255), random(0, 255), cycle_num, SIGNAL_DELAY, SIGNAL_INCREMENT);
            }
            else if (command.charAt(0) == 'f' && command.length() == 13) {
                int cycle_num = command.substring(1, 4).toInt();
                int r = command.substring(4, 7).toInt();
                int g = command.substring(7, 10).toInt();
                int b = command.substring(10, 13).toInt();
                fadeColors(255, 255, 255, 1, SIGNAL_DELAY, SIGNAL_INCREMENT);
                fadeColors(r, g, b, cycle_num, SIGNAL_DELAY, SIGNAL_INCREMENT);
                fadeColors(0, 0, 0, 1, SIGNAL_DELAY, SIGNAL_INCREMENT);
            }
            else if (command.length() > 4 && command.substring(0, 4).equals("wipe")) {
                pauseCycle = true;
                int r = command.substring(4, 7).toInt();
                int g = command.substring(7, 10).toInt();
                int b = command.substring(10, 13).toInt();
                int wait = command.substring(13, 16).toInt();
                colorWipe(strip.Color(r, g, b), wait);
            }
            else if (command.charAt(0) == 'g' && command.length() == 3) {
                pauseCycle = true;
                fancyGradient(command.substring(1,3).toInt());
            }
        }
    }

    if (!buggy.isPaused() && !pauseCycle)
    {
        strip.setPixelColor(i, Wheel(((i * 0x100 / strip.numPixels()) + j) & 0xff));
        i++;
        if (i >= strip.numPixels())
        {
            if (time0 > millis())  time0 = millis();
            if ((millis() - time0) > 2)
            {
                i = 0;
                j++;
                if (j >= 0x500) {  // 256 * 5
                   j = 0;
                }
                if (time0 > millis()) {
                    time0 = millis();
                }
                time0 = millis();
            }
            if (time1 > millis())  time1 = millis();
            if ((millis() - time1) > 15) {
                strip.show();
                time1 = millis();
            }
            // delay(1);
        }
    }
}

void fancyGradient(int start){
  //strip.setPixelColor(index, strip.Color(r, g, b))
  int index = 0;
  for(int x = 0; x < LED_NUM; x++) {
    index = (start + x) % LED_NUM;
    byte r = (byte)(sin(x/10) * 255.0);
    byte g = (byte)(x*10);
    byte b = 100;
    strip.setPixelColor(index,strip.Color(r,g,b));
    Serial.print("rgb: ");
    Serial.print(r,HEX);
    Serial.print(g,HEX);
    Serial.println(b,HEX);
  }
  strip.show();
}

void fadeColors(int r, int g, int b, uint16_t cycles, uint8_t wait, int increment) {
    fadeColors(signal_r, signal_g, signal_b, r, g, b, cycles, wait, increment);
}

void fadeColors(int r1, int g1, int b1, int r2, int g2, int b2, uint16_t cycles, uint8_t wait, int increment)
{
    // Serial.print(r1); Serial.print('\t');
    // Serial.print(g1); Serial.print('\t');
    // Serial.print(b1); Serial.print('\n');
    // Serial.print(r2); Serial.print('\t');
    // Serial.print(g2); Serial.print('\t');
    // Serial.print(b2); Serial.print('\n');

    if (cycles % 2 == 0) {
        signal_r = r1;
        signal_g = g1;
        signal_b = b1;
    }
    else {
        signal_r = r2;
        signal_g = g2;
        signal_b = b2;
    }
    int red_diff = abs(r2 - r1);
    int green_diff = abs(g2 - g1);
    int blue_diff = abs(b2 - b1);

    char max_channel = 'r';
    int max_diff = red_diff;

    if (green_diff > max_diff) {
        max_diff = green_diff;
        max_channel = 'g';
    }
    if (blue_diff > max_diff) {
        max_diff = blue_diff;
        max_channel = 'b';
    }
    // Serial.println(max_channel);

    float red_slope = 0.0;
    float green_slope = 0.0;
    float blue_slope = 0.0;

    int start = 0;
    int end = 0;

    bool condition = true;

    switch (max_channel) {
        case 'r':
            if (r2 < r1) {
                increment *= -1;
            }
            break;
        case 'g':
            if (g2 < g1) {
                increment *= -1;
            }
            break;
        case 'b':
            if (b2 < b1) {
                increment *= -1;
            }
            break;
    }

    // Serial.println(cycles);
    for (uint16_t cycle = 0; cycle < cycles; cycle++)
    {
        switch (max_channel) {
            case 'r':
                condition = r1 < r2;
                if (increment < 0) {
                    condition = !condition;
                }
                if (condition) {
                    start = r1;
                    end = r2;
                }
                else {
                    start = r2;
                    end = r1;
                }
                green_slope = (float)(g2 - g1) / (r2 - r1);
                blue_slope = (float)(b2 - b1) / (r2 - r1);

                if (start < end) {
                    for (int value = start; value <= end; value += increment) {
                        setColor(strip.Color(
                                value,
                                green_slope * (value - r1) + g1,
                                blue_slope * (value - r1) + b1
                            )
                        );
                        delay(wait);
                    }
                }
                else if (start > end) {
                    for (int value = start; value >= end; value += increment) {
                        setColor(strip.Color(
                                value,
                                green_slope * (value - r1) + g1,
                                blue_slope * (value - r1) + b1
                            )
                        );
                        delay(wait);
                    }
                }
                break;

            case 'g':
                condition = g1 < g2;
                if (increment < 0) {
                    condition = !condition;
                }
                if (condition) {
                    start = g1;
                    end = g2;
                }
                else {
                    start = g2;
                    end = g1;
                }


                red_slope = (float)(r2 - r1) / (g2 - g1);
                blue_slope = (float)(b2 - b1) / (g2 - g1);
                if (start < end) {
                    for (int value = start; value <= end; value += increment) {
                        setColor(strip.Color(
                                red_slope * (value - g1) + r1,
                                value,
                                blue_slope * (value - g1) + b1
                            )
                        );
                        delay(wait);
                    }
                }
                else {
                    for (int value = start; value >= end; value += increment) {
                        setColor(strip.Color(
                                red_slope * (value - g1) + r1,
                                value,
                                blue_slope * (value - g1) + b1
                            )
                        );
                        delay(wait);
                    }
                }
                break;
            case 'b':
                condition = b1 < b2;
                if (increment < 0) {
                    condition = !condition;
                }
                if (condition) {
                    start = b1;
                    end = b2;
                }
                else {
                    start = b2;
                    end = b1;
                }
                red_slope = (float)(r2 - r1) / (b2 - b1);
                green_slope = (float)(g2 - g1) / (b2 - b1);

                if (start < end) {
                    for (int value = start; value <= end; value += increment) {
                        setColor(strip.Color(
                                red_slope * (value - b1) + r1,
                                green_slope * (value - b1) + g1,
                                value
                            )
                        );
                        delay(wait);
                    }
                }
                else {
                    for (int value = start; value >= end; value += increment) {
                        setColor(strip.Color(
                                red_slope * (value - b1) + r1,
                                green_slope * (value - b1) + g1,
                                value
                            )
                        );
                        delay(wait);
                    }
                }

                break;
        }
        increment *= -1;

    }
}

void flashColors(uint32_t c1, uint32_t c2, uint16_t cycles, uint8_t wait)
{
    for (uint16_t cycle = 0; cycle < cycles; cycle++) {
        setColor(c1);
        delay(wait);
        setColor(c2);
        delay(wait);
    }
}

void setColor(uint32_t c)
{
    for(uint16_t i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, c);
    }
    strip.show();
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
    for(uint16_t i=0; i<strip.numPixels(); i++) {
        strip.setPixelColor(i, c);
        strip.show();
        delay(wait);
    }
}

void rainbow(uint8_t wait) {
    uint16_t i, j;

    for(j=0; j<256; j++) {
        for(i=0; i<strip.numPixels(); i++) {
            strip.setPixelColor(i, Wheel((i+j) & 255));
        }
        strip.show();
        delay(wait);
    }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
    uint16_t i, j;

    for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
        for(i=0; i< strip.numPixels(); i++) {
            strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
        }
        strip.show();
        delay(wait);
    }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
    for (int j=0; j<10; j++) {  //do 10 cycles of chasing
        for (int q=0; q < 3; q++) {
            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
                strip.setPixelColor(i+q, c);    //turn every third pixel on
            }
            strip.show();

            delay(wait);

            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
                strip.setPixelColor(i+q, 0);        //turn every third pixel off
            }
        }
    }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
    for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
        for (int q=0; q < 3; q++) {
            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
                strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
            }
            strip.show();

            delay(wait);

            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
                strip.setPixelColor(i+q, 0);        //turn every third pixel off
            }
        }
    }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85) {
        return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if(WheelPos < 170) {
        WheelPos -= 85;
        return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
