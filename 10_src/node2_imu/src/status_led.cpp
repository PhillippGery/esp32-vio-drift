#include "status_led.h"
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// ESP32 Feather V2 specific pins
#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL 0
#endif
#ifndef NEOPIXEL_POWER
#define NEOPIXEL_POWER 2 
#endif

namespace drift {

// Create the NeoPixel object (1 LED, standard GRB format)
static Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void ledInit() {
    // 1. Turn on the physical power to the NeoPixel!
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);

    // 2. Start the data stream
    pixel.begin();
    pixel.setBrightness(20); // Keep this low so it doesn't blind you
    pixel.setPixelColor(0, pixel.Color(0, 0, 0)); // Start OFF
    pixel.show();
}

void ledSet(StatusColor color) {
    switch(color) {
        case StatusColor::RED:
            pixel.setPixelColor(0, pixel.Color(50, 0, 0));
            break;
        case StatusColor::GREEN:
            pixel.setPixelColor(0, pixel.Color(0, 50, 0));
            break;
        case StatusColor::OFF:
        default:
            pixel.setPixelColor(0, pixel.Color(0, 0, 0));
            break;
    }
    pixel.show(); // Push the data to the hardware
}

} // namespace drift