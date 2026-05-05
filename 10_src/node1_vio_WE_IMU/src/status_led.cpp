#include "status_led.h"
#include <Arduino.h>

// The Arduino compiler automatically maps LED_BUILTIN to GPIO 21 for the XIAO S3
#ifndef LED_BUILTIN
#define LED_BUILTIN 21 
#endif

namespace drift {

void ledInit() {
    pinMode(LED_BUILTIN, OUTPUT);
    ledSet(false); // Ensure it starts completely OFF
}

void ledSet(bool isOn) {
    // XIAO LEDs are typically active LOW. 
    // If it behaves backwards on your specific board, swap HIGH and LOW here.
    if (isOn) {
        digitalWrite(LED_BUILTIN, LOW);  // Turn ON
    } else {
        digitalWrite(LED_BUILTIN, HIGH); // Turn OFF
    }
}

} // namespace drift