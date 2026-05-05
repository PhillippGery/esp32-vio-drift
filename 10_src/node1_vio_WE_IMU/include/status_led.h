#pragma once

namespace drift {

// Initialize the hardware pin
void ledInit();

// Turn the XIAO built-in LED on or off
void ledSet(bool isOn);

} // namespace drift