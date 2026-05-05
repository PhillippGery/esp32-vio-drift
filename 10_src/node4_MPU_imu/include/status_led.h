#pragma once

namespace drift {

// Define our clean status states
enum class StatusColor {
    OFF,
    RED,
    GREEN
};

// Initialize the hardware
void ledInit();

// The one-liner to change the color
void ledSet(StatusColor color);

} // namespace drift
