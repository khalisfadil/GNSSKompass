#pragma once

#include <cstdint>

struct DataFrameID26 {
    // Orientation Standard Deviations (Bytes 0-11)
    float sigmaRoll = 0.0f;    // Roll standard deviation in radians
    float sigmaPitch = 0.0f;   // Pitch standard deviation in radians
    float sigmaHeading = 0.0f; // Heading standard deviation in radians

    // Clear all fields
    void clear() {
        sigmaRoll = 0.0f;
        sigmaPitch = 0.0f;
        sigmaHeading = 0.0f;
    }
};