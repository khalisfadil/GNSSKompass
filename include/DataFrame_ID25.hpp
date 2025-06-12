#pragma once

#include <cstdint>

namespace decodeNav {

    struct DataFrameID25 {
        // Velocity Standard Deviations (Bytes 0-11)
        float sigmaVelocityNorth = 0.0f; // Velocity north standard deviation in m/s
        float sigmaVelocityEast = 0.0f;  // Velocity east standard deviation in m/s
        float sigmaVelocityDown = 0.0f;  // Velocity down standard deviation in m/s

        // Clear all fields
        void clear() {
            sigmaVelocityNorth = 0.0f;
            sigmaVelocityEast = 0.0f;
            sigmaVelocityDown = 0.0f;
        }
    };

} // namespace decodeNav