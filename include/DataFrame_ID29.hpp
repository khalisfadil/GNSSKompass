#pragma once

#include <cstdint>

namespace decodeNav {

    struct DataFrameID29 {
        // Navigation Data (Bytes 0-71)
        double unixTime = 0.0;          // Unix time in seconds (with microsecond precision)
        double latitude = 0.0;          // Latitude in radians
        double longitude = 0.0;         // Longitude in radians
        double height = 0.0;            // Height in meters
        float velocityNorth = 0.0f;     // Velocity north in m/s
        float velocityEast = 0.0f;      // Velocity east in m/s
        float velocityDown = 0.0f;      // Velocity down in m/s
        float sigmaLatitude = 0.0f;     // Latitude standard deviation in meters
        float sigmaLongitude = 0.0f;    // Longitude standard deviation in meters
        float sigmaHeight = 0.0f;       // Height standard deviation in meters
        float tilt = 0.0f;              // Tilt in radians
        float heading = 0.0f;           // Heading in radians
        float sigmaTilt = 0.0f;         // Tilt standard deviation in radians
        float sigmaHeading = 0.0f;      // Heading standard deviation in radians

        // Status Flags (Bytes 72-73)
        uint8_t gnssFixStatus = 0;      // GNSS fix status (0-7)
        bool dopplerVelocityValid = false; // Doppler velocity valid
        bool timeValid = false;         // Time valid
        bool externalGNSS = false;      // External GNSS
        bool tiltValid = false;         // Tilt valid

        // Clear all fields
        void clear() {
            unixTime = 0.0;
            latitude = 0.0;
            longitude = 0.0;
            height = 0.0;
            velocityNorth = 0.0f;
            velocityEast = 0.0f;
            velocityDown = 0.0f;
            sigmaLatitude = 0.0f;
            sigmaLongitude = 0.0f;
            sigmaHeight = 0.0f;
            tilt = 0.0f;
            heading = 0.0f;
            sigmaTilt = 0.0f;
            sigmaHeading = 0.0f;
            gnssFixStatus = 0;
            dopplerVelocityValid = false;
            timeValid = false;
            externalGNSS = false;
            tiltValid = false;
        }
    };

} // namespace decodeNav