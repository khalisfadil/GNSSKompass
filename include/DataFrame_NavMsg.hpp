#pragma once

#include <cstdint>

namespace decodeNav {

    struct DataFrameNavMsg {

        // Navigation Data (Bytes 0-71)
        double timestamp = 0.0;          // UTC second of the day
        double latitude = 0.0;          // Latitude in radians
        double longitude = 0.0;         // Longitude in radians
        float altitude = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        float velU = 0.0f;
        float velV = 0.0f;
        float velW = 0.0f;
        float velP = 0.0f;
        float velQ = 0.0f;
        float velR = 0.0f;
        float accU = 0.0f;
        float accV = 0.0f;
        float accW = 0.0f;
        float accP = 0.0f;
        float accQ = 0.0f; 
        float accR = 0.0f; 
        float velN = 0.0f;
        float velE = 0.0f;
        float velD = 0.0f;

        // Clear all fields
        void clear() {
            timestamp = 0.0;
            latitude = 0.0;
            longitude = 0.0;
            altitude = 0.0f;
            roll = 0.0f;
            pitch = 0.0f;
            yaw = 0.0f;
            velU = 0.0f;
            velV = 0.0f;
            velW = 0.0f;
            velP = 0.0f;
            velQ = 0.0f;
            velR = 0.0f;
            accU = 0.0f;
            accV = 0.0f;
            accW = 0.0f;
            accP = 0.0f;
            accQ = 0.0f; 
            accR = 0.0f; 
            velN = 0.0f;
            velE = 0.0f;
            velD = 0.0f;
        }
    };

} // namespace decodeNav