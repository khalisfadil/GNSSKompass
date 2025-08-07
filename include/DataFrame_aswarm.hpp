#pragma once
#include <cstdint>

namespace decodeNav {
    struct DataFrameAswarm {
        // Header and validity
        uint8_t header = 0;
        bool isValid = false;
        // Navigation data (all doubles for high precision)
        double navTimestampUTC = 0.0;  // Navigation timestamp in UTC (seconds)
        double navLat = 0.0;           // Navigation latitude (degrees or radians, depending on convention)
        double navLon = 0.0;           // Navigation longitude (degrees or radians)
        double navAlt = 0.0;           // Navigation altitude (meters)
        double navRoll = 0.0;          // Navigation roll angle (radians)
        double navPitch = 0.0;         // Navigation pitch angle (radians)
        double navYaw = 0.0;           // Navigation yaw angle (radians)
        double navVelU = 0.0;          // Navigation velocity U (body frame, m/s)
        double navVelV = 0.0;          // Navigation velocity V (body frame, m/s)
        double navVelW = 0.0;          // Navigation velocity W (body frame, m/s)
        double navVelP = 0.0;          // Navigation angular velocity P (body frame, rad/s)
        double navVelQ = 0.0;          // Navigation angular velocity Q (body frame, rad/s)
        double navVelR = 0.0;          // Navigation angular velocity R (body frame, rad/s)
        double navStdNorth = 0.0;      // Standard deviation north (position, meters)
        double navStdEast = 0.0;       // Standard deviation east (position, meters)
        double navStdDown = 0.0;       // Standard deviation down (position, meters)
        double navStdVelNorth = 0.0;   // Standard deviation velocity north (m/s)
        double navStdVelEast = 0.0;    // Standard deviation velocity east (m/s; 
        double navStdVelDown = 0.0;    // Standard deviation velocity down (m/s)
        double navStdRoll = 0.0;       // Standard deviation roll (radians)
        double navStdPitch = 0.0;      // Standard deviation pitch (radians)
        double navStdYaw = 0.0;        // Standard deviation yaw (radians)
        // IMU data
        double imuTimestampUTC = 0.0;  // IMU timestamp in UTC (seconds)
        // Acceleration (3D vector, body frame, m/s^2)
        double accelerationX = 0.0;
        double accelerationY = 0.0;
        double accelerationZ = 0.0;
        // Angular rate (3D vector, body frame, rad/s)
        double angularRateX = 0.0;
        double angularRateY = 0.0;
        double angularRateZ = 0.0;
        // Clear all fields
        void clear() {
            header = 0;
            isValid = false;
            navTimestampUTC = 0.0;
            navLat = 0.0;
            navLon = 0.0;
            navAlt = 0.0;
            navRoll = 0.0;
            navPitch = 0.0;
            navYaw = 0.0;
            navVelU = 0.0;
            navVelV = 0.0;
            navVelW = 0.0;
            navVelP = 0.0;
            navVelQ = 0.0;
            navVelR = 0.0;
            navStdNorth = 0.0;
            navStdEast = 0.0;
            navStdDown = 0.0;
            navStdVelNorth = 0.0;
            navStdVelEast = 0.0;
            navStdVelDown = 0.0;
            navStdRoll = 0.0;
            navStdPitch = 0.0;
            navStdYaw = 0.0;
            imuTimestampUTC = 0.0;
            accelerationX = 0.0;
            accelerationY = 0.0;
            accelerationZ = 0.0;
            angularRateX = 0.0;
            angularRateY = 0.0;
            angularRateZ = 0.0;
        }
    };
} // namespace decodeNav