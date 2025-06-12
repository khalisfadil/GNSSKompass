#pragma once

#include <cstdint>

namespace decodeNav {

    struct DataFrameID20 {
        // System Status (Bytes 6-7)
        bool SystemFailure = false;
        bool AccelerometerSensorFailure = false;
        bool GyroscopeSensorFailure = false;
        bool MagnetometerSensorFailure = false;
        bool GNSSFailureSecondaryAntenna = false;
        bool GNSSFailurePrimaryAntenna = false;
        bool AccelerometerOverRange = false;
        bool GyroscopeOverRange = false;
        bool MagnetometerOverRange = false;
        bool MinimumTemperatureAlarm = false;
        bool MaximumTemperatureAlarm = false;
        bool GNSSAntennaConnectionBroken = false;
        bool DataOutputOverflowAlarm = false;

        // Filter Status (Bytes 8-9)
        bool OrientationFilterInitialised = false;
        bool NavigationFilterInitialised = false;
        bool HeadingInitialised = false;
        bool UTCTimeInitialised = false;
        uint8_t GNSSFixStatus = 0; // 3-bit field (0-7) for GNSS fix type
        bool Event1 = false;
        bool Event2 = false;
        bool InternalGNSSEnabled = false;
        bool DualAntennaHeadingActive = false;
        bool VelocityHeadingEnabled = false;
        bool GNSSFixInterrupted = false;
        bool ExternalPositionActive = false;
        bool ExternalVelocityActive = false;
        bool ExternalHeadingActive = false;

        // Navigation Data
        double unixTime = 0.0; // Unix time in seconds (with microsecond precision)
        double latitude = 0.0; // Latitude in radians
        double longitude = 0.0; // Longitude in radians
        double altitude = 0.0; // Altitude in meters
        float velocityNorth = 0.0f; // Velocity north in m/s
        float velocityEast = 0.0f; // Velocity east in m/s
        float velocityDown = 0.0f; // Velocity down in m/s
        float accelX = 0.0f; // Body acceleration X in m/s^2
        float accelY = 0.0f; // Body acceleration Y in m/s^2
        float accelZ = 0.0f; // Body acceleration Z in m/s^2
        float gForce = 0.0f; // G force in g
        float roll = 0.0f; // Roll angle in radians
        float pitch = 0.0f; // Pitch angle in radians
        float yaw = 0.0f; // Yaw (heading) angle in radians
        float angularVelocityX = 0.0f; // Angular velocity X in rad/s
        float angularVelocityY = 0.0f; // Angular velocity Y in rad/s
        float angularVelocityZ = 0.0f; // Angular velocity Z in rad/s
        float sigmaLatitude = 0.0f; // Latitude standard deviation in meters
        float sigmaLongitude = 0.0f; // Longitude standard deviation in meters
        float sigmaAltitude = 0.0f; // Altitude standard deviation in meters

        // Clear all vectors
        void clear() {
            SystemFailure = false;
            AccelerometerSensorFailure = false;
            GyroscopeSensorFailure = false;
            MagnetometerSensorFailure = false;
            GNSSFailureSecondaryAntenna = false;
            GNSSFailurePrimaryAntenna = false;
            AccelerometerOverRange = false;
            GyroscopeOverRange = false;
            MagnetometerOverRange = false;
            MinimumTemperatureAlarm = false;
            MaximumTemperatureAlarm = false;
            GNSSAntennaConnectionBroken = false;
            DataOutputOverflowAlarm = false;

            // Filter Status (Bytes 8-9)
            OrientationFilterInitialised = false;
            NavigationFilterInitialised = false;
            HeadingInitialised = false;
            UTCTimeInitialised = false;
            GNSSFixStatus = 0; // 3-bit field (0-7) for GNSS fix type
            Event1 = false;
            Event2 = false;
            InternalGNSSEnabled = false;
            DualAntennaHeadingActive = false;
            VelocityHeadingEnabled = false;
            GNSSFixInterrupted = false;
            ExternalPositionActive = false;
            ExternalVelocityActive = false;
            ExternalHeadingActive = false;

            // Navigation Data
            unixTime = 0.0; // Unix time in seconds (with microsecond precision)
            latitude = 0.0; // Latitude in radians
            longitude = 0.0; // Longitude in radians
            altitude = 0.0; // Altitude in meters
            velocityNorth = 0.0f; // Velocity north in m/s
            velocityEast = 0.0f; // Velocity east in m/s
            velocityDown = 0.0f; // Velocity down in m/s
            accelX = 0.0f; // Body acceleration X in m/s^2
            accelY = 0.0f; // Body acceleration Y in m/s^2
            accelZ = 0.0f; // Body acceleration Z in m/s^2
            gForce = 0.0f; // G force in g
            roll = 0.0f; // Roll angle in radians
            pitch = 0.0f; // Pitch angle in radians
            yaw = 0.0f; // Yaw (heading) angle in radians
            angularVelocityX = 0.0f; // Angular velocity X in rad/s
            angularVelocityY = 0.0f; // Angular velocity Y in rad/s
            angularVelocityZ = 0.0f; // Angular velocity Z in rad/s
            sigmaLatitude = 0.0f; // Latitude standard deviation in meters
            sigmaLongitude = 0.0f; // Longitude standard deviation in meters
            sigmaAltitude = 0.0f; // Altitude standard deviation in meters
        }
    };

} // namespace decodeNav