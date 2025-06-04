#pragma once

#include <cstdint>

struct DataFrameID28 {
    // IMU and Environmental Sensor Measurements (Bytes 0-47)
    float accelX = 0.0f;          // Accelerometer X in m/s^2
    float accelY = 0.0f;          // Accelerometer Y in m/s^2
    float accelZ = 0.0f;          // Accelerometer Z in m/s^2
    float gyroX = 0.0f;           // Gyroscope X in rad/s
    float gyroY = 0.0f;           // Gyroscope Y in rad/s
    float gyroZ = 0.0f;           // Gyroscope Z in rad/s
    float magX = 0.0f;            // Magnetometer X in mG
    float magY = 0.0f;            // Magnetometer Y in mG
    float magZ = 0.0f;            // Magnetometer Z in mG
    float imuTemperature = 0.0f;  // IMU temperature in deg C
    float pressure = 0.0f;        // Pressure in Pascals
    float pressureTemperature = 0.0f; // Pressure temperature in deg C

    // Clear all fields
    void clear() {
        accelX = 0.0f;
        accelY = 0.0f;
        accelZ = 0.0f;
        gyroX = 0.0f;
        gyroY = 0.0f;
        gyroZ = 0.0f;
        magX = 0.0f;
        magY = 0.0f;
        magZ = 0.0f;
        imuTemperature = 0.0f;
        pressure = 0.0f;
        pressureTemperature = 0.0f;
    }
};