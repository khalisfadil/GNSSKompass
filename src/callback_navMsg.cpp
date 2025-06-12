#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cstdint>
#include <iostream>

#include <callback_navMsg.hpp> 

namespace decodeNav {

    // -----------------------------------------------------------------------------

    NavMsgCallback::NavMsgCallback(){}

    // -----------------------------------------------------------------------------

    void NavMsgCallback::decode_NavMsg(const std::vector<uint8_t>& packet, DataFrameNavMsg& frame) {
        // Define constants
        constexpr double tscorr = 17.427051482643094;
        constexpr double tsgain = 2.224677453845612e-05;
        constexpr size_t expected_size_ = 100; // Total packet size

        // Validate packet size
        if (packet.size() != expected_size_) {
            std::cerr << "Invalid packet size: " << packet.size() << ", expected: " << expected_size_ << std::endl;
            return;
        }

        // Clear the output frame
        frame.clear();

        // Latitude, Longitude (Bytes 0-15, fp64)
        std::memcpy(&frame.latitude, packet.data() + 0, sizeof(double));
        std::memcpy(&frame.longitude, packet.data() + 8, sizeof(double));

        // Altitude (Bytes 16-19, fp32)
        std::memcpy(&frame.altitude, packet.data() + 16, sizeof(float));

        // Roll, Pitch, Yaw (Bytes 20-31, fp32)
        std::memcpy(&frame.roll, packet.data() + 20, sizeof(float));
        std::memcpy(&frame.pitch, packet.data() + 24, sizeof(float));
        std::memcpy(&frame.yaw, packet.data() + 28, sizeof(float));

        // Body Velocities U, V, W (Bytes 32-43, fp32)
        std::memcpy(&frame.velU, packet.data() + 32, sizeof(float));
        std::memcpy(&frame.velV, packet.data() + 36, sizeof(float));
        std::memcpy(&frame.velW, packet.data() + 40, sizeof(float));

        // Angular Velocities P, Q, R (Bytes 44-55, fp32)
        std::memcpy(&frame.velP, packet.data() + 44, sizeof(float));
        std::memcpy(&frame.velQ, packet.data() + 48, sizeof(float));
        std::memcpy(&frame.velR, packet.data() + 52, sizeof(float));

        // Body Accelerations U, V, W (Bytes 56-67, fp32)
        std::memcpy(&frame.accU, packet.data() + 56, sizeof(float));
        std::memcpy(&frame.accV, packet.data() + 60, sizeof(float));
        std::memcpy(&frame.accW, packet.data() + 64, sizeof(float));

        // Angular Accelerations P, Q, R (Bytes 68-79, fp32)
        std::memcpy(&frame.accP, packet.data() + 68, sizeof(float));
        std::memcpy(&frame.accQ, packet.data() + 72, sizeof(float));
        std::memcpy(&frame.accR, packet.data() + 76, sizeof(float));

        // NED Velocities N, E, D (Bytes 80-91, fp32)
        std::memcpy(&frame.velN, packet.data() + 80, sizeof(float));
        std::memcpy(&frame.velE, packet.data() + 84, sizeof(float));
        std::memcpy(&frame.velD, packet.data() + 88, sizeof(float));

        // Timestamp (Bytes 92-99, fp64)
        double timestamp_raw;
        std::memcpy(&timestamp_raw, packet.data() + 92, sizeof(double));
        frame.timestamp = timestamp_raw - ((timestamp_raw * tsgain) + tscorr);
    }

} // namespace decodeNav



