#include <callback_aswarm.hpp> 

namespace decodeNav {

        // -----------------------------------------------------------------------------
        AswarmCallback::AswarmCallback() {}
        
        // -----------------------------------------------------------------------------
        void AswarmCallback::decode_Aswarm(const std::vector<uint8_t>& packet, DataFrameAswarm& frame) {
            // Define constants
            constexpr size_t expected_size_ = 234; // Total packet size based on structure

            // Validate packet size
            if (packet.size() != expected_size_) {
                std::cerr << "Invalid packet size: " << packet.size() << ", expected: " << expected_size_ << std::endl;
                return;
            }

            // Clear the output frame
            frame.clear();

            // Header (Byte 0, uint8_t)
            std::memcpy(&frame.header, packet.data() + 0, sizeof(uint8_t));

            // isValid (Byte 1, bool)
            std::memcpy(&frame.isValid, packet.data() + 1, sizeof(bool));

            // Navigation doubles (Bytes 2-177, 22 doubles)
            std::memcpy(&frame.navTimestampUTC, packet.data() + 2, sizeof(double));
            std::memcpy(&frame.navLat, packet.data() + 10, sizeof(double));
            std::memcpy(&frame.navLon, packet.data() + 18, sizeof(double));
            std::memcpy(&frame.navAlt, packet.data() + 26, sizeof(double));
            std::memcpy(&frame.navRoll, packet.data() + 34, sizeof(double));
            std::memcpy(&frame.navPitch, packet.data() + 42, sizeof(double));
            std::memcpy(&frame.navYaw, packet.data() + 50, sizeof(double));
            std::memcpy(&frame.navVelU, packet.data() + 58, sizeof(double));
            std::memcpy(&frame.navVelV, packet.data() + 66, sizeof(double));
            std::memcpy(&frame.navVelW, packet.data() + 74, sizeof(double));
            std::memcpy(&frame.navVelP, packet.data() + 82, sizeof(double));
            std::memcpy(&frame.navVelQ, packet.data() + 90, sizeof(double));
            std::memcpy(&frame.navVelR, packet.data() + 98, sizeof(double));
            std::memcpy(&frame.navStdNorth, packet.data() + 106, sizeof(double));
            std::memcpy(&frame.navStdEast, packet.data() + 114, sizeof(double));
            std::memcpy(&frame.navStdDown, packet.data() + 122, sizeof(double));
            std::memcpy(&frame.navStdVelNorth, packet.data() + 130, sizeof(double));
            std::memcpy(&frame.navStdVelEast, packet.data() + 138, sizeof(double));
            std::memcpy(&frame.navStdVelDown, packet.data() + 146, sizeof(double));
            std::memcpy(&frame.navStdRoll, packet.data() + 154, sizeof(double));
            std::memcpy(&frame.navStdPitch, packet.data() + 162, sizeof(double));
            std::memcpy(&frame.navStdYaw, packet.data() + 170, sizeof(double));

            // IMU timestamp (Bytes 178-185, double)
            std::memcpy(&frame.imuTimestampUTC, packet.data() + 178, sizeof(double));

            // Acceleration (Bytes 186-209, 3 doubles)
            std::memcpy(&frame.accelerationX, packet.data() + 186, sizeof(double));
            std::memcpy(&frame.accelerationY, packet.data() + 194, sizeof(double));
            std::memcpy(&frame.accelerationZ, packet.data() + 202, sizeof(double));

            // Angular rates (Bytes 210-233, 3 doubles)
            std::memcpy(&frame.angularRateX, packet.data() + 210, sizeof(double));
            std::memcpy(&frame.angularRateY, packet.data() + 218, sizeof(double));
            std::memcpy(&frame.angularRateZ, packet.data() + 226, sizeof(double));
        }
} // namesoace decodeNav