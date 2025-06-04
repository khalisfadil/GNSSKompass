#include "callback_gnssComp.hpp" 

// -----------------------------------------------------------------------------

void decode_ID20(const std::vector<uint8_t>& packet, DataFrameID20& frame) {

    // Define static expected size
    static constexpr size_t expected_size_ = 105; // 5-byte header + 100-byte data
    static constexpr uint8_t packet_id_ = 0x14;   // Packet ID is 20 (0x14)
    static constexpr uint8_t data_size_ = 100;    // Packet data length is 100 bytes

    // Validate packet size
    if (packet.size() != expected_size_) {
        std::cerr << "Invalid packet size: " << packet.size() << ", expected: " << expected_size_ << std::endl;
        return;
    }

    // Validate Packet ID (at offset 1)
    uint8_t packet_id = packet[1];
    if (packet_id != packet_id_) {
        std::cerr << "Invalid packet ID: 0x" << std::hex << static_cast<int>(packet_id)
                    << std::dec << " (expected 0x14)" << std::endl;
        return;
    }

    // Validate Packet Length (at offset 2)
    uint8_t packet_length = packet[2];
    if (packet_length != data_size_) {
        std::cerr << "Invalid packet length: " << static_cast<int>(packet_length)
                    << ", expected: " << data_size_ << std::endl;
        return;
    }

    // Optional: Validate Header LRC (at offset 0)
    // uint8_t header_lrc = packet[0];
    // TODO: Implement LRC validation (requires LRC calculation logic)
    // if (!validate_lrc(packet.data(), header_lrc)) {
    //     std::cerr << "Invalid Header LRC: 0x" << std::hex << static_cast<int>(header_lrc) << std::dec << std::endl;
    //     return;
    // }

    // Optional: Validate CRC (at offset 3-4)
    // uint16_t crc_raw;
    // std::memcpy(&crc_raw, packet.data() + 3, sizeof(uint16_t));
    // uint16_t crc = le16toh(crc_raw);
    // TODO: Implement CRC16-CCITT validation (covers packet data from byte 5 onwards)
    // if (!validate_crc(packet.data() + header_size_, data_size_, crc)) {
    //     std::cerr << "Invalid CRC: 0x" << std::hex << crc << std::dec << std::endl;
    //     return;
    // }

    // Clear the output frame to ensure a clean state
    frame.clear();

    // System Status (Bytes 5-6, u16)
    uint16_t system_status_raw;
    std::memcpy(&system_status_raw, packet.data() + 5, sizeof(uint16_t));
    uint16_t system_status = le16toh(system_status_raw);
    frame.SystemFailure = (system_status & 0x0001) != 0; // Bit 0
    frame.AccelerometerSensorFailure = (system_status & 0x0002) != 0; // Bit 1
    frame.GyroscopeSensorFailure = (system_status & 0x0004) != 0; // Bit 2
    frame.MagnetometerSensorFailure = (system_status & 0x0008) != 0; // Bit 3
    frame.GNSSFailureSecondaryAntenna = (system_status & 0x0010) != 0; // Bit 4
    frame.GNSSFailurePrimaryAntenna = (system_status & 0x0020) != 0; // Bit 5
    frame.AccelerometerOverRange = (system_status & 0x0040) != 0; // Bit 6
    frame.GyroscopeOverRange = (system_status & 0x0080) != 0; // Bit 7
    frame.MagnetometerOverRange = (system_status & 0x0100) != 0; // Bit 8
    frame.MinimumTemperatureAlarm = (system_status & 0x0400) != 0; // Bit 10
    frame.MaximumTemperatureAlarm = (system_status & 0x0800) != 0; // Bit 11
    frame.GNSSAntennaConnectionBroken = (system_status & 0x4000) != 0; // Bit 14
    frame.DataOutputOverflowAlarm = (system_status & 0x8000) != 0; // Bit 15

    // Filter Status (Bytes 7-8, u16)
    uint16_t filter_status_raw;
    std::memcpy(&filter_status_raw, packet.data() + 7, sizeof(uint16_t));
    uint16_t filter_status = le16toh(filter_status_raw);
    frame.OrientationFilterInitialised = (filter_status & 0x0001) != 0; // Bit 0
    frame.NavigationFilterInitialised = (filter_status & 0x0002) != 0; // Bit 1
    frame.HeadingInitialised = (filter_status & 0x0004) != 0; // Bit 2
    frame.UTCTimeInitialised = (filter_status & 0x0008) != 0; // Bit 3
    frame.GNSSFixStatus = (filter_status >> 4) & 0x07; // Bits 4-6 (3-bit field)
    frame.Event1 = (filter_status & 0x0080) != 0; // Bit 7
    frame.Event2 = (filter_status & 0x0100) != 0; // Bit 8
    frame.InternalGNSSEnabled = (filter_status & 0x0200) != 0; // Bit 9
    frame.DualAntennaHeadingActive = (filter_status & 0x0400) != 0; // Bit 10
    frame.VelocityHeadingEnabled = (filter_status & 0x0800) != 0; // Bit 11
    frame.GNSSFixInterrupted = (filter_status & 0x1000) != 0; // Bit 12
    frame.ExternalPositionActive = (filter_status & 0x2000) != 0; // Bit 13
    frame.ExternalVelocityActive = (filter_status & 0x4000) != 0; // Bit 14
    frame.ExternalHeadingActive = (filter_status & 0x8000) != 0; // Bit 15

    // Unix Time (Bytes 9-16, u32 seconds + u32 microseconds)
    uint32_t seconds_raw, microseconds_raw;
    std::memcpy(&seconds_raw, packet.data() + 9, sizeof(uint32_t));
    std::memcpy(&microseconds_raw, packet.data() + 13, sizeof(uint32_t));
    uint32_t seconds = le32toh(seconds_raw);
    uint32_t microseconds = le32toh(microseconds_raw);
    if (microseconds > 999999) {
        std::cerr << "Invalid microseconds: " << microseconds << " (expected 0-999999)" << std::endl;
        return;
    }
    frame.unixTime = static_cast<double>(seconds) + static_cast<double>(microseconds) * 1e-6;

    // Latitude, Longitude, Altitude (Bytes 17-40, fp64)
    std::memcpy(&frame.latitude, packet.data() + 17, sizeof(double));
    std::memcpy(&frame.longitude, packet.data() + 25, sizeof(double));
    std::memcpy(&frame.altitude, packet.data() + 33, sizeof(double));

    // Velocity North, East, Down (Bytes 41-52, fp32)
    std::memcpy(&frame.velocityNorth, packet.data() + 41, sizeof(float));
    std::memcpy(&frame.velocityEast, packet.data() + 45, sizeof(float));
    std::memcpy(&frame.velocityDown, packet.data() + 49, sizeof(float));

    // Body Acceleration X, Y, Z (Bytes 53-64, fp32)
    std::memcpy(&frame.accelX, packet.data() + 53, sizeof(float));
    std::memcpy(&frame.accelY, packet.data() + 57, sizeof(float));
    std::memcpy(&frame.accelZ, packet.data() + 61, sizeof(float));

    // G Force (Bytes 65-68, fp32)
    std::memcpy(&frame.gForce, packet.data() + 65, sizeof(float));

    // Roll, Pitch, Yaw (Bytes 69-80, fp32)
    std::memcpy(&frame.roll, packet.data() + 69, sizeof(float));
    std::memcpy(&frame.pitch, packet.data() + 73, sizeof(float));
    std::memcpy(&frame.yaw, packet.data() + 77, sizeof(float));

    // Angular Velocity X, Y, Z (Bytes 81-92, fp32)
    std::memcpy(&frame.angularVelocityX, packet.data() + 81, sizeof(float));
    std::memcpy(&frame.angularVelocityY, packet.data() + 85, sizeof(float));
    std::memcpy(&frame.angularVelocityZ, packet.data() + 89, sizeof(float));

    // Standard Deviations (Bytes 93-104, fp32)
    std::memcpy(&frame.sigmaLatitude, packet.data() + 93, sizeof(float));
    std::memcpy(&frame.sigmaLongitude, packet.data() + 97, sizeof(float));
    std::memcpy(&frame.sigmaAltitude, packet.data() + 101, sizeof(float));
}

// -----------------------------------------------------------------------------

void decode_ID25(const std::vector<uint8_t>& packet, DataFrameID25& frame) {
    // Define static expected size
    static constexpr size_t header_size_ = 5;      // 5-byte header (LRC, ID, Length, CRC)
    static constexpr size_t data_size_ = 12;       // Packet data length is 12 bytes
    static constexpr size_t expected_size_ = header_size_ + data_size_; // Total packet size is 17 bytes
    static constexpr uint8_t packet_id_ = 0x19;    // Packet ID is 25 (0x19)

    // Validate packet size
    if (packet.size() != expected_size_) {
        std::cerr << "Invalid packet size: " << packet.size() << ", expected: " << expected_size_ << std::endl;
        return;
    }

    // Validate Packet ID (at offset 1)
    uint8_t packet_id = packet[1];
    if (packet_id != packet_id_) {
        std::cerr << "Invalid packet ID: 0x" << std::hex << static_cast<int>(packet_id)
                  << std::dec << " (expected 0x19)" << std::endl;
        return;
    }

    // Validate Packet Length (at offset 2)
    uint8_t packet_length = packet[2];
    if (packet_length != data_size_) {
        std::cerr << "Invalid packet length: " << static_cast<int>(packet_length)
                  << ", expected: " << static_cast<int>(data_size_) << std::endl;
        return;
    }

    // Optional: Validate Header LRC (at offset 0)
    // uint8_t header_lrc = packet[0];
    // TODO: Implement LRC validation (requires LRC calculation logic)
    // if (!validate_lrc(packet.data(), header_lrc)) {
    //     std::cerr << "Invalid Header LRC: 0x" << std::hex << static_cast<int>(header_lrc) << std::dec << std::endl;
    //     return;
    // }

    // Optional: Validate CRC (at offset 3-4)
    // uint16_t crc_raw;
    // std::memcpy(&crc_raw, packet.data() + 3, sizeof(uint16_t));
    // uint16_t crc = le16toh(crc_raw);
    // TODO: Implement CRC16-CCITT validation (covers packet data from byte 5 onwards)
    // if (!validate_crc(packet.data() + header_size_, data_size_, crc)) {
    //     std::cerr << "Invalid CRC: 0x" << std::hex << crc << std::dec << std::endl;
    //     return;
    // }

    // Clear the output frame to ensure a clean state
    frame.clear();

    // Velocity Standard Deviations (Bytes 5-16, fp32)
    std::memcpy(&frame.sigmaVelocityNorth, packet.data() + 5, sizeof(float));
    std::memcpy(&frame.sigmaVelocityEast, packet.data() + 9, sizeof(float));
    std::memcpy(&frame.sigmaVelocityDown, packet.data() + 13, sizeof(float));
}

// -----------------------------------------------------------------------------

void decode_ID26(const std::vector<uint8_t>& packet, DataFrameID26& frame) {
    // Define static expected size
    static constexpr size_t header_size_ = 5;      // 5-byte header (LRC, ID, Length, CRC)
    static constexpr size_t data_size_ = 12;       // Packet data length is 12 bytes
    static constexpr size_t expected_size_ = header_size_ + data_size_; // Total packet size is 17 bytes
    static constexpr uint8_t packet_id_ = 0x1A;    // Packet ID is 26 (0x1A)

    // Validate packet size
    if (packet.size() != expected_size_) {
        std::cerr << "Invalid packet size: " << packet.size() << ", expected: " << expected_size_ << std::endl;
        return;
    }

    // Validate Packet ID (at offset 1)
    uint8_t packet_id = packet[1];
    if (packet_id != packet_id_) {
        std::cerr << "Invalid packet ID: 0x" << std::hex << static_cast<int>(packet_id)
                  << std::dec << " (expected 0x1A)" << std::endl;
        return;
    }

    // Validate Packet Length (at offset 2)
    uint8_t packet_length = packet[2];
    if (packet_length != data_size_) {
        std::cerr << "Invalid packet length: " << static_cast<int>(packet_length)
                  << ", expected: " << static_cast<int>(data_size_) << std::endl;
        return;
    }

    // Optional: Validate Header LRC (at offset 0)
    // uint8_t header_lrc = packet[0];
    // TODO: Implement LRC validation (requires LRC calculation logic)
    // if (!validate_lrc(packet.data(), header_lrc)) {
    //     std::cerr << "Invalid Header LRC: 0x" << std::hex << static_cast<int>(header_lrc) << std::dec << std::endl;
    //     return;
    // }

    // Optional: Validate CRC (at offset 3-4)
    // uint16_t crc_raw;
    // std::memcpy(&crc_raw, packet.data() + 3, sizeof(uint16_t));
    // uint16_t crc = le16toh(crc_raw);
    // TODO: Implement CRC16-CCITT validation (covers packet data from byte 5 onwards)
    // if (!validate_crc(packet.data() + header_size_, data_size_, crc)) {
    //     std::cerr << "Invalid CRC: 0x" << std::hex << crc << std::dec << std::endl;
    //     return;
    // }

    // Clear the output frame to ensure a clean state
    frame.clear();

    // Orientation Standard Deviations (Bytes 5-16, fp32)
    std::memcpy(&frame.sigmaRoll, packet.data() + 5, sizeof(float));
    std::memcpy(&frame.sigmaPitch, packet.data() + 9, sizeof(float));
    std::memcpy(&frame.sigmaHeading, packet.data() + 13, sizeof(float));
}

// -----------------------------------------------------------------------------

void decode_ID28(const std::vector<uint8_t>& packet, DataFrameID28& frame) {
    // Define static expected size
    static constexpr size_t header_size_ = 5;      // 5-byte header (LRC, ID, Length, CRC)
    static constexpr size_t data_size_ = 48;       // Packet data length is 48 bytes
    static constexpr size_t expected_size_ = header_size_ + data_size_; // Total packet size is 53 bytes
    static constexpr uint8_t packet_id_ = 0x1C;    // Packet ID is 28 (0x1C)

    // Validate packet size
    if (packet.size() != expected_size_) {
        std::cerr << "Invalid packet size: " << packet.size() << ", expected: " << expected_size_ << std::endl;
        return;
    }

    // Validate Packet ID (at offset 1)
    uint8_t packet_id = packet[1];
    if (packet_id != packet_id_) {
        std::cerr << "Invalid packet ID: 0x" << std::hex << static_cast<int>(packet_id)
                  << std::dec << " (expected 0x1C)" << std::endl;
        return;
    }

    // Validate Packet Length (at offset 2)
    uint8_t packet_length = packet[2];
    if (packet_length != data_size_) {
        std::cerr << "Invalid packet length: " << static_cast<int>(packet_length)
                  << ", expected: " << static_cast<int>(data_size_) << std::endl;
        return;
    }

    // Optional: Validate Header LRC (at offset 0)
    // uint8_t header_lrc = packet[0];
    // TODO: Implement LRC validation (requires LRC calculation logic)
    // if (!validate_lrc(packet.data(), header_lrc)) {
    //     std::cerr << "Invalid Header LRC: 0x" << std::hex << static_cast<int>(header_lrc) << std::dec << std::endl;
    //     return;
    // }

    // Optional: Validate CRC (at offset 3-4)
    // uint16_t crc_raw;
    // std::memcpy(&crc_raw, packet.data() + 3, sizeof(uint16_t));
    // uint16_t crc = le16toh(crc_raw);
    // TODO: Implement CRC16-CCITT validation (covers packet data from byte 5 onwards)
    // if (!validate_crc(packet.data() + header_size_, data_size_, crc)) {
    //     std::cerr << "Invalid CRC: 0x" << std::hex << crc << std::dec << std::endl;
    //     return;
    // }

    // Clear the output frame to ensure a clean state
    frame.clear();

    // IMU and Environmental Sensor Measurements (Bytes 5-52, fp32)
    std::memcpy(&frame.accelX, packet.data() + 5, sizeof(float));
    std::memcpy(&frame.accelY, packet.data() + 9, sizeof(float));
    std::memcpy(&frame.accelZ, packet.data() + 13, sizeof(float));
    std::memcpy(&frame.gyroX, packet.data() + 17, sizeof(float));
    std::memcpy(&frame.gyroY, packet.data() + 21, sizeof(float));
    std::memcpy(&frame.gyroZ, packet.data() + 25, sizeof(float));
    std::memcpy(&frame.magX, packet.data() + 29, sizeof(float));
    std::memcpy(&frame.magY, packet.data() + 33, sizeof(float));
    std::memcpy(&frame.magZ, packet.data() + 37, sizeof(float));
    std::memcpy(&frame.imuTemperature, packet.data() + 41, sizeof(float));
    std::memcpy(&frame.pressure, packet.data() + 45, sizeof(float));
    std::memcpy(&frame.pressureTemperature, packet.data() + 49, sizeof(float));
}

// -----------------------------------------------------------------------------

void decode_ID29(const std::vector<uint8_t>& packet, DataFrameID29& frame) {
    // Define static expected size
    static constexpr size_t header_size_ = 5;      // 5-byte header (LRC, ID, Length, CRC)
    static constexpr size_t data_size_ = 74;       // Packet data length is 74 bytes
    static constexpr size_t expected_size_ = header_size_ + data_size_; // Total packet size is 79 bytes
    static constexpr uint8_t packet_id_ = 0x1D;    // Packet ID is 29 (0x1D)

    // Validate packet size
    if (packet.size() != expected_size_) {
        std::cerr << "Invalid packet size: " << packet.size() << ", expected: " << expected_size_ << std::endl;
        return;
    }

    // Validate Packet ID (at offset 1)
    uint8_t packet_id = packet[1];
    if (packet_id != packet_id_) {
        std::cerr << "Invalid packet ID: 0x" << std::hex << static_cast<int>(packet_id)
                  << std::dec << " (expected 0x1D)" << std::endl;
        return;
    }

    // Validate Packet Length (at offset 2)
    uint8_t packet_length = packet[2];
    if (packet_length != data_size_) {
        std::cerr << "Invalid packet length: " << static_cast<int>(packet_length)
                  << ", expected: " << static_cast<int>(data_size_) << std::endl;
        return;
    }

    // Optional: Validate Header LRC (at offset 0)
    // uint8_t header_lrc = packet[0];
    // TODO: Implement LRC validation (requires LRC calculation logic)
    // if (!validate_lrc(packet.data(), header_lrc)) {
    //     std::cerr << "Invalid Header LRC: 0x" << std::hex << static_cast<int>(header_lrc) << std::dec << std::endl;
    //     return;
    // }

    // Optional: Validate CRC (at offset 3-4)
    // uint16_t crc_raw;
    // std::memcpy(&crc_raw, packet.data() + 3, sizeof(uint16_t));
    // uint16_t crc = le16toh(crc_raw);
    // TODO: Implement CRC16-CCITT validation (covers packet data from byte 5 onwards)
    // if (!validate_crc(packet.data() + header_size_, data_size_, crc)) {
    //     std::cerr << "Invalid CRC: 0x" << std::hex << crc << std::dec << std::endl;
    //     return;
    // }

    // Clear the output frame to ensure a clean state
    frame.clear();

    // Unix Time (Bytes 5-12, u32 seconds + u32 microseconds)
    uint32_t seconds_raw, microseconds_raw;
    std::memcpy(&seconds_raw, packet.data() + 5, sizeof(uint32_t));
    std::memcpy(&microseconds_raw, packet.data() + 9, sizeof(uint32_t));
    uint32_t seconds = le32toh(seconds_raw);
    uint32_t microseconds = le32toh(microseconds_raw);
    if (microseconds > 999999) {
        std::cerr << "Invalid microseconds: " << microseconds << " (expected 0-999999)" << std::endl;
        return;
    }
    frame.unixTime = static_cast<double>(seconds) + static_cast<double>(microseconds) * 1e-6;

    // Latitude, Longitude, Height (Bytes 13-36, fp64)
    std::memcpy(&frame.latitude, packet.data() + 13, sizeof(double));
    std::memcpy(&frame.longitude, packet.data() + 21, sizeof(double));
    std::memcpy(&frame.height, packet.data() + 29, sizeof(double));

    // Velocity North, East, Down (Bytes 37-48, fp32)
    std::memcpy(&frame.velocityNorth, packet.data() + 37, sizeof(float));
    std::memcpy(&frame.velocityEast, packet.data() + 41, sizeof(float));
    std::memcpy(&frame.velocityDown, packet.data() + 45, sizeof(float));

    // Standard Deviations (Bytes 49-60, fp32)
    std::memcpy(&frame.sigmaLatitude, packet.data() + 49, sizeof(float));
    std::memcpy(&frame.sigmaLongitude, packet.data() + 53, sizeof(float));
    std::memcpy(&frame.sigmaHeight, packet.data() + 57, sizeof(float));

    // Tilt and Heading (Bytes 61-68, fp32)
    std::memcpy(&frame.tilt, packet.data() + 61, sizeof(float));
    std::memcpy(&frame.heading, packet.data() + 65, sizeof(float));

    // Tilt and Heading Standard Deviations (Bytes 69-76, fp32)
    std::memcpy(&frame.sigmaTilt, packet.data() + 69, sizeof(float));
    std::memcpy(&frame.sigmaHeading, packet.data() + 73, sizeof(float));

    // Status (Bytes 77-78, u16)
    uint16_t status_raw;
    std::memcpy(&status_raw, packet.data() + 77, sizeof(uint16_t));
    uint16_t status = le16toh(status_raw);
    frame.gnssFixStatus = status & 0x07; // Bits 0-2
    frame.dopplerVelocityValid = (status & 0x08) != 0; // Bit 3
    frame.timeValid = (status & 0x10) != 0; // Bit 4
    frame.externalGNSS = (status & 0x20) != 0; // Bit 5
    frame.tiltValid = (status & 0x40) != 0; // Bit 6
}

// -----------------------------------------------------------------------------
