#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cstdint>
#include <iostream>

#include <DataFrame_ID20.hpp>
#include <DataFrame_ID25.hpp>
#include <DataFrame_ID26.hpp>
#include <DataFrame_ID28.hpp>
#include <DataFrame_ID29.hpp>

namespace decodeNav {

    class GnssCompassCallback {
        public:
            explicit GnssCompassCallback();

            void decode_ID20(const std::vector<uint8_t>& packet, DataFrameID20& frame); 
            void decode_ID25(const std::vector<uint8_t>& packet, DataFrameID25& frame); 
            void decode_ID26(const std::vector<uint8_t>& packet, DataFrameID26& frame); 
            void decode_ID28(const std::vector<uint8_t>& packet, DataFrameID28& frame); 
            void decode_ID29(const std::vector<uint8_t>& packet, DataFrameID29& frame); 
        
    };

} // namespace decodeNav