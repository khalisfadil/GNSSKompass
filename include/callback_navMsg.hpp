#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cstdint>
#include <iostream>

#include <DataFrame_NavMsg.hpp>

namespace decodeNav {

    class NavMsgCallback {
        public:
            explicit NavMsgCallback();

            void decode_NavMsg(const std::vector<uint8_t>& packet, DataFrameNavMsg& frame); 
        
    };

} // namespace decodeNav

