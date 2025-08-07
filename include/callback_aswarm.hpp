#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cstdint>
#include <iostream>

#include <DataFrame_aswarm.hpp>
// Include the header where DataFrameAswarm is defined, e.g., <callback_aswarm.hpp>
// Assuming it's similar to the example's setup.

namespace decodeNav {

    // -----------------------------------------------------------------------------
    class AswarmCallback {
        public:
            explicit AswarmCallback();
            // -----------------------------------------------------------------------------
            void decode_Aswarm(const std::vector<uint8_t>& packet, DataFrameAswarm& frame);
    };

} // namespace decodeNav