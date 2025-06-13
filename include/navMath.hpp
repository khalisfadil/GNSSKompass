#pragma once

#include <cstdint>

namespace navMath {
    class NavMath{
        public:

            NavMath();

            void LLA2NED(const double lat, const double lon, const double alt, const double originlat, const double originlon, const double originalt);

        private:

            double SymmetricalAngle(double x);

            double OMEGA_EARTH = 7.292115e-5;           //Angular rate [rad/s] of the earth w.r.t. the inertial frame.

    };
} // namespace decodeNav