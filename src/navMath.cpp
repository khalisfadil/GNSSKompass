#include <navMath.hpp>
#include <cmath>

namespace navMath {


    NavMath::NavMath(){}

    void NavMath::LLA2NED(const double lat, const double lon, const double alt, const double originlat, const double originlon, const double originalt){
        //Constants according to WGS84
        constexpr double a = 6378137.0;
        constexpr double e2 = 0.00669437999014132;

        // Location of data points
        double dphi = lat - originlat;
        double dlam = SymmetricalAngle(lon - originlon);
        double dh = alt - originalt;
    }

    double NavMath::SymmetricalAngle(double x){
        //Convert a given angle x [rad] to an output angle y [rad] with y being in range [-pi, +pi).
        //x ... Input angle in radians, either scalar or n-dimensional.

        // Constants for 2π and π
        constexpr double TWO_PI = 6.28318530717959; // 2 * π
        constexpr double PI = 3.14159265358979;     // π
        constexpr double ONE_OVER_TWO_PI = 0.159154943091895; // 1 / (2 * π)

        // Normalize angle: x = x - 2π * fix(x / (2π))
        x = x - TWO_PI * std::trunc(x * ONE_OVER_TWO_PI);
        // Adjust to range [-π, +π): y = x + 2π * (x < -π) - 2π * (x >= π)
        x = x + TWO_PI * static_cast<double>(x < -PI) - TWO_PI * static_cast<double>(x >= PI);
        return x;
    }

} // namespace decodeNav