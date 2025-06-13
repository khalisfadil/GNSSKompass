#include "navMath.hpp"
#include <cmath>
#include <cassert>

namespace navMath {

    Eigen::Vector3d NavMath::LLA2NED(double lat, double lon, double alt,
                                    double originlat, double originlon, double originalt) {
        // Input validation
        assert(std::abs(lat) <= M_PI / 2 && std::abs(originlat) <= M_PI / 2 &&
            "Latitude must be in [-pi/2, pi/2]");

        // Constants according to WGS84
        constexpr double a = 6378137.0;              // Semi-major axis (m)
        constexpr double e2 = 0.00669437999014132;   // Squared eccentricity

        // Location of data points
        double dphi = lat - originlat;
        double dlam = SymmetricalAngle(lon - originlon);
        double dh = alt - originalt;

        // Useful definitions
        double cp = std::cos(originlat);
        double sp = std::sin(originlat); // Fixed: was sin(originlon)
        double tmp1 = std::sqrt(1 - e2 * sp * sp);
        double tmp3 = tmp1 * tmp1 * tmp1;
        double dlam2 = dlam * dlam;   // Fixed: was dlam.*dlam
        double dphi2 = dphi * dphi;   // Fixed: was dphi.*dphi

        // Transformations
        double E = (a / tmp1 + originalt) * cp * dlam -
                (a * (1 - e2) / tmp3 + originalt) * sp * dphi * dlam + // Fixed: was dphi.*dlam
                cp * dlam * dh;                                       // Fixed: was dlam.*dh
        double N = (a * (1 - e2) / tmp3 + originalt) * dphi +
                1.5 * cp * sp * a * e2 * dphi2 +
                sp * sp * dh * dphi +                              // Fixed: was dh.*dphi
                0.5 * sp * cp * (a / tmp1 + originalt) * dlam2;
        double D = -(dh - 0.5 * (a - 1.5 * a * e2 * cp * cp + 0.5 * a * e2 + originalt) * dphi2 -
                    0.5 * cp * cp * (a / tmp1 - originalt) * dlam2);

        return Eigen::Vector3d(N, E, D);
    }

    Eigen::MatrixXd NavMath::LLA2NED(const Eigen::VectorXd& lat, const Eigen::VectorXd& lon, const Eigen::VectorXd& alt,
                                    double originlat, double originlon, double originalt) {
        // Input validation
        assert(lat.size() == lon.size() && lat.size() == alt.size() &&
            "Inputs lat, lon, alt must have the same size!");
        assert(lat.size() > 0 && "Input arrays must not be empty!");
        assert((lat.array().abs() <= M_PI / 2).all() && std::abs(originlat) <= M_PI / 2 &&
            "Latitude must be in [-pi/2, pi/2]");

        // Constants according to WGS84
        constexpr double a = 6378137.0;              // Semi-major axis (m)
        constexpr double e2 = 0.00669437999014132;   // Squared eccentricity

        // Location of data points
        Eigen::VectorXd dphi = lat.array() - originlat;
        Eigen::VectorXd dlam = SymmetricalAngle(lon.array() - originlon);
        Eigen::VectorXd dh = alt.array() - originalt;

        // Useful definitions
        double cp = std::cos(originlat);
        double sp = std::sin(originlat);
        double tmp1 = std::sqrt(1 - e2 * sp * sp);
        double tmp3 = tmp1 * tmp1 * tmp1;
        Eigen::VectorXd dlam2 = dlam.array().square();
        Eigen::VectorXd dphi2 = dphi.array().square();

        // Transformations
        Eigen::MatrixXd result(lat.size(), 3); // Columns: N, E, D
        result.col(1) = ((a / tmp1 + originalt) * cp * dlam.array() -
                        (a * (1 - e2) / tmp3 + originalt) * sp * (dphi.array() * dlam.array()) +
                        cp * (dlam.array() * dh.array())).matrix(); // E
        result.col(0) = ((a * (1 - e2) / tmp3 + originalt) * dphi.array() +
                        1.5 * cp * sp * a * e2 * dphi2.array() +
                        sp * sp * (dh.array() * dphi.array()) +
                        0.5 * sp * cp * (a / tmp1 + originalt) * dlam2.array()).matrix(); // N
        result.col(2) = -(dh.array() -
                        0.5 * (a - 1.5 * a * e2 * cp * cp + 0.5 * a * e2 + originalt) * dphi2.array() -
                        0.5 * cp * cp * (a / tmp1 - originalt) * dlam2.array()).matrix(); // D

        return result;
    }

    double NavMath::SymmetricalAngle(double x) {
        // Constants for 2π and π
        constexpr double TWO_PI = 6.28318530717959;       // 2 * π
        constexpr double PI = 3.14159265358979;           // π
        constexpr double ONE_OVER_TWO_PI = 0.159154943091895; // 1 / (2 * π)

        // Normalize angle: x = x - 2π * trunc(x / (2π))
        x = x - TWO_PI * std::trunc(x * ONE_OVER_TWO_PI);
        // Adjust to range [-π, +π): y = x + 2π * (x < -π) - 2π * (x >= π)
        x = x + TWO_PI * static_cast<double>(x < -PI) - TWO_PI * static_cast<double>(x >= PI);
        return x;
    }

    Eigen::VectorXd NavMath::SymmetricalAngle(const Eigen::VectorXd& x) {
        constexpr double TWO_PI = 6.28318530717959;
        constexpr double PI = 3.14159265358979;
        constexpr double ONE_OVER_TWO_PI = 0.159154943091895;

        Eigen::VectorXd y = x;
        Eigen::ArrayXd scaled = y.array() * ONE_OVER_TWO_PI;
        y = y.array() - TWO_PI * scaled.unaryExpr([](double v) { return std::trunc(v); });
        y = y.array() + TWO_PI * (y.array() < -PI).cast<double>() - TWO_PI * (y.array() >= PI).cast<double>();
        return y;
    }

} // namespace navMath