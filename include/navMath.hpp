#pragma once

#include <cstdint>
#include <cmath>
#include <Eigen/Dense>

namespace navMath {
    class NavMath{
        public:

            NavMath() = default;

            // Scalar version: Convert a single LLA coordinate to NED
            Eigen::Vector3d LLA2NED(double lat, double lon, double alt,
                                    double originlat, double originlon, double originalt);

            // Vector version: Convert multiple LLA coordinates to NED
            Eigen::MatrixXd LLA2NED(const Eigen::VectorXd& lat, const Eigen::VectorXd& lon, const Eigen::VectorXd& alt,
                                    double originlat, double originlon, double originalt);

        private:

            // Scalar SymmetricalAngle: Normalize angle to [-π, +π)
            static double SymmetricalAngle(double x);

            // Vector SymmetricalAngle: Normalize array of angles to [-π, +π)
            static Eigen::VectorXd SymmetricalAngle(const Eigen::VectorXd& x);

    };
} // namespace navMath
