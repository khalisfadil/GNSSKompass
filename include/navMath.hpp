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
            
            //Safe normalization of an input vector.
            static Eigen::VectorXd Normalize(const Eigen::VectorXd& u);
            
            // Computes a normalized quaternion from roll, pitch, and yaw angles.
            static Eigen::Vector4d getQuat(double roll, double pitch, double yaw);
            
            // Calculates the rotation matrix for a given unit quaternion.
            //q Normalized unit quaternion as [qw, qx, qy, qz] (b-frame to n-frame).
            static Eigen::Matrix3d Cb2n(const Eigen::Vector4d& q);

            // Computes a 4x4 homogeneous transformation matrix from a state vector.
            static Eigen::Matrix4d TransformMatrix(const Eigen::VectorXd& x);
    };
} // namespace navMath
