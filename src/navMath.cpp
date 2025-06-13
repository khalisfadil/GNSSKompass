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
        constexpr double PI = M_PI;
        constexpr double TWO_PI = 2.0 * M_PI;

        double y = std::remainder(x, TWO_PI);

        // std::remainder(x, 2*PI) returns a value in [-PI, PI].
        // For the range [-PI, PI), if the result is PI, it should be mapped to -PI.
        if (y == PI) { // Note: direct floating point comparison. std::remainder should be exact for PI.
            y = -PI;
        }
        return y;
    }

    Eigen::VectorXd NavMath::SymmetricalAngle(const Eigen::VectorXd& x) {
        constexpr double PI = M_PI;
        constexpr double TWO_PI = 2.0 * M_PI;

        return x.unaryExpr([PI, TWO_PI](double val) {
            double y_val = std::remainder(val, TWO_PI);
            return (y_val == PI) ? -PI : y_val;
        });
    }

    Eigen::VectorXd NavMath::Normalize(const Eigen::VectorXd& u) {
        // Ensure input vector has at least one element
        if (u.size() == 0) {
            return Eigen::VectorXd();
        }

        // Compute the Euclidean norm
        double len = u.norm();

        // Handle near-zero vector
        if (len < 1e-12) {
            Eigen::VectorXd y = Eigen::VectorXd::Zero(u.size());
            y(0) = 1.0;
            return y;
        }

        // Normalize the vector
        return u / len;
    }

    Eigen::Vector4d NavMath::getQuat(double roll, double pitch, double yaw) {
        // Compute half-angles
        double phi = 0.5 * roll;
        double theta = 0.5 * pitch;
        double psi = 0.5 * yaw;

        // Compute trigonometric functions
        double c1 = std::cos(phi);
        double c2 = std::cos(theta);
        double c3 = std::cos(psi);
        double s1 = std::sin(phi);
        double s2 = std::sin(theta);
        double s3 = std::sin(psi);

        // Construct quaternion [qw, qx, qy, qz]
        Eigen::Vector4d u;
        u << c1 * c2 * c3 + s1 * s2 * s3,  // qw
            s1 * c2 * c3 - c1 * s2 * s3,  // qx
            c1 * s2 * c3 + s1 * c2 * s3,  // qy
            c1 * c2 * s3 - s1 * s2 * c3;  // qz

        // Normalize and return
        return Normalize(u).cast<double>();
    }

    Eigen::Matrix3d NavMath::Cb2n(const Eigen::Vector4d& q) {
        // Ensure input quaternion has correct size
        if (q.size() != 4) {
            return Eigen::Matrix3d::Identity(); // Return identity matrix for invalid input
        }

        // Quaternion components
        double q0 = q(0); // qw
        double q1 = q(1); // qx
        double q2 = q(2); // qy
        double q3 = q(3); // qz

        // Compute products
        double q0q0 = q0 * q0;
        double q1q1 = q1 * q1;
        double q2q2 = q2 * q2;
        double q3q3 = q3 * q3;
        double q1q2 = q1 * q2;
        double q0q3 = q0 * q3;
        double q1q3 = q1 * q3;
        double q0q2 = q0 * q2;
        double q2q3 = q2 * q3;
        double q0q1 = q0 * q1;

        // Construct rotation matrix
        Eigen::Matrix3d C;
        C << q0q0 + q1q1 - q2q2 - q3q3,  2.0 * (q1q2 - q0q3),          2.0 * (q1q3 + q0q2),
            2.0 * (q1q2 + q0q3),        q0q0 - q1q1 + q2q2 - q3q3,    2.0 * (q2q3 - q0q1),
            2.0 * (q1q3 - q0q2),        2.0 * (q2q3 + q0q1),          q0q0 - q1q1 - q2q2 + q3q3;

        return C;
    }

    Eigen::Matrix4d NavMath::TransformMatrix(const Eigen::VectorXd& x) {
        // Ensure input vector has at least 7 elements
        if (x.size() < 7) {
            return Eigen::Matrix4d::Identity(); // Return identity matrix for invalid input
        }

        // Extract position (x(1:3)) and quaternion (x(4:7))
        Eigen::Vector3d p = x.segment<3>(0); // [px, py, pz]
        Eigen::Vector4d q = x.segment<4>(3); // [qw, qx, qy, qz]

        // Compute rotation matrix
        Eigen::Matrix3d C = Cb2n(q);

        // Construct 4x4 transformation matrix
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = C; // Set rotation part
        T.block<3,1>(0,3) = p; // Set translation part

        return T;
    }

} // namespace navMath