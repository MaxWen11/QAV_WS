#ifndef UADL_CONTROL_GEOMETRY_H
#define UADL_CONTROL_GEOMETRY_H

#include <Eigen/Geometry>
#include <array>

namespace uadl {
struct ThrustConfig {
    double gravity = 9.81;
    double mass = 0.9;
    double hover_fraction = 0.5;
    double max_tilt = 1.3962634015954636;
    bool voltage_model = false;
    double k1 = 2.10, k2 = 1.0, k3 = 0.0;
};
struct MappedCommand {
    bool valid = false;
    Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
    Eigen::Vector3d executed_input = Eigen::Vector3d::Zero();
    double thrust = 0.0;
};
// u is the commanded net inertial acceleration after gravity compensation
// (Eq. 56). The collective thrust and reference attitude satisfy
// F_T,cmd R_cmd e3 = m0 (u + g e3). executed_input is reconstructed from the
// final attitude and thrust command after ALL clipping/mapping operations,
// u = (F_T,cmd/m0) R_cmd e3 - g e3 (Section VI-B).
MappedCommand mapAcceleration(const Eigen::Vector3d& u, double yaw,
                              double voltage, const ThrustConfig& config,
                              const std::array<Eigen::Vector2d, 3>& limits);
// Response label: the IMU specific force rotated into the inertial frame
// plus the gravity vector -g e3 (Section VI-C).
Eigen::Vector3d specificForceToNetAcceleration(const Eigen::Vector3d& force,
                                              const Eigen::Quaterniond& body_to_world,
                                              double gravity);
} // namespace uadl
#endif
