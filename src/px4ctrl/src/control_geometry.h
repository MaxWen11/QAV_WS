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
// u is net inertial acceleration. executed_input is reconstructed from the
// final attitude and thrust command, after ALL clipping/mapping operations.
MappedCommand mapAcceleration(const Eigen::Vector3d& u, double yaw,
                              double voltage, const ThrustConfig& config,
                              const std::array<Eigen::Vector2d, 3>& limits);
Eigen::Vector3d specificForceToNetAcceleration(const Eigen::Vector3d& force,
                                              const Eigen::Quaterniond& body_to_world,
                                              double gravity);
} // namespace uadl
#endif
