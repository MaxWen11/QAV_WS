#include "control_geometry.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace uadl {
Eigen::Vector3d specificForceToNetAcceleration(const Eigen::Vector3d& force,
                                              const Eigen::Quaterniond& q,
                                              double gravity) {
    if (!force.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-8 ||
        !std::isfinite(gravity) || gravity <= 0.0)
        return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    return q.normalized() * force - gravity * Eigen::Vector3d::UnitZ();
}

MappedCommand mapAcceleration(const Eigen::Vector3d& input, double yaw,
                              double voltage, const ThrustConfig& c,
                              const std::array<Eigen::Vector2d, 3>& limits) {
    MappedCommand out;
    if (!input.allFinite() || !std::isfinite(yaw) || !std::isfinite(c.gravity) ||
        c.gravity <= 0.0 || !std::isfinite(c.mass) || c.mass <= 0.0 ||
        !std::isfinite(c.hover_fraction) || c.hover_fraction <= 0.0 || c.hover_fraction > 1.0 ||
        !std::isfinite(c.max_tilt) || c.max_tilt < 0.0 || c.max_tilt >= 1.5707963267948966)
        return out;
    Eigen::Vector3d acc = input;
    for (int i = 0; i < 3; ++i) {
        if (!limits[i].allFinite() || limits[i](0) > limits[i](1)) return out;
        acc(i) = std::clamp(acc(i), limits[i](0), limits[i](1));
    }
    acc.z() += c.gravity;
    // A nonpositive vertical thrust request cannot be represented in the
    // upright flight domain used by this controller.
    if (acc.z() <= 1e-8) return out;
    const double horizontal = acc.head<2>().norm();
    const double max_horizontal = acc.z() * std::tan(c.max_tilt);
    if (horizontal > max_horizontal && horizontal > 0.0)
        acc.head<2>() *= max_horizontal / horizontal;
    const double requested_magnitude = acc.norm();
    const Eigen::Vector3d zb = acc / requested_magnitude;
    const Eigen::Vector3d xc(std::cos(yaw), std::sin(yaw), 0.0);
    Eigen::Vector3d yb = zb.cross(xc);
    if (yb.norm() < 1e-8) return out;
    yb.normalize();
    Eigen::Matrix3d rotation;
    rotation.col(0) = yb.cross(zb).normalized();
    rotation.col(1) = yb;
    rotation.col(2) = zb;
    out.attitude = Eigen::Quaterniond(rotation).normalized();
    double actual_magnitude;
    if (c.voltage_model) {
        if (!std::isfinite(voltage) || voltage <= 0.0 || !std::isfinite(c.k1) || c.k1 <= 0.0 ||
            !std::isfinite(c.k2) || !std::isfinite(c.k3) || c.k3 < 0.0 || c.k3 > 1.0)
            return MappedCommand();
        const double scale = c.k1 * std::pow(voltage, c.k2);
        if (!std::isfinite(scale) || scale <= 0.0) return MappedCommand();
        const double target = c.mass * requested_magnitude / scale;
        const double linear = 1.0 - c.k3;
        // Rationalized positive root avoids cancellation for small k3.
        const double command = c.k3 < 1e-10 ? target :
            2.0 * target / (linear + std::sqrt(linear * linear + 4.0 * c.k3 * target));
        out.thrust = std::clamp(command, 0.0, 1.0);
        actual_magnitude = scale * (c.k3 * out.thrust * out.thrust + linear * out.thrust) / c.mass;
    } else {
        const double acceleration_per_thrust = c.gravity / c.hover_fraction;
        out.thrust = std::clamp(requested_magnitude / acceleration_per_thrust, 0.0, 1.0);
        actual_magnitude = out.thrust * acceleration_per_thrust;
    }
    out.executed_input = actual_magnitude * zb - c.gravity * Eigen::Vector3d::UnitZ();
    if (!out.executed_input.allFinite() || !std::isfinite(out.thrust)) return MappedCommand();
    for (int i = 0; i < 3; ++i)
        if (out.executed_input(i) < limits[i](0) - 1e-8 || out.executed_input(i) > limits[i](1) + 1e-8)
            return MappedCommand();
    out.valid = true;
    return out;
}
} // namespace uadl
