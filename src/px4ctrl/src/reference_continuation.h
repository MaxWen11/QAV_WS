#ifndef UADL_REFERENCE_CONTINUATION_H
#define UADL_REFERENCE_CONTINUATION_H

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace uadl {

struct ReferenceSample {
    bool valid = false;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d a = Eigen::Vector3d::Zero();
    double yaw = 0.0;
};

struct ReferenceBounds {
    bool valid = false;
    // [px, py, pz, vx, vy, vz], over all times from the requested instant.
    Eigen::Matrix<double, 6, 1> lower = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> upper = Eigen::Matrix<double, 6, 1>::Zero();
};

// Compatible reference continuation for the maintained backup of Remark 8.
// Each axis brakes smoothly to a fixed position, then remains there forever.
// The same initial velocity/acceleration and acceleration limits produce the
// same displacement, preserving a constant formation offset between vehicles.
struct ReferenceContinuation {
    bool valid = false;
    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d a0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d durations = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration_limits = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double start_stamp = 0.0;

    static ReferenceContinuation create(const Eigen::Vector3d& p,
            const Eigen::Vector3d& v, const Eigen::Vector3d& a,
            double yaw_value, double stamp, const Eigen::Vector3d& limits) {
        ReferenceContinuation result;
        if (!p.allFinite() || !v.allFinite() || !a.allFinite() ||
            !limits.allFinite() || (limits.array() < 0.0).any() ||
            !std::isfinite(yaw_value) || !std::isfinite(stamp) || stamp < 0.0 ||
            (a.cwiseAbs().array() > limits.array()).any()) return result;
        result.p0 = p;
        result.v0 = v;
        result.a0 = a;
        result.yaw = yaw_value;
        result.start_stamp = stamp;
        result.acceleration_limits = limits;
        for (int axis = 0; axis < 3; ++axis) {
            if (v(axis) == 0.0 && a(axis) == 0.0) continue;
            if (limits(axis) <= 0.0) return result;
            double duration = std::max(1.0, 2.0 * std::abs(v(axis)) / limits(axis));
            bool admitted = false;
            for (int iteration = 0; iteration < 64; ++iteration) {
                if (!std::isfinite(duration)) return result;
                // a(s) = a0 + c1*s + c2*s^2, s in [0,1]. The endpoints
                // and its single interior stationary point give exact extrema.
                const double c1 = -4.0 * a(axis) - 6.0 * v(axis) / duration;
                const double c2 = 3.0 * a(axis) + 6.0 * v(axis) / duration;
                if (!std::isfinite(c1) || !std::isfinite(c2)) return result;
                double maximum = std::abs(a(axis));
                if (c2 != 0.0) {
                    const double stationary = -c1 / (2.0 * c2);
                    if (stationary > 0.0 && stationary < 1.0)
                        maximum = std::max(maximum,
                            std::abs(a(axis) + stationary * (c1 + stationary * c2)));
                }
                if (std::isfinite(maximum) && maximum <= limits(axis)) {
                    admitted = true;
                    break;
                }
                duration *= 2.0;
            }
            if (!admitted) return result;
            result.durations(axis) = duration;
            const double stop = p(axis) + 0.5 * v(axis) * duration +
                                a(axis) * duration * duration / 12.0;
            if (!std::isfinite(stop)) return result;
        }
        result.valid = true;
        return result;
    }

    ReferenceSample sample(double stamp) const {
        ReferenceSample result;
        if (!valid || !std::isfinite(stamp) || stamp < start_stamp) return result;
        result.yaw = yaw;
        const double elapsed = stamp - start_stamp;
        for (int axis = 0; axis < 3; ++axis) {
            const double duration = durations(axis);
            if (duration == 0.0 || elapsed >= duration) {
                result.p(axis) = p0(axis) + 0.5 * v0(axis) * duration +
                                 a0(axis) * duration * duration / 12.0;
                continue;
            }
            const double s = elapsed / duration;
            const double s2 = s * s;
            const double s3 = s2 * s;
            const double s4 = s3 * s;
            result.p(axis) = p0(axis) + v0(axis) * duration * (s - s3 + 0.5 * s4) +
                a0(axis) * duration * duration * (0.5 * s2 - (2.0 / 3.0) * s3 + 0.25 * s4);
            result.v(axis) = v0(axis) * (1.0 - 3.0 * s2 + 2.0 * s3) +
                a0(axis) * duration * (s - 2.0 * s2 + s3);
            result.a(axis) = a0(axis) * (1.0 - 4.0 * s + 3.0 * s2) +
                (v0(axis) / duration) * (-6.0 * s + 6.0 * s2);
        }
        result.valid = result.p.allFinite() && result.v.allFinite() && result.a.allFinite();
        return result;
    }

    ReferenceBounds bounds(double from_stamp) const {
        ReferenceBounds result;
        if (!valid || !std::isfinite(from_stamp) || from_stamp < start_stamp) return result;
        const double elapsed = from_stamp - start_stamp;
        for (int axis = 0; axis < 3; ++axis) {
            const double duration = durations(axis);
            const double stop = p0(axis) + 0.5 * v0(axis) * duration +
                                a0(axis) * duration * duration / 12.0;
            if (duration == 0.0 || elapsed >= duration) {
                result.lower(axis) = result.upper(axis) = stop;
                result.lower(axis + 3) = result.upper(axis + 3) = 0.0;
                continue;
            }
            const double s = elapsed / duration;
            // Position and velocity Bezier control points. Convex hulls bound
            // the entire remaining curves; no time-grid extrema are used.
            const std::array<double, 5> positions{{p0(axis),
                p0(axis) + v0(axis) * duration / 4.0, stop, stop, stop}};
            const std::array<double, 4> velocities{{v0(axis),
                v0(axis) + a0(axis) * duration / 3.0, 0.0, 0.0}};
            const auto p_bounds = suffixHull(positions, s);
            const auto v_bounds = suffixHull(velocities, s);
            result.lower(axis) = p_bounds[0];
            result.upper(axis) = p_bounds[1];
            result.lower(axis + 3) = v_bounds[0];
            result.upper(axis + 3) = v_bounds[1];
        }
        result.valid = result.lower.allFinite() && result.upper.allFinite();
        return result;
    }

    Eigen::Vector3d accelerationChangeBound(double dt) const {
        if (!valid || !std::isfinite(dt) || dt < 0.0)
            return Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
        Eigen::Vector3d result = Eigen::Vector3d::Zero();
        for (int axis = 0; axis < 3; ++axis) {
            const double duration = durations(axis);
            if (duration == 0.0) continue;
            const double c1 = -4.0 * a0(axis) - 6.0 * v0(axis) / duration;
            const double c2 = 3.0 * a0(axis) + 6.0 * v0(axis) / duration;
            // Jerk is affine in s during braking and zero in the static tail.
            const double max_jerk = std::max(std::abs(c1), std::abs(c1 + 2.0 * c2)) / duration;
            result(axis) = std::min(2.0 * acceleration_limits(axis), max_jerk * dt);
        }
        return result;
    }

private:
    template <std::size_t N>
    static std::array<double, 2> suffixHull(std::array<double, N> points, double s) {
        std::array<double, N> suffix;
        suffix[N - 1] = points[N - 1];
        // Right branch of de Casteljau's split at s describes the curve from
        // s to 1. Its last point also bounds the infinite stationary tail.
        for (std::size_t level = 1; level < N; ++level) {
            for (std::size_t j = 0; j < N - level; ++j)
                points[j] = (1.0 - s) * points[j] + s * points[j + 1];
            suffix[N - level - 1] = points[N - level - 1];
        }
        const auto limits = std::minmax_element(suffix.begin(), suffix.end());
        return {{std::nextafter(*limits.first, -std::numeric_limits<double>::infinity()),
                 std::nextafter(*limits.second, std::numeric_limits<double>::infinity())}};
    }
};

}  // namespace uadl
#endif
