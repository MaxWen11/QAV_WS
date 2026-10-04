#include "reference_continuation.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {
void require(bool condition, const std::string& description) {
    if (!condition) {
        std::cerr << "FAIL: " << description << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void near(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected,
          const std::string& description, double tolerance = 1e-9) {
    require(actual.allFinite() && (actual - expected).norm() <= tolerance * (1.0 + expected.norm()),
            description);
}

void testSmoothStopAndBounds() {
    const Eigen::Vector3d p(0.3, -0.4, 1.0), v(0.8, -0.6, 0.2), a(0.4, 0.3, -0.2);
    const Eigen::Vector3d limits(0.7, 0.5, 0.4);
    const double epoch = 12.0;
    const auto reference = uadl::ReferenceContinuation::create(p, v, a, 0.3, epoch, limits);
    require(reference.valid, "finite bounded braking continuation admitted");
    const auto initial = reference.sample(epoch);
    require(initial.valid, "initial reference sample valid");
    near(initial.p, p, "continuation starts at supplied position");
    near(initial.v, v, "continuation starts at supplied velocity");
    near(initial.a, a, "continuation starts at supplied acceleration");

    const double final_stamp = epoch + reference.durations.maxCoeff();
    const auto stopped = reference.sample(final_stamp);
    const auto tail = reference.sample(final_stamp + 1e6);
    require(stopped.valid && tail.valid, "infinite stationary tail remains defined");
    near(stopped.v, Eigen::Vector3d::Zero(), "velocity is zero at completion");
    near(stopped.a, Eigen::Vector3d::Zero(), "acceleration is zero at completion");
    near(tail.p, stopped.p, "stationary tail retains its endpoint");
    const auto tail_bounds = reference.bounds(final_stamp);
    require(tail_bounds.valid, "stationary tail enclosure valid");
    near(tail_bounds.lower.head<3>(), stopped.p, "tail position lower bound is endpoint");
    near(tail_bounds.upper.head<3>(), stopped.p, "tail position upper bound is endpoint");

    const double dt = 0.01;
    const auto change = reference.accelerationChangeBound(dt);
    for (int origin = 0; origin <= 5; ++origin) {
        const double from_stamp = epoch + reference.durations.maxCoeff() * origin / 5.0;
        const auto bounds = reference.bounds(from_stamp);
        require(bounds.valid, "future reference enclosure valid");
        for (int k = 0; k <= 100; ++k) {
            const double stamp = from_stamp + 2.0 * reference.durations.maxCoeff() * k / 100.0;
            const auto sample = reference.sample(stamp);
            Eigen::Matrix<double, 6, 1> state;
            state << sample.p, sample.v;
            require(sample.valid && (state.array() >= bounds.lower.array() - 1e-10).all() &&
                    (state.array() <= bounds.upper.array() + 1e-10).all(),
                    "Bezier suffix hull encloses all future states and static tail");
            require((sample.a.cwiseAbs().array() <= limits.array() + 1e-10).all(),
                    "analytic quadratic-extrema check enforces acceleration limits");
            const auto next = reference.sample(stamp + dt);
            require(((next.a - sample.a).cwiseAbs().array() <= change.array() + 1e-10).all(),
                    "jerk envelope bounds one-period reference acceleration changes");
        }
    }
    // An independent derivative check ensures that the commanded acceleration,
    // velocity and position belong to one continuous reference trajectory.
    const double stamp = epoch + 0.37 * reference.durations.minCoeff();
    const double step = 1e-5;
    const auto before = reference.sample(stamp - step);
    const auto center = reference.sample(stamp);
    const auto after = reference.sample(stamp + step);
    near((after.p - before.p) / (2.0 * step), center.v, "position derivative equals velocity", 1e-7);
    near((after.v - before.v) / (2.0 * step), center.a, "velocity derivative equals acceleration", 1e-7);
}

void testFormationAndAdmission() {
    const Eigen::Vector3d offset(0.0, 1.5, 0.0);
    const Eigen::Vector3d v(0.5, -0.2, 0.0), a(0.2, 0.1, 0.0), limits(0.4, 0.4, 0.0);
    const auto first = uadl::ReferenceContinuation::create(Eigen::Vector3d::Zero(), v, a, 0.0, 0.0, limits);
    const auto second = uadl::ReferenceContinuation::create(offset, v, a, 0.0, 0.0, limits);
    require(first.valid && second.valid, "synchronized formation references admitted");
    near(first.durations, second.durations, "formation offset does not change stop durations");
    for (double stamp : {0.0, 0.25, 1.0, 3.0, 1000.0})
        near(second.sample(stamp).p - first.sample(stamp).p, offset,
             "compatible continuations preserve formation displacement");
    require(!first.sample(-0.1).valid && !first.bounds(-0.1).valid,
            "a timestamp before the reference origin is rejected");
    require(!uadl::ReferenceContinuation::create(Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero(), 0.0, 0.0,
        Eigen::Vector3d::Zero()).valid, "nonzero velocity requires positive braking authority");
    require(!uadl::ReferenceContinuation::create(Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(), 0.0, 0.0,
        Eigen::Vector3d::Constant(0.5)).valid, "initial acceleration outside the domain is rejected");
}
}  // namespace

int main() {
    testSmoothStopAndBounds();
    testFormationAndAdmission();
    std::cout << "All reference continuation tests passed.\n";
    return EXIT_SUCCESS;
}
