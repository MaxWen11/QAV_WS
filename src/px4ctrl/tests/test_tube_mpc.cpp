#include "tube_mpc.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

using uadl::MPCConfig;
using uadl::SafetyLimits;
using uadl::TubeMPC;
using Eigen::Vector2d;

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, const std::string& message, double tolerance) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
            message + ": got " + std::to_string(actual) + ", expected " + std::to_string(expected));
}
MPCConfig horizontal(double max_envelope = 0.1) {
    MPCConfig config;
    config.max_envelope = max_envelope;
    return config;
}
MPCConfig vertical(double max_envelope) {
    MPCConfig config;
    config.max_envelope = max_envelope;
    config.Q_diag = Vector2d(15.0, 2.0);
    config.R = 0.5;
    config.Q_anc_diag = Vector2d(30.0, 5.0);
    config.state_limits = Vector2d(1.5, 1.0);
    config.correction_domain = Vector2d(-2.0, 5.0);
    return config;
}
SafetyLimits along(const TubeMPC& mpc, int horizon, const Vector2d& start,
                   const Vector2d& lower, const Vector2d& upper) {
    SafetyLimits safety;
    for (int i = 0; i <= horizon + 1; ++i)
        safety.reference.push_back(Vector2d(start(0) + i * mpc.A()(0, 1) * start(1), start(1)));
    safety.lower = lower;
    safety.upper = upper;
    return safety;
}

// Section VI-C weights give the ancillary LQR gains, and the base set of the
// homothetic tube is the finite-sum set scaled to be lambda-contractive.
void ancillary_gains_and_base_tube() {
    for (bool is_vertical : {false, true}) {
        const MPCConfig config = is_vertical ? vertical(0.95) : horizontal(1.2);
        const TubeMPC mpc(config);
        require(mpc.configured(), mpc.configuration_status());
        const Eigen::RowVector2d gain = mpc.ancillary_gain();
        near(gain(0), is_vertical ? -48.422970 : -41.077002, "K_anc position", 1e-6);
        near(gain(1), is_vertical ? -22.082647 : -15.839382, "K_anc velocity", 1e-6);
        require(mpc.tube_alpha_required() <= config.tube_alpha, "A_K^s W0 within alpha W0");
        require(mpc.tube_scale() >= 1.0 / (1.0 - config.tube_alpha), "base set contains the RPI set");
    }
}

// Homothetic scaling applies to state and input support functions alike.
void homothetic_geometry() {
    const TubeMPC mpc(horizontal(1.2));
    require(mpc.configured(), "ratio fixture");
    require((mpc.tube_radii(0.1) - 2.0 * mpc.tube_radii(0.05)).norm() < 1e-12, "homothetic scaling");
    near(mpc.input_reserve(0.1), 2.0 * mpc.input_reserve(0.05),
         "homothetic ancillary-input reserve", 1e-12);
}

void stable_and_terminal() {
    for (bool is_vertical : {false, true}) {
        MPCConfig config = is_vertical ? vertical(0.95) : horizontal(1.2);
        TubeMPC mpc(config);
        require(mpc.configured(), mpc.configuration_status());
        const auto result = mpc.solve(Vector2d(0.06, 0.0), config.max_envelope,
                                      config.correction_domain, 1.0);
        require(result.valid, "max-envelope adaptive tube infeasible: " + result.status);
        Eigen::EigenSolver<Eigen::Matrix2d> eig(mpc.A() + mpc.B() * mpc.ancillary_gain());
        require(eig.eigenvalues().cwiseAbs().maxCoeff() < 1.0, "ancillary sign / stability");
        const Eigen::Matrix2d Af = mpc.A() + mpc.B() * mpc.terminal_gain();
        const Eigen::Matrix2d residual = Af.transpose() * mpc.terminal_weight() * Af -
            mpc.terminal_weight() + config.Q_diag.asDiagonal().toDenseMatrix() +
            config.R * mpc.terminal_gain().transpose() * mpc.terminal_gain();
        require(residual.norm() < 1e-7, "DARE terminal decrease, Eq. (50)");
        for (int angle = 0; angle < 360; ++angle) {
            const Vector2d direction(std::cos(angle * 0.0174532925199433),
                                     std::sin(angle * 0.0174532925199433));
            double lo = 0.0, hi = 10.0;
            for (int i = 0; i < 60; ++i) {
                const double middle = (lo + hi) / 2.0;
                if (mpc.terminal_contains(middle * direction)) lo = middle;
                else hi = middle;
            }
            const Vector2d vertex = lo * 0.999999 * direction;
            require(mpc.terminal_contains(Af * vertex), "terminal positive invariance");
        }
    }
}

void rpi_at_disturbance_corners() {
    TubeMPC mpc(horizontal());
    require(mpc.configured(), mpc.configuration_status());
    const Eigen::Matrix2d Ak = mpc.A() + mpc.B() * mpc.ancillary_gain();
    const double envelope = 0.1;
    for (int angle = 0; angle < 360; ++angle) {
        Vector2d direction(std::cos(angle * 0.0174532925199433),
                           std::sin(angle * 0.0174532925199433));
        double lo = 0.0, hi = 10.0;
        for (int i = 0; i < 60; ++i) {
            const double middle = (lo + hi) / 2.0;
            if (mpc.tube_contains(middle * direction, envelope)) lo = middle;
            else hi = middle;
        }
        const Vector2d boundary = 0.99999 * lo * direction;
        for (double sign_p : {-1.0, 1.0}) {
            for (double sign_v : {-1.0, 1.0}) {
                const Vector2d w(sign_p * mpc.B()(0) * envelope,
                                 sign_v * mpc.B()(1) * envelope);
                require(mpc.tube_contains(Ak * boundary + w, 0.999 * envelope),
                        "lambda-contraction fails at a disturbance-box corner");
            }
        }
    }
}

void initial_and_deadline_backup() {
    MPCConfig config = horizontal();
    TubeMPC mpc(config);
    Vector2d state(0.025, 0.0);
    auto no_plan = mpc.solve(state, 0.1, config.correction_domain, 0.0);
    require(!no_plan.valid && !mpc.has_backup(), "cutoff without backup must fail");
    auto command = mpc.solve(state, 0.1, config.correction_domain, 1.0);
    require(command.valid && !command.used_backup, "initial QP: " + command.status);
    for (int k = 0; k < 200; ++k) {
        // Box corners include all bounded continuous-acceleration sample
        // integrals of a residual envelope of 0.1.
        Vector2d w = mpc.B().cwiseProduct(Vector2d(k % 2 ? 0.1 : -0.1,
                                                  k % 3 ? 0.1 : -0.1));
        state = mpc.A() * state + mpc.B() * command.correction + w;
        command = mpc.solve(state, 0.1, config.correction_domain, 0.0);
        require(command.valid && command.used_backup, "shift/terminal backup: " + command.status);
        require(command.correction >= -3.0 - 1e-7 && command.correction <= 3.0 + 1e-7,
                "backup actuator bound");
        require((state.cwiseAbs().array() <= config.state_limits.array() + 1e-7).all(),
                "backup true-state bounds");
    }
}

void candidate_update_and_copy_transaction() {
    MPCConfig config = horizontal();
    TubeMPC original(config);
    Vector2d state(0.01, 0.0);
    auto first = original.solve(state, 0.05, config.correction_domain, 1.0);
    require(first.valid, "seed transaction plan: " + first.status);
    state = original.A() * state + original.B() * first.correction;
    TubeMPC candidate = original;
    auto expanded = candidate.solve(state, 0.1, config.correction_domain, 0.0);
    require(expanded.valid && expanded.used_backup, "valid expansion rejected: " + expanded.status);
    // The lambda-contractive base tube admits a decrease down to lambda*old.
    const double admissible = original.min_next_envelope();
    require(std::abs(admissible - config.tube_contraction * 0.05) < 1e-12, "contraction limit");
    TubeMPC contracted = original;
    auto shrunk = contracted.solve(state, admissible, config.correction_domain, 0.0);
    require(shrunk.valid && shrunk.used_backup, "admissible contraction rejected: " + shrunk.status);
    TubeMPC too_fast = original;
    auto rejected = too_fast.solve(state, 0.0, config.correction_domain, 0.0);
    require(!rejected.valid, "shrink faster than the base-tube contraction kept its backup");
    auto narrow = original.solve(state, 0.05, Vector2d(-0.001, 0.001), 1.0);
    require(!narrow.valid, "empty tightening was silently relaxed");
    // A state/reference jump that invalidates the retained sequence cannot
    // bypass the update check by solving a different OCP.
    TubeMPC jumped = original;
    auto state_jump = jumped.solve(state + Vector2d(0.2, 0.0), 0.05,
                                   config.correction_domain, 1.0);
    require(!state_jump.valid,
            "reference/state jump bypassed shifted-plan feasibility");
    auto old_backup = original.solve(state, 0.05, config.correction_domain, 0.0);
    require(old_backup.valid && old_backup.used_backup,
            "candidate evaluation corrupted the original backup: " + old_backup.status);
    original = candidate;
    state = candidate.A() * state + candidate.B() * expanded.correction;
    auto committed = original.solve(state, 0.1, config.correction_domain, 0.0);
    require(committed.valid, "copied accepted plan failed");
    original.reset();
    require(!original.has_backup(), "reset retained old trajectory");
}

void soft_tracking_constraints_and_nans() {
    MPCConfig config = horizontal();
    TubeMPC mpc(config);
    // A performance violation is allowed only when the hard terminal set is
    // reachable with hard-bounded inputs. Enlarge the horizon for this case.
    MPCConfig soft_config = horizontal(0.01);
    soft_config.horizon = 100;
    soft_config.state_limits = Vector2d(0.04, 0.5);
    TubeMPC soft(soft_config);
    auto relaxed = soft.solve(Vector2d(0.05, 0.0), 0.01, soft_config.correction_domain, 1.0);
    require(relaxed.valid && relaxed.slack(0) > 0.0,
            "soft tracking constraint: " + relaxed.status);
    require(relaxed.correction >= config.correction_domain(0) - 1e-7 &&
            relaxed.correction <= config.correction_domain(1) + 1e-7, "hard correction bound");
    // A distant state cannot reach X_f in 20 steps; terminal slack is forbidden.
    auto far = mpc.solve(Vector2d(3.0, 0.0), 0.1, config.correction_domain, 1.0);
    require(!far.valid, "unreachable hard terminal set was relaxed");
    auto nonfinite = mpc.solve(Vector2d(std::numeric_limits<double>::quiet_NaN(), 0.0),
                              0.1, config.correction_domain, 1.0);
    require(!nonfinite.valid, "NaN input accepted");
    auto above_max = mpc.solve(Vector2d::Zero(), 0.10001, config.correction_domain, 1.0);
    require(!above_max.valid, "envelope above max_envelope accepted");
    MPCConfig invalid = config;
    invalid.max_envelope = 100.0;
    require(!TubeMPC(invalid).configured(), "empty fixed terminal domain accepted");
    invalid = config;
    invalid.Q_diag(0) = -1.0;
    require(!TubeMPC(invalid).configured(), "negative quadratic weight accepted");
    invalid = config;
    invalid.tube_contraction = 1.5;
    require(!TubeMPC(invalid).configured(), "expanding base tube accepted");
    invalid = config;
    invalid.tube_terms = 20;
    require(!TubeMPC(invalid).configured(), "finite sum with A_K^s W0 outside alpha W0 accepted");
    invalid = config;
    invalid.tube_contraction = 0.99;
    require(!TubeMPC(invalid).configured(), "contraction below the attainable rate accepted");
}

void ancillary_is_actually_applied() {
    MPCConfig config = horizontal();
    TubeMPC mpc(config);
    const Vector2d error(0.0001, 0.0001);
    auto result = mpc.solve(error, 0.1, config.correction_domain, 1.0);
    require(result.valid, "small tube-only QP: " + result.status);
    require(result.nominal_state.norm() < 1e-6, "free z0 was fixed to measured error");
    require(std::abs(result.correction - mpc.ancillary_gain().dot(error)) < 1e-5,
            "ancillary feedback missing from applied correction");
}

void exact_model_backup_and_solver_failure() {
    MPCConfig config = horizontal();
    TubeMPC exact(config);
    Vector2d state(0.025, 0.0);
    auto command = exact.solve(state, 0.05, config.correction_domain, 1.0);
    require(command.valid, "exact-model QP: " + command.status);
    TubeMPC broad(config);
    const auto broad_command = broad.solve(state, 0.05, Vector2d(-30.0, 30.0), 1.0);
    require(broad_command.valid && std::abs(broad_command.correction - command.correction) < 1e-8,
            "candidate bounds expanded the fixed common compact input domain");
    for (int k = 0; k < 30; ++k) {
        state = exact.A() * state + exact.B() * command.correction;
        command = exact.solve(state, 0.05, config.correction_domain, 0.0);
        require(command.valid && command.used_backup, "exact-model backup: " + command.status);
    }
    config.max_working_set_recalculations = 1;
    TubeMPC limited(config);
    auto failed = limited.solve(Vector2d(0.15, 0.0), 0.05, config.correction_domain, 1.0);
    require(!failed.valid && !limited.has_backup(),
            "unfinished QP was accepted without a feasible backup");
}

void hard_safety_limits() {
    MPCConfig config = horizontal();
    TubeMPC mpc(config);
    const int H = config.horizon;
    // Reference 4.9 m inside an upper position limit of 5 m, measured error
    // +0.05 m: the plan must keep reference + nominal state + tube inside.
    const Vector2d lower(-5.0, -3.0), upper(5.0, 3.0);
    const SafetyLimits safety = along(mpc, H, Vector2d(4.9, 0.0), lower, upper);
    const auto result = mpc.solve(Vector2d(0.05, 0.0), 0.1, config.correction_domain, safety, 1.0);
    require(result.valid && !result.used_backup, "safe re-optimization: " + result.status);
    const double radius = mpc.tube_radii(0.1)(0);
    require(4.9 + result.nominal_state(0) <= 5.0 - radius + 1e-7, "hard safety limit on z_0");
    // A reference whose tube cannot fit inside the limits is infeasible.
    TubeMPC blocked(config);
    const SafetyLimits violated = along(blocked, H, Vector2d(4.999, 0.0), lower, upper);
    const auto infeasible = blocked.solve(Vector2d(0.01, 0.0), 0.1, config.correction_domain, violated, 1.0);
    require(!infeasible.valid, "violated safety limit accepted: " + infeasible.status);
    SafetyLimits malformed = safety;
    malformed.reference.pop_back();
    require(!mpc.solve(Vector2d::Zero(), 0.1, config.correction_domain, malformed, 1.0).valid,
            "safety reference without the continuation instant accepted");

    TubeMPC continuation(config);
    SafetyLimits tail = along(continuation, H, Vector2d::Zero(), lower, upper);
    tail.have_tail_bounds = true;
    tail.tail_reference_lower = Vector2d(-0.5, -0.2);
    tail.tail_reference_upper = Vector2d(0.5, 0.2);
    require(continuation.solve(Vector2d::Zero(), 0.1, config.correction_domain, tail, 1.0).valid,
            "safe terminal reference envelope rejected");
    TubeMPC invalid_tail(config);
    tail.tail_reference_upper(0) = upper(0);
    require(!invalid_tail.solve(Vector2d::Zero(), 0.1, config.correction_domain, tail, 1.0).valid,
            "terminal continuation could leave physical safety limits");
}

void state_information_box() {
    MPCConfig config = horizontal();
    config.max_estimation_error = Vector2d::Constant(1.0e-9);
    config.feasibility_tolerance = 1.0e-12;
    TubeMPC mpc(config);
    require(mpc.configured(), mpc.configuration_status());
    SafetyLimits information;
    information.estimation_error = 0.5 * config.max_estimation_error;
    auto feasible = mpc.solve(Vector2d::Zero(), 0.1, config.correction_domain, information, 1.0);
    require(feasible.valid, "bounded state information rejected: " + feasible.status);
    const double additional_reserve = mpc.ancillary_gain().cwiseAbs().dot(config.max_estimation_error);
    near(feasible.input_reserve, mpc.input_reserve(0.1) + additional_reserve,
         "estimator contribution to ancillary-input tightening", 1e-12);

    TubeMPC too_small(config);
    information.estimation_error = config.max_estimation_error;
    const double minimum = too_small.min_next_envelope();
    require(minimum > 0.0, "future estimator-information boxes did not set an envelope floor");
    auto infeasible = too_small.solve(Vector2d::Zero(), 0.5 * minimum,
                                      config.correction_domain, information, 1.0);
    require(!infeasible.valid, "common tube could not contain future estimator-information boxes");

    MPCConfig inadequate = config;
    inadequate.max_estimation_error = Vector2d::Constant(0.001);
    TubeMPC no_invariant_information_tube(inadequate);
    require(!no_invariant_information_tube.configured(),
            "maximum envelope without sufficient information-set propagation margin was accepted");

    TubeMPC physical(config);
    SafetyLimits boundary = along(physical, config.horizon, Vector2d(5.0 - 0.5e-9, 0.0),
                                  Vector2d(-5.0, -3.0), Vector2d(5.0, 3.0));
    boundary.estimation_error = config.max_estimation_error;
    require(!physical.solve(Vector2d::Zero(), 0.1, config.correction_domain, boundary, 1.0).valid,
            "physical safety checked only the state-estimate center");
}
} // namespace

int main() {
    try {
        ancillary_gains_and_base_tube();
        homothetic_geometry();
        stable_and_terminal();
        rpi_at_disturbance_corners();
        initial_and_deadline_backup();
        candidate_update_and_copy_transaction();
        soft_tracking_constraints_and_nans();
        ancillary_is_actually_applied();
        exact_model_backup_and_solver_failure();
        hard_safety_limits();
        state_information_box();
        std::cout << "Tube MPC tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Tube MPC test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
