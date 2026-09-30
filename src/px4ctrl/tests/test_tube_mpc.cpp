#include "tube_mpc.h"

#include <cmath>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

using uadl::MPCConfig;
using uadl::TubeMPC;
using Eigen::Vector2d;

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
MPCConfig horizontal() {
    MPCConfig config;
    config.max_envelope = 0.1;
    return config;
}
void stable_and_terminal() {
    for (bool vertical : {false, true}) {
        MPCConfig config = horizontal();
        config.max_envelope = vertical ? 1.2 : 1.5;
        if (vertical) {
            config.Q_diag = Vector2d(15.0, 2.0);
            config.R = 0.5;
            config.Q_anc_diag = Vector2d(30.0, 5.0);
            config.state_limits = Vector2d(1.5, 1.0);
            config.correction_domain = Vector2d(-2.0, 5.0);
        }
        TubeMPC mpc(config);
        require(mpc.configured(), mpc.configuration_status());
        const auto solve_started = std::chrono::steady_clock::now();
        const auto fixed_envelope = mpc.solve(Vector2d(0.06, 0.0), config.max_envelope,
                                               config.correction_domain, 1.0);
        const double solve_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - solve_started).count();
        require(fixed_envelope.valid, "paper fixed-envelope baseline infeasible: " + fixed_envelope.status);
        Eigen::EigenSolver<Eigen::Matrix2d> eig(mpc.A() + mpc.B() * mpc.ancillary_gain());
        require(eig.eigenvalues().cwiseAbs().maxCoeff() < 1.0, "ancillary sign / stability");
        const Eigen::Matrix2d Af = mpc.A() + mpc.B() * mpc.terminal_gain();
        const Eigen::Matrix2d residual = Af.transpose() * mpc.terminal_weight() * Af -
            mpc.terminal_weight() + config.Q_diag.asDiagonal().toDenseMatrix() +
            config.R * mpc.terminal_gain().transpose() * mpc.terminal_gain();
        require(residual.norm() < 1e-7, "DARE terminal decrease");
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
        std::cout << (vertical ? "z" : "xy") << " tube radii/unit "
                  << mpc.tube_radii(1.0).transpose() << ", input reserve/unit "
                  << mpc.input_reserve(1.0) << ", faces " << mpc.tube_face_count()
                  << ", fixed-envelope solve_ms " << solve_ms << '\n';
    }
}

void rpi_continuous_residual() {
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
                require(mpc.tube_contains(Ak * boundary + w, envelope),
                        "RPI fails at rectangular disturbance corner");
            }
        }
    }
    require((mpc.tube_radii(0.1) - 2.0 * mpc.tube_radii(0.05)).norm() < 1e-12,
            "homothetic scaling");
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
        // Independent box corners include all bounded continuous-acceleration
        // sample integrals and deliberately exceed the held-disturbance line.
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
    // A state or reference jump outside the retained tube re-solves the OCP.
    TubeMPC jumped = original;
    auto state_jump = jumped.solve(state + Vector2d(0.2, 0.0), 0.05,
                                   config.correction_domain, 1.0);
    require(state_jump.valid && !state_jump.used_backup,
            "reference/state jump was not re-optimized: " + state_jump.status);
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
    // Tracking constraints are soft (Remark 9): a large error yields a valid
    // plan with positive slack while the applied correction stays hard-bounded.
    auto far = mpc.solve(Vector2d(3.0, 0.0), 0.1, config.correction_domain, 1.0);
    require(far.valid && far.slack(0) > 0.0, "soft tracking constraint: " + far.status);
    require(far.correction >= config.correction_domain(0) - 1e-7 &&
            far.correction <= config.correction_domain(1) + 1e-7, "hard correction bound");
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

void exact_model_and_solver_failure() {
    MPCConfig config;
    TubeMPC exact(config);
    Vector2d state(0.025, 0.0);
    auto command = exact.solve(state, 0.0, config.correction_domain, 1.0);
    require(command.valid, "zero-envelope QP: " + command.status);
    require((command.nominal_state - state).norm() < 1e-7,
            "zero disturbance must force nominal initial equality");
    TubeMPC broad(config);
    const auto broad_command = broad.solve(state, 0.0, Vector2d(-30.0, 30.0), 1.0);
    require(broad_command.valid && std::abs(broad_command.correction - command.correction) < 1e-8,
            "candidate bounds expanded the fixed common compact input domain");
    for (int k = 0; k < 30; ++k) {
        state = exact.A() * state + exact.B() * command.correction;
        command = exact.solve(state, 0.0, config.correction_domain, 0.0);
        require(command.valid && command.used_backup, "exact-model backup: " + command.status);
    }
    config.max_working_set_recalculations = 1;
    TubeMPC limited(config);
    auto failed = limited.solve(Vector2d(0.15, 0.0), 0.0, config.correction_domain, 1.0);
    require(!failed.valid && !limited.has_backup(),
            "unfinished QP was accepted without a feasible backup");
}

void estimator_updates_preserve_backup() {
    MPCConfig config = horizontal();
    config.min_envelope = 0.05;
    config.state_estimation_bound = 1e-5;
    TubeMPC mpc(config);
    require(mpc.configured(), "estimator configuration: " + mpc.configuration_status());
    Vector2d truth(0.01, 0.0);
    Vector2d estimate = truth + Vector2d(config.state_estimation_bound, 0.0);
    auto command = mpc.solve(estimate, 0.05, config.correction_domain, 1.0);
    require(command.valid, "estimated initial QP: " + command.status);
    for (int k = 0; k < 200; ++k) {
        truth = mpc.A() * truth + mpc.B() * command.correction +
                mpc.B() * (k % 2 ? 0.05 : -0.05);
        const double angle = k * 1.71;
        estimate = truth + config.state_estimation_bound * Vector2d(std::cos(angle), std::sin(angle));
        command = mpc.solve(estimate, 0.05, config.correction_domain, 0.0);
        require(command.valid && command.used_backup, "estimator update backup: " + command.status);
        require(mpc.tube_contains(truth - command.nominal_state, 0.05),
                "consistent true state outside tube");
    }
    config.min_envelope = 0.0;
    require(!TubeMPC(config).configured(), "nonzero estimation with zero envelope floor accepted");
}
} // namespace

int main() {
    try {
        stable_and_terminal();
        rpi_continuous_residual();
        initial_and_deadline_backup();
        candidate_update_and_copy_transaction();
        soft_tracking_constraints_and_nans();
        ancillary_is_actually_applied();
        exact_model_and_solver_failure();
        estimator_updates_preserve_backup();
        std::cout << "Tube MPC tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Tube MPC test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
