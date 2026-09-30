#include "tube_mpc.h"

#include <qpOASES.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace uadl {
namespace {
constexpr double kPi = 3.14159265358979323846;
using Clock = std::chrono::steady_clock;
double elapsed(const Clock::time_point& start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
bool positive(const Eigen::Vector2d& value) {
    return value.allFinite() && (value.array() > 0.0).all();
}
} // namespace

double TubeMPC::Polytope::support(const Eigen::Vector2d& direction) const {
    double result = -std::numeric_limits<double>::infinity();
    for (const auto& vertex : vertices)
        result = std::max(result, direction.dot(vertex));
    return result;
}

bool TubeMPC::Polytope::contains(const Eigen::Vector2d& point, double scale,
                               double tolerance) const {
    return point.allFinite() && std::isfinite(scale) && scale >= 0.0 &&
           (normals * point).maxCoeff() <= scale * radius + tolerance;
}

bool TubeMPC::dare(const Eigen::Matrix2d& A, const Eigen::Vector2d& B,
                   const Eigen::Matrix2d& Q, double R,
                   Eigen::Matrix2d& P, Eigen::RowVector2d& K) {
    P = Q;
    bool converged = false;
    for (int iteration = 0; iteration < 100000; ++iteration) {
        const double denominator = R + B.dot(P * B);
        if (!std::isfinite(denominator) || denominator <= 0.0) return false;
        const Eigen::Vector2d coupling = A.transpose() * P * B;
        Eigen::Matrix2d next = A.transpose() * P * A -
                              coupling * coupling.transpose() / denominator + Q;
        next = (0.5 * (next + next.transpose())).eval();
        if (!next.allFinite()) return false;
        const double difference = (next - P).norm();
        P = next;
        if (difference <= 1e-12 * std::max(1.0, P.norm())) {
            converged = true;
            break;
        }
    }
    if (!converged) return false;
    K = -(B.transpose() * P * A) / (R + B.dot(P * B));
    const Eigen::Matrix2d closed = A + B * K;
    Eigen::EigenSolver<Eigen::Matrix2d> eigenvalues(closed, false);
    if (eigenvalues.info() != Eigen::Success ||
        eigenvalues.eigenvalues().cwiseAbs().maxCoeff() >= 1.0) return false;
    const Eigen::Matrix2d residual = closed.transpose() * P * closed - P +
                                     Q + R * K.transpose() * K;
    return residual.norm() <= 1e-8 * std::max(1.0, Q.norm());
}

bool TubeMPC::contracting_polytope(const Eigen::Matrix2d& closed_loop,
                                  const Eigen::Matrix2d& metric,
                                  Polytope& result) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(metric);
    if (eig.info() != Eigen::Success || eig.eigenvalues().minCoeff() <= 0.0)
        return false;
    // ||x||_metric = ||T*x||_2. The regular polygon is circumscribed
    // around the unit Euclidean ball in these transformed coordinates.
    const Eigen::Matrix2d T = eig.eigenvalues().cwiseSqrt().asDiagonal() *
                              eig.eigenvectors().transpose();
    const Eigen::Matrix2d Ti = T.inverse();
    const Eigen::Matrix2d transformed = T * closed_loop * Ti;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> singular(
        transformed.transpose() * transformed);
    if (singular.info() != Eigen::Success) return false;
    const double r = std::sqrt(std::max(0.0, singular.eigenvalues().maxCoeff()));
    if (!std::isfinite(r) || r >= 1.0) return false;
    int faces = 16;
    while (faces <= 2048 && r / std::cos(kPi / faces) > (1.0 + r) / 2.0)
        faces *= 2;
    if (faces > 2048) return false;
    result.contraction = r / std::cos(kPi / faces);
    result.radius = 1.0;
    result.normals.resize(faces, 2);
    result.vertices.clear();
    for (int i = 0; i < faces; ++i) {
        const double angle = 2.0 * kPi * i / faces;
        const Eigen::Vector2d normal(std::cos(angle), std::sin(angle));
        result.normals.row(i) = normal.transpose() * T;
        const double vertex_angle = angle + kPi / faces;
        const Eigen::Vector2d vertex(std::cos(vertex_angle), std::sin(vertex_angle));
        result.vertices.push_back(Ti * vertex / std::cos(kPi / faces));
    }
    return result.normals.allFinite();
}

bool TubeMPC::build_rpi_polytope(const Eigen::Matrix2d& closed_loop,
                               const Eigen::Matrix2d& metric) {
    // The base set is lambda-contractive, A_K Z (+) W <= lambda Z, i.e. RPI
    // for (A_K/lambda, W/lambda). A box outer-bounds the integrated residual
    // plus estimator terms. For M^s W <= alpha W, alpha<1, the finite
    // Minkowski sum Z=(W (+) ... (+) M^(s-1) W)/(1-alpha) is RPI for M.
    const double lambda = config_.tube_contraction;
    const Eigen::Matrix2d scaled = closed_loop / lambda;
    const auto scaled_disturbance = [&](const Eigen::Vector2d& direction) {
        return disturbance_support(direction) / lambda;
    };
    const Eigen::Vector2d box(scaled_disturbance(Eigen::Vector2d::UnitX()),
                              scaled_disturbance(Eigen::Vector2d::UnitY()));
    if (!positive(box)) return false;
    Eigen::Matrix2d power = Eigen::Matrix2d::Identity();
    std::vector<Eigen::Vector2d> generators;
    double alpha = 1.0;
    for (int i = 0; i < 20000; ++i) {
        generators.push_back(power.col(0) * box(0));
        generators.push_back(power.col(1) * box(1));
        power = (scaled * power).eval();
        alpha = (power.cwiseAbs() * box).cwiseQuotient(box).maxCoeff();
        if (alpha <= 0.001) break;
    }
    if (!std::isfinite(alpha) || alpha > 0.001) return false;
    for (auto& generator : generators) generator /= (1.0 - alpha);
    auto zonotope_support = [&](const Eigen::Vector2d& normal) {
        double support = 0.0;
        for (const auto& generator : generators) support += std::abs(normal.dot(generator));
        return support;
    };
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(metric);
    if (eig.info() != Eigen::Success || eig.eigenvalues().minCoeff() <= 0.0) return false;
    const Eigen::Matrix2d transform = eig.eigenvalues().cwiseSqrt().asDiagonal() *
                                       eig.eigenvectors().transpose();
    Polytope best;
    double best_reserve = std::numeric_limits<double>::infinity();
    for (int faces = 32; faces <= config_.max_tube_faces; faces *= 2) {
        Polytope candidate;
        candidate.radius = 1.0;
        candidate.normals.resize(faces, 2);
        for (int i = 0; i < faces; ++i) {
            const double angle = 2.0 * kPi * i / faces;
            const Eigen::Vector2d n = transform.transpose() *
                Eigen::Vector2d(std::cos(angle), std::sin(angle));
            const double support = zonotope_support(n);
            if (!std::isfinite(support) || support <= 0.0) return false;
            candidate.normals.row(i) = n.transpose() / support;
        }
        // Adjacent supporting lines give the vertices of the 2D outer
        // polygon. Normalization stores every face with right-hand side 1.
        for (int i = 0; i < faces; ++i) {
            Eigen::Matrix2d adjacent;
            adjacent.row(0) = candidate.normals.row(i);
            adjacent.row(1) = candidate.normals.row((i + 1) % faces);
            if (std::abs(adjacent.determinant()) < 1e-14) return false;
            candidate.vertices.push_back(adjacent.inverse() * Eigen::Vector2d::Ones());
        }
        bool valid = true;
        for (const auto& vertex : candidate.vertices) {
            if (!vertex.allFinite() || (candidate.normals * vertex).maxCoeff() > 1.0 + 1e-8) {
                valid = false;
                break;
            }
        }
        double inflation = 1.0;
        if (valid) {
            for (int i = 0; i < faces; ++i) {
                const Eigen::Vector2d n = candidate.normals.row(i).transpose();
                const double gap = 1.0 - candidate.support(scaled.transpose() * n);
                if (gap <= 1e-12) { valid = false; break; }
                inflation = std::max(inflation, scaled_disturbance(n) / gap);
            }
        }
        if (!valid || !std::isfinite(inflation)) continue;
        inflation *= (1.0 + 1e-9);
        candidate.radius = inflation;
        for (auto& vertex : candidate.vertices) vertex *= inflation;
        // A candidate is accepted only after the finite-face contraction
        // A_K Z (+) W <= lambda Z is verified.
        for (int i = 0; i < faces; ++i) {
            const Eigen::Vector2d n = candidate.normals.row(i).transpose();
            if (candidate.support(closed_loop.transpose() * n) + disturbance_support(n) >
                lambda * candidate.radius * (1.0 + 1e-10)) { valid = false; break; }
        }
        if (!valid) continue;
        const double reserve = candidate.support(K_.transpose());
        if (reserve < best_reserve) { best = candidate; best_reserve = reserve; }
        if (inflation <= 1.10) break;
    }
    if (!std::isfinite(best_reserve)) return false;
    tube_ = best;
    unit_tube_radii_ = Eigen::Vector2d(tube_.support(Eigen::Vector2d::UnitX()),
                                       tube_.support(Eigen::Vector2d::UnitY()));
    unit_input_reserve_ = tube_.support(K_.transpose());
    propagated_face_support_.resize(tube_.normals.rows());
    disturbance_face_support_.resize(tube_.normals.rows());
    for (int i = 0; i < tube_.normals.rows(); ++i) {
        const Eigen::Vector2d n = tube_.normals.row(i).transpose();
        propagated_face_support_(i) = tube_.support(closed_loop.transpose() * n);
        disturbance_face_support_(i) = disturbance_support(n);
    }
    return true;
}

TubeMPC::TubeMPC(const MPCConfig& config) : config_(config) {
    config_status_ = "invalid_configuration";
    if (!std::isfinite(config.dt) || config.dt <= 0.0 ||
        config.horizon < 1 || config.horizon > 200 ||
        !positive(config.Q_diag) || !positive(config.Q_anc_diag) ||
        !positive(config.state_limits) || !std::isfinite(config.R) || config.R <= 0.0 ||
        !std::isfinite(config.R_anc) || config.R_anc <= 0.0 ||
        !std::isfinite(config.max_envelope) || config.max_envelope < 0.0 ||
        !std::isfinite(config.min_envelope) || config.min_envelope < 0.0 ||
        config.min_envelope > config.max_envelope ||
        !std::isfinite(config.state_estimation_bound) || config.state_estimation_bound < 0.0 ||
        (config.state_estimation_bound > 0.0 && config.min_envelope <= 0.0) ||
        !config.correction_domain.allFinite() || config.correction_domain(0) >= 0.0 ||
        config.correction_domain(1) <= 0.0 ||
        !std::isfinite(config.tube_contraction) || config.tube_contraction <= 0.0 ||
        config.tube_contraction > 1.0 ||
        config.max_tube_faces < 32 || config.max_tube_faces > 1024 ||
        !std::isfinite(config.slack_linear_weight) || config.slack_linear_weight < 0.0 ||
        !std::isfinite(config.slack_quadratic_weight) || config.slack_quadratic_weight <= 0.0 ||
        config.max_working_set_recalculations < 1 ||
        !std::isfinite(config.feasibility_tolerance) || config.feasibility_tolerance <= 0.0)
        return;
    A_ << 1.0, config.dt, 0.0, 1.0;
    B_ << 0.5 * config.dt * config.dt, config.dt;
    Eigen::Matrix2d ancillary_metric;
    if (!dare(A_, B_, config.Q_diag.asDiagonal(), config.R, P_, Kf_) ||
        !dare(A_, B_, config.Q_anc_diag.asDiagonal(), config.R_anc,
              ancillary_metric, K_)) {
        config_status_ = "dare_failed";
        return;
    }
    if (!build_rpi_polytope(A_ + B_ * K_, ancillary_metric) ||
        !contracting_polytope(A_ + B_ * Kf_, P_, terminal_)) {
        config_status_ = "contractive_polytope_failed";
        return;
    }
    const Eigen::Vector2d room = config.state_limits - tube_radii(config.max_envelope);
    const double reserve = input_reserve(config.max_envelope);
    const double input_room = std::min(-config.correction_domain(0),
                                       config.correction_domain(1)) - reserve;
    if ((room.array() <= 0.0).any() || input_room <= 0.0) {
        config_status_ = "empty_common_terminal_domain";
        return;
    }
    double scale = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 2; ++i)
        scale = std::min(scale, room(i) / terminal_.support(Eigen::Vector2d::Unit(i)));
    const double terminal_input_support = terminal_.support(Kf_.transpose());
    if (terminal_input_support > 0.0) scale = std::min(scale, input_room / terminal_input_support);
    scale *= (1.0 - 1e-8);
    if (!std::isfinite(scale) || scale <= 1e-12) {
        config_status_ = "empty_common_terminal_set";
        return;
    }
    terminal_.radius = scale;
    for (auto& vertex : terminal_.vertices) vertex *= scale;
    // Verify each finite face of the invariant sets after construction.
    const Eigen::Matrix2d closed = A_ + B_ * K_;
    for (int i = 0; i < tube_.normals.rows(); ++i) {
        const Eigen::Vector2d n = tube_.normals.row(i).transpose();
        if (tube_.support(closed.transpose() * n) + disturbance_support(n) >
            config.tube_contraction * tube_.radius * (1.0 + 1e-9)) {
            config_status_ = "rpi_verification_failed";
            return;
        }
    }
    const Eigen::Matrix2d terminal_closed = A_ + B_ * Kf_;
    for (const auto& vertex : terminal_.vertices) {
        if (!terminal_.contains(terminal_closed * vertex, 1.0, 1e-9)) {
            config_status_ = "terminal_invariance_failed";
            return;
        }
    }
    configured_ = true;
    config_status_ = "ready";
}

void TubeMPC::reset() {
    has_plan_ = false;
    plan_ = Plan();
    envelope_ = 0.0;
    bounds_.setZero();
}

double TubeMPC::disturbance_support(const Eigen::Vector2d& direction) const {
    double result = direction.cwiseAbs().dot(B_);
    if (config_.state_estimation_bound > 0.0) {
        // With e_hat=e+epsilon, the estimated-state disturbance includes
        // epsilon_next-A*epsilon. A further E ball is reserved so that the
        // NEXT consistent true-state set e_hat_next+E, not just its center,
        // fits the initial tube constraint. Thus A_K*Z + W_hat + E <= Z.
        // Scaling by envelope>=min_envelope covers these fixed error terms
        // while preserving one common homothetic base tube.
        result += (2.0 * direction.norm() + (A_.transpose() * direction).norm()) *
                  config_.state_estimation_bound / config_.min_envelope;
    }
    return result;
}

Eigen::Vector2d TubeMPC::tube_radii(double envelope) const {
    if (tube_.vertices.empty()) return Eigen::Vector2d::Constant(
        std::numeric_limits<double>::infinity());
    return envelope * unit_tube_radii_;
}

double TubeMPC::input_reserve(double envelope) const {
    return tube_.vertices.empty() ? std::numeric_limits<double>::infinity() :
                                   envelope * unit_input_reserve_;
}

bool TubeMPC::tube_contains(const Eigen::Vector2d& delta, double envelope) const {
    return configured_ && tube_.contains(delta, envelope, config_.feasibility_tolerance);
}

bool TubeMPC::terminal_contains(const Eigen::Vector2d& state) const {
    return configured_ && terminal_.contains(state, 1.0, config_.feasibility_tolerance);
}

bool TubeMPC::context_valid(double envelope, const Eigen::Vector2d& bounds) const {
    if (!std::isfinite(envelope) || envelope < config_.min_envelope || envelope > config_.max_envelope ||
        !bounds.allFinite() || bounds(0) >= 0.0 || bounds(1) <= 0.0) return false;
    const Eigen::Vector2d room = config_.state_limits - tube_radii(envelope);
    const double reserve = input_reserve(envelope);
    const double lo = bounds(0) + reserve, hi = bounds(1) - reserve;
    if ((room.array() <= 0.0).any() || lo >= 0.0 || hi <= 0.0) return false;
    // The terminal neighborhood and weights are fixed across every update.
    for (const auto& vertex : terminal_.vertices) {
        const double command = Kf_.dot(vertex);
        if ((vertex.cwiseAbs().array() > room.array() + config_.feasibility_tolerance).any() ||
            command < lo - config_.feasibility_tolerance ||
            command > hi + config_.feasibility_tolerance) return false;
    }
    return true;
}

bool TubeMPC::transition_valid(double next_envelope) const {
    // Check A_K Z_old (+) W <= Z_new face by face. max(old,new) also covers
    // a larger candidate residual at the update boundary. The contraction of
    // the base tube admits decreases down to lambda*old.
    const double disturbance_envelope = std::max(envelope_, next_envelope);
    for (int i = 0; i < tube_.normals.rows(); ++i) {
        const double lhs = envelope_ * propagated_face_support_(i) +
                           disturbance_envelope * disturbance_face_support_(i);
        if (lhs > next_envelope * tube_.radius + config_.feasibility_tolerance)
            return false;
    }
    return true;
}

TubeMPC::Plan TubeMPC::shifted(const Plan& old, const Eigen::Vector2d& input_interval) const {
    Plan result;
    result.states.reserve(config_.horizon + 1);
    result.inputs.reserve(config_.horizon);
    result.states.push_back(old.states[1]);
    for (int i = 1; i < config_.horizon; ++i) result.inputs.push_back(old.inputs[i]);
    // Terminal continuation. The terminal set is a soft constraint, so the
    // terminal law is saturated to the tightened correction interval.
    result.inputs.push_back(std::min(input_interval(1),
        std::max(input_interval(0), Kf_.dot(old.states.back()))));
    for (int i = 0; i < config_.horizon; ++i)
        result.states.push_back(A_ * result.states.back() + B_ * result.inputs[i]);
    return result;
}

Eigen::Vector3d TubeMPC::required_slack(const Plan& plan, double envelope) const {
    const Eigen::Vector2d room = config_.state_limits - tube_radii(envelope);
    Eigen::Vector3d slack = Eigen::Vector3d::Zero();
    for (const auto& state : plan.states)
        for (int j = 0; j < 2; ++j)
            slack(j) = std::max(slack(j), std::abs(state(j)) - room(j));
    if (!plan.states.empty())
        slack(2) = std::max(0.0, (terminal_.normals * plan.states.back()).maxCoeff() -
                                 terminal_.radius);
    return slack;
}

bool TubeMPC::plan_valid(const Plan& plan, const Eigen::Vector2d& error,
                         double envelope, const Eigen::Vector2d& bounds) const {
    // Hard constraints only: dynamics, tightened correction inputs, initial
    // containment and the applied correction. Tracking and terminal
    // constraints are soft and priced through the slack variables.
    if (plan.states.size() != static_cast<std::size_t>(config_.horizon + 1) ||
        plan.inputs.size() != static_cast<std::size_t>(config_.horizon) ||
        !error.allFinite() || !plan.slack.allFinite() || (plan.slack.array() < 0.0).any())
        return false;
    const double tolerance = config_.feasibility_tolerance;
    for (int i = 0; i < tube_.normals.rows(); ++i) {
        const Eigen::RowVector2d n = tube_.normals.row(i);
        if (n.dot(error - plan.states.front()) + n.norm() * config_.state_estimation_bound >
            envelope * tube_.radius + tolerance) return false;
    }
    const double reserve = input_reserve(envelope);
    for (int i = 0; i <= config_.horizon; ++i) {
        if (!plan.states[i].allFinite()) return false;
        if (i < config_.horizon) {
            const double input = plan.inputs[i];
            if (!std::isfinite(input) || input < bounds(0) + reserve - tolerance ||
                input > bounds(1) - reserve + tolerance ||
                (A_ * plan.states[i] + B_ * input - plan.states[i + 1]).norm() > tolerance)
                return false;
        }
    }
    const double actual = plan.inputs.front() + K_.dot(error - plan.states.front());
    return std::isfinite(actual) && actual >= bounds(0) - tolerance &&
           actual <= bounds(1) + tolerance;
}

MPCResult TubeMPC::make_result(const Plan& plan, const Eigen::Vector2d& error,
                               double envelope, bool backup,
                               const std::string& status) const {
    MPCResult result;
    result.valid = true;
    result.used_backup = backup;
    result.envelope = envelope;
    result.nominal_state = plan.states.front();
    result.correction = plan.inputs.front() + K_.dot(error - result.nominal_state);
    result.tube_radii = tube_radii(envelope);
    result.input_reserve = input_reserve(envelope);
    result.slack = plan.slack;
    result.status = status;
    return result;
}

MPCResult TubeMPC::solve(const Eigen::Vector2d& error, double envelope,
                        const Eigen::Vector2d& supplied_correction_bounds,
                        double time_budget_seconds) {
    const auto started = Clock::now();
    MPCResult failure;
    failure.status = configured_ ? "invalid_context" : config_status_;
    if (!supplied_correction_bounds.allFinite()) return failure;
    const Eigen::Vector2d correction_bounds(
        std::max(supplied_correction_bounds(0), config_.correction_domain(0)),
        std::min(supplied_correction_bounds(1), config_.correction_domain(1)));
    if (!configured_ || !error.allFinite() || !std::isfinite(time_budget_seconds) ||
        !context_valid(envelope, correction_bounds)) return failure;
    const double reserve = input_reserve(envelope);
    const Eigen::Vector2d nominal_interval(correction_bounds(0) + reserve,
                                           correction_bounds(1) - reserve);

    // Remark 10: the shifted sequence with its terminal continuation is the
    // backup whenever the tube transition (Remark 8(i)) and its hard
    // constraints hold at the measured error. After a reference switch or a
    // disturbance outside the previous tube, the OCP is re-solved from the
    // measured error without a backup.
    Plan backup;
    bool have_backup = false;
    if (has_plan_ && transition_valid(envelope)) {
        backup = shifted(plan_, nominal_interval);
        backup.slack = required_slack(backup, envelope);
        have_backup = plan_valid(backup, error, envelope, correction_bounds);
    }
    auto commit = [&](const Plan& plan, bool used_backup, const std::string& status) {
        plan_ = plan;
        envelope_ = envelope;
        bounds_ = correction_bounds;
        has_plan_ = true;
        return make_result(plan_, error, envelope, used_backup, status);
    };
    auto use_backup = [&](const std::string& reason) -> MPCResult {
        if (!have_backup) {
            failure.status = reason + "_without_backup";
            return failure;
        }
        return commit(backup, true, reason);
    };
    if (time_budget_seconds <= elapsed(started)) return use_backup("solver_cutoff");

    // The zero nominal sequence has objective exactly zero and is therefore
    // the unique optimum whenever the measured error's consistent set fits
    // the tube. This also avoids a degenerate active-set start at equilibrium.
    Plan zero;
    zero.states.assign(config_.horizon + 1, Eigen::Vector2d::Zero());
    zero.inputs.assign(config_.horizon, 0.0);
    if (plan_valid(zero, error, envelope, correction_bounds)) {
        if (elapsed(started) > time_budget_seconds) return use_backup("solver_cutoff");
        return commit(zero, false, "optimal_zero_nominal");
    }

    // Decision vector [z0 (2), vbar_0..vbar_{H-1}, s_p, s_v, s_f].
    const int H = config_.horizon;
    const int slack_index = H + 2, variables = H + 5;
    const int tube_faces = static_cast<int>(tube_.normals.rows());
    const int terminal_faces = static_cast<int>(terminal_.normals.rows());
    const int constraints = tube_faces + 4 * (H + 1) + terminal_faces;
    std::vector<Eigen::MatrixXd> prediction(H + 1);
    prediction[0] = Eigen::MatrixXd::Zero(2, variables);
    prediction[0].leftCols(2).setIdentity();
    for (int i = 0; i < H; ++i) {
        prediction[i + 1] = A_ * prediction[i];
        prediction[i + 1].col(2 + i) += B_;
    }
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(variables, variables);
    for (int i = 0; i < H; ++i)
        hessian += 2.0 * prediction[i].transpose() * config_.Q_diag.asDiagonal() * prediction[i];
    hessian += 2.0 * prediction[H].transpose() * P_ * prediction[H];
    for (int i = 0; i < H; ++i) hessian(2 + i, 2 + i) += 2.0 * config_.R;
    for (int j = 0; j < 3; ++j)
        hessian(slack_index + j, slack_index + j) += 2.0 * config_.slack_quadratic_weight;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(variables);
    gradient.tail(3).setConstant(config_.slack_linear_weight);
    Eigen::VectorXd lower = Eigen::VectorXd::Constant(variables, -qpOASES::INFTY);
    Eigen::VectorXd upper = Eigen::VectorXd::Constant(variables, qpOASES::INFTY);
    lower.segment(2, H).setConstant(nominal_interval(0));
    upper.segment(2, H).setConstant(nominal_interval(1));
    lower.tail(3).setZero();
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(constraints, variables);
    Eigen::VectorXd lower_a = Eigen::VectorXd::Constant(constraints, -qpOASES::INFTY);
    Eigen::VectorXd upper_a = Eigen::VectorXd::Constant(constraints, qpOASES::INFTY);
    int row = 0;
    // Hard initial containment of the consistent true-state set, Eq. (46).
    matrix.block(row, 0, tube_faces, 2) = -tube_.normals;
    for (int i = 0; i < tube_faces; ++i)
        upper_a(row + i) = envelope * tube_.radius - tube_.normals.row(i).dot(error) -
                           tube_.normals.row(i).norm() * config_.state_estimation_bound;
    row += tube_faces;
    // Soft tightened tracking constraints z_i in E (-) Z_k (Remark 9).
    const Eigen::Vector2d room = config_.state_limits - tube_radii(envelope);
    for (int i = 0; i <= H; ++i) {
        for (int j = 0; j < 2; ++j) {
            matrix.row(row) = prediction[i].row(j);
            matrix(row, slack_index + j) = -1.0;
            upper_a(row++) = room(j);
            matrix.row(row) = prediction[i].row(j);
            matrix(row, slack_index + j) = 1.0;
            lower_a(row++) = -room(j);
        }
    }
    // Soft terminal set X_f.
    matrix.block(row, 0, terminal_faces, variables) = terminal_.normals * prediction[H];
    matrix.block(row, slack_index + 2, terminal_faces, 1).setConstant(-1.0);
    upper_a.segment(row, terminal_faces).setConstant(terminal_.radius);

    const double remaining = time_budget_seconds - elapsed(started);
    if (remaining <= 0.0) return use_backup("solver_cutoff");
    // qpOASES's default real_t is double, but casting also supports a library
    // built with single precision.
    using QPMatrix = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using QPVector = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>;
    QPMatrix qp_hessian = hessian.cast<qpOASES::real_t>();
    QPMatrix qp_matrix = matrix.cast<qpOASES::real_t>();
    QPVector qp_gradient = gradient.cast<qpOASES::real_t>();
    QPVector qp_lower = lower.cast<qpOASES::real_t>(), qp_upper = upper.cast<qpOASES::real_t>();
    QPVector qp_lower_a = lower_a.cast<qpOASES::real_t>(), qp_upper_a = upper_a.cast<qpOASES::real_t>();
    QPVector solution = QPVector::Zero(variables);
    QPVector seed = QPVector::Zero(variables);
    // Warm start from the backup, or from the saturated terminal law.
    Plan seed_plan;
    if (have_backup) {
        seed_plan = backup;
    } else {
        seed_plan.states.push_back(error);
        for (int i = 0; i < H; ++i) {
            const double input = std::min(nominal_interval(1),
                std::max(nominal_interval(0), Kf_.dot(seed_plan.states.back())));
            seed_plan.inputs.push_back(input);
            seed_plan.states.push_back(A_ * seed_plan.states.back() + B_ * input);
        }
        seed_plan.slack = required_slack(seed_plan, envelope);
    }
    const bool seed_feasible = plan_valid(seed_plan, error, envelope, correction_bounds);
    if (seed_feasible) {
        seed.head(2) = seed_plan.states.front().cast<qpOASES::real_t>();
        for (int i = 0; i < H; ++i) seed(2 + i) = static_cast<qpOASES::real_t>(seed_plan.inputs[i]);
        for (int j = 0; j < 3; ++j) seed(slack_index + j) = static_cast<qpOASES::real_t>(seed_plan.slack(j));
    }
    qpOASES::QProblem solver(variables, constraints);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    solver.setOptions(options);
    qpOASES::int_t iterations = config_.max_working_set_recalculations;
    const double solver_remaining = time_budget_seconds - elapsed(started);
    if (solver_remaining <= 0.0) return use_backup("solver_cutoff");
    qpOASES::real_t cpu_budget = static_cast<qpOASES::real_t>(solver_remaining);
    const auto solved = solver.init(qp_hessian.data(), qp_gradient.data(), qp_matrix.data(),
        qp_lower.data(), qp_upper.data(), qp_lower_a.data(), qp_upper_a.data(), iterations, &cpu_budget,
        seed_feasible ? seed.data() : nullptr);
    if (elapsed(started) > time_budget_seconds) return use_backup("solver_cutoff");
    if (solved != qpOASES::SUCCESSFUL_RETURN)
        return use_backup("solver_failed_" + std::to_string(static_cast<int>(solved)));
    if (solver.getPrimalSolution(solution.data()) != qpOASES::SUCCESSFUL_RETURN || !solution.allFinite())
        return use_backup("solver_solution_unavailable");
    Plan candidate;
    candidate.states.reserve(H + 1);
    candidate.inputs.reserve(H);
    const Eigen::VectorXd decision = solution.cast<double>();
    for (int i = 0; i <= H; ++i) candidate.states.push_back(prediction[i] * decision);
    for (int i = 0; i < H; ++i) candidate.inputs.push_back(decision(2 + i));
    candidate.slack = required_slack(candidate, envelope);
    const Plan continuation = shifted(candidate, nominal_interval);
    if (!plan_valid(candidate, error, envelope, correction_bounds) ||
        !plan_valid(continuation, candidate.states[1], envelope, correction_bounds))
        return use_backup("solution_validation_failed");
    if (elapsed(started) > time_budget_seconds) return use_backup("solver_cutoff");
    return commit(candidate, false, "optimal");
}

} // namespace uadl
