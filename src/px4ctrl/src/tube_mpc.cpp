#include "tube_mpc.h"

#include <qpOASES.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

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
double clamp_to(const Eigen::Vector2d& interval, double value) {
    return std::min(interval(1), std::max(interval(0), value));
}
} // namespace

double TubeMPC::Zonotope::support(const Eigen::Vector2d& direction) const {
    double result = 0.0;
    for (const auto& generator : generators) result += std::abs(direction.dot(generator));
    return scale * result;
}

bool TubeMPC::Zonotope::contains(const Eigen::Vector2d& point, double envelope,
                                 double tolerance) const {
    return point.allFinite() && std::isfinite(envelope) && envelope >= 0.0 &&
           (normals * point - envelope * offsets).maxCoeff() <= tolerance;
}

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

bool TubeMPC::build_base_tube(const Eigen::Matrix2d& closed_loop) {
    // Unit-envelope disturbance box W0 of Eq. (49): a residual bounded by 1
    // over a sampling interval gives |w_p| <= T_s^2/2 and |w_v| <= T_s.
    const Eigen::Vector2d half_widths = B_.cwiseAbs();
    Zonotope base;
    base.generators.reserve(2 * static_cast<std::size_t>(config_.tube_terms));
    Eigen::Matrix2d power = Eigen::Matrix2d::Identity();
    for (int j = 0; j < config_.tube_terms; ++j) {
        base.generators.push_back(power.col(0) * half_widths(0));
        base.generators.push_back(power.col(1) * half_widths(1));
        power = (closed_loop * power).eval();
    }
    if (!power.allFinite()) return false;
    // Smallest scaling with A_K^s W0 <= alpha_req W0 for the box W0.
    base.alpha_required = (power.cwiseAbs() * half_widths).cwiseQuotient(half_widths).maxCoeff();
    if (!std::isfinite(base.alpha_required) || base.alpha_required > config_.tube_alpha)
        return false;

    // Exact half-space representation: every edge of a planar zonotope is
    // parallel to a generator, so the face normals are the generator normals.
    std::vector<std::pair<double, Eigen::Vector2d>> candidates;
    candidates.reserve(2 * base.generators.size());
    for (const auto& generator : base.generators) {
        const double length = generator.norm();
        if (!std::isfinite(length) || length <= 0.0) continue;
        const Eigen::Vector2d normal(-generator(1) / length, generator(0) / length);
        candidates.emplace_back(std::atan2(normal(1), normal(0)), normal);
        candidates.emplace_back(std::atan2(-normal(1), -normal(0)), -normal);
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
    std::vector<Eigen::Vector2d> normals;
    double last_angle = -std::numeric_limits<double>::infinity();
    for (const auto& candidate : candidates) {
        // Parallel generators share one face; merge directions equal to
        // working precision.
        if (candidate.first - last_angle > 1e-12) {
            normals.push_back(candidate.second);
            last_angle = candidate.first;
        }
    }
    if (normals.size() > 1 &&
        (normals.front() - normals.back()).norm() <= 1e-12) normals.pop_back();
    if (normals.size() < 4) return false;

    const auto unscaled_support = [&](const Eigen::Vector2d& direction) {
        double result = 0.0;
        for (const auto& generator : base.generators) result += std::abs(direction.dot(generator));
        return result;
    };
    // c = 1/(1-alpha) gives the RPI set A_K Z0 (+) W0 <= Z0. For lambda < 1,
    // c(h_S - h_W + h_{A^s W}) + h_W <= lambda c h_S on every face fixes the
    // smallest scaling of the same set that is lambda-contractive.
    const double lambda = config_.tube_contraction;
    double scale = 1.0 / (1.0 - config_.tube_alpha);
    if (lambda < 1.0) {
        for (const auto& normal : normals) {
            const double h_w = normal.cwiseAbs().dot(half_widths);
            const double h_tail = (power.transpose() * normal).cwiseAbs().dot(half_widths);
            const double denominator = h_w - h_tail - (1.0 - lambda) * unscaled_support(normal);
            if (!(denominator > 0.0)) return false;
            scale = std::max(scale, h_w / denominator);
        }
        scale *= 1.0 + 1e-9;
    }
    base.scale = scale;
    const Eigen::Index faces = static_cast<Eigen::Index>(normals.size());
    base.normals.resize(faces, 2);
    base.offsets.resize(faces);
    propagated_face_support_.resize(faces);
    disturbance_face_support_.resize(faces);
    for (Eigen::Index i = 0; i < faces; ++i) {
        const Eigen::Vector2d& normal = normals[static_cast<std::size_t>(i)];
        base.normals.row(i) = normal.transpose();
        base.offsets(i) = scale * unscaled_support(normal);
        propagated_face_support_(i) = scale * unscaled_support(closed_loop.transpose() * normal);
        disturbance_face_support_(i) = normal.cwiseAbs().dot(half_widths);
        // Face-by-face verification of A_K Z0 (+) W0 <= lambda Z0.
        if (propagated_face_support_(i) + disturbance_face_support_(i) >
            lambda * base.offsets(i) * (1.0 + 1e-12)) return false;
    }
    if (!base.normals.allFinite() || !base.offsets.allFinite() || (base.offsets.array() <= 0.0).any())
        return false;
    tube_ = std::move(base);
    unit_tube_radii_ = Eigen::Vector2d(tube_.support(Eigen::Vector2d::UnitX()),
                                       tube_.support(Eigen::Vector2d::UnitY()));
    unit_input_reserve_ = tube_.support(K_.transpose());
    return unit_tube_radii_.allFinite() && std::isfinite(unit_input_reserve_);
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
        !config.correction_domain.allFinite() || config.correction_domain(0) >= 0.0 ||
        config.correction_domain(1) <= 0.0 ||
        !config.max_estimation_error.allFinite() ||
        (config.max_estimation_error.array() < 0.0).any() ||
        config.tube_terms < 1 || config.tube_terms > 10000 ||
        !std::isfinite(config.tube_alpha) || config.tube_alpha <= 0.0 || config.tube_alpha >= 1.0 ||
        !std::isfinite(config.tube_contraction) || config.tube_contraction <= 0.0 ||
        config.tube_contraction > 1.0 ||
        !std::isfinite(config.slack_linear_weight) || config.slack_linear_weight <= 0.0 ||
        !std::isfinite(config.slack_quadratic_weight) || config.slack_quadratic_weight <= 0.0 ||
        config.max_working_set_recalculations < 1 ||
        !std::isfinite(config.feasibility_tolerance) || config.feasibility_tolerance <= 0.0)
        return;
    A_ << 1.0, config.dt, 0.0, 1.0;
    B_ << 0.5 * config.dt * config.dt, config.dt;
    // Terminal weight P and gain K_f from the DARE of (Q, R); ancillary LQR
    // gain K_anc from (Q_anc, R_anc), Section VI-C.
    Eigen::Matrix2d ancillary_weight;
    if (!dare(A_, B_, config.Q_diag.asDiagonal(), config.R, P_, Kf_) ||
        !dare(A_, B_, config.Q_anc_diag.asDiagonal(), config.R_anc, ancillary_weight, K_)) {
        config_status_ = "dare_failed";
        return;
    }
    if (!build_base_tube(A_ + B_ * K_)) {
        config_status_ = "finite_sum_tube_failed";
        return;
    }
    // A new estimator center can differ from the true state by E_info;
    // its complete consistency box contributes a second E_info. The
    // complete W already includes feedback-estimation error, so 2 E_info
    // here encloses the next information set rather than another W term.
    information_face_support_ = 2.0 * tube_.normals.cwiseAbs() * config.max_estimation_error;
    for (Eigen::Index i = 0; i < tube_.normals.rows(); ++i) {
        const double margin = tube_.offsets(i) - propagated_face_support_(i) -
                              disturbance_face_support_(i);
        if (information_face_support_(i) > 0.0) {
            if (margin <= 0.0) {
                config_status_ = "state_information_tube_has_no_invariant_margin";
                return;
            }
            minimum_information_envelope_ = std::max(minimum_information_envelope_,
                information_face_support_(i) / margin);
        }
    }
    if (!information_propagation_valid(config.max_envelope, config.max_envelope) ||
        minimum_information_envelope_ > config.max_envelope) {
        config_status_ = "state_information_tube_not_invariant_at_max_envelope";
        return;
    }
    if (!contracting_polytope(A_ + B_ * Kf_, P_, terminal_)) {
        config_status_ = "terminal_set_failed";
        return;
    }
    // Common terminal set of Eq. (50) for every admissible envelope:
    // X_f <= E (-) Z_max and K_f X_f <= V (-) K_anc Z_max.
    const Eigen::Vector2d room = config.state_limits - tube_radii(config.max_envelope);
    const double reserve = input_reserve(config.max_envelope);
    const double input_room = std::min(-config.correction_domain(0),
                                       config.correction_domain(1)) - reserve -
                              K_.cwiseAbs().dot(config.max_estimation_error);
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
}

Eigen::Vector2d TubeMPC::tube_radii(double envelope) const {
    if (tube_.generators.empty()) return Eigen::Vector2d::Constant(
        std::numeric_limits<double>::infinity());
    return envelope * unit_tube_radii_;
}

double TubeMPC::input_reserve(double envelope) const {
    return tube_.generators.empty() ? std::numeric_limits<double>::infinity() :
                                      envelope * unit_input_reserve_;
}

bool TubeMPC::tube_contains(const Eigen::Vector2d& delta, double envelope) const {
    return configured_ && tube_.contains(delta, envelope, config_.feasibility_tolerance);
}

bool TubeMPC::terminal_contains(const Eigen::Vector2d& state) const {
    return configured_ && terminal_.contains(state, 1.0, config_.feasibility_tolerance);
}

double TubeMPC::input_reserve(double envelope, const SafetyLimits&) const {
    // For every admissible estimator update, the feedback correction lies
    // in K_anc Z (+) K_anc [-max_estimation_error, max_estimation_error].
    // The common maximum also protects every future backup input when the
    // current state-information box happens to be smaller.
    return input_reserve(envelope) + K_.cwiseAbs().dot(config_.max_estimation_error);
}

bool TubeMPC::context_valid(double envelope, const Eigen::Vector2d& bounds,
                            const SafetyLimits& safety) const {
    if (!std::isfinite(envelope) || envelope < config_.min_envelope || envelope > config_.max_envelope ||
        !bounds.allFinite() || bounds(0) >= 0.0 || bounds(1) <= 0.0 ||
        !safety.estimation_error.allFinite() ||
        (safety.estimation_error.array() < 0.0).any() ||
        (safety.estimation_error.array() > config_.max_estimation_error.array()).any()) return false;
    const Eigen::Vector2d room = config_.state_limits - tube_radii(envelope);
    const double reserve = input_reserve(envelope, safety);
    const double lo = bounds(0) + reserve, hi = bounds(1) - reserve;
    if ((room.array() <= 0.0).any() || lo >= 0.0 || hi <= 0.0) return false;
    // The terminal neighborhood and weights are fixed across every update.
    for (const auto& vertex : terminal_.vertices) {
        const double command = Kf_.dot(vertex);
        if ((vertex.cwiseAbs().array() > room.array() + config_.feasibility_tolerance).any() ||
            command < lo - config_.feasibility_tolerance ||
            command > hi + config_.feasibility_tolerance) return false;
    }
    if (safety.have_tail_bounds) {
        if (!safety.active() || !safety.lower.allFinite() || !safety.upper.allFinite() ||
            !safety.tail_reference_lower.allFinite() || !safety.tail_reference_upper.allFinite() ||
            (safety.tail_reference_lower.array() > safety.tail_reference_upper.array()).any())
            return false;
        // Once the nominal trajectory reaches X_f, its complete K_f tail
        // remains in X_f. This enclosure therefore certifies physical safety
        // for every future compatible reference and allowed estimator update.
        Eigen::Vector2d terminal_radii;
        for (int c = 0; c < 2; ++c)
            terminal_radii(c) = terminal_.support(Eigen::Vector2d::Unit(c));
        const Eigen::Vector2d tail_margin = terminal_radii +
            tube_radii(config_.max_envelope) + config_.max_estimation_error;
        if ((safety.tail_reference_lower.array() - tail_margin.array() <
             safety.lower.array() - config_.feasibility_tolerance).any() ||
            (safety.tail_reference_upper.array() + tail_margin.array() >
             safety.upper.array() + config_.feasibility_tolerance).any()) return false;
    }
    return true;
}

double TubeMPC::min_next_envelope() const {
    if (!configured_) return std::numeric_limits<double>::infinity();
    double lower = std::max(config_.min_envelope, minimum_information_envelope_);
    if (!has_plan_) return lower;
    lower = std::max(lower, config_.tube_contraction * envelope_);
    for (Eigen::Index i = 0; i < tube_.normals.rows(); ++i) {
        const double p = propagated_face_support_(i);
        const double w = disturbance_face_support_(i);
        const double h = tube_.offsets(i);
        const double information = information_face_support_(i);
        // e_old*p + max(e_old,e_new)*w + information <= e_new*h.
        // The max gives two linear lower bounds; enforcing both is exact.
        lower = std::max(lower, (envelope_ * (p + w) + information) / h);
        if (h <= w) return std::numeric_limits<double>::infinity();
        lower = std::max(lower, (envelope_ * p + information) / (h - w));
    }
    return lower;
}

bool TubeMPC::information_propagation_valid(double current_envelope, double next_envelope) const {
    // Remark 8's state-information requirement is checked explicitly:
    // A_K Z_current (+) W_current (+) 2 E_info <= Z_next. max(old,new)
    // also covers an increased complete residual at the update boundary.
    const double disturbance_envelope = std::max(current_envelope, next_envelope);
    for (Eigen::Index i = 0; i < tube_.normals.rows(); ++i) {
        const double lhs = current_envelope * propagated_face_support_(i) +
                           disturbance_envelope * disturbance_face_support_(i) +
                           information_face_support_(i);
        if (lhs > next_envelope * tube_.offsets(i) + config_.feasibility_tolerance)
            return false;
    }
    return true;
}

bool TubeMPC::transition_valid(double next_envelope) const {
    return information_propagation_valid(envelope_, next_envelope);
}

TubeMPC::Plan TubeMPC::shifted(const Plan& old) const {
    Plan result;
    result.states.reserve(config_.horizon + 1);
    result.inputs.reserve(config_.horizon);
    result.states.push_back(old.states[1]);
    for (int i = 1; i < config_.horizon; ++i) result.inputs.push_back(old.inputs[i]);
    // Theorem 3 appends K_f z_H exactly. The hard terminal set guarantees
    // this input is tightened-admissible; modifying it would change A_f and
    // invalidate the terminal invariance and DARE decrease conditions.
    result.inputs.push_back(Kf_.dot(old.states.back()));
    for (int i = 0; i < config_.horizon; ++i)
        result.states.push_back(A_ * result.states.back() + B_ * result.inputs[i]);
    return result;
}

void TubeMPC::assign_slack(Plan& plan, double envelope) const {
    // Smallest slacks of the soft tracking-performance constraints, Eq. (54):
    // |z_j| <= limit - rho + s_j componentwise. The terminal set stays hard.
    const Eigen::Vector2d room = config_.state_limits - tube_radii(envelope);
    plan.slack.assign(plan.states.size(), Eigen::Vector2d::Zero());
    for (std::size_t j = 0; j < plan.states.size(); ++j)
        plan.slack[j] = (plan.states[j].cwiseAbs() - room).cwiseMax(0.0);
}

bool TubeMPC::plan_valid(const Plan& plan, const Eigen::Vector2d& error, double envelope,
                         const Eigen::Vector2d& bounds, const SafetyLimits& safety) const {
    // Hard constraints only: dynamics, tightened correction inputs, initial
    // tube containment, terminal membership, safety and applied correction.
    // Only tracking-performance constraints use penalized slacks.
    const std::size_t H = static_cast<std::size_t>(config_.horizon);
    if (plan.states.size() != H + 1 || plan.inputs.size() != H || plan.slack.size() != H + 1 ||
        !error.allFinite())
        return false;
    const double tolerance = config_.feasibility_tolerance;
    // Entire state-information box, not only its estimated center, must fit
    // inside the initial tube.
    if ((tube_.normals * (error - plan.states.front()) +
         tube_.normals.cwiseAbs() * safety.estimation_error -
         envelope * tube_.offsets).maxCoeff() > tolerance) return false;
    const double reserve = input_reserve(envelope, safety);
    const Eigen::Vector2d radii = tube_radii(envelope);
    const Eigen::Vector2d room = config_.state_limits - radii;
    if (!terminal_.contains(plan.states.back(), 1.0, tolerance)) return false;
    if (safety.active()) {
        const Eigen::Vector2d measured_state = safety.reference.front() + error;
        if ((measured_state.array() - safety.estimation_error.array() <
             safety.lower.array() - tolerance).any() ||
            (measured_state.array() + safety.estimation_error.array() >
             safety.upper.array() + tolerance).any()) return false;
    }
    for (std::size_t i = 0; i <= H; ++i) {
        if (!plan.states[i].allFinite() || !plan.slack[i].allFinite() ||
            (plan.slack[i].array() < 0.0).any()) return false;
        if ((plan.states[i].cwiseAbs().array() >
             (room + plan.slack[i]).array() + tolerance).any()) return false;
        if (safety.active()) {
            const Eigen::Vector2d state = safety.reference[i] + plan.states[i];
            if ((state.array() < (safety.lower + radii).array() - tolerance).any() ||
                (state.array() > (safety.upper - radii).array() + tolerance).any()) return false;
        }
        if (i < H) {
            const double input = plan.inputs[i];
            if (!std::isfinite(input) || input < bounds(0) + reserve - tolerance ||
                input > bounds(1) - reserve + tolerance ||
                (A_ * plan.states[i] + B_ * input - plan.states[i + 1]).norm() > tolerance)
                return false;
        }
    }
    const double actual = plan.inputs.front() + K_.dot(error - plan.states.front());
    const double estimator_reserve = K_.cwiseAbs().dot(safety.estimation_error);
    return std::isfinite(actual) && actual - estimator_reserve >= bounds(0) - tolerance &&
           actual + estimator_reserve <= bounds(1) + tolerance;
}

MPCResult TubeMPC::make_result(const Plan& plan, const Eigen::Vector2d& error,
                               double envelope, bool backup,
                               const SafetyLimits& safety, const std::string& status) const {
    MPCResult result;
    result.valid = true;
    result.used_backup = backup;
    result.envelope = envelope;
    result.nominal_state = plan.states.front();
    // Applied correction v_k = vbar_0 + K_anc (e_k - z_0), Eq. (45).
    result.correction = plan.inputs.front() + K_.dot(error - result.nominal_state);
    result.tube_radii = tube_radii(envelope);
    result.input_reserve = input_reserve(envelope, safety);
    for (const auto& slack : plan.slack) {
        result.slack(0) = std::max(result.slack(0), slack(0));
        result.slack(1) = std::max(result.slack(1), slack(1));
    }
    result.status = status;
    return result;
}

MPCResult TubeMPC::solve(const Eigen::Vector2d& error, double envelope,
                        const Eigen::Vector2d& supplied_correction_bounds,
                        const SafetyLimits& safety, double time_budget_seconds) {
    const auto started = Clock::now();
    MPCResult failure;
    failure.status = configured_ ? "invalid_context" : config_status_;
    if (!supplied_correction_bounds.allFinite()) return failure;
    const Eigen::Vector2d correction_bounds(
        std::max(supplied_correction_bounds(0), config_.correction_domain(0)),
        std::min(supplied_correction_bounds(1), config_.correction_domain(1)));
    if (!configured_ || !error.allFinite() || std::isnan(time_budget_seconds) ||
        !context_valid(envelope, correction_bounds, safety)) return failure;
    // Even the first accepted normal command must leave a feasible backup
    // for every admitted next estimator update, then for all later shifts
    // under this same common envelope. A pointwise next-state check is not
    // enough to establish this containment.
    if (envelope < minimum_information_envelope_ ||
        !information_propagation_valid(envelope, envelope)) {
        failure.status = "common_envelope_cannot_contain_future_information_boxes";
        return failure;
    }
    const int H = config_.horizon;
    if (safety.active()) {
        bool finite = safety.reference.size() == static_cast<std::size_t>(H + 2) &&
                      safety.lower.allFinite() && safety.upper.allFinite() &&
                      (safety.lower.array() < safety.upper.array()).all();
        for (const auto& point : safety.reference) finite = finite && point.allFinite();
        if (!finite) {
            failure.status = "invalid_safety_reference";
            return failure;
        }
    }
    // The continuation of a new plan is checked one sample ahead, against
    // the reference prediction shifted by one instant.
    SafetyLimits next_safety = safety;
    if (safety.active()) next_safety.reference.erase(next_safety.reference.begin());
    SafetyLimits current_safety = safety;
    if (safety.active()) current_safety.reference.pop_back();
    const double reserve = input_reserve(envelope, safety);
    const Eigen::Vector2d nominal_interval(correction_bounds(0) + reserve,
                                           correction_bounds(1) - reserve);

    // Remark 8: the shifted sequence with its terminal continuation is the
    // backup whenever the tube transition (i) and its hard constraints hold
    // at the measured error. A retained context can accept a model, tube or
    // reference update only if this shifted plan remains feasible; solving
    // a different OCP does not replace either update condition.
    Plan backup;
    bool have_backup = false;
    if (has_plan_) {
        if (!transition_valid(envelope)) {
            failure.status = "tube_update_breaks_propagation";
            return failure;
        }
        backup = shifted(plan_);
        assign_slack(backup, envelope);
        Plan backup_continuation = shifted(backup);
        assign_slack(backup_continuation, envelope);
        have_backup = plan_valid(backup, error, envelope, correction_bounds, current_safety) &&
            plan_valid(backup_continuation, backup.states[1], envelope, correction_bounds, next_safety);
        if (!have_backup) {
            failure.status = "update_breaks_shifted_plan_feasibility";
            return failure;
        }
    }
    auto commit = [&](const Plan& plan, bool used_backup, const std::string& status) {
        plan_ = plan;
        envelope_ = envelope;
        has_plan_ = true;
        return make_result(plan_, error, envelope, used_backup, current_safety, status);
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
    // the unique optimum whenever the measured error fits the tube and the
    // safety limits hold along the reference.
    Plan zero;
    zero.states.assign(H + 1, Eigen::Vector2d::Zero());
    zero.inputs.assign(H, 0.0);
    assign_slack(zero, envelope);
    Plan zero_continuation = shifted(zero);
    assign_slack(zero_continuation, envelope);
    if (plan_valid(zero, error, envelope, correction_bounds, current_safety) &&
        plan_valid(zero_continuation, Eigen::Vector2d::Zero(), envelope, correction_bounds, next_safety)) {
        if (elapsed(started) > time_budget_seconds) return use_backup("solver_cutoff");
        return commit(zero, false, "optimal_zero_nominal");
    }

    // Decision vector [z_0 (2), vbar_0..vbar_{H-1} (H),
    //                  s_p,0..s_p,H (H+1), s_v,0..s_v,H (H+1)].
    const int position_slack = 2 + H, velocity_slack = 3 + 2 * H;
    const int variables = 4 + 3 * H;
    const int tube_faces = static_cast<int>(tube_.normals.rows());
    const int terminal_faces = static_cast<int>(terminal_.normals.rows());
    const int safety_rows = safety.active() ? 2 * (H + 1) : 0;
    const int constraints = tube_faces + 4 * (H + 1) + safety_rows + terminal_faces;
    std::vector<Eigen::MatrixXd> prediction(H + 1);
    prediction[0] = Eigen::MatrixXd::Zero(2, variables);
    prediction[0].leftCols(2).setIdentity();
    for (int i = 0; i < H; ++i) {
        prediction[i + 1] = A_ * prediction[i];
        prediction[i + 1].col(2 + i) += B_;
    }
    // Eq. (48): sum ||z_i||_Q^2 + ||vbar_i||_R^2 + ||z_H||_P^2, plus the
    // linear and quadratic slack penalties of Eq. (54).
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(variables, variables);
    for (int i = 0; i < H; ++i)
        hessian += 2.0 * prediction[i].transpose() * config_.Q_diag.asDiagonal() * prediction[i];
    hessian += 2.0 * prediction[H].transpose() * P_ * prediction[H];
    for (int i = 0; i < H; ++i) hessian(2 + i, 2 + i) += 2.0 * config_.R;
    for (int j = position_slack; j < variables; ++j)
        hessian(j, j) += 2.0 * config_.slack_quadratic_weight;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(variables);
    gradient.tail(variables - position_slack).setConstant(config_.slack_linear_weight);
    Eigen::VectorXd lower = Eigen::VectorXd::Constant(variables, -qpOASES::INFTY);
    Eigen::VectorXd upper = Eigen::VectorXd::Constant(variables, qpOASES::INFTY);
    // Hard tightened correction inputs vbar_i in V (-) K_anc Z_kappa.
    lower.segment(2, H).setConstant(nominal_interval(0));
    upper.segment(2, H).setConstant(nominal_interval(1));
    lower.tail(variables - position_slack).setZero();
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(constraints, variables);
    Eigen::VectorXd lower_a = Eigen::VectorXd::Constant(constraints, -qpOASES::INFTY);
    Eigen::VectorXd upper_a = Eigen::VectorXd::Constant(constraints, qpOASES::INFTY);
    int row = 0;
    // Hard initial containment e_k - z_0 in Z_kappa, Eq. (48).
    for (int i = 0; i < tube_faces; ++i, ++row) {
        matrix.block(row, 0, 1, 2) = -tube_.normals.row(i);
        upper_a(row) = envelope * tube_.offsets(i) - tube_.normals.row(i).dot(error) -
                       tube_.normals.row(i).cwiseAbs().dot(safety.estimation_error);
    }
    // Soft tracking-performance constraints z_j in E (-) Z_kappa, Eq. (54).
    const Eigen::Vector2d room = config_.state_limits - tube_radii(envelope);
    for (int j = 0; j <= H; ++j) {
        for (int c = 0; c < 2; ++c) {
            const int slack_column = (c == 0 ? position_slack : velocity_slack) + j;
            matrix.row(row) = prediction[j].row(c);
            matrix(row, slack_column) = -1.0;
            upper_a(row++) = room(c);
            matrix.row(row) = prediction[j].row(c);
            matrix(row, slack_column) = 1.0;
            lower_a(row++) = -room(c);
        }
    }
    // Hard safety limits on the physical state, tightened by the tube.
    if (safety.active()) {
        const Eigen::Vector2d radii = tube_radii(envelope);
        for (int j = 0; j <= H; ++j) {
            for (int c = 0; c < 2; ++c) {
                matrix.row(row) = prediction[j].row(c);
                lower_a(row) = safety.lower(c) + radii(c) - safety.reference[j](c);
                upper_a(row++) = safety.upper(c) - radii(c) - safety.reference[j](c);
            }
        }
    }
    // Hard terminal set X_f, retained under every accepted update.
    matrix.block(row, 0, terminal_faces, variables) = terminal_.normals * prediction[H];
    upper_a.segment(row, terminal_faces).setConstant(terminal_.radius);

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
            const double input = clamp_to(nominal_interval, Kf_.dot(seed_plan.states.back()));
            seed_plan.inputs.push_back(input);
            seed_plan.states.push_back(A_ * seed_plan.states.back() + B_ * input);
        }
        assign_slack(seed_plan, envelope);
    }
    const bool seed_feasible = plan_valid(seed_plan, error, envelope, correction_bounds, current_safety);
    if (seed_feasible) {
        seed.head(2) = seed_plan.states.front().cast<qpOASES::real_t>();
        for (int i = 0; i < H; ++i) seed(2 + i) = static_cast<qpOASES::real_t>(seed_plan.inputs[i]);
        for (int j = 0; j <= H; ++j) {
            seed(position_slack + j) = static_cast<qpOASES::real_t>(seed_plan.slack[j](0));
            seed(velocity_slack + j) = static_cast<qpOASES::real_t>(seed_plan.slack[j](1));
        }
    }
    qpOASES::QProblem solver(variables, constraints);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    solver.setOptions(options);
    qpOASES::int_t iterations = config_.max_working_set_recalculations;
    const double solver_remaining = time_budget_seconds - elapsed(started);
    if (solver_remaining <= 0.0) return use_backup("solver_cutoff");
    qpOASES::real_t cpu_budget = static_cast<qpOASES::real_t>(
        std::min(solver_remaining, 1.0));
    const auto solved = solver.init(qp_hessian.data(), qp_gradient.data(), qp_matrix.data(),
        qp_lower.data(), qp_upper.data(), qp_lower_a.data(), qp_upper_a.data(), iterations, &cpu_budget,
        seed_feasible ? seed.data() : nullptr);
    if (solved == qpOASES::RET_MAX_NWSR_REACHED) return use_backup("solver_cutoff");
    if (solver.isInfeasible() || solved == qpOASES::RET_INIT_FAILED_INFEASIBILITY ||
        solved == qpOASES::RET_QP_INFEASIBLE)
        return use_backup("mpc_infeasible");
    if (solved != qpOASES::SUCCESSFUL_RETURN)
        return use_backup("solver_failed_" + std::to_string(static_cast<int>(solved)));
    // Late results are discarded even when qpOASES reports success.
    if (elapsed(started) > time_budget_seconds) return use_backup("solver_cutoff");
    if (solver.getPrimalSolution(solution.data()) != qpOASES::SUCCESSFUL_RETURN || !solution.allFinite())
        return use_backup("solver_solution_unavailable");
    Plan candidate;
    candidate.states.reserve(H + 1);
    candidate.inputs.reserve(H);
    const Eigen::VectorXd decision = solution.cast<double>();
    // Numerical acceptance checks the actual returned decision against every
    // current-QP row and bound, including nonnegative performance slacks.
    const Eigen::VectorXd row_values = matrix * decision;
    const double tolerance = config_.feasibility_tolerance;
    if (!row_values.allFinite() ||
        (decision.array() < lower.array() - tolerance).any() ||
        (decision.array() > upper.array() + tolerance).any() ||
        (row_values.array() < lower_a.array() - tolerance).any() ||
        (row_values.array() > upper_a.array() + tolerance).any())
        return use_backup("current_qp_feasibility_failed");
    for (int i = 0; i <= H; ++i) candidate.states.push_back(prediction[i] * decision);
    for (int i = 0; i < H; ++i) candidate.inputs.push_back(decision(2 + i));
    for (int j = 0; j <= H; ++j)
        candidate.slack.emplace_back(std::max(0.0, decision(position_slack + j)),
                                     std::max(0.0, decision(velocity_slack + j)));
    // Backup eligibility: the shifted continuation of the new plan is feasible.
    Plan continuation = shifted(candidate);
    assign_slack(continuation, envelope);
    if (!plan_valid(candidate, error, envelope, correction_bounds, current_safety) ||
        !plan_valid(continuation, candidate.states[1], envelope, correction_bounds, next_safety))
        return use_backup("solution_validation_failed");
    if (elapsed(started) > time_budget_seconds) return use_backup("solver_cutoff");
    return commit(candidate, false, "optimal");
}

} // namespace uadl
