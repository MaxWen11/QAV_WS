#include "controller.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#ifdef UADL_WITH_TORCH
#include <ATen/Parallel.h>
#endif

namespace {
// Tolerated offset between MAVROS source stamps and the local ROS clock.
constexpr double kClockSkewTolerance = 0.02;
const char* const kAxisNames[3] = {"x", "y", "z"};
bool nonnegative(double value) { return std::isfinite(value) && value >= 0.0; }
bool unitQuaternion(const Eigen::Quaterniond& q) {
    return q.coeffs().allFinite() && std::abs(q.norm() - 1.0) < 0.05;
}
uadl::State stateOf(const Odom_Data_t& odom) {
    uadl::State state;
    state << odom.p, odom.v;
    return state;
}
}

Controller::Controller(Parameter_t& parameters) : param(parameters) {
    configured_ = configure();
    if (!configured_) ROS_ERROR_STREAM("[UADL] Controller configuration failed: " << status_);
}

bool Controller::configure() {
    if (param.method != "A" && param.method != "B" && param.method != "C" && param.method != "D") {
        status_ = "controller/method must be A, B, C or D"; return false;
    }
    if (!param.analysis_lower.allFinite() || !param.analysis_upper.allFinite() ||
        !(param.analysis_lower.array() < param.analysis_upper.array()).all()) {
        status_ = "bounds/state_lower must lie below bounds/state_upper"; return false;
    }
    if (!std::isfinite(param.ctrl_freq_max) || std::abs(param.ctrl_freq_max - 100.0) > 1e-6 ||
        !std::isfinite(param.gain_floor) || param.gain_floor <= 0.0 ||
        !nonnegative(param.predicted_input_radius) ||
        !nonnegative(param.sensor_max_skew) || !nonnegative(param.input_delay) ||
        !std::isfinite(param.command_max_age) || param.command_max_age <= 0.0 ||
        !std::isfinite(param.solver_cutoff) || param.solver_cutoff <= 0.0 ||
        !std::isfinite(param.control_deadline) || param.control_deadline > 0.01 ||
        param.solver_cutoff >= param.control_deadline) {
        status_ = "control timing (100 Hz, solver cutoff below the 10 ms deadline) or GP sampling settings invalid";
        return false;
    }
    thrust_config_.gravity = param.gra;
    thrust_config_.mass = param.mass;
    thrust_config_.hover_fraction = param.thr_map.hover_percentage;
    thrust_config_.max_tilt = param.max_angle;
    thrust_config_.voltage_model = param.thr_map.accurate_thrust_model;
    thrust_config_.k1 = param.thr_map.K1;
    thrust_config_.k2 = param.thr_map.K2;
    thrust_config_.k3 = param.thr_map.K3;
    if (param.thr_map.accurate_thrust_model &&
        (!std::isfinite(param.low_voltage) || param.low_voltage <= 0.0 || param.thr_map.K2 < 0.0)) {
        status_ = "accurate thrust model needs a positive low_voltage and nonnegative K2"; return false;
    }
    for (int axis = 0; axis < 3; ++axis) {
        const auto& b = param.bounds[axis];
        const double required[] = {b.rkhs_norm, b.rkhs_norm_nominal, b.disturbance, b.measurement,
            b.state_error, b.synchronization, b.command_modification, b.hold_error, b.lipschitz_f,
            b.lipschitz_g, b.prior_abs_f, b.reference_acceleration, b.max_envelope,
            b.min_envelope, b.fixed_envelope};
        if (!std::all_of(std::begin(required), std::end(required), nonnegative) ||
            !std::isfinite(b.prior_min_g) || b.prior_min_g < 1e-6 ||
            !std::isfinite(b.prior_max_g) || b.prior_max_g < b.prior_min_g) {
            status_ = std::string("residual budget on axis ") + kAxisNames[axis] +
                      " must be finite and nonnegative";
            return false;
        }
        if (!param.physical_input_limits[axis].allFinite() ||
            param.physical_input_limits[axis](0) >= 0.0 || param.physical_input_limits[axis](1) <= 0.0 ||
            std::abs(param.mpc[axis].dt - 0.01) > 1e-9) {
            status_ = "physical input bounds must contain zero and rtmpc/dt must equal 0.01 s"; return false;
        }
        accepted_.gp.emplace_back(param.gp[axis]);
        accepted_.mpc.emplace_back(param.mpc[axis]);
        if (!accepted_.gp.back().valid() || !accepted_.mpc.back().configured()) {
            status_ = std::string("GP/MPC configuration on axis ") + kAxisNames[axis] + ": " +
                      accepted_.mpc.back().configuration_status();
            return false;
        }
    }
    // The whole physical input box must pass the attitude/thrust mapping
    // without tilt or thrust saturation, evaluated at the lowest voltage.
    const double voltage = param.thr_map.accurate_thrust_model ? param.low_voltage : 0.0;
    for (int vertex = 0; vertex < 8; ++vertex) {
        Eigen::Vector3d input;
        for (int i = 0; i < 3; ++i) input(i) = param.physical_input_limits[i]((vertex >> i) & 1);
        const auto mapped = uadl::mapAcceleration(input, 0.0, voltage, thrust_config_, param.physical_input_limits);
        if (!mapped.valid || (mapped.executed_input - input).norm() > 1e-7) {
            status_ = "physical input box exceeds the thrust/tilt range of the thrust model"; return false;
        }
    }
    if (param.method != "C") {
#ifdef UADL_WITH_TORCH
        try {
            at::set_num_threads(1);
            torch::NoGradGuard no_grad;
            for (int i = 0; i < 3; ++i) {
                if (param.prior_paths[i].empty()) throw std::runtime_error("prior/model_" + std::string(kAxisNames[i]) + " is empty");
                prior_models_[i] = torch::jit::load(param.prior_paths[i], torch::kCPU);
                prior_models_[i].eval();
                auto tuple = prior_models_[i].forward({torch::zeros({1, 6}, torch::kFloat32)}).toTuple();
                if (tuple->elements().size() != 2 ||
                    tuple->elements()[0].toTensor().numel() != 1 || tuple->elements()[1].toTensor().numel() != 1)
                    throw std::runtime_error("prior must map float32[N,6] to (f0[N,1], g0[N,1])");
            }
        } catch (const std::exception& error) {
            status_ = std::string("offline prior load failed: ") + error.what(); return false;
        }
#else
        status_ = "configurations A, B and D use the offline prior; build with UADL_ENABLE_TORCH=ON"; return false;
#endif
    }
    status_ = "ready";
    return true;
}

bool Controller::priors(const std::vector<uadl::State>& states,
                        std::vector<Eigen::Vector3d>& f0, std::vector<Eigen::Vector3d>& g0) {
    f0.assign(states.size(), Eigen::Vector3d::Zero());
    g0.assign(states.size(), Eigen::Vector3d::Ones());
    for (const auto& state : states)
        if (!state.allFinite()) return false;
    if (param.method != "C") {
#ifdef UADL_WITH_TORCH
        try {
            torch::NoGradGuard no_grad;
            const auto count = static_cast<int64_t>(states.size());
            auto input = torch::empty({count, 6}, torch::kFloat32);
            float* data = input.data_ptr<float>();
            for (int64_t k = 0; k < count; ++k)
                for (int j = 0; j < 6; ++j) data[k * 6 + j] = static_cast<float>(states[k](j));
            for (int i = 0; i < 3; ++i) {
                auto result = prior_models_[i].forward({input}).toTuple();
                if (result->elements().size() != 2) return false;
                const auto f = result->elements()[0].toTensor().to(torch::kFloat64).contiguous().view({-1});
                const auto g = result->elements()[1].toTensor().to(torch::kFloat64).contiguous().view({-1});
                if (f.numel() != count || g.numel() != count) return false;
                const auto fa = f.accessor<double, 1>();
                const auto ga = g.accessor<double, 1>();
                for (int64_t k = 0; k < count; ++k) {
                    f0[k](i) = fa[k];
                    g0[k](i) = ga[k];
                }
            }
        } catch (const std::exception& error) {
            status_ = std::string("offline prior inference failed: ") + error.what(); return false;
        }
#else
        return false;
#endif
    }
    // The prior envelope of Remark 3: f0 and g0 are kept inside the declared
    // bounds used by the residual budget; g0 stays strictly positive.
    for (std::size_t k = 0; k < states.size(); ++k) {
        if (!f0[k].allFinite() || !g0[k].allFinite()) return false;
        for (int i = 0; i < 3; ++i) {
            const auto& b = param.bounds[i];
            f0[k](i) = std::max(-b.prior_abs_f, std::min(b.prior_abs_f, f0[k](i)));
            g0[k](i) = std::max(b.prior_min_g, std::min(b.prior_max_g, g0[k](i)));
        }
    }
    return true;
}

bool Controller::priors(const uadl::State& state, Eigen::Vector3d& f0, Eigen::Vector3d& g0) {
    std::vector<Eigen::Vector3d> f, g;
    if (!priors(std::vector<uadl::State>{state}, f, g)) return false;
    f0 = f.front();
    g0 = g.front();
    return true;
}

bool Controller::insideDomain(const uadl::State& state) const {
    return state.allFinite() && (state.array() >= param.analysis_lower.array()).all() &&
           (state.array() <= param.analysis_upper.array()).all();
}

double Controller::historicalError(int axis, double input) const {
    // Per-sample observation error of Assumption 4.
    const auto& b = param.bounds[axis];
    return b.disturbance + b.measurement + b.synchronization +
        (b.lipschitz_f + std::abs(input) * b.lipschitz_g) * b.state_error;
}

double Controller::residualMargin(int axis) const {
    // Non-GP terms of Eq. (44): c + d + (Lf + u_bar*Lg)*e_x, plus hold error.
    const auto& b = param.bounds[axis];
    const double max_u = param.physical_input_limits[axis].cwiseAbs().maxCoeff();
    return b.command_modification + b.disturbance +
           (b.lipschitz_f + max_u * b.lipschitz_g) * b.state_error + b.hold_error;
}

Eigen::Vector3d Controller::lastExecutedInput() const {
    return history_.empty() ? Eigen::Vector3d::Zero() : history_.back().input;
}

std::vector<uadl::State> Controller::predictedStates(const uadl::State& state,
                                                     const Desired_State_t& ref) const {
    // Reference continuation over the horizon, shifted by the current error.
    const double horizon = param.mpc[0].dt * param.mpc[0].horizon;
    std::vector<uadl::State> points{state};
    for (double tau : {0.5 * horizon, horizon}) {
        uadl::State point = state;
        for (int i = 0; i < 3; ++i) {
            point(i) = state(i) + tau * ref.v(i) + 0.5 * tau * tau * ref.a(i);
            point(i + 3) = state(i + 3) + tau * ref.a(i);
        }
        points.push_back(point);
    }
    return points;
}

bool Controller::targetEnvelopes(const Context& context, const Desired_State_t& ref,
                                 const uadl::State& state, Eigen::Vector3d& targets) {
    targets.setZero();
    if (param.method == "A") return true;
    if (param.method == "B") {
        for (int i = 0; i < 3; ++i) targets(i) = param.bounds[i].fixed_envelope;
        return true;
    }
    // Eq. (44) with E_q from Eq. (34) over the predicted state-input set.
    const auto points = predictedStates(state, ref);
    std::vector<Eigen::Vector3d> f0, g0;
    if (!priors(points, f0, g0)) return false;
    const Eigen::Vector3d last_input = lastExecutedInput();
    for (int i = 0; i < 3; ++i) {
        std::vector<double> f(points.size()), g(points.size());
        for (std::size_t k = 0; k < points.size(); ++k) {
            f[k] = f0[k](i);
            g[k] = g0[k](i);
        }
        const auto& limits = param.physical_input_limits[i];
        const double u_min = std::max(limits(0), std::min(limits(1), last_input(i) - param.predicted_input_radius));
        const double u_max = std::min(limits(1), std::max(limits(0), last_input(i) + param.predicted_input_radius));
        const auto bound = context.gp[i].predictedSetBound(points, f, g, u_min, u_max);
        if (!bound.valid) return false;
        targets(i) = bound.error_bound + residualMargin(i);
    }
    return true;
}

Controller::Evaluation Controller::evaluate(Context& context, const Desired_State_t& ref,
        const uadl::State& state, const Eigen::Vector3d& f0, const Eigen::Vector3d& g0,
        double voltage, double stamp, const Clock::time_point& cutoff, bool backup_only) {
    Evaluation result;
    if (!state.allFinite() || !ref.p.allFinite() || !ref.v.allFinite() ||
        !ref.a.allFinite() || !std::isfinite(ref.yaw)) {
        result.status = "nonfinite state or reference"; return result;
    }
    result.inside_domain = insideDomain(state);
    Eigen::Vector3d targets;
    if (!targetEnvelopes(context, ref, state, targets)) {
        result.status = "residual envelope evaluation failed"; return result;
    }
    Eigen::Vector3d input;
    for (int i = 0; i < 3; ++i) {
        const auto& b = param.bounds[i];
        auto& post = result.posterior[i];
        post = context.gp[i].predict(state, f0(i), g0(i));
        if (!post.valid) {
            result.status = std::string("GP posterior on axis ") + kAxisNames[i] + ": " +
                            uadl::gpStatusName(post.status);
            return result;
        }
        // Homothetic scaling of Eq. (45) under the tube transition of Remark 8.
        double envelope = targets(i);
        if (param.method != "A") {
            envelope = std::max({envelope, context.mpc[i].min_next_envelope(), param.mpc[i].min_envelope});
            if (envelope > param.mpc[i].max_envelope) {
                envelope = param.mpc[i].max_envelope;
                result.envelope_saturated = true;
            }
        }
        result.envelopes(i) = envelope;
        const double budget = backup_only ? 0.0 :
            std::max(0.0, std::chrono::duration<double>(cutoff - Clock::now()).count()) / (3 - i);
        const Eigen::Vector2d error(state(i) - ref.p(i), state(i + 3) - ref.v(i));
        result.mpc[i] = context.mpc[i].solve(error, envelope, param.mpc[i].correction_domain, budget);
        if (!result.mpc[i].valid) {
            result.status = std::string("MPC on axis ") + kAxisNames[i] + ": " + result.mpc[i].status;
            return result;
        }
        result.used_backup = result.used_backup || result.mpc[i].used_backup;
        // eta = v + reference acceleration, followed by the protected inverse.
        const double feedforward = std::max(-b.reference_acceleration,
                                            std::min(b.reference_acceleration, ref.a(i)));
        result.eta(i) = result.mpc[i].correction + feedforward;
        input(i) = (result.eta(i) - post.f) / std::max(param.gain_floor, post.g);
    }
    if (param.thr_map.accurate_thrust_model && (!std::isfinite(voltage) || voltage <= 0.0)) {
        result.status = "battery voltage unavailable for the thrust model"; return result;
    }
    result.mapped = uadl::mapAcceleration(input, ref.yaw, voltage, thrust_config_, param.physical_input_limits);
    if (!result.mapped.valid) { result.status = "attitude/thrust mapping failed"; return result; }
    context.reference = ref;
    context.reference_stamp = stamp;
    context.have_reference = true;
    result.valid = true;
    result.status = result.used_backup ? "backup" : "normal";
    if (result.envelope_saturated) result.status += "; envelope held at max_envelope";
    return result;
}

Desired_State_t Controller::continuedReference(double stamp) const {
    Desired_State_t ref = accepted_.reference;
    const double dt = std::max(0.0, stamp - accepted_.reference_stamp);
    ref.p += dt * ref.v + 0.5 * dt * dt * ref.a;
    ref.v += dt * ref.a;
    ref.j.setZero();
    // The backup retains its reference acceleration and yaw (Remark 10).
    return ref;
}

quadrotor_msgs::Px4ctrlDebug Controller::update(const Desired_State_t& des,
        const Odom_Data_t& odom, const Imu_Data_t& imu,
        Controller_Output_t& output, double voltage, bool active, bool learning) {
    const auto started = cycle_announced_ ? cycle_started_ : Clock::now();
    cycle_started_ = started;
    cycle_announced_ = false;
    active_cycle_ = active;
    pending_.reset();
    const auto cutoff = started + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(param.solver_cutoff));
    const double now = ros::Time::now().toSec();
    if (reset_epoch_ > 0.0 && now >= reset_epoch_) {
        const double epoch = reset_epoch_;
        resetOnline();
        sample_epoch_ = epoch;
    }
    output = Controller_Output_t();
    debug = quadrotor_msgs::Px4ctrlDebug();
    if (!active) {
        if (was_active_) resetOnline();
        was_active_ = false;
        if (unitQuaternion(imu.q)) {
            // Attitude-hold setpoint streamed outside automatic control. It is
            // neither an adaptive command nor a source of GP samples.
            output.q = imu.q.normalized();
            output.thrust = 0.0;
            output.valid = true;
        }
        return debug;
    }
    was_active_ = true;
    if (!configured_) return debug;
    const uadl::State state = stateOf(odom);
    const double stamp = odom.msg.header.stamp.toSec();
    const double imu_stamp = imu.msg.header.stamp.toSec();
    if (!unitQuaternion(odom.q) || !unitQuaternion(imu.q) || !state.allFinite() ||
        stamp <= 0.0 || imu_stamp <= 0.0 ||
        stamp - now > kClockSkewTolerance || imu_stamp - now > kClockSkewTolerance ||
        now - stamp > param.msg_timeout.odom || now - imu_stamp > param.msg_timeout.imu) {
        status_ = "stale or invalid odometry/IMU"; return debug;
    }
    // Response label: IMU specific force rotated by the odometry attitude,
    // plus the gravity vector (Section VI-C).
    const auto observed = uadl::specificForceToNetAcceleration(imu.a, odom.q, param.gra);
    Eigen::Vector3d f0, g0;
    if (!observed.allFinite() || !priors(state, f0, g0)) {
        status_ = "nonfinite acceleration label or prior output"; return debug;
    }
    Context candidate = accepted_;
    bool inserted = false;
    bool sample_rejected = false;
    // A new sample needs an advancing state timestamp and a synchronized IMU
    // label. It is paired with the command published before the measurement
    // (plus the configured actuation delay), never with the current action.
    if (learning && (param.method == "C" || param.method == "D") &&
        stamp >= sample_epoch_ && imu_stamp >= sample_epoch_ && stamp > last_sample_stamp_ + 1e-9 &&
        std::abs(stamp - imu_stamp) <= param.sensor_max_skew) {
        last_sample_stamp_ = stamp;
        const ExecutedCommand* executed = nullptr;
        for (auto it = history_.rbegin(); it != history_.rend(); ++it)
            if (it->stamp + param.input_delay <= imu_stamp) { executed = &*it; break; }
        if (executed && imu_stamp - executed->stamp - param.input_delay <= param.command_max_age) {
            bool accepted = true;
            for (int i = 0; i < 3 && accepted; ++i) {
                uadl::GPSample sample;
                sample.state = state;
                sample.u_ex = executed->input(i);
                sample.y = observed(i);
                sample.f0 = f0(i);
                sample.g0 = g0(i);
                sample.timestamp = stamp;
                sample.error_bound = historicalError(i, sample.u_ex);
                // Pre-insertion residual gate against the retained posterior.
                accepted = candidate.gp[i].insert(sample).accepted;
            }
            if (accepted) {
                // The updated model must keep the predicted-set envelope within
                // the common terminal domain (Remark 8).
                Eigen::Vector3d targets;
                accepted = targetEnvelopes(candidate, des, state, targets);
                for (int i = 0; i < 3 && accepted; ++i)
                    accepted = param.method == "A" || targets(i) <= param.mpc[i].max_envelope;
            }
            if (accepted) {
                inserted = true;
            } else {
                candidate.gp = accepted_.gp;
                sample_rejected = true;
            }
        }
    }
    // Remark 10: prepare the feasible continuation with the retained model and
    // a compatible reference before spending the remaining solver budget.
    Context backup = accepted_;
    Evaluation backup_result;
    Desired_State_t backup_ref;
    if (accepted_.have_reference) {
        backup_ref = continuedReference(now);
        backup_result = evaluate(backup, backup_ref, state, f0, g0, voltage, now, cutoff, true);
    }
    Evaluation selected = evaluate(candidate, des, state, f0, g0, voltage, now, cutoff, false);
    Desired_State_t selected_ref = des;
    if (!selected.valid && inserted && Clock::now() < cutoff) {
        // Reject the model update and keep tracking the commanded reference.
        candidate = accepted_;
        inserted = false;
        sample_rejected = true;
        selected = evaluate(candidate, des, state, f0, g0, voltage, now, cutoff, false);
    }
    // Late results are never committed, even if qpOASES reports success.
    if (!selected.valid || Clock::now() > cutoff) {
        const std::string reason = selected.valid ? "solver cutoff" : selected.status;
        if (!backup_result.valid) {
            status_ = reason + "; no feasible backup"; return debug;
        }
        candidate = std::move(backup);
        selected = std::move(backup_result);
        selected.used_backup = true;
        selected.status = "backup: " + reason;
        selected_ref = backup_ref;
        inserted = false;
    }
    const double elapsed = std::chrono::duration<double>(Clock::now() - started).count();
    if (elapsed >= param.control_deadline) {
        status_ = "control deadline exceeded; late command discarded"; return debug;
    }
    output.q = (imu.q.normalized() * odom.q.normalized().inverse() * selected.mapped.attitude).normalized();
    output.thrust = selected.mapped.thrust;
    output.used_backup = selected.used_backup;
    output.adaptive_command = true;
    output.executed_input = selected.mapped.executed_input;
    output.valid = unitQuaternion(output.q) && std::isfinite(output.thrust);
    if (!output.valid) { status_ = "invalid mapped attitude/thrust command"; return debug; }
    pending_.reset(new Context(std::move(candidate)));
    status_ = selected.status;
    populateDebug(selected_ref, odom, observed, selected, output, voltage, elapsed * 1000.0);
    debug.gp_sample_inserted = inserted;
    debug.gp_sample_rejected = sample_rejected;
    return debug;
}

void Controller::resetOnline() {
    for (auto& gp : accepted_.gp) gp.reset();
    for (auto& mpc : accepted_.mpc) mpc.reset();
    accepted_.have_reference = false;
    pending_.reset();
    history_.clear();
    last_sample_stamp_ = 0.0;
    reset_epoch_ = 0.0;
    sample_epoch_ = ros::Time::now().toSec();
    if (configured_) status_ = "online state reset";
}

bool Controller::scheduleReset(double epoch) {
    if (!configured_ || !std::isfinite(epoch) || epoch <= ros::Time::now().toSec()) return false;
    reset_epoch_ = epoch;
    return true;
}

void Controller::beginCycle(Clock::time_point started) {
    cycle_started_ = started;
    cycle_announced_ = true;
    pending_.reset();
}

bool Controller::publicationAllowed() const {
    return !active_cycle_ ||
        std::chrono::duration<double>(Clock::now() - cycle_started_).count() < param.control_deadline;
}

void Controller::discardPending() { pending_.reset(); }

void Controller::commitPublished(const Controller_Output_t& output, const ros::Time& stamp) {
    // The candidate model/plan and its command history are committed only
    // after the command has been published.
    if (output.valid && output.adaptive_command && pending_) {
        accepted_ = std::move(*pending_);
        pending_.reset();
        history_.push_back({stamp.toSec(), output.executed_input});
        while (history_.size() > 200 || (!history_.empty() && stamp.toSec() - history_.front().stamp > 2.0))
            history_.pop_front();
    }
    debug.header.stamp = stamp;
    debug.cycle_ms = 1000.0 * std::chrono::duration<double>(Clock::now() - cycle_started_).count();
    // Valid automatic command; attitude-hold and motor-idle setpoints are not.
    debug.controller_valid = output.valid && output.adaptive_command;
    if (active_cycle_ && debug.cycle_ms >= 1000.0 * param.control_deadline) {
        debug.online_status = "publication exceeded the control deadline";
        status_ = debug.online_status;
    }
}

void Controller::populateDebug(const Desired_State_t& ref, const Odom_Data_t& odom,
        const Eigen::Vector3d& observation, const Evaluation& eval,
        const Controller_Output_t& output, double voltage, double elapsed_ms) {
    debug.des_p_x = ref.p.x(); debug.des_p_y = ref.p.y(); debug.des_p_z = ref.p.z();
    debug.des_v_x = ref.v.x(); debug.des_v_y = ref.v.y(); debug.des_v_z = ref.v.z();
    debug.real_x = odom.p.x(); debug.real_y = odom.p.y(); debug.real_z = odom.p.z();
    debug.real_vx = odom.v.x(); debug.real_vy = odom.v.y(); debug.real_vz = odom.v.z();
    debug.fb_a_x = observation.x(); debug.fb_a_y = observation.y(); debug.fb_a_z = observation.z();
    debug.des_a_x = eval.mapped.executed_input.x();
    debug.des_a_y = eval.mapped.executed_input.y();
    debug.des_a_z = eval.mapped.executed_input.z();
    debug.des_q_x = output.q.x(); debug.des_q_y = output.q.y();
    debug.des_q_z = output.q.z(); debug.des_q_w = output.q.w();
    debug.des_thr = output.thrust; debug.hover_percentage = param.thr_map.hover_percentage;
    debug.voltage = voltage; debug.controller_valid = output.valid;
    debug.used_backup = output.used_backup; debug.cycle_ms = elapsed_ms;
    debug.online_status = status_;
    debug.envelope_saturated = eval.envelope_saturated;
    debug.inside_analysis_domain = eval.inside_domain;
    for (int i = 0; i < 3; ++i) {
        const auto response = eval.posterior[i].response(eval.mapped.executed_input(i));
        debug.eta[i] = eval.eta(i);
        debug.gp_f[i] = eval.posterior[i].f; debug.gp_g[i] = eval.posterior[i].g;
        debug.gp_cov_fg[i] = eval.posterior[i].cov_fg;
        debug.response_sigma[i] = response.sigma; debug.response_error_bound[i] = response.error_bound;
        debug.residual_envelope[i] = eval.envelopes(i);
        debug.command_error[i] = eval.posterior[i].f + eval.posterior[i].g * eval.mapped.executed_input(i) - eval.eta(i);
        debug.nominal_position_error[i] = eval.mpc[i].nominal_state(0);
        debug.tube_position_radius[i] = eval.mpc[i].tube_radii(0);
        debug.tube_velocity_radius[i] = eval.mpc[i].tube_radii(1);
        debug.input_reserve[i] = eval.mpc[i].input_reserve;
        debug.tracking_slack[i] = eval.mpc[i].slack.maxCoeff();
        debug.gp_window_size[i] = static_cast<uint32_t>(pending_ ? pending_->gp[i].size() : accepted_.gp[i].size());
    }
}
