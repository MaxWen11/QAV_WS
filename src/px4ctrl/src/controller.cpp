#include "controller.h"
#include <ATen/Parallel.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>

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
    if (!param.analysis_lower.allFinite() || !param.analysis_upper.allFinite() ||
        !(param.analysis_lower.array() < param.analysis_upper.array()).all()) {
        status_ = "bounds/state_lower must lie below bounds/state_upper"; return false;
    }
    if (!std::isfinite(param.ctrl_freq_max) || std::abs(param.ctrl_freq_max - 100.0) > 1e-6 ||
        !std::isfinite(param.gain_floor) || param.gain_floor <= 0.0 ||
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
        const double required[] = {b.rkhs_norm, b.disturbance, b.measurement, b.state_error,
            b.synchronization, b.command_modification, b.hold_error, b.lipschitz_f, b.lipschitz_g,
            b.reference_acceleration, b.max_envelope, b.min_envelope};
        if (!std::all_of(std::begin(required), std::end(required), nonnegative)) {
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
            // Frozen network interval propagation covers the entire analysis
            // domain, including every predicted state and sampling interval.
            auto lower = torch::empty({6}, torch::kFloat64);
            auto upper = torch::empty({6}, torch::kFloat64);
            for (int j = 0; j < 6; ++j) {
                lower.data_ptr<double>()[j] = param.analysis_lower(j);
                upper.data_ptr<double>()[j] = param.analysis_upper(j);
            }
            const auto interval = prior_models_[i].get_method("interval_bounds")({lower, upper}).toTuple();
            if (interval->elements().size() != 4)
                throw std::runtime_error("prior interval_bounds must return f0 lower/upper, g0 lower/upper");
            auto& region = prior_regions_[i];
            region.state_lower = param.analysis_lower;
            region.state_upper = param.analysis_upper;
            region.f0_lower = interval->elements()[0].toTensor().item<double>();
            region.f0_upper = interval->elements()[1].toTensor().item<double>();
            region.g0_lower = interval->elements()[2].toTensor().item<double>();
            region.g0_upper = interval->elements()[3].toTensor().item<double>();
            if (!std::isfinite(region.f0_lower) || !std::isfinite(region.f0_upper) ||
                !std::isfinite(region.g0_lower) || !std::isfinite(region.g0_upper) ||
                region.f0_lower > region.f0_upper || region.g0_lower < 1e-6 ||
                region.g0_lower > region.g0_upper)
                throw std::runtime_error("offline prior has no valid positive-gain interval on the analysis domain");
        }
    } catch (const std::exception& error) {
        status_ = std::string("offline prior load failed: ") + error.what(); return false;
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
    // Frozen generative prior of Eq. (15), f0 = -Theta1/Theta2, g0 = 1/Theta2.
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
    // Remark 3: the prior is used as evaluated. Nonfinite values or raw gains
    // g0 < 1e-6 are rejected, never clipped; the protected gain of Eq. (28)
    // applies only to the control denominator.
    for (std::size_t k = 0; k < states.size(); ++k) {
        if (!f0[k].allFinite() || !g0[k].allFinite() || (g0[k].array() < 1e-6).any()) {
            status_ = "offline prior rejected: nonfinite value or raw gain below 1e-6";
            return false;
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
    // Per-sample observation-error bound of Assumption 4:
    // e_bar_j = d_bar + eps_bar + (L_f + |u_j| L_g) e_bar_x + e_bar_sync.
    const auto& b = param.bounds[axis];
    return b.disturbance + b.measurement + b.synchronization +
        (b.lipschitz_f + std::abs(input) * b.lipschitz_g) * b.state_error;
}

double Controller::residualMargin(int axis, double command_bound, double reference_hold_bound) const {
    // Non-GP terms of Eq. (46): c_bar + d_bar + (L_f + u_bar L_g) e_bar_x,
    // where u_bar bounds the executed input, plus the acceleration-level
    // bound of the hold and reference-discretization errors that enter w_k
    // (Section V, Eq. 49).
    const auto& b = param.bounds[axis];
    const double max_u = param.physical_input_limits[axis].cwiseAbs().maxCoeff();
    const double ancillary_error = accepted_.mpc[axis].ancillary_gain().cwiseAbs().sum() * b.state_error;
    return command_bound + b.disturbance +
           (b.lipschitz_f + max_u * b.lipschitz_g) * b.state_error +
           b.hold_error + reference_hold_bound + ancillary_error;
}

uadl::SafetyLimits Controller::safetyLimits(int axis, const uadl::ReferenceContinuation& path,
                                           double stamp) const {
    // Hard safety limits (Remark 8) on the analysis domain X of Assumption 1,
    // evaluated along the reference continuation at the H+2 instants used
    // by the plan and its shifted continuation.
    uadl::SafetyLimits limits;
    const double dt = param.mpc[axis].dt;
    for (int i = 0; i <= param.mpc[axis].horizon + 1; ++i) {
        const auto sample = path.sample(stamp + i * dt);
        limits.reference.emplace_back(sample.p(axis), sample.v(axis));
    }
    limits.lower << param.analysis_lower(axis), param.analysis_lower(axis + 3);
    limits.upper << param.analysis_upper(axis), param.analysis_upper(axis + 3);
    limits.estimation_error.setConstant(param.bounds[axis].state_error);
    // Intersample reachability: |a| <= sup|v| + sup|x''_ref| + zeta_bar.
    // Shrink the sampling-state box by the maximum one-period excursion.
    const double acceleration = param.mpc[axis].correction_domain.cwiseAbs().maxCoeff() +
        param.bounds[axis].reference_acceleration + param.bounds[axis].max_envelope;
    const double velocity = std::max(std::abs(limits.lower(1)), std::abs(limits.upper(1)));
    const Eigen::Vector2d excursion(velocity * dt + 0.5 * acceleration * dt * dt,
                                    acceleration * dt);
    limits.lower += excursion;
    limits.upper -= excursion;
    const auto tail = path.bounds(stamp + param.mpc[axis].horizon * dt);
    limits.have_tail_bounds = tail.valid;
    limits.tail_reference_lower << tail.lower(axis), tail.lower(axis + 3);
    limits.tail_reference_upper << tail.upper(axis), tail.upper(axis + 3);
    return limits;
}

bool Controller::targetEnvelopes(const Context& context, Eigen::Vector3d& targets,
        std::array<Eigen::Vector2d, 3>& correction_domains,
        Eigen::Vector3d& command_bounds, const Eigen::Vector3d& reference_hold_bounds) const {
    // Full analysis-domain enclosure, not an extremum of a few samples.
    // It also covers intersample states admitted by safetyLimits().
    targets.setZero();
    for (int i = 0; i < 3; ++i) {
        const auto& limits = param.physical_input_limits[i];
        const auto bound = context.gp[i].predictedRegionBound(prior_regions_[i], limits(0), limits(1));
        if (!bound.valid) return false;
        const double protected_lower = std::max(param.gain_floor, bound.g_lower);
        const double reference_acceleration = param.bounds[i].reference_acceleration;
        // A common V whose protected inverse stays in the physical input
        // box for every state and every admitted reference acceleration.
        correction_domains[i](0) = std::max(param.mpc[i].correction_domain(0),
            bound.f_upper + protected_lower * limits(0) + reference_acceleration);
        correction_domains[i](1) = std::min(param.mpc[i].correction_domain(1),
            bound.f_lower + protected_lower * limits(1) - reference_acceleration);
        if (correction_domains[i](0) >= 0.0 || correction_domains[i](1) <= 0.0)
            return false;
        command_bounds(i) = std::max(param.bounds[i].command_modification,
            std::max(0.0, param.gain_floor - bound.g_lower) * limits.cwiseAbs().maxCoeff());
        targets(i) = bound.error_bound + residualMargin(i, command_bounds(i), reference_hold_bounds(i));
    }
    return true;
}

Controller::Evaluation Controller::evaluate(Context& context, const Desired_State_t& ref,
        const uadl::State& state, const Eigen::Vector3d& f0, const Eigen::Vector3d& g0,
        double voltage, double stamp, const Clock::time_point& cutoff, bool backup_only) {
    Evaluation prepared;
    if (!state.allFinite() || !ref.p.allFinite() || !ref.v.allFinite() ||
        !ref.a.allFinite() || !std::isfinite(ref.yaw)) {
        prepared.status = "nonfinite state or reference"; return prepared;
    }
    prepared.inside_domain = insideDomain(state);
    if (!prepared.inside_domain) {
        prepared.status = "state outside the analysis domain"; return prepared;
    }
    for (int i = 0; i < 3; ++i) {
        if (std::abs(ref.a(i)) > param.bounds[i].reference_acceleration) {
            prepared.status = "reference acceleration outside the admitted range"; return prepared;
        }
    }
    if (param.thr_map.accurate_thrust_model && (!std::isfinite(voltage) || voltage < param.low_voltage)) {
        prepared.status = "battery voltage outside the thrust-model domain"; return prepared;
    }
    Eigen::Vector3d targets, command_bounds;
    Eigen::Vector3d acceleration_limits;
    for (int i = 0; i < 3; ++i) acceleration_limits(i) = param.bounds[i].reference_acceleration;
    const auto path = backup_only ? context.reference_path :
        uadl::ReferenceContinuation::create(ref.p, ref.v, ref.a, ref.yaw, stamp, acceleration_limits);
    if (!path.valid) {
        prepared.status = "reference has no admissible terminal continuation"; return prepared;
    }
    if (!path.sample(stamp).valid) {
        prepared.status = "reference timestamp precedes its committed origin"; return prepared;
    }
    std::array<Eigen::Vector2d, 3> correction_domains, errors;
    std::array<uadl::SafetyLimits, 3> safety;
    if (!targetEnvelopes(context, targets, correction_domains, command_bounds,
                         path.accelerationChangeBound(param.mpc[0].dt))) {
        prepared.status = "invalid uniform residual bound or empty inverse-admissible correction set";
        return prepared;
    }
    for (int i = 0; i < 3; ++i) {
        prepared.posterior[i] = context.gp[i].predict(state, f0(i), g0(i));
        if (!prepared.posterior[i].valid) {
            prepared.status = std::string("invalid GP posterior on axis ") + kAxisNames[i];
            return prepared;
        }
        prepared.envelopes(i) = std::max({targets(i), context.mpc[i].min_next_envelope(),
                                           param.mpc[i].min_envelope});
        if (prepared.envelopes(i) > param.mpc[i].max_envelope + 1e-12) {
            prepared.envelope_exceeded = true;
            prepared.status = std::string("residual envelope exceeds the terminal design bound on axis ") + kAxisNames[i];
            return prepared;
        }
        errors[i] << state(i) - ref.p(i), state(i + 3) - ref.v(i);
        safety[i] = safetyLimits(i, path, stamp);
    }
    // Copy every solve input, including the model, tube and reference version.
    // The worker owns no reference to Controller, ROS inputs or accepted_.
    auto job = std::make_shared<SolveJob>();
    job->context = context;
    job->evaluation = prepared;
    const auto thrust_config = thrust_config_;
    const auto physical_limits = param.physical_input_limits;
    const double gain_floor = param.gain_floor;
    auto solve = [job, errors, safety, correction_domains, command_bounds, ref, path,
                  thrust_config, physical_limits, gain_floor, voltage, stamp, cutoff, backup_only]() {
        auto& result = job->evaluation;
        auto& snapshot = job->context;
        try {
            Eigen::Vector3d input;
            bool admissible = true;
            for (int i = 0; i < 3; ++i) {
                const double budget = backup_only ? 0.0 :
                    std::max(0.0, std::chrono::duration<double>(cutoff - Clock::now()).count()) / (3 - i);
                result.mpc[i] = snapshot.mpc[i].solve(errors[i], result.envelopes(i),
                                                     correction_domains[i], safety[i], budget);
                if (!result.mpc[i].valid) {
                    result.status = std::string("MPC on axis ") + kAxisNames[i] + ": " + result.mpc[i].status;
                    admissible = false; break;
                }
                result.used_backup = result.used_backup || result.mpc[i].used_backup;
                result.eta(i) = result.mpc[i].correction + ref.a(i);
                input(i) = (result.eta(i) - result.posterior[i].f) /
                           std::max(gain_floor, result.posterior[i].g);
            }
            if (admissible) {
                result.mapped = uadl::mapAcceleration(input, ref.yaw, voltage, thrust_config, physical_limits);
                admissible = result.mapped.valid;
                if (!admissible) result.status = "attitude/thrust mapping failed";
            }
            for (int i = 0; i < 3 && admissible; ++i) {
                // Check the actually mapped command, using raw posterior g.
                const double u_ex = result.mapped.executed_input(i);
                const auto response = result.posterior[i].response(u_ex);
                const double c = result.posterior[i].f + result.posterior[i].g * u_ex - result.eta(i);
                admissible = response.valid && std::isfinite(c) &&
                    std::abs(c) <= command_bounds(i) + 1e-9 &&
                    u_ex >= physical_limits[i](0) - 1e-9 && u_ex <= physical_limits[i](1) + 1e-9;
                if (!admissible) result.status = "executed command violates its residual or physical-input bound";
            }
            if (admissible) {
                snapshot.reference_path = path;
                snapshot.have_reference = true;
                result.valid = true;
                result.status = result.used_backup ? "backup" : "normal";
            }
        } catch (const std::exception& error) {
            result.valid = false;
            result.status = std::string("solve failed: ") + error.what();
        } catch (...) {
            result.valid = false;
            result.status = "solve failed with an unknown exception";
        }
        {
            std::lock_guard<std::mutex> lock(job->mutex);
            job->finished = Clock::now();
            job->done = true;
        }
        job->completed.notify_one();
    };
    if (backup_only) {
        // budget=0 shifts the retained plan and never invokes the QP solver.
        solve();
    } else {
        if (solve_job_) {
            std::lock_guard<std::mutex> lock(solve_job_->mutex);
            if (!solve_job_->done) {
                prepared.status = "previous QP worker is overdue; use the maintained backup";
                return prepared;
            }
        }
        if (Clock::now() >= cutoff) {
            prepared.status = "solver cutoff reached before launch"; return prepared;
        }
        solve_job_ = job;
        std::thread(std::move(solve)).detach();
        std::unique_lock<std::mutex> lock(job->mutex);
        if (!job->completed.wait_until(lock, cutoff, [&job] { return job->done; }) ||
            job->finished > cutoff) {
            prepared.status = "solver cutoff; late worker result discarded";
            return prepared;
        }
    }
    if (job->evaluation.valid) context = std::move(job->context);
    return job->evaluation;
}

Desired_State_t Controller::continuedReference(double stamp) const {
    // Follow the committed path through its stationary terminal tail.
    // Repeated backup cycles never restart the braking continuation.
    const auto sample = accepted_.reference_path.sample(stamp);
    Desired_State_t ref;
    ref.p = sample.p;
    ref.v = sample.v;
    ref.a = sample.a;
    ref.yaw = sample.yaw;
    ref.j.setZero();
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
    // Result-acceptance cutoff, measured from the scheduled release.
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
    // Online GP response label (Section VI-C): the IMU specific force rotated
    // into the inertial frame, plus the gravity vector -g e3.
    const auto observed = uadl::specificForceToNetAcceleration(imu.a, odom.q, param.gra);
    Eigen::Vector3d f0, g0;
    const bool observation_valid = observed.allFinite();
    if (!priors(state, f0, g0)) {
        status_ = "rejected prior"; return debug;
    }
    Context candidate = accepted_;
    bool inserted = false;
    bool sample_rejected = false;
    // A sample is new and valid for insertion when its state timestamp
    // advances (Section VI-C), at up to 100 Hz. It requires a synchronized
    // IMU label and is paired with the command executed before the
    // measurement (plus the configured actuation delay), never with the
    // current action.
    if (learning && observation_valid && stamp >= sample_epoch_ && imu_stamp >= sample_epoch_ &&
        stamp > last_sample_stamp_ + 1e-9 && std::abs(stamp - imu_stamp) <= param.sensor_max_skew) {
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
                // Pre-insertion residual check against the retained posterior.
                accepted = candidate.gp[i].insert(sample).accepted;
            }
            // The candidate remains provisional until evaluate() checks the
            // full-domain bound, changed input set and shifted backup plan.
            if (accepted) {
                inserted = true;
            } else {
                candidate.gp = accepted_.gp;
                sample_rejected = true;
            }
        }
    }
    // Remark 8: prepare the feasible continuation with the retained model and
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
    accepted_.reference_path = uadl::ReferenceContinuation();
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
    debug.response_label_valid = observation.allFinite();
    debug.des_a_x = eval.mapped.executed_input.x();
    debug.des_a_y = eval.mapped.executed_input.y();
    debug.des_a_z = eval.mapped.executed_input.z();
    debug.des_q_x = output.q.x(); debug.des_q_y = output.q.y();
    debug.des_q_z = output.q.z(); debug.des_q_w = output.q.w();
    debug.des_thr = output.thrust; debug.hover_percentage = param.thr_map.hover_percentage;
    debug.voltage = voltage; debug.controller_valid = output.valid;
    debug.used_backup = output.used_backup; debug.cycle_ms = elapsed_ms;
    debug.online_status = status_;
    debug.envelope_exceeded = eval.envelope_exceeded;
    debug.inside_analysis_domain = eval.inside_domain;
    for (int i = 0; i < 3; ++i) {
        const auto response = eval.posterior[i].response(eval.mapped.executed_input(i));
        debug.eta[i] = eval.eta(i);
        debug.gp_f[i] = eval.posterior[i].f; debug.gp_g[i] = eval.posterior[i].g;
        debug.gp_cov_fg[i] = eval.posterior[i].cov_fg;
        debug.response_sigma[i] = response.sigma; debug.response_error_bound[i] = response.error_bound;
        debug.residual_envelope[i] = eval.envelopes(i);
        // c_k = f_hat + g_hat u_ex - eta of Theorem 2(2), with the raw gain.
        debug.command_error[i] = eval.posterior[i].f + eval.posterior[i].g * eval.mapped.executed_input(i) - eval.eta(i);
        debug.nominal_position_error[i] = eval.mpc[i].nominal_state(0);
        debug.tube_position_radius[i] = eval.mpc[i].tube_radii(0);
        debug.tube_velocity_radius[i] = eval.mpc[i].tube_radii(1);
        debug.input_reserve[i] = eval.mpc[i].input_reserve;
        debug.tracking_slack[i] = eval.mpc[i].slack.maxCoeff();
        debug.gp_window_size[i] = static_cast<uint32_t>(pending_ ? pending_->gp[i].size() : accepted_.gp[i].size());
    }
}
