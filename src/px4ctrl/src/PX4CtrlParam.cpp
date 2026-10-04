#include "PX4CtrlParam.h"
#include <cmath>
#include <vector>

namespace {
// Per-axis residual budget. The horizontal axes share one budget; the
// vertical axis has its own input range and disturbance level.
Parameter_t::ResidualBounds defaultBounds(int axis)
{
    Parameter_t::ResidualBounds b;
    if (axis == 2) {
        b.rkhs_norm = 0.30;
        b.disturbance = 0.20;
        b.measurement = 0.08;
        b.command_modification = 0.08;
        b.hold_error = 0.04;
        b.reference_acceleration = 0.5;
        b.min_envelope = 0.4;
        b.max_envelope = 0.95;
    }
    return b;
}
} // namespace

Parameter_t::Parameter_t()
{
}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{
    read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
    read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
    read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
    read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
    read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

    read_essential_param(nh, "mass", mass);
    read_essential_param(nh, "gra", gra);
    read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
    read_essential_param(nh, "max_manual_vel", max_manual_vel);
    read_essential_param(nh, "max_angle", max_angle);
    read_essential_param(nh, "low_voltage", low_voltage);

    read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
    read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
    read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
    read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

    read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
    read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
    read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
    read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
    read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);

    read_essential_param(nh, "thrust_model/K1", thr_map.K1);
    read_essential_param(nh, "thrust_model/K2", thr_map.K2);
    read_essential_param(nh, "thrust_model/K3", thr_map.K3);
    read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
    read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);

    nh.param("controller/solver_cutoff", solver_cutoff, 0.0085);
    nh.param("controller/deadline", control_deadline, 0.01);
    nh.param("online_gp/gain_floor", gain_floor, 0.5);
    nh.param("online_gp/max_sensor_skew", sensor_max_skew, 0.02);
    nh.param("online_gp/input_delay", input_delay, 0.0);
    nh.param("online_gp/command_max_age", command_max_age, 0.03);

    // Analysis domain X of Assumption 1 and hard safety limits of Remark 8:
    // [p_x, p_y, p_z, v_x, v_y, v_z].
    const std::vector<double> default_lower{-5.0, -5.0, -0.5, -3.0, -3.0, -2.0};
    const std::vector<double> default_upper{5.0, 5.0, 3.5, 3.0, 3.0, 2.0};
    std::vector<double> lower, upper;
    nh.param("bounds/state_lower", lower, default_lower);
    nh.param("bounds/state_upper", upper, default_upper);
    if (lower.size() != 6 || upper.size() != 6) {
        ROS_WARN("bounds/state_lower and state_upper need six entries; using the arena defaults.");
        lower = default_lower;
        upper = default_upper;
    }
    for (int j = 0; j < 6; ++j) {
        analysis_lower(j) = lower[j];
        analysis_upper(j) = upper[j];
    }

    const std::array<std::string, 3> axes{{"x", "y", "z"}};
    for (int i = 0; i < 3; ++i) {
        auto& g = gp[i];
        auto& m = mpc[i];
        auto& b = bounds[i];
        const auto d = defaultBounds(i);
        const bool vertical = i == 2;
        const std::string prefix = std::string("rtmpc/") + (vertical ? "z/" : "xy/");

        nh.param("online_gp/l", g.lengthscale, 0.5);
        nh.param("online_gp/variance_a", g.variance_a, 1.0);
        nh.param("online_gp/variance_b", g.variance_b, 1.0);
        nh.param("online_gp/noise_variance", g.noise_variance, 0.01);
        nh.param("online_gp/minimum_sample_interval", g.minimum_sample_interval, 0.01);
        int window = 50;
        nh.param("online_gp/N_max", window, 50);
        g.max_samples = window > 0 ? static_cast<std::size_t>(window) : 0;

        nh.param("rtmpc/dt", m.dt, 0.01);
        nh.param("rtmpc/H", m.horizon, 20);
        nh.param("rtmpc/tube_terms", m.tube_terms, 500);
        nh.param("rtmpc/tube_alpha", m.tube_alpha, 1.0e-4);
        nh.param("rtmpc/tube_contraction", m.tube_contraction, 0.999);
        nh.param("rtmpc/slack_linear_weight", m.slack_linear_weight, 1.0e3);
        nh.param("rtmpc/slack_quadratic_weight", m.slack_quadratic_weight, 1.0e4);
        nh.param(prefix + "Q_p", m.Q_diag(0), vertical ? 15.0 : 10.0);
        nh.param(prefix + "Q_v", m.Q_diag(1), vertical ? 2.0 : 1.0);
        nh.param(prefix + "R", m.R, vertical ? 0.5 : 0.1);
        nh.param(prefix + "Q_anc_p", m.Q_anc_diag(0), vertical ? 30.0 : 20.0);
        nh.param(prefix + "Q_anc_v", m.Q_anc_diag(1), vertical ? 5.0 : 2.0);
        nh.param(prefix + "R_anc", m.R_anc, 0.01);
        nh.param(prefix + "limit_p", m.state_limits(0), vertical ? 1.5 : 2.0);
        nh.param(prefix + "limit_v", m.state_limits(1), vertical ? 1.0 : 2.0);
        nh.param(prefix + "limit_u_min", physical_input_limits[i](0), vertical ? -2.0 : -3.0);
        nh.param(prefix + "limit_u_max", physical_input_limits[i](1), vertical ? 5.0 : 3.0);
        nh.param(prefix + "correction_min", m.correction_domain(0), physical_input_limits[i](0));
        nh.param(prefix + "correction_max", m.correction_domain(1), physical_input_limits[i](1));

        const std::string bp = "bounds/" + axes[i] + "/";
        nh.param(bp + "rkhs_norm", b.rkhs_norm, d.rkhs_norm);
        nh.param(bp + "disturbance", b.disturbance, d.disturbance);
        nh.param(bp + "measurement", b.measurement, d.measurement);
        nh.param(bp + "state_error", b.state_error, d.state_error);
        nh.param(bp + "synchronization", b.synchronization, d.synchronization);
        nh.param(bp + "command_modification", b.command_modification, d.command_modification);
        nh.param(bp + "hold_error", b.hold_error, d.hold_error);
        nh.param(bp + "lipschitz_f", b.lipschitz_f, d.lipschitz_f);
        nh.param(bp + "lipschitz_g", b.lipschitz_g, d.lipschitz_g);
        nh.param(bp + "reference_acceleration", b.reference_acceleration, d.reference_acceleration);
        nh.param(bp + "min_envelope", b.min_envelope, d.min_envelope);
        nh.param(bp + "max_envelope", b.max_envelope, d.max_envelope);
        nh.param<std::string>("prior/model_" + axes[i], prior_paths[i], "");

        // Adaptive homothetic tube sized by Eq. (46) with the RKHS bound B.
        g.rkhs_bound = b.rkhs_norm;
        m.max_envelope = b.max_envelope;
        m.min_envelope = b.min_envelope;
        m.max_estimation_error.setConstant(b.state_error);
    }

    max_angle /= (180.0 / M_PI);

    if ( takeoff_land.enable_auto_arm && !takeoff_land.enable )
    {
        takeoff_land.enable_auto_arm = false;
        ROS_ERROR("\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
    }
    if ( takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable) )
    {
        takeoff_land.no_RC = false;
        ROS_ERROR("\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
    }
};
