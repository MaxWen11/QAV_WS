#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <ros/ros.h>
#include <array>
#include <string>
#include "online_gp.h"
#include "tube_mpc.h"

class Parameter_t
{
public:
    struct MsgTimeout
    {
        double odom;
        double rc;
        double cmd;
        double imu;
        double bat;
    };

    // F = K1 * V^K2 * (K3*u^2 + (1-K3)*u) with accurate_thrust_model;
    // otherwise hover_percentage maps gravity to the normalized thrust.
    struct ThrustMapping
    {
        double K1;
        double K2;
        double K3;
        bool accurate_thrust_model;
        double hover_percentage;
    };

    struct RCReverse
    {
        bool roll;
        bool pitch;
        bool yaw;
        bool throttle;
    };

    struct AutoTakeoffLand
    {
        bool enable;
        bool enable_auto_arm;
        bool no_RC;
        double height;
        double speed;
    };

    MsgTimeout msg_timeout;
    RCReverse rc_reverse;
    ThrustMapping thr_map;
    AutoTakeoffLand takeoff_land;

    double mass;
    double gra;
    double max_angle;
    double ctrl_freq_max;
    double max_manual_vel;
    double low_voltage;

    // Frozen offline priors (Eq. 15); the prior enters both the GP mean and
    // the task-error covariance (Eq. 18).
    std::array<std::string, 3> prior_paths;
    std::array<uadl::GPConfig, 3> gp;
    std::array<uadl::MPCConfig, 3> mpc;
    std::array<Eigen::Vector2d, 3> physical_input_limits;

    // Residual budget of Assumption 4, Theorem 2 and Eq. (46), per axis, SI units.
    struct ResidualBounds {
        double rkhs_norm = 0.36;              // B of Assumption 4
        double disturbance = 0.25;            // d_bar
        double measurement = 0.10;            // eps_bar, IMU label error
        double state_error = 0.03;            // e_bar_x
        double synchronization = 0.02;        // e_bar_sync
        double command_modification = 0.10;   // c_bar: gain floor, saturation, mapping
        double hold_error = 0.05;             // hold and reference discretization (enter w_k)
        double lipschitz_f = 0.8;             // L_f
        double lipschitz_g = 0.1;             // L_g
        double reference_acceleration = 1.5;  // allowed reference acceleration
        double min_envelope = 0.5;            // admissible tube scaling
        double max_envelope = 1.20;
    };
    std::array<ResidualBounds, 3> bounds;
    // Analysis domain X of Assumption 1, also the hard safety limits of
    // Remark 8: [p_x, p_y, p_z, v_x, v_y, v_z].
    uadl::State analysis_lower;
    uadl::State analysis_upper;
    // Gain floor g_on of the protected inverse, Eq. (28).
    double gain_floor = 0.5;
    double sensor_max_skew = 0.02;
    double input_delay = 0.0;
    double command_max_age = 0.03;
    // Result-acceptance cutoff from the scheduled cycle release, shared by
    // the three axis solves, and the command deadline.
    double solver_cutoff = 0.0085;
    double control_deadline = 0.01;

    Parameter_t();
    void config_from_ros_handle(const ros::NodeHandle &nh);

private:
    template <typename TName, typename TVal>
    void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
    {
        if (!nh.getParam(name, val))
        {
            ROS_ERROR_STREAM("Read param: " << name << " failed.");
            ROS_BREAK();
        }
    };
};

#endif
