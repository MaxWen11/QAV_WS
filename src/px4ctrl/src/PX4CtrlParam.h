#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <ros/ros.h>

class Parameter_t
{
public:
    struct Gain
    {
        double Kp0, Kp1, Kp2;
        double Kv0, Kv1, Kv2;
        double Kvi0, Kvi1, Kvi2;
        double Kvd0, Kvd1, Kvd2;
        double KAngR, KAngP, KAngY;
    };

    struct RotorDrag
    {
        double x, y, z;
        double k_thrust_horz;
    };

    struct MsgTimeout
    {
        double odom;
        double rc;
        double cmd;
        double imu;
        double bat;
    };

    struct ThrustMapping
    {
        bool print_val;
        double K1;
        double K2;
        double K3;
        bool accurate_thrust_model;
        double hover_percentage;
        bool noisy_imu;
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

    Gain gain;
    RotorDrag rt_drag;
    MsgTimeout msg_timeout;
    RCReverse rc_reverse;
    ThrustMapping thr_map;
    AutoTakeoffLand takeoff_land;

    int pose_solver;
    double mass;
    double gra;
    double max_angle;
    double ctrl_freq_max;
    double max_manual_vel;
    double low_voltage;

    // =========================================================
    // =============== 新增算法参数声明 ========================
    // =========================================================

    // =========== Online GP 参数 ===========
    double gp_l;
    double gp_sigma_f;
    double gp_beta;
    int gp_N_max;

    // =========== RTMPC 参数 ===========
    double mpc_dt;
    int mpc_H;

    double mpc_xy_Q_p, mpc_xy_Q_v, mpc_xy_R;
    double mpc_xy_Q_anc_p, mpc_xy_Q_anc_v, mpc_xy_R_anc;
    double mpc_xy_limit_p, mpc_xy_limit_v;
    double mpc_xy_limit_u_min, mpc_xy_limit_u_max;

    double mpc_z_Q_p, mpc_z_Q_v, mpc_z_R;
    double mpc_z_Q_anc_p, mpc_z_Q_anc_v, mpc_z_R_anc;
    double mpc_z_limit_p, mpc_z_limit_v;
    double mpc_z_limit_u_min, mpc_z_limit_u_max;

    // =========================================================
    // Uncertainty-Aware PID Parameters
    bool use_mpc;
    double ua_pid_alpha;
    // =========================================================

    // bool print_dbg;

    Parameter_t();
    void config_from_ros_handle(const ros::NodeHandle &nh);
    void config_full_thrust(double hov);

private:
    template <typename TName, typename TVal>
    void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
    {
        if (nh.getParam(name, val))
        {
            // pass
        }
        else
        {
            ROS_ERROR_STREAM("Read param: " << name << " failed.");
            ROS_BREAK();
        }
    };
};

#endif