#include "PX4CtrlParam.h"

Parameter_t::Parameter_t()
{
}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{
    read_essential_param(nh, "gain/Kp0", gain.Kp0);
    read_essential_param(nh, "gain/Kp1", gain.Kp1);
    read_essential_param(nh, "gain/Kp2", gain.Kp2);
    read_essential_param(nh, "gain/Kv0", gain.Kv0);
    read_essential_param(nh, "gain/Kv1", gain.Kv1);
    read_essential_param(nh, "gain/Kv2", gain.Kv2);
    read_essential_param(nh, "gain/Kvi0", gain.Kvi0);
    read_essential_param(nh, "gain/Kvi1", gain.Kvi1);
    read_essential_param(nh, "gain/Kvi2", gain.Kvi2);
    read_essential_param(nh, "gain/KAngR", gain.KAngR);
    read_essential_param(nh, "gain/KAngP", gain.KAngP);
    read_essential_param(nh, "gain/KAngY", gain.KAngY);

    read_essential_param(nh, "rotor_drag/x", rt_drag.x);
    read_essential_param(nh, "rotor_drag/y", rt_drag.y);
    read_essential_param(nh, "rotor_drag/z", rt_drag.z);
    read_essential_param(nh, "rotor_drag/k_thrust_horz", rt_drag.k_thrust_horz);

    read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
    read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
    read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
    read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
    read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

    read_essential_param(nh, "pose_solver", pose_solver);
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

    read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
    read_essential_param(nh, "thrust_model/K1", thr_map.K1);
    read_essential_param(nh, "thrust_model/K2", thr_map.K2);
    read_essential_param(nh, "thrust_model/K3", thr_map.K3);
    read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
    read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);
    read_essential_param(nh, "thrust_model/noisy_imu", thr_map.noisy_imu);
    
    // =========================================================
    // =============== 新增算法参数读取 ========================
    // =========================================================

    // 1. 加载 Online GP 参数
    nh.param("online_gp/l", gp_l, 0.5);
    nh.param("online_gp/sigma_f", gp_sigma_f, 1.0);
    nh.param("online_gp/beta", gp_beta, 2.0);
    nh.param("online_gp/N_max", gp_N_max, 50);

    // 2. 加载 RTMPC 整体参数
    nh.param("rtmpc/dt", mpc_dt, 0.05);
    nh.param("rtmpc/H", mpc_H, 20);

    // 3. 加载 RTMPC X/Y 轴参数
    nh.param("rtmpc/xy/Q_p", mpc_xy_Q_p, 10.0);
    nh.param("rtmpc/xy/Q_v", mpc_xy_Q_v, 1.0);
    nh.param("rtmpc/xy/R", mpc_xy_R, 0.1);
    nh.param("rtmpc/xy/Q_anc_p", mpc_xy_Q_anc_p, 20.0);
    nh.param("rtmpc/xy/Q_anc_v", mpc_xy_Q_anc_v, 2.0);
    nh.param("rtmpc/xy/R_anc", mpc_xy_R_anc, 0.01);
    nh.param("rtmpc/xy/limit_p", mpc_xy_limit_p, 2.0);
    nh.param("rtmpc/xy/limit_v", mpc_xy_limit_v, 2.0);
    nh.param("rtmpc/xy/limit_u_min", mpc_xy_limit_u_min, -3.0);
    nh.param("rtmpc/xy/limit_u_max", mpc_xy_limit_u_max, 3.0);

    // 4. 加载 RTMPC Z 轴参数
    nh.param("rtmpc/z/Q_p", mpc_z_Q_p, 15.0);
    nh.param("rtmpc/z/Q_v", mpc_z_Q_v, 2.0);
    nh.param("rtmpc/z/R", mpc_z_R, 0.5);
    nh.param("rtmpc/z/Q_anc_p", mpc_z_Q_anc_p, 30.0);
    nh.param("rtmpc/z/Q_anc_v", mpc_z_Q_anc_v, 5.0);
    nh.param("rtmpc/z/R_anc", mpc_z_R_anc, 0.01);
    nh.param("rtmpc/z/limit_p", mpc_z_limit_p, 1.5);
    nh.param("rtmpc/z/limit_v", mpc_z_limit_v, 1.0);
    nh.param("rtmpc/z/limit_u_min", mpc_z_limit_u_min, -2.0);
    nh.param("rtmpc/z/limit_u_max", mpc_z_limit_u_max, 5.0);
    // =========================================================
    
    // 5. Controller Selection & PID Parameters
    nh.param("controller/use_mpc", use_mpc, true);
    nh.param("controller/ua_pid_alpha", ua_pid_alpha, 0.5);

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

    if ( thr_map.print_val )
    {
        ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
    }
};