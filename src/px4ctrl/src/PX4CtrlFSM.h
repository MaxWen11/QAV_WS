#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <utility>
#include "input.h"
#include "controller.h"

struct AutoTakeoffLand_t {
    bool landed = true;
    ros::Time toggle_takeoff_land_time;
    std::pair<bool, ros::Time> delay_trigger{false, ros::Time(0)};
    Eigen::Vector4d start_pose = Eigen::Vector4d::Zero();
    static constexpr double MOTORS_SPEEDUP_TIME = 3.0;
    static constexpr double DELAY_TRIGGER_TIME = 2.0;
};

class PX4CtrlFSM {
public:
    Parameter_t& param;
    RC_Data_t rc_data;
    State_Data_t state_data;
    ExtendedState_Data_t extended_state_data;
    Odom_Data_t odom_data;
    Imu_Data_t imu_data;
    Command_Data_t cmd_data;
    Battery_Data_t bat_data;
    Takeoff_Land_Data_t takeoff_land_data;
    Controller& controller;
    ros::Publisher traj_start_trigger_pub, ctrl_FCU_pub, debug_pub, ready_pub;
    ros::ServiceClient set_FCU_mode_srv, arming_client_srv;
    quadrotor_msgs::Px4ctrlDebug debug_msg;
    Eigen::Vector4d hover_pose = Eigen::Vector4d::Zero();
    Eigen::Vector3d filter_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d filter_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d filter_a = Eigen::Vector3d::Zero();
    double time_const = 0.8, damp_ratio = 1.5;
    double fcu_state_timeout = 2.0;
    ros::Time last_set_hover_pose_time;
    enum State_t { MANUAL_CTRL = 1, AUTO_HOVER, CMD_CTRL, AUTO_TAKEOFF, AUTO_LAND };
    PX4CtrlFSM(Parameter_t&, Controller&);
    void process(std::chrono::steady_clock::time_point scheduled_release);
    bool reset_online(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
    bool rc_is_received(const ros::Time&) const;
    bool cmd_is_received(const ros::Time&) const;
    bool odom_is_received(const ros::Time&) const;
    bool imu_is_received(const ros::Time&) const;
    bool bat_is_received(const ros::Time&) const;
    bool state_is_received(const ros::Time&) const;
    bool sensors_ready(const ros::Time&) const;
    bool recv_new_odom();
    State_t get_state() const { return state; }
    bool get_landed() const { return takeoff_land.landed; }
private:
    State_t state = MANUAL_CTRL;
    AutoTakeoffLand_t takeoff_land;
    bool awaiting_offboard_ = false, arm_requested_ = false, takeoff_started_ = false;
    bool last_output_valid_ = false, suppress_prestream_ = false;
    ros::Time transition_time_, arm_request_time_, last_disarm_attempt_, landing_condition_time_;
    bool landing_condition_last_ = false;
    Desired_State_t get_hover_des() const;
    Desired_State_t get_cmd_des() const;
    Desired_State_t get_takeoff_land_des(double speed) const;
    void set_hov_with_odom();
    void set_hov_with_rc();
    void set_start_pose_for_takeoff_land();
    void land_detector(const Desired_State_t&, const ros::Time&);
    void motors_idling(Controller_Output_t&) const;
    bool toggle_offboard_mode(bool);
    bool toggle_arm_disarm(bool);
    void return_to_manual(const std::string& reason);
    bool publish_attitude_ctrl(const Controller_Output_t&);
    void publish_trigger(const nav_msgs::Odometry&);
    void publish_ready(const ros::Time&);
};
#endif
