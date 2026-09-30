#ifndef __INPUT_H
#define __INPUT_H
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <uav_utils/utils.h>
#include "PX4CtrlParam.h"

// Both source and receipt must be recent; republishing old data is not freshness.
bool input_is_fresh(const ros::Time& now, const ros::Time& source,
                    const ros::Time& receipt, double timeout);

class RC_Data_t {
public:
    double mode = 0.0, gear = 0.0, reboot_cmd = 0.0;
    double last_mode = -1.0, last_gear = -1.0, last_reboot_cmd = 0.0;
    bool have_init_last_mode = false, have_init_last_gear = false;
    double ch[4] = {0.0, 0.0, 0.0, 0.0};
    mavros_msgs::RCIn msg;
    ros::Time rcv_stamp;
    bool received = false;
    // FSM requires fresh RC except under explicitly configured no_RC operation.
    bool is_command_mode = true, enter_command_mode = false;
    bool is_hover_mode = true, enter_hover_mode = false, toggle_reboot = false;
    static constexpr double GEAR_SHIFT_VALUE = 0.75;
    static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
    static constexpr double DEAD_ZONE = 0.25;
    RC_Data_t() = default;
    void check_validity();
    bool check_centered() const;
    void feed(mavros_msgs::RCInConstPtr message);
};
class Odom_Data_t {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p = Eigen::Vector3d::Zero(), v = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;
    bool received = false, recv_new_msg = false;
    Odom_Data_t() = default;
    void feed(nav_msgs::OdometryConstPtr message);
};
class Imu_Data_t {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d w = Eigen::Vector3d::Zero(), a = Eigen::Vector3d::Zero();
    sensor_msgs::Imu msg;
    ros::Time rcv_stamp;
    bool received = false;
    Imu_Data_t() = default;
    void feed(sensor_msgs::ImuConstPtr message);
};
class State_Data_t {
public:
    mavros_msgs::State current_state, state_before_offboard;
    ros::Time rcv_stamp;
    bool received = false;
    State_Data_t() = default;
    void feed(mavros_msgs::StateConstPtr message);
};
class ExtendedState_Data_t {
public:
    mavros_msgs::ExtendedState current_extended_state;
    ros::Time rcv_stamp;
    bool received = false;
    ExtendedState_Data_t() = default;
    void feed(mavros_msgs::ExtendedStateConstPtr message);
};
class Command_Data_t {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p = Eigen::Vector3d::Zero(), v = Eigen::Vector3d::Zero();
    Eigen::Vector3d a = Eigen::Vector3d::Zero(), j = Eigen::Vector3d::Zero();
    double yaw = 0.0, yaw_rate = 0.0;
    quadrotor_msgs::PositionCommand msg;
    ros::Time rcv_stamp;
    bool received = false;
    Command_Data_t() = default;
    void feed(quadrotor_msgs::PositionCommandConstPtr message);
};
class Battery_Data_t {
public:
    double volt = 0.0, percentage = 0.0;
    sensor_msgs::BatteryState msg;
    ros::Time rcv_stamp;
    bool received = false;
    Battery_Data_t() = default;
    void feed(sensor_msgs::BatteryStateConstPtr message);
};
class Takeoff_Land_Data_t {
public:
    bool triggered = false;
    uint8_t takeoff_land_cmd = 0;
    quadrotor_msgs::TakeoffLand msg;
    ros::Time rcv_stamp;
    Takeoff_Land_Data_t() = default;
    void feed(quadrotor_msgs::TakeoffLandConstPtr message);
};
#endif
