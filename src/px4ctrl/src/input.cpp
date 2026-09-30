#include "input.h"
#include <algorithm>
#include <cmath>

namespace {
bool advancing(const ros::Time& stamp, const ros::Time& previous, bool received) {
    return stamp.toSec() > 0.0 && std::isfinite(stamp.toSec()) &&
           (!received || stamp > previous);
}
bool validQuaternion(const Eigen::Quaterniond& quaternion) {
    return quaternion.coeffs().allFinite() && std::isfinite(quaternion.norm()) &&
           quaternion.norm() > 1e-6;
}
}
bool input_is_fresh(const ros::Time& now, const ros::Time& source,
                    const ros::Time& receipt, double timeout) {
    // MAVROS time synchronization can place source stamps slightly ahead of
    // the local clock; a small offset is tolerated.
    constexpr double clock_skew_tolerance = 0.02;
    if (!std::isfinite(timeout) || timeout <= 0.0 || source.toSec() <= 0.0 || receipt.toSec() <= 0.0)
        return false;
    const double source_age = (now - source).toSec();
    const double receipt_age = (now - receipt).toSec();
    return std::isfinite(source_age) && std::isfinite(receipt_age) &&
           source_age >= -clock_skew_tolerance && receipt_age >= 0.0 &&
           source_age < timeout && receipt_age < timeout;
}
void RC_Data_t::feed(mavros_msgs::RCInConstPtr incoming) {
    if (incoming->channels.size() < 8 || !advancing(incoming->header.stamp, msg.header.stamp, received)) return;
    for (int i : {0, 1, 2, 3, 6, 7}) {
        if (incoming->channels[i] < 800 || incoming->channels[i] > 2200) return;
    }
    msg = *incoming;
    rcv_stamp = ros::Time::now();
    received = true;
    for (int i = 0; i < 4; ++i) {
        const double raw = std::max(-1.0, std::min(1.0, (static_cast<double>(msg.channels[i]) - 1500.0) / 500.0));
        ch[i] = std::abs(raw) <= DEAD_ZONE ? 0.0 :
                (raw - std::copysign(DEAD_ZONE, raw)) / (1.0 - DEAD_ZONE);
    }
    mode = (static_cast<double>(msg.channels[6]) - 1000.0) / 1000.0;
    gear = (static_cast<double>(msg.channels[7]) - 1000.0) / 1000.0;
    // No reboot channel mapping is specified; never infer one from an index.
    reboot_cmd = 0.0;
    toggle_reboot = false;
    if (!have_init_last_mode) { last_mode = mode; have_init_last_mode = true; }
    if (!have_init_last_gear) { last_gear = gear; have_init_last_gear = true; }
    enter_hover_mode = enter_hover_mode || (last_mode <= API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE);
    is_hover_mode = mode > API_MODE_THRESHOLD_VALUE;
    is_command_mode = is_hover_mode && gear > GEAR_SHIFT_VALUE;
    enter_command_mode = enter_command_mode || (is_hover_mode && last_gear <= GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE);
    if (!is_hover_mode) enter_hover_mode = false;
    if (!is_command_mode) enter_command_mode = false;
    last_mode = mode;
    last_gear = gear;
}
void RC_Data_t::check_validity() {
    if (!received || msg.channels.size() < 8) ROS_ERROR("RC data is unavailable or has fewer than 8 channels");
}
bool RC_Data_t::check_centered() const {
    for (double channel : ch) if (std::abs(channel) >= 1e-5) return false;
    return true;
}
void Odom_Data_t::feed(nav_msgs::OdometryConstPtr incoming) {
    if (!advancing(incoming->header.stamp, msg.header.stamp, received)) return;
    const auto& pose = incoming->pose.pose;
    const auto& twist = incoming->twist.twist;
    Eigen::Vector3d next_p(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Vector3d next_v(twist.linear.x, twist.linear.y, twist.linear.z);
    Eigen::Vector3d next_w(twist.angular.x, twist.angular.y, twist.angular.z);
    Eigen::Quaterniond next_q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    if (!next_p.allFinite() || !next_v.allFinite() || !next_w.allFinite() || !validQuaternion(next_q)) return;
    next_q.normalize();
    // MAVROS local_position/odom publishes twist in child/body coordinates.
    next_v = next_q * next_v;
    if (!next_v.allFinite()) return;
    p = next_p; v = next_v; w = next_w; q = next_q;
    msg = *incoming;
    rcv_stamp = ros::Time::now();
    received = true;
    recv_new_msg = true;
}
void Imu_Data_t::feed(sensor_msgs::ImuConstPtr incoming) {
    if (!advancing(incoming->header.stamp, msg.header.stamp, received)) return;
    Eigen::Vector3d next_w(incoming->angular_velocity.x, incoming->angular_velocity.y, incoming->angular_velocity.z);
    Eigen::Vector3d next_a(incoming->linear_acceleration.x, incoming->linear_acceleration.y, incoming->linear_acceleration.z);
    Eigen::Quaterniond next_q(incoming->orientation.w, incoming->orientation.x, incoming->orientation.y, incoming->orientation.z);
    if (!next_w.allFinite() || !next_a.allFinite() || !validQuaternion(next_q) ||
        incoming->orientation_covariance[0] < 0.0 || incoming->linear_acceleration_covariance[0] < 0.0) return;
    q = next_q.normalized(); w = next_w; a = next_a;
    msg = *incoming;
    rcv_stamp = ros::Time::now();
    received = true;
}
void State_Data_t::feed(mavros_msgs::StateConstPtr incoming) {
    if (!advancing(incoming->header.stamp, current_state.header.stamp, received)) return;
    current_state = *incoming;
    rcv_stamp = ros::Time::now();
    received = true;
}
void ExtendedState_Data_t::feed(mavros_msgs::ExtendedStateConstPtr incoming) {
    if (!advancing(incoming->header.stamp, current_extended_state.header.stamp, received)) return;
    current_extended_state = *incoming;
    rcv_stamp = ros::Time::now();
    received = true;
}
void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr incoming) {
    if (!advancing(incoming->header.stamp, msg.header.stamp, received)) return;
    Eigen::Vector3d next_p(incoming->position.x, incoming->position.y, incoming->position.z);
    Eigen::Vector3d next_v(incoming->velocity.x, incoming->velocity.y, incoming->velocity.z);
    Eigen::Vector3d next_a(incoming->acceleration.x, incoming->acceleration.y, incoming->acceleration.z);
    Eigen::Vector3d next_j(incoming->jerk.x, incoming->jerk.y, incoming->jerk.z);
    if (!next_p.allFinite() || !next_v.allFinite() || !next_a.allFinite() || !next_j.allFinite() ||
        !std::isfinite(incoming->yaw) || !std::isfinite(incoming->yaw_dot)) return;
    p = next_p; v = next_v; a = next_a; j = next_j;
    yaw = uav_utils::normalize_angle(incoming->yaw);
    yaw_rate = incoming->yaw_dot;
    msg = *incoming;
    rcv_stamp = ros::Time::now();
    received = true;
}
void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr incoming) {
    if (!advancing(incoming->header.stamp, msg.header.stamp, received)) return;
    double voltage = incoming->voltage;
    if (!incoming->cell_voltage.empty()) {
        double sum = 0.0;
        bool valid_cells = true;
        for (double cell : incoming->cell_voltage) {
            if (!std::isfinite(cell) || cell <= 0.0) { valid_cells = false; break; }
            sum += cell;
        }
        if (valid_cells) voltage = sum;
    }
    if (!std::isfinite(voltage) || voltage <= 0.0) return;
    volt = received ? 0.8 * volt + 0.2 * voltage : voltage;
    percentage = std::isfinite(incoming->percentage) ? incoming->percentage : 0.0;
    msg = *incoming;
    rcv_stamp = ros::Time::now();
    received = true;
}
void Takeoff_Land_Data_t::feed(quadrotor_msgs::TakeoffLandConstPtr incoming) {
    if (incoming->takeoff_land_cmd != quadrotor_msgs::TakeoffLand::TAKEOFF &&
        incoming->takeoff_land_cmd != quadrotor_msgs::TakeoffLand::LAND) return;
    msg = *incoming;
    rcv_stamp = ros::Time::now();
    triggered = true;
    takeoff_land_cmd = incoming->takeoff_land_cmd;
}
