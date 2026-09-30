#include "PX4CtrlFSM.h"
#include <algorithm>
#include <cmath>
#include <chrono>

PX4CtrlFSM::PX4CtrlFSM(Parameter_t& parameters, Controller& control)
    : param(parameters), controller(control) {}

bool PX4CtrlFSM::rc_is_received(const ros::Time& now) const {
    return rc_data.received && input_is_fresh(now, rc_data.msg.header.stamp, rc_data.rcv_stamp, param.msg_timeout.rc);
}
bool PX4CtrlFSM::cmd_is_received(const ros::Time& now) const {
    return cmd_data.received && input_is_fresh(now, cmd_data.msg.header.stamp, cmd_data.rcv_stamp, param.msg_timeout.cmd);
}
bool PX4CtrlFSM::odom_is_received(const ros::Time& now) const {
    return odom_data.received && input_is_fresh(now, odom_data.msg.header.stamp, odom_data.rcv_stamp, param.msg_timeout.odom);
}
bool PX4CtrlFSM::imu_is_received(const ros::Time& now) const {
    return imu_data.received && input_is_fresh(now, imu_data.msg.header.stamp, imu_data.rcv_stamp, param.msg_timeout.imu);
}
bool PX4CtrlFSM::bat_is_received(const ros::Time& now) const {
    return bat_data.received && input_is_fresh(now, bat_data.msg.header.stamp, bat_data.rcv_stamp, param.msg_timeout.bat);
}
bool PX4CtrlFSM::state_is_received(const ros::Time& now) const {
    return state_data.received && state_data.current_state.connected &&
        input_is_fresh(now, state_data.current_state.header.stamp, state_data.rcv_stamp, fcu_state_timeout);
}
bool PX4CtrlFSM::sensors_ready(const ros::Time& now) const {
    return state_is_received(now) && odom_is_received(now) && imu_is_received(now) &&
        (param.takeoff_land.no_RC || rc_is_received(now)) &&
        (!param.thr_map.accurate_thrust_model || bat_is_received(now));
}
bool PX4CtrlFSM::recv_new_odom() {
    const bool result = odom_data.recv_new_msg;
    odom_data.recv_new_msg = false;
    return result;
}
void PX4CtrlFSM::publish_ready(const ros::Time& now) {
    std_msgs::Bool message;
    message.data = state == AUTO_HOVER && last_output_valid_ && controller.ready() &&
        sensors_ready(now) && state_data.current_state.armed && state_data.current_state.mode == "OFFBOARD";
    ready_pub.publish(message);
}
bool PX4CtrlFSM::reset_online(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    const ros::Time now = ros::Time::now();
    if (!controller.ready() || (state != AUTO_HOVER && state != CMD_CTRL) ||
        !sensors_ready(now) || !state_data.current_state.armed ||
        state_data.current_state.mode != "OFFBOARD" || !last_output_valid_) {
        response.success = false;
        response.message = "Reset requires valid active AUTO_HOVER or CMD_CTRL";
        return true;
    }
    double epoch = 0.0;
    ros::NodeHandle("~").param("reset_epoch", epoch, 0.0);
    if (!std::isfinite(epoch) || epoch < 0.0) {
        response.success = false;
        response.message = "reset_epoch must be zero or a finite future ROS timestamp";
    } else if (epoch > 0.0) {
        response.success = controller.scheduleReset(epoch);
        response.message = response.success ? "Online reset scheduled at epoch " + std::to_string(epoch) :
                                              "Scheduled reset rejected: epoch must be in the future";
    } else {
        controller.resetOnline();
        last_output_valid_ = false;
        response.success = true;
        response.message = "GP windows, plans and executed-command history cleared; next cycle rechecks feasibility";
    }
    return true;
}
void PX4CtrlFSM::return_to_manual(const std::string& reason) {
    // If mode restoration fails, silence setpoints until PX4 leaves OFFBOARD.
    // Neutral prestream must not indefinitely mask the flight controller's failsafe.
    suppress_prestream_ = state_data.current_state.mode == "OFFBOARD";
    if (suppress_prestream_) toggle_offboard_mode(false);
    state = MANUAL_CTRL;
    awaiting_offboard_ = false;
    arm_requested_ = false;
    takeoff_started_ = false;
    last_output_valid_ = false;
    takeoff_land.delay_trigger.first = false;
    landing_condition_last_ = false;
    controller.resetOnline();
    ROS_WARN("[px4ctrl] Returning to MANUAL_CTRL: %s", reason.c_str());
}

void PX4CtrlFSM::process() {
    controller.beginCycle(std::chrono::steady_clock::now());
    const ros::Time now = ros::Time::now();
    const bool enter_hover = rc_data.enter_hover_mode;
    const bool enter_command = rc_data.enter_command_mode;
    const bool takeoff_event = takeoff_land_data.triggered &&
        takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF;
    const bool land_event = takeoff_land_data.triggered &&
        takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND;
    const double event_age = (now - takeoff_land_data.rcv_stamp).toSec();
    const bool fresh_event = event_age >= 0.0 && event_age < param.msg_timeout.cmd;
    rc_data.enter_hover_mode = false;
    rc_data.enter_command_mode = false;
    rc_data.toggle_reboot = false;
    takeoff_land_data.triggered = false;

    if (state != MANUAL_CTRL) {
        if (!controller.ready() || !sensors_ready(now) ||
            (!param.takeoff_land.no_RC && !rc_data.is_hover_mode)) {
            return_to_manual("controller, odometry, IMU, RC, battery or FCU gate failed");
            publish_ready(now);
            return;
        }
        if (awaiting_offboard_) {
            if (state_data.current_state.mode == "OFFBOARD") awaiting_offboard_ = false;
            else if ((now - transition_time_).toSec() > 1.0) {
                return_to_manual("OFFBOARD acknowledgement timed out");
                publish_ready(now);
                return;
            }
        } else if (state_data.current_state.mode != "OFFBOARD") {
            return_to_manual("PX4 left OFFBOARD; preserving pilot/FCU-selected mode");
            publish_ready(now);
            return;
        }
        if (!state_data.current_state.armed && state != AUTO_TAKEOFF) {
            return_to_manual("PX4 disarmed during automatic control");
            publish_ready(now);
            return;
        }
    }

    Desired_State_t desired(odom_data);
    bool idle_motors = false;
    switch (state) {
    case MANUAL_CTRL: {
        if (state_data.current_state.mode != "OFFBOARD") suppress_prestream_ = false;
        if (takeoff_event && fresh_event && param.takeoff_land.enable) {
            const bool ground = extended_state_data.received &&
                input_is_fresh(now, extended_state_data.current_extended_state.header.stamp,
                               extended_state_data.rcv_stamp, fcu_state_timeout) &&
                extended_state_data.current_extended_state.landed_state ==
                    mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
            if (!controller.ready() || !sensors_ready(now) || !ground || cmd_is_received(now) ||
                odom_data.v.norm() > 0.1 ||
                (!param.takeoff_land.no_RC && (!rc_data.is_hover_mode ||
                    !rc_data.is_command_mode || !rc_data.check_centered())) ||
                (!param.takeoff_land.enable_auto_arm && !state_data.current_state.armed)) {
                ROS_ERROR("[px4ctrl] Reject takeoff: valid controller/sensors, ground state and centered RC are required");
                break;
            }
            controller.resetOnline();
            set_start_pose_for_takeoff_land();
            if (!toggle_offboard_mode(true)) break;
            state = AUTO_TAKEOFF;
            transition_time_ = now;
            awaiting_offboard_ = state_data.current_state.mode != "OFFBOARD";
            takeoff_started_ = false;
            arm_requested_ = false;
            suppress_prestream_ = false;
            idle_motors = true;
            ROS_INFO("[px4ctrl] MANUAL_CTRL -> AUTO_TAKEOFF");
        } else if (enter_hover) {
            if (!controller.ready() || !sensors_ready(now) || !state_data.current_state.armed ||
                cmd_is_received(now) || odom_data.v.norm() > 3.0) {
                ROS_ERROR("[px4ctrl] Reject AUTO_HOVER: require ready controller, fresh sensors, armed vehicle and no commands");
                break;
            }
            controller.resetOnline();
            set_hov_with_odom();
            if (!toggle_offboard_mode(true)) break;
            state = AUTO_HOVER;
            desired = get_hover_des();
            transition_time_ = now;
            awaiting_offboard_ = state_data.current_state.mode != "OFFBOARD";
            suppress_prestream_ = false;
            takeoff_land.landed = false;
            ROS_INFO("[px4ctrl] MANUAL_CTRL -> AUTO_HOVER");
        }
        break;
    }
    case AUTO_HOVER:
        if (land_event && fresh_event && param.takeoff_land.enable) {
            state = AUTO_LAND;
            set_start_pose_for_takeoff_land();
            desired = get_takeoff_land_des(-param.takeoff_land.speed);
        } else if (rc_data.is_command_mode && cmd_is_received(now) && !awaiting_offboard_) {
            state = CMD_CTRL;
            desired = get_cmd_des();
        } else {
            set_hov_with_rc();
            desired = get_hover_des();
            if (enter_command || (takeoff_land.delay_trigger.first && now > takeoff_land.delay_trigger.second)) {
                takeoff_land.delay_trigger.first = false;
                publish_trigger(odom_data.msg);
            }
        }
        break;
    case CMD_CTRL:
        if (!rc_data.is_command_mode || !cmd_is_received(now)) {
            state = AUTO_HOVER;
            set_hov_with_odom();
            desired = get_hover_des();
        } else {
            desired = get_cmd_des();
        }
        if (land_event) ROS_WARN("[px4ctrl] Stop position commands and enter AUTO_HOVER before requesting LAND");
        break;
    case AUTO_TAKEOFF:
        if (awaiting_offboard_) break;
        if (!state_data.current_state.armed) {
            if (!param.takeoff_land.enable_auto_arm || takeoff_started_) {
                return_to_manual("Takeoff arming was disabled or PX4 unexpectedly disarmed");
                publish_ready(now);
                return;
            }
            if (!arm_requested_) {
                if (!toggle_arm_disarm(true)) {
                    return_to_manual("PX4 rejected arming");
                    publish_ready(now);
                    return;
                }
                arm_requested_ = true;
                arm_request_time_ = now;
            } else if ((now - arm_request_time_).toSec() > 1.0) {
                return_to_manual("Arming acknowledgement timed out");
                publish_ready(now);
                return;
            }
            break;
        }
        if (!takeoff_started_) {
            takeoff_started_ = true;
            takeoff_land.toggle_takeoff_land_time = now;
            takeoff_land.landed = false;
            controller.resetOnline();
        }
        if ((now - takeoff_land.toggle_takeoff_land_time).toSec() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) {
            // Ground spinup is not a free-flight acceleration observation.
            idle_motors = true;
        } else if (odom_data.p.z() >= takeoff_land.start_pose.z() + param.takeoff_land.height) {
            state = AUTO_HOVER;
            set_hov_with_odom();
            desired = get_hover_des();
            takeoff_land.delay_trigger = {true, now + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME)};
        } else {
            desired = get_takeoff_land_des(param.takeoff_land.speed);
        }
        break;
    case AUTO_LAND:
        if (!rc_data.is_command_mode) {
            state = AUTO_HOVER;
            set_hov_with_odom();
            desired = get_hover_des();
        } else if (!takeoff_land.landed) {
            desired = get_takeoff_land_des(-param.takeoff_land.speed);
        } else {
            idle_motors = true;
            const bool on_ground = extended_state_data.received &&
                input_is_fresh(now, extended_state_data.current_extended_state.header.stamp,
                               extended_state_data.rcv_stamp, fcu_state_timeout) &&
                extended_state_data.current_extended_state.landed_state ==
                    mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
            if (on_ground && (now - last_disarm_attempt_).toSec() > 1.0) {
                last_disarm_attempt_ = now;
                if (toggle_arm_disarm(false)) {
                    return_to_manual("Landing completed");
                    publish_ready(now);
                    return;
                }
            }
        }
        break;
    }

    // Automatic commands are computed as soon as an automatic state is entered
    // on an armed vehicle, so the first setpoint PX4 executes after switching
    // to OFFBOARD is already the controller command. Online learning starts
    // once OFFBOARD is confirmed and the vehicle is in free flight.
    const bool offboard_confirmed = !awaiting_offboard_ && state_data.current_state.mode == "OFFBOARD";
    const bool active = state != MANUAL_CTRL && state_data.current_state.armed &&
        (state != AUTO_TAKEOFF || takeoff_started_);
    const bool learning = offboard_confirmed && (state == AUTO_HOVER || state == CMD_CTRL);
    if (!sensors_ready(now) || (state == MANUAL_CTRL &&
        (suppress_prestream_ || state_data.current_state.mode == "OFFBOARD"))) {
        last_output_valid_ = false;
        publish_ready(now);
        return;
    }
    Controller_Output_t output;
    if (idle_motors && active) {
        // Let the controller clear pending/adaptive history before overriding
        // its attitude-hold setpoint with the ground motor-idle command.
        controller.update(desired, odom_data, imu_data, output, bat_data.volt, false);
        motors_idling(output);
    } else {
        debug_msg = controller.update(desired, odom_data, imu_data, output, bat_data.volt, active, learning);
    }
    output.valid = output.valid && output.q.coeffs().allFinite() &&
        std::isfinite(output.thrust) && output.thrust >= 0.0 && output.thrust <= 1.0;
    last_output_valid_ = output.valid;
    if (output.valid) {
        if (publish_attitude_ctrl(output)) {
            land_detector(desired, now);
        } else {
            last_output_valid_ = false;
            if (state != MANUAL_CTRL) return_to_manual("Publication deadline/gate rejected prepared command");
        }
    } else if (state != MANUAL_CTRL) {
        return_to_manual("No valid normal/backup command: " + controller.lastStatus());
    }
    publish_ready(now);
}

Desired_State_t PX4CtrlFSM::get_hover_des() const {
    Desired_State_t desired;
    desired.p = filter_p; desired.v = filter_v; desired.a = filter_a;
    desired.yaw = hover_pose(3);
    desired.yaw_rate = rc_data.ch[3] * param.max_manual_vel * (param.rc_reverse.yaw ? 1.0 : -1.0);
    return desired;
}
Desired_State_t PX4CtrlFSM::get_cmd_des() const {
    Desired_State_t desired;
    desired.p = cmd_data.p; desired.v = cmd_data.v; desired.a = cmd_data.a; desired.j = cmd_data.j;
    desired.yaw = cmd_data.yaw; desired.yaw_rate = cmd_data.yaw_rate;
    return desired;
}
void PX4CtrlFSM::set_hov_with_odom() {
    hover_pose.head<3>() = odom_data.p;
    hover_pose(3) = uav_utils::get_yaw_from_quaternion(odom_data.q);
    filter_p = odom_data.p;
    filter_v.setZero(); filter_a.setZero();
    last_set_hover_pose_time = ros::Time::now();
}
void PX4CtrlFSM::set_hov_with_rc() {
    const ros::Time now = ros::Time::now();
    const double dt = std::max(0.0, std::min(0.05, (now - last_set_hover_pose_time).toSec()));
    last_set_hover_pose_time = now;
    Eigen::Vector3d target;
    target << rc_data.ch[1] * param.max_manual_vel * (param.rc_reverse.pitch ? 1.0 : -1.0),
              rc_data.ch[0] * param.max_manual_vel * (param.rc_reverse.roll ? 1.0 : -1.0),
              rc_data.ch[2] * param.max_manual_vel * (param.rc_reverse.throttle ? 1.0 : -1.0);
    filter_a = (target - filter_v) * (2.0 * damp_ratio / time_const);
    if (filter_a.norm() > 5.0) filter_a *= 5.0 / filter_a.norm();
    filter_v += filter_a * dt;
    filter_p += filter_v * dt;
    hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * (param.rc_reverse.yaw ? 1.0 : -1.0) * dt;
}
void PX4CtrlFSM::set_start_pose_for_takeoff_land() {
    takeoff_land.start_pose.head<3>() = odom_data.p;
    takeoff_land.start_pose(3) = uav_utils::get_yaw_from_quaternion(odom_data.q);
    takeoff_land.toggle_takeoff_land_time = ros::Time::now();
    landing_condition_last_ = false;
}
Desired_State_t PX4CtrlFSM::get_takeoff_land_des(double speed) const {
    const double elapsed = (ros::Time::now() - takeoff_land.toggle_takeoff_land_time).toSec() -
        (speed > 0.0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0.0);
    Desired_State_t desired;
    desired.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0.0, 0.0, speed * std::max(0.0, elapsed));
    desired.v.z() = speed;
    desired.yaw = takeoff_land.start_pose(3);
    return desired;
}
void PX4CtrlFSM::land_detector(const Desired_State_t& desired, const ros::Time& now) {
    if (state == MANUAL_CTRL && !state_data.current_state.armed) {
        takeoff_land.landed = true;
        landing_condition_last_ = false;
        return;
    }
    if (state != AUTO_LAND || takeoff_land.landed) return;
    const bool condition = desired.p.z() - odom_data.p.z() < -0.5 && odom_data.v.norm() < 0.1;
    if (condition && !landing_condition_last_) landing_condition_time_ = now;
    if (condition && landing_condition_last_ && (now - landing_condition_time_).toSec() > 3.0)
        takeoff_land.landed = true;
    landing_condition_last_ = condition;
}
void PX4CtrlFSM::motors_idling(Controller_Output_t& output) const {
    output.q = imu_data.q;
    output.bodyrates.setZero();
    output.thrust = 0.04;
    output.valid = output.q.coeffs().allFinite();
}
bool PX4CtrlFSM::publish_attitude_ctrl(const Controller_Output_t& output) {
    if (!output.valid || !output.q.coeffs().allFinite() || !std::isfinite(output.thrust) ||
        output.thrust < 0.0 || output.thrust > 1.0) {
        controller.discardPending();
        return false;
    }
    mavros_msgs::AttitudeTarget message;
    message.header.frame_id = "FCU";
    message.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    message.orientation.x = output.q.x(); message.orientation.y = output.q.y();
    message.orientation.z = output.q.z(); message.orientation.w = output.q.w();
    message.thrust = output.thrust;
    if (!controller.publicationAllowed()) {
        controller.discardPending();
        return false;
    }
    const ros::Time publication_stamp = ros::Time::now();
    message.header.stamp = publication_stamp;
    ctrl_FCU_pub.publish(message);
    controller.commitPublished(output, publication_stamp);
    debug_msg = controller.debug;
    debug_msg.header.stamp = publication_stamp;
    debug_msg.des_thr = output.thrust;
    debug_msg.des_q_x = output.q.x(); debug_msg.des_q_y = output.q.y();
    debug_msg.des_q_z = output.q.z(); debug_msg.des_q_w = output.q.w();
    debug_pub.publish(debug_msg);
    return true;
}
void PX4CtrlFSM::publish_trigger(const nav_msgs::Odometry& odometry) {
    geometry_msgs::PoseStamped message;
    message.header = odometry.header;
    message.pose = odometry.pose.pose;
    traj_start_trigger_pub.publish(message);
}
bool PX4CtrlFSM::toggle_offboard_mode(bool enable) {
    mavros_msgs::SetMode service;
    if (enable) {
        state_data.state_before_offboard = state_data.current_state;
        if (state_data.state_before_offboard.mode == "OFFBOARD")
            state_data.state_before_offboard.mode = "POSCTL";
        service.request.custom_mode = "OFFBOARD";
    } else {
        service.request.custom_mode = state_data.state_before_offboard.mode;
        if (service.request.custom_mode.empty()) return false;
    }
    const bool success = set_FCU_mode_srv.call(service) && service.response.mode_sent;
    if (!success) ROS_ERROR("[px4ctrl] PX4 rejected requested mode %s", service.request.custom_mode.c_str());
    return success;
}
bool PX4CtrlFSM::toggle_arm_disarm(bool arm) {
    mavros_msgs::CommandBool service;
    service.request.value = arm;
    const bool success = arming_client_srv.call(service) && service.response.success;
    if (!success) ROS_ERROR("[px4ctrl] PX4 rejected %s", arm ? "arming" : "disarming");
    return success;
}
