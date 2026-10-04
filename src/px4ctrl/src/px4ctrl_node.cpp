#include <ros/ros.h>
#include <cmath>
#include <thread>
#include "PX4CtrlFSM.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle vehicle;
    ros::NodeHandle private_node("~");
    Parameter_t parameters;
    parameters.config_from_ros_handle(private_node);
    Controller controller(parameters);
    PX4CtrlFSM fsm(parameters, controller);
    private_node.param("fcu_state_timeout_s", fsm.fcu_state_timeout, 2.0);
    if (!std::isfinite(fsm.fcu_state_timeout) || fsm.fcu_state_timeout <= 0.0 ||
        !std::isfinite(parameters.ctrl_freq_max) || parameters.ctrl_freq_max <= 0.0) {
        ROS_FATAL("[px4ctrl] Invalid FCU timeout or control rate");
        return 2;
    }

    auto state_sub = vehicle.subscribe<mavros_msgs::State>("mavros/state", 10,
        [&fsm](mavros_msgs::StateConstPtr message) { fsm.state_data.feed(message); });
    auto extended_sub = vehicle.subscribe<mavros_msgs::ExtendedState>("mavros/extended_state", 10,
        [&fsm](mavros_msgs::ExtendedStateConstPtr message) { fsm.extended_state_data.feed(message); });
    auto odom_sub = private_node.subscribe<nav_msgs::Odometry>("odom", 10,
        [&fsm](nav_msgs::OdometryConstPtr message) { fsm.odom_data.feed(message); },
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    auto command_sub = private_node.subscribe<quadrotor_msgs::PositionCommand>("cmd", 10,
        [&fsm](quadrotor_msgs::PositionCommandConstPtr message) { fsm.cmd_data.feed(message); },
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    auto imu_sub = vehicle.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10,
        [&fsm](sensor_msgs::ImuConstPtr message) { fsm.imu_data.feed(message); },
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    ros::Subscriber rc_sub;
    if (!parameters.takeoff_land.no_RC) {
        rc_sub = vehicle.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 10,
            [&fsm](mavros_msgs::RCInConstPtr message) { fsm.rc_data.feed(message); });
    }
    auto battery_sub = vehicle.subscribe<sensor_msgs::BatteryState>("mavros/battery", 10,
        [&fsm](sensor_msgs::BatteryStateConstPtr message) { fsm.bat_data.feed(message); },
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    auto takeoff_sub = private_node.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land", 1,
        [&fsm](quadrotor_msgs::TakeoffLandConstPtr message) { fsm.takeoff_land_data.feed(message); });

    fsm.ctrl_FCU_pub = vehicle.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);
    fsm.traj_start_trigger_pub = vehicle.advertise<geometry_msgs::PoseStamped>("traj_start_trigger", 1);
    fsm.debug_pub = vehicle.advertise<quadrotor_msgs::Px4ctrlDebug>("debugPx4ctrl", 10);
    fsm.ready_pub = private_node.advertise<std_msgs::Bool>("controller_ready", 1);
    fsm.set_FCU_mode_srv = vehicle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    fsm.arming_client_srv = vehicle.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    auto reset_service = private_node.advertiseService("reset_online", &PX4CtrlFSM::reset_online, &fsm);

    // ROS callbacks execute on this thread, so reset and update cannot race.
    if (controller.ready())
        ROS_INFO("[px4ctrl] UADL controller ready");
    else
        ROS_ERROR("[px4ctrl] UADL controller configuration failed: %s", controller.lastStatus().c_str());
    ROS_INFO("[px4ctrl] Waiting for odometry, IMU, FCU state and RC/battery inputs");
    ros::Rate startup_rate(50.0);
    while (ros::ok() && !fsm.sensors_ready(ros::Time::now())) {
        ros::spinOnce();
        std_msgs::Bool ready;
        ready.data = false;
        fsm.ready_pub.publish(ready);
        startup_rate.sleep();
    }
    fsm.rc_data.enter_hover_mode = false;
    fsm.rc_data.enter_command_mode = false;
    fsm.takeoff_land_data.triggered = false;
    using Clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / parameters.ctrl_freq_max));
    auto release = Clock::now();
    while (ros::ok()) {
        std::this_thread::sleep_until(release);
        ros::spinOnce();
        fsm.process(release);
        release += period;
        // Preserve scheduled releases after an overrun; never reset the
        // acceptance cutoff to the delayed start of an individual solve.
        while (release + period <= Clock::now()) release += period;
    }
    return 0;
}
