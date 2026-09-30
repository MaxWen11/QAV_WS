#ifndef UADL_CONTROLLER_H
#define UADL_CONTROLLER_H

#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <Eigen/Geometry>
#include <array>
#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <vector>
#include "input.h"
#include "online_gp.h"
#include "tube_mpc.h"
#include "control_geometry.h"
#ifdef UADL_WITH_TORCH
#include <torch/script.h>
#endif

struct Desired_State_t {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d a = Eigen::Vector3d::Zero();
    Eigen::Vector3d j = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    double yaw = 0.0, yaw_rate = 0.0;
    Desired_State_t() = default;
    explicit Desired_State_t(const Odom_Data_t& odom)
        : p(odom.p), q(odom.q), yaw(uav_utils::get_yaw_from_quaternion(odom.q)) {}
};

struct Controller_Output_t {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d bodyrates = Eigen::Vector3d::Zero();
    double thrust = 0.0;
    bool valid = false;
    bool used_backup = false;
    bool adaptive_command = false;
    Eigen::Vector3d executed_input = Eigen::Vector3d::Zero();
};

// Uncertainty-aware dynamics learning controller (Sections IV-B and V):
// frozen offline prior + task-error GP (Eqs. 18-23), protected inverse,
// axis-wise homothetic tube MPC with a maintained feasible backup.
class Controller {
public:
    explicit Controller(Parameter_t& parameters);
    Parameter_t& param;
    quadrotor_msgs::Px4ctrlDebug debug;
    // active: compute the automatic command for this cycle.
    // learning: admit new online GP samples (free flight under OFFBOARD).
    quadrotor_msgs::Px4ctrlDebug update(const Desired_State_t& des,
        const Odom_Data_t& odom, const Imu_Data_t& imu,
        Controller_Output_t& output, double voltage, bool active = false,
        bool learning = true);
    bool ready() const { return configured_; }
    const std::string& lastStatus() const { return status_; }
    // Reset adaptation, plans and command history; retain frozen priors/kernels.
    void resetOnline();
    bool scheduleReset(double epoch);
    void beginCycle(std::chrono::steady_clock::time_point started);
    bool publicationAllowed() const;
    void commitPublished(const Controller_Output_t&, const ros::Time& stamp);
    void discardPending();

private:
    using Clock = std::chrono::steady_clock;
    struct Context {
        std::vector<uadl::OnlineGP> gp;
        std::vector<uadl::TubeMPC> mpc;
        Desired_State_t reference;
        double reference_stamp = 0.0;
        bool have_reference = false;
    };
    struct ExecutedCommand {
        double stamp;
        Eigen::Vector3d input;
    };
    struct Evaluation {
        bool valid = false;
        bool used_backup = false;
        bool envelope_saturated = false;
        bool inside_domain = true;
        uadl::MappedCommand mapped;
        std::array<uadl::GPPrediction, 3> posterior;
        std::array<uadl::MPCResult, 3> mpc;
        Eigen::Vector3d eta = Eigen::Vector3d::Zero();
        Eigen::Vector3d envelopes = Eigen::Vector3d::Zero();
        std::string status;
    };
    Context accepted_;
    std::unique_ptr<Context> pending_;
    std::deque<ExecutedCommand> history_;
    bool configured_ = false;
    bool was_active_ = false;
    double last_sample_stamp_ = 0.0;
    double reset_epoch_ = 0.0;
    double sample_epoch_ = 0.0;
    Clock::time_point cycle_started_;
    bool cycle_announced_ = false;
    bool active_cycle_ = false;
    std::string status_;
    uadl::ThrustConfig thrust_config_;
#ifdef UADL_WITH_TORCH
    std::array<torch::jit::script::Module, 3> prior_models_;
#endif
    bool configure();
    bool priors(const std::vector<uadl::State>& states,
                std::vector<Eigen::Vector3d>& f0, std::vector<Eigen::Vector3d>& g0);
    bool priors(const uadl::State&, Eigen::Vector3d& f0, Eigen::Vector3d& g0);
    bool insideDomain(const uadl::State&) const;
    double historicalError(int axis, double executed_input) const;
    double residualMargin(int axis) const;
    Eigen::Vector3d lastExecutedInput() const;
    std::vector<uadl::State> predictedStates(const uadl::State&, const Desired_State_t&) const;
    bool targetEnvelopes(const Context&, const Desired_State_t&, const uadl::State&,
                         Eigen::Vector3d& targets);
    Evaluation evaluate(Context&, const Desired_State_t&, const uadl::State&,
        const Eigen::Vector3d& f0, const Eigen::Vector3d& g0, double voltage,
        double stamp, const Clock::time_point& cutoff, bool backup_only);
    Desired_State_t continuedReference(double stamp) const;
    void populateDebug(const Desired_State_t&, const Odom_Data_t&,
        const Eigen::Vector3d& measured_acceleration, const Evaluation&,
        const Controller_Output_t&, double voltage, double elapsed_ms);
};
#endif
