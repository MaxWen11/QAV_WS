#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>
#include <deque>
#include <Eigen/Dense>

#include "input.h"
#include <qpOASES.hpp> // Required for RTMPC

// Include LibTorch
#include <torch/script.h> 
#include <torch/torch.h>

struct Desired_State_t
{
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Quaterniond q;
    double yaw;
    double yaw_rate;

    Desired_State_t(){};

    Desired_State_t(Odom_Data_t &odom)
        : p(odom.p),
          v(Eigen::Vector3d::Zero()),
          a(Eigen::Vector3d::Zero()),
          j(Eigen::Vector3d::Zero()),
          q(odom.q),
          yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
          yaw_rate(0){};
};

struct Controller_Output_t
{
    Eigen::Quaterniond q;
    Eigen::Vector3d bodyrates; 
    double thrust;
};

// ==========================================================
// 1. Online Gaussian Process (Online GP) 
// ==========================================================
class OnlineGP {
public:
    OnlineGP(double l, double sigma_f, double beta, int N_max);
    void add_data(const Eigen::Vector2d& x, double u, double y);
    void predict(const Eigen::Vector2d& x_query, double u_query, 
                 double f_prior, double g_prior,
                 double& f_post, double& g_post, double& sigma_f, double& sigma_g);

private:
    double lengthscale;
    double var_f;
    double beta_factor;
    int max_window_size;
    double noise_var; 

    std::deque<Eigen::Vector2d> X_buffer;
    std::deque<double> U_buffer;
    std::deque<double> Y_buffer;

    double kernel_se(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2);
};

// ==========================================================
// 2. Robust Tube-based MPC (RTMPC)
// ==========================================================
class RTMPC {
public:
    RTMPC(double dt, int H, const Eigen::Vector2d& Q_diag, double R, 
          const Eigen::Vector2d& Q_anc_diag, double R_anc,
          const Eigen::Vector2d& state_limit_p_v, const Eigen::Vector2d& input_limit);

    double solve(const Eigen::Vector2d& current_state, const Eigen::Vector2d& ref_state, double w_kappa);
    Eigen::RowVector2d get_K_anc() const { return K_anc; }

private:
    double dt_;
    int H_;
    Eigen::Matrix2d A_d;
    Eigen::Vector2d B_d;
    
    Eigen::Matrix2d Q_;
    double R_;
    Eigen::RowVector2d K_anc; 

    Eigen::Vector2d state_limit_;
    Eigen::Vector2d input_limit_;
    qpOASES::QProblem qp_solver_;

    void compute_dlqr(const Eigen::Matrix2d& Q_anc, double R_anc);
};

// ==========================================================
// 3. Controller Main Class
// ==========================================================
class Controller {
public:
    Parameter_t &param;
    quadrotor_msgs::Px4ctrlDebug debug;

    Controller(Parameter_t &param_);

    quadrotor_msgs::Px4ctrlDebug update(
        const Desired_State_t &des,
        const Odom_Data_t &odom,
        const Imu_Data_t &imu,
        Controller_Output_t &u,
        double voltage);

    void resetThrustMapping();
    bool estimateThrustModel(const Eigen::Vector3d &est_a, double voltage, const Eigen::Vector3d &est_v, const Parameter_t &param);

private:
    Eigen::Vector3d Gravity;

    OnlineGP* gp_x;
    OnlineGP* gp_y;
    OnlineGP* gp_z;

    RTMPC* mpc_x;
    RTMPC* mpc_y;
    RTMPC* mpc_z;

    double last_u_x, last_u_y, last_u_z;

    // LibTorch Modules
    torch::jit::script::Module prior_model_x;
    torch::jit::script::Module prior_model_y;
    torch::jit::script::Module prior_model_z;
    bool models_loaded = false;

    void computeFlatInput(const Eigen::Vector3d &thr_acc,
                          const double &yaw,
                          const Eigen::Quaterniond &att_est,
                          Eigen::Quaterniond &att,
                          double &thrust) const;

    // Unified inference function for LibTorch
    void get_prior(const Eigen::Vector2d& x, char axis, double& f0, double& g0);
};

#endif