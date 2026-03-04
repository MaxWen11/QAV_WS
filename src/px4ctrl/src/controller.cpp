#include "controller.h"
#include <iostream>
#include <cmath>

using namespace std;

// ==========================================
// OnlineGP Implementation
// ==========================================
OnlineGP::OnlineGP(double l, double sigma_f, double beta, int N_max) 
    : lengthscale(l), var_f(sigma_f), beta_factor(beta), max_window_size(N_max) {
    noise_var = 0.01; 
}

double OnlineGP::kernel_se(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2) {
    return var_f * exp(- (x1 - x2).squaredNorm() / (2.0 * lengthscale * lengthscale));
}

void OnlineGP::add_data(const Eigen::Vector2d& x, double u, double y) {
    X_buffer.push_back(x);
    U_buffer.push_back(u);
    Y_buffer.push_back(y);
    if (X_buffer.size() > max_window_size) {
        X_buffer.pop_front();
        U_buffer.pop_front();
        Y_buffer.pop_front();
    }
}

void OnlineGP::predict(const Eigen::Vector2d& x_query, double u_query, 
                       double f_prior, double g_prior,
                       double& f_post, double& g_post, double& sigma_f, double& sigma_g) {
    int N = X_buffer.size();
    if (N == 0) {
        f_post = f_prior; g_post = g_prior;
        sigma_f = var_f;  sigma_g = var_f;
        return;
    }

    Eigen::MatrixXd K_fg(N, N);
    Eigen::VectorXd Y_err(N);
    Eigen::VectorXd k_f(N), k_g(N);
    Eigen::MatrixXd U_mat = Eigen::MatrixXd::Zero(N, N);

    for (int i = 0; i < N; ++i) {
        U_mat(i, i) = U_buffer[i];
        k_f(i) = kernel_se(X_buffer[i], x_query);
        k_g(i) = kernel_se(X_buffer[i], x_query);
        
        double mu_i = f_prior + U_buffer[i] * g_prior; 
        Y_err(i) = Y_buffer[i] - mu_i;

        for (int j = 0; j <= i; ++j) {
            double kf_ij = kernel_se(X_buffer[i], X_buffer[j]);
            double kg_ij = kernel_se(X_buffer[i], X_buffer[j]);
            double val = kf_ij + U_buffer[i] * kg_ij * U_buffer[j];
            if (i == j) val += noise_var;
            K_fg(i, j) = val;
            K_fg(j, i) = val;
        }
    }

    Eigen::VectorXd K_inv_Y = K_fg.ldlt().solve(Y_err);
    Eigen::MatrixXd K_inv = K_fg.ldlt().solve(Eigen::MatrixXd::Identity(N, N));

    f_post = f_prior + k_f.transpose() * K_inv_Y;
    g_post = g_prior + (U_mat * k_g).transpose() * K_inv_Y;

    sigma_f = kernel_se(x_query, x_query) - k_f.transpose() * K_inv * k_f;
    sigma_g = kernel_se(x_query, x_query) - (U_mat * k_g).transpose() * K_inv * (U_mat * k_g);

    sigma_f = sqrt(max(0.0, sigma_f));
    sigma_g = sqrt(max(0.0, sigma_g));
}

// ==========================================
// RTMPC Implementation
// ==========================================
RTMPC::RTMPC(double dt, int H, const Eigen::Vector2d& Q_diag, double R, 
             const Eigen::Vector2d& Q_anc_diag, double R_anc,
             const Eigen::Vector2d& state_limit, const Eigen::Vector2d& input_limit)
    : dt_(dt), H_(H), Q_(Q_diag.asDiagonal()), R_(R), state_limit_(state_limit), input_limit_(input_limit),
      qp_solver_(H_, H_) 
{
    A_d << 1.0, dt_, 
           0.0, 1.0;
    B_d << 0.5 * dt_ * dt_, 
           dt_;

    compute_dlqr(Q_anc_diag.asDiagonal(), R_anc);
    
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    qp_solver_.setOptions(options);
}

void RTMPC::compute_dlqr(const Eigen::Matrix2d& Q_anc, double R_anc) {
    Eigen::Matrix2d P = Q_anc;
    for (int i = 0; i < 100; ++i) {
        double R_eff = R_anc + (B_d.transpose() * P * B_d)(0, 0); 
        P = A_d.transpose() * P * A_d - 
            A_d.transpose() * P * B_d * (1.0 / R_eff) * B_d.transpose() * P * A_d + Q_anc;
    }
    double R_eff = R_anc + (B_d.transpose() * P * B_d)(0, 0);
    K_anc = (1.0 / R_eff) * B_d.transpose() * P * A_d;
}

double RTMPC::solve(const Eigen::Vector2d& current_state, const Eigen::Vector2d& ref_state, double w_kappa) {
    Eigen::MatrixXd H_qp = Eigen::MatrixXd::Identity(H_, H_) * R_;
    Eigen::VectorXd g_qp = Eigen::VectorXd::Zero(H_);
    
    // Placeholder for real Condensed QP using qpOASES
    double bar_eta = 0.0; 
    return bar_eta;
}

// ==========================================
// Controller Implementation
// ==========================================
Controller::Controller(Parameter_t &param_) : param(param_) {
    Gravity = Eigen::Vector3d(0.0, 0.0, -param.gra);

    // Initialize GP
    gp_x = new OnlineGP(param.gp_l, param.gp_sigma_f, param.gp_beta, param.gp_N_max);
    gp_y = new OnlineGP(param.gp_l, param.gp_sigma_f, param.gp_beta, param.gp_N_max);
    gp_z = new OnlineGP(param.gp_l, param.gp_sigma_f, param.gp_beta, param.gp_N_max);

    // Initialize RTMPC
    mpc_x = new RTMPC(param.mpc_dt, param.mpc_H, 
                      Eigen::Vector2d(param.mpc_xy_Q_p, param.mpc_xy_Q_v), param.mpc_xy_R, 
                      Eigen::Vector2d(param.mpc_xy_Q_anc_p, param.mpc_xy_Q_anc_v), param.mpc_xy_R_anc, 
                      Eigen::Vector2d(param.mpc_xy_limit_p, param.mpc_xy_limit_v), 
                      Eigen::Vector2d(param.mpc_xy_limit_u_min, param.mpc_xy_limit_u_max));
                      
    mpc_y = new RTMPC(param.mpc_dt, param.mpc_H, 
                      Eigen::Vector2d(param.mpc_xy_Q_p, param.mpc_xy_Q_v), param.mpc_xy_R, 
                      Eigen::Vector2d(param.mpc_xy_Q_anc_p, param.mpc_xy_Q_anc_v), param.mpc_xy_R_anc, 
                      Eigen::Vector2d(param.mpc_xy_limit_p, param.mpc_xy_limit_v), 
                      Eigen::Vector2d(param.mpc_xy_limit_u_min, param.mpc_xy_limit_u_max));

    mpc_z = new RTMPC(param.mpc_dt, param.mpc_H, 
                      Eigen::Vector2d(param.mpc_z_Q_p, param.mpc_z_Q_v), param.mpc_z_R, 
                      Eigen::Vector2d(param.mpc_z_Q_anc_p, param.mpc_z_Q_anc_v), param.mpc_z_R_anc, 
                      Eigen::Vector2d(param.mpc_z_limit_p, param.mpc_z_limit_v), 
                      Eigen::Vector2d(param.mpc_z_limit_u_min, param.mpc_z_limit_u_max));

    last_u_x = 0; last_u_y = 0; last_u_z = 0;

    // Load LibTorch Models
    try {
        // NOTE: Ensure these .pt files are in the directory where the ROS node is executed
        // or provide absolute paths.
        prior_model_x = torch::jit::load("generator_prior_X.pt");
        prior_model_y = torch::jit::load("generator_prior_Y.pt");
        prior_model_z = torch::jit::load("generator_prior_Z.pt");
        models_loaded = true;
        std::cout << "[Controller] LibTorch prior models loaded successfully.\n";
    } catch (const c10::Error& e) {
        std::cerr << "[Controller] Warning: Could not load LibTorch models. Using 0/1 prior.\n";
        models_loaded = false;
    }
}

void Controller::get_prior(const Eigen::Vector2d& x, char axis, double& f0, double& g0) {
    if (!models_loaded) {
        f0 = 0.0;
        g0 = 1.0;
        return;
    }

    // Pass the state velocity x(1) to the 1D network
    torch::Tensor input_tensor = torch::tensor({{static_cast<float>(x(1))}}); 
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_tensor);

    torch::jit::script::Module* target_module;
    if (axis == 'X') target_module = &prior_model_x;
    else if (axis == 'Y') target_module = &prior_model_y;
    else target_module = &prior_model_z;

    // Forward inference
    auto output = target_module->forward(inputs).toTuple();
    f0 = output->elements()[0].toTensor().item<float>();
    g0 = output->elements()[1].toTensor().item<float>();
}

quadrotor_msgs::Px4ctrlDebug Controller::update(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu,
    Controller_Output_t &u,
    double voltage) 
{
    Eigen::Vector2d state_x(odom.p(0), odom.v(0));
    Eigen::Vector2d state_y(odom.p(1), odom.v(1));
    Eigen::Vector2d state_z(odom.p(2), odom.v(2));

    Eigen::Vector2d ref_x(des.p(0), des.v(0));
    Eigen::Vector2d ref_y(des.p(1), des.v(1));
    Eigen::Vector2d ref_z(des.p(2), des.v(2));

    Eigen::Vector3d obs_acc = imu.a - Gravity;
    gp_x->add_data(state_x, last_u_x, obs_acc(0));
    gp_y->add_data(state_y, last_u_y, obs_acc(1));
    gp_z->add_data(state_z, last_u_z, obs_acc(2));

    // Get Neural Network Priors
    double f0_x, g0_x; get_prior(state_x, 'X', f0_x, g0_x);
    double f0_y, g0_y; get_prior(state_y, 'Y', f0_y, g0_y);
    double f0_z, g0_z; get_prior(state_z, 'Z', f0_z, g0_z);

    // Get GP Posteriors
    double fx_post, gx_post, sig_fx, sig_gx;
    double fy_post, gy_post, sig_fy, sig_gy;
    double fz_post, gz_post, sig_fz, sig_gz;

    gp_x->predict(state_x, last_u_x, f0_x, g0_x, fx_post, gx_post, sig_fx, sig_gx);
    gp_y->predict(state_y, last_u_y, f0_y, g0_y, fy_post, gy_post, sig_fy, sig_gy);
    gp_z->predict(state_z, last_u_z, f0_z, g0_z, fz_post, gz_post, sig_fz, sig_gz);

    double beta = param.gp_beta;
    double w_kappa_x = beta * sig_fx + abs(last_u_x) * beta * sig_gx;
    double w_kappa_y = beta * sig_fy + abs(last_u_y) * beta * sig_gy;
    double w_kappa_z = beta * sig_fz + abs(last_u_z) * beta * sig_gz;

    double bar_eta_x = mpc_x->solve(state_x, ref_x, w_kappa_x);
    double bar_eta_y = mpc_y->solve(state_y, ref_y, w_kappa_y);
    double bar_eta_z = mpc_z->solve(state_z, ref_z, w_kappa_z);

    double anc_eta_x = -mpc_x->get_K_anc() * (state_x - ref_x);
    double anc_eta_y = -mpc_y->get_K_anc() * (state_y - ref_y);
    double anc_eta_z = -mpc_z->get_K_anc() * (state_z - ref_z);

    double eta_x = bar_eta_x + anc_eta_x;
    double eta_y = bar_eta_y + anc_eta_y;
    double eta_z = bar_eta_z + anc_eta_z;

    double u_cmd_x = (eta_x - fx_post) / max(0.1, gx_post);
    double u_cmd_y = (eta_y - fy_post) / max(0.1, gy_post);
    double u_cmd_z = (eta_z - fz_post) / max(0.1, gz_post);

    last_u_x = u_cmd_x; last_u_y = u_cmd_y; last_u_z = u_cmd_z;

    Eigen::Vector3d thr_acc(u_cmd_x, u_cmd_y, u_cmd_z + param.gra); 
    
    Eigen::Quaterniond desired_attitude;
    computeFlatInput(thr_acc, des.yaw, odom.q, desired_attitude, u.thrust);

    u.q = imu.q * odom.q.inverse() * desired_attitude; 
    u.bodyrates = Eigen::Vector3d::Zero();

    debug.des_a_x = thr_acc(0);
    debug.des_a_y = thr_acc(1);
    debug.des_a_z = thr_acc(2);
    return debug;
}

void Controller::computeFlatInput(const Eigen::Vector3d &thr_acc,
                                  const double &yaw,
                                  const Eigen::Quaterniond &att_est,
                                  Eigen::Quaterniond &att,
                                  double &thrust) const 
{
    if (thr_acc.norm() < 1e-4) {
        att = att_est;
        thrust = param.mass * param.gra; 
        return;
    }
    Eigen::Vector3d zb = thr_acc.normalized();
    Eigen::Vector3d xc(cos(yaw), sin(yaw), 0.0);
    Eigen::Vector3d yc = zb.cross(xc).normalized();
    Eigen::Vector3d xb = yc.cross(zb).normalized();
    Eigen::Matrix3d R;
    R << xb, yc, zb;
    att = Eigen::Quaterniond(R);
    
    thrust = thr_acc.dot(zb) / (param.gra / 0.04); 
}

void Controller::resetThrustMapping() {}

bool Controller::estimateThrustModel(const Eigen::Vector3d &est_a, double voltage, const Eigen::Vector3d &est_v, const Parameter_t &param) {
    return true;
}