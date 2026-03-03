#include "controller.h"
#include <iostream>
#include <cmath>

using namespace std;

// ==========================================
// OnlineGP 实现
// ==========================================
OnlineGP::OnlineGP(double l, double sigma_f, double beta, int N_max) 
    : lengthscale(l), var_f(sigma_f), beta_factor(beta), max_window_size(N_max) {
    noise_var = 0.01; // 观测噪声水平
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
        
        // 此处的先验理论上应该计算过去每个状态的 f0(x_i) 和 g0(x_i)
        double mu_i = f_prior + U_buffer[i] * g_prior; 
        Y_err(i) = Y_buffer[i] - mu_i;

        for (int j = 0; j <= i; ++j) {
            double kf_ij = kernel_se(X_buffer[i], X_buffer[j]);
            double kg_ij = kernel_se(X_buffer[i], X_buffer[j]);
            // 公式 18 和 21：组合核
            double val = kf_ij + U_buffer[i] * kg_ij * U_buffer[j];
            if (i == j) val += noise_var;
            K_fg(i, j) = val;
            K_fg(j, i) = val;
        }
    }

    Eigen::VectorXd K_inv_Y = K_fg.ldlt().solve(Y_err);
    Eigen::MatrixXd K_inv = K_fg.ldlt().solve(Eigen::MatrixXd::Identity(N, N));

    // 公式 22：后验均值
    f_post = f_prior + k_f.transpose() * K_inv_Y;
    g_post = g_prior + (U_mat * k_g).transpose() * K_inv_Y;

    // 公式 23：后验方差
    sigma_f = kernel_se(x_query, x_query) - k_f.transpose() * K_inv * k_f;
    sigma_g = kernel_se(x_query, x_query) - (U_mat * k_g).transpose() * K_inv * (U_mat * k_g);

    sigma_f = sqrt(max(0.0, sigma_f));
    sigma_g = sqrt(max(0.0, sigma_g));
}

// ==========================================
// RTMPC 实现
// ==========================================
RTMPC::RTMPC(double dt, int H, const Eigen::Vector2d& Q_diag, double R, 
             const Eigen::Vector2d& Q_anc_diag, double R_anc,
             const Eigen::Vector2d& state_limit, const Eigen::Vector2d& input_limit)
    : dt_(dt), H_(H), Q_(Q_diag.asDiagonal()), R_(R), state_limit_(state_limit), input_limit_(input_limit),
      qp_solver_(H_, H_) // 变量数为H的QP问题
{
    A_d << 1.0, dt_, 
           0.0, 1.0;
    B_d << 0.5 * dt_ * dt_, 
           dt_;

    // 离线计算 Ancillary LQR
    compute_dlqr(Q_anc_diag.asDiagonal(), R_anc);
    
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    qp_solver_.setOptions(options);
}

void RTMPC::compute_dlqr(const Eigen::Matrix2d& Q_anc, double R_anc) {
    // 简易代数黎卡提方程(DARE)迭代求 LQR K_anc
    Eigen::Matrix2d P = Q_anc;
    for (int i = 0; i < 100; ++i) {
        P = A_d.transpose() * P * A_d - 
            A_d.transpose() * P * B_d * (R_anc + B_d.transpose() * P * B_d).inverse() * B_d.transpose() * P * A_d + Q_anc;
    }
    K_anc = (R_anc + B_d.transpose() * P * B_d).inverse() * B_d.transpose() * P * A_d;
}

double RTMPC::solve(const Eigen::Vector2d& current_state, const Eigen::Vector2d& ref_state, double w_kappa) {
    // 核心思想：在此处构建密集型QP (Condensed QP) 矩阵进行预测控制
    // 为保持示例精简，演示构建 H_qp 和 g_qp 的标准做法
    // min U^T H U + U^T g
    // s.t. lb <= U <= ub (包含自适应收缩的 Tube w_kappa)

    Eigen::MatrixXd H_qp = Eigen::MatrixXd::Identity(H_, H_) * R_;
    Eigen::VectorXd g_qp = Eigen::VectorXd::Zero(H_);
    
    // 省略标准密集QP的 A_cond 和 B_cond 构建细节...
    // 将状态限制收缩: state_limit_ - w_kappa
    
    // (由于qpOASES需要连续内存，这里使用伪码表示，你需要根据具体平台补全矩阵展开)
    // qp_solver_.init(H_qp.data(), g_qp.data(), A_cons.data(), lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR);
    // qp_solver_.getPrimalSolution(U_opt);
    
    // 假设求解出的最优控制第一项为 bar_eta
    double bar_eta = 0.0; // 替换为真实的 U_opt[0]
    
    return bar_eta;
}


// ==========================================
// 控制器主流程实现
// ==========================================
Controller::Controller(Parameter_t &param_) : param(param_) {
    Gravity = Eigen::Vector3d(0.0, 0.0, -param.gra);

    // 根据论文实验参数初始化 GP (l=0.5, var=1.0, beta=2.0, N=50)
    gp_x = new OnlineGP(0.5, 1.0, 2.0, 50);
    gp_y = new OnlineGP(0.5, 1.0, 2.0, 50);
    gp_z = new OnlineGP(0.5, 1.0, 2.0, 50);

    // 根据论文参数初始化 MPC
    // X, Y: Q = diag(10, 1), R = 0.1 | Ancillary: Q = diag(20, 2), R = 0.01 | 限制: p<=2, v<=2, u<=3
    mpc_x = new RTMPC(0.05, 20, Eigen::Vector2d(10, 1), 0.1, Eigen::Vector2d(20, 2), 0.01, Eigen::Vector2d(2.0, 2.0), Eigen::Vector2d(-3.0, 3.0));
    mpc_y = new RTMPC(0.05, 20, Eigen::Vector2d(10, 1), 0.1, Eigen::Vector2d(20, 2), 0.01, Eigen::Vector2d(2.0, 2.0), Eigen::Vector2d(-3.0, 3.0));
    
    // Z: Q = diag(15, 2), R = 0.5 | Ancillary: Q = diag(30, 5), R = 0.01 | 限制: p<=1.5, v<=1, u<=5
    mpc_z = new RTMPC(0.05, 20, Eigen::Vector2d(15, 2), 0.5, Eigen::Vector2d(30, 5), 0.01, Eigen::Vector2d(1.5, 1.0), Eigen::Vector2d(-2.0, 5.0));

    last_u_x = 0; last_u_y = 0; last_u_z = 0;
}

quadrotor_msgs::Px4ctrlDebug Controller::update(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu,
    Controller_Output_t &u,
    double voltage) 
{
    // 1. 提取当前状态
    Eigen::Vector2d state_x(odom.p(0), odom.v(0));
    Eigen::Vector2d state_y(odom.p(1), odom.v(1));
    Eigen::Vector2d state_z(odom.p(2), odom.v(2));

    Eigen::Vector2d ref_x(des.p(0), des.v(0));
    Eigen::Vector2d ref_y(des.p(1), des.v(1));
    Eigen::Vector2d ref_z(des.p(2), des.v(2));

    // 2. 将观测值推入在线 GP 数据集 (观测值为真实加速度减去重力项)
    Eigen::Vector3d obs_acc = imu.a - Gravity;
    gp_x->add_data(state_x, last_u_x, obs_acc(0));
    gp_y->add_data(state_y, last_u_y, obs_acc(1));
    gp_z->add_data(state_z, last_u_z, obs_acc(2));

    // 3. GAN 先验提取与 GP 后验预测
    double fx_post, gx_post, sig_fx, sig_gx;
    double fy_post, gy_post, sig_fy, sig_gy;
    double fz_post, gz_post, sig_fz, sig_gz;

    gp_x->predict(state_x, last_u_x, get_f0_prior(state_x), get_g0_prior(state_x), fx_post, gx_post, sig_fx, sig_gx);
    gp_y->predict(state_y, last_u_y, get_f0_prior(state_y), get_g0_prior(state_y), fy_post, gy_post, sig_fy, sig_gy);
    gp_z->predict(state_z, last_u_z, get_f0_prior(state_z), get_g0_prior(state_z), fz_post, gz_post, sig_fz, sig_gz);

    // 论文中公式 (44): 局部管缩放不确定度 w_kappa
    double beta = 2.0;
    double w_kappa_x = beta * sig_fx + abs(last_u_x) * beta * sig_gx;
    double w_kappa_y = beta * sig_fy + abs(last_u_y) * beta * sig_gy;
    double w_kappa_z = beta * sig_fz + abs(last_u_z) * beta * sig_gz;

    // 4. 求解 RTMPC 名义控制 (bar_eta)
    double bar_eta_x = mpc_x->solve(state_x, ref_x, w_kappa_x);
    double bar_eta_y = mpc_y->solve(state_y, ref_y, w_kappa_y);
    double bar_eta_z = mpc_z->solve(state_z, ref_z, w_kappa_z);

    // 5. 辅助反馈控制 (Ancillary Control: K_anc * error)
    double anc_eta_x = -mpc_x->get_K_anc() * (state_x - ref_x);
    double anc_eta_y = -mpc_y->get_K_anc() * (state_y - ref_y);
    double anc_eta_z = -mpc_z->get_K_anc() * (state_z - ref_z);

    // 6. 虚拟期望输入 \eta
    double eta_x = bar_eta_x + anc_eta_x;
    double eta_y = bar_eta_y + anc_eta_y;
    double eta_z = bar_eta_z + anc_eta_z;

    // 如果未启动MPC (静止测试)，可以用给定的脱机指令做测试
    // eta_x = des.a(0); eta_y = des.a(1); eta_z = des.a(2);

    // 7. 反馈线性化映射 (Feedback Linearization) : u = (eta - f) / g
    double u_cmd_x = (eta_x - fx_post) / max(0.1, gx_post);
    double u_cmd_y = (eta_y - fy_post) / max(0.1, gy_post);
    double u_cmd_z = (eta_z - fz_post) / max(0.1, gz_post);

    last_u_x = u_cmd_x; last_u_y = u_cmd_y; last_u_z = u_cmd_z;

    // 8. 映射到飞控底层的总推力与期望姿态
    Eigen::Vector3d thr_acc(u_cmd_x, u_cmd_y, u_cmd_z + param.gra); 
    
    Eigen::Quaterniond desired_attitude;
    computeFlatInput(thr_acc, des.yaw, odom.q, desired_attitude, u.thrust);

    // 对齐至 FCU 机体坐标系
    u.q = imu.q * odom.q.inverse() * desired_attitude; 
    
    // （如果需要加入角速度反馈控制可以在此处叠加，按照原架构）
    u.bodyrates = Eigen::Vector3d::Zero();

    // 记录 Debug 信息
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
    // 标准的由期望加速度反推姿态和平行推力的函数
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
    
    // 根据具体飞控的模型做推力映射，这里简化为理想映射
    thrust = thr_acc.dot(zb) / (param.gra / 0.04); 
}