#ifndef UADL_ONLINE_GP_H
#define UADL_ONLINE_GP_H

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace uadl {

// Inertial state ordered [p_x, p_y, p_z, v_x, v_y, v_z].
using State = Eigen::Matrix<double, 6, 1>;

// Task-error GP of Section IV-B. k_a and k_b are squared-exponential kernels
// on the six-dimensional state with a common lengthscale; their
// hyperparameters are selected offline and remain fixed during flight.
struct GPConfig {
    double lengthscale = 0.5;
    double variance_a = 1.0;
    double variance_b = 1.0;
    // Working likelihood Sigma_e = sigma_on^2 I of Eq. (21).
    double noise_variance = 0.01;
    // Insertion rate up to 100 Hz: the interval must not be below 0.01 s.
    double minimum_sample_interval = 0.01;
    // RKHS norm bound B of Assumption 4, Eq. (39); not a Gaussian
    // confidence multiplier.
    double rkhs_bound = 0.36;
    // N_max, retained observations per axis.
    std::size_t max_samples = 50;
};

enum class GPStatus {
    Ok,
    InvalidConfiguration,
    InvalidState,
    InvalidPrior,
    InvalidObservation,
    InvalidTimestamp,
    NonIncreasingTimestamp,
    RateLimited,
    ResidualRejected,
    NumericalFailure
};

const char* gpStatusName(GPStatus status);

struct GPSample {
    State state = State::Zero();
    // Recorded executed input u^(i).
    double u_ex = 0.0;
    // Response label h^(i).
    double y = 0.0;
    // Frozen prior f0, g0 at the sample state.
    double f0 = 0.0;
    double g0 = 1.0;
    // Observation-error bound e_bar_j of Assumption 4.
    double error_bound = 0.0;
    // Timestamp of the measured state, in seconds, not callback receipt time.
    double timestamp = 0.0;
};

struct GPResponse {
    bool valid = false;
    GPStatus status = GPStatus::NumericalFailure;
    // h0 + q_hat = f_hat + u g_hat, Eq. (26).
    double mean = 0.0;
    // sigma_q^2, Eq. (26).
    double variance = 0.0;
    double sigma = 0.0;
    // E_q = B sigma_q + sum_j |w_j| e_bar_j, Eq. (40).
    double error_bound = 0.0;
    double historical_error = 0.0;
};

struct GPPrediction {
    bool valid = false;
    GPStatus status = GPStatus::NumericalFailure;
    // Eq. (22): a_hat, b_hat; Eq. (23): f_hat, g_hat.
    double a = 0.0;
    double b = 0.0;
    double f = 0.0;
    double g = 0.0;
    // Eq. (24): sigma_a^2, sigma_b^2, c_ab; Eq. (25): sigma_f^2, sigma_g^2;
    // c_fg = g0 c_ab + f0 g0 sigma_b^2.
    double var_a = 0.0;
    double var_b = 0.0;
    double cov_ab = 0.0;
    double var_f = 0.0;
    double var_g = 0.0;
    double cov_fg = 0.0;

    // Noiseless response f + g*u (Eq. 26) and the pointwise bound of Eq. (40).
    // Retains its own solve results, so it remains valid after GP mutations.
    GPResponse response(double u) const;

private:
    friend class OnlineGP;
    double f0_ = 0.0;
    double g0_ = 1.0;
    double rkhs_bound_ = 0.0;
    Eigen::VectorXd weights_a_;
    Eigen::VectorXd weights_b_;
    Eigen::VectorXd errors_;
};

struct GPInsertResult {
    bool accepted = false;
    GPStatus status = GPStatus::NumericalFailure;
    double residual = 0.0;
    double allowed_residual = 0.0;
};

struct GPSetBound {
    bool valid = false;
    GPStatus status = GPStatus::NumericalFailure;
    double error_bound = 0.0;
    double sigma_bound = 0.0;
    double historical_error = 0.0;
    // Raw posterior means over the supplied state set. The controller applies
    // gain protection after inference and uses these intervals to construct V.
    double f_lower = 0.0;
    double f_upper = 0.0;
    double g_lower = 0.0;
    double g_upper = 0.0;
};

// A closed continuous state region, with enclosures of the frozen prior over
// every state in the box. The prior intervals must come from the frozen model
// itself (for example interval propagation), not extrema of sampled values.
struct GPStateRegion {
    State state_lower = State::Zero();
    State state_upper = State::Zero();
    double f0_lower = 0.0;
    double f0_upper = 0.0;
    double g0_lower = 1.0;
    double g0_upper = 1.0;
};

// Independent of ROS and Torch. Copying preserves a complete estimator window
// and factorization; a caller can validate a candidate copy before committing it.
class OnlineGP {
public:
    explicit OnlineGP(const GPConfig& config = GPConfig());
    GPPrediction predict(const State& state, double f0, double g0) const;
    GPResponse response(const State& state, double f0, double g0, double u) const;
    // Inserts a new, valid sample into the window D_N (oldest sample removed
    // beyond N_max). The prediction residual is checked against the retained
    // posterior before insertion, with the allowance E_q + e_bar_j.
    GPInsertResult insert(const GPSample& sample);
    // Eq. (40) over a finite state set and the input interval [u_min,u_max].
    // f0/g0 hold the frozen prior at each state. At a fixed state the bound is
    // convex in h0=f0+g0*u, so the interval endpoints attain its maximum.
    // This does not bound states between the supplied points.
    GPSetBound predictedSetBound(const std::vector<State>& states,
                                 const std::vector<double>& f0,
                                 const std::vector<double>& g0,
                                 double u_min, double u_max) const;
    // A uniform bound over the entire continuous box and input interval.
    // Bounds SE kernels analytically over the box, propagates intervals
    // through the Cholesky solves, and includes historical contamination.
    // Region construction must also cover between-sample states for Eq. (46).
    GPSetBound predictedRegionBound(const GPStateRegion& region,
                                    double u_min, double u_max) const;

    std::size_t size() const { return samples_.size(); }
    bool valid() const { return config_valid_; }
    const GPConfig& config() const { return config_; }
    void reset();

private:
    bool rebuild();
    double baseKernel(const State& lhs, const State& rhs) const;

    GPConfig config_;
    bool config_valid_ = false;
    std::vector<GPSample> samples_;
    // Cholesky factor of K_ab, Eq. (21).
    Eigen::LLT<Eigen::MatrixXd> factor_;
    // Diagonal of H0, Eq. (19).
    Eigen::VectorXd h0_;
    // K_ab^{-1} r.
    Eigen::VectorXd alpha_;
    Eigen::VectorXd errors_;
};

}  // namespace uadl
#endif
