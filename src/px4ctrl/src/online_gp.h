#ifndef UADL_ONLINE_GP_H
#define UADL_ONLINE_GP_H

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace uadl {

// Inertial state ordered [p_x, p_y, p_z, v_x, v_y, v_z].
using State = Eigen::Matrix<double, 6, 1>;

struct GPConfig {
    double lengthscale = 0.5;
    double variance_a = 1.0;
    double variance_b = 1.0;
    double noise_variance = 0.01;
    // May reduce the insertion rate, but must not be below 0.01 s (100 Hz).
    double minimum_sample_interval = 0.01;
    // This is an assumed RKHS norm bound, not a Gaussian confidence multiplier.
    double rkhs_bound = 2.0;
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
    double u_ex = 0.0;
    double y = 0.0;
    double f0 = 0.0;
    double g0 = 1.0;
    double error_bound = 0.0;
    // Timestamp of the measured state, in seconds, not callback receipt time.
    double timestamp = 0.0;
};

struct GPResponse {
    bool valid = false;
    GPStatus status = GPStatus::NumericalFailure;
    double mean = 0.0;
    double variance = 0.0;
    double sigma = 0.0;
    double error_bound = 0.0;
    double historical_error = 0.0;
};

struct GPPrediction {
    bool valid = false;
    GPStatus status = GPStatus::NumericalFailure;
    double a = 0.0;
    double b = 0.0;
    double f = 0.0;
    double g = 0.0;
    double var_a = 0.0;
    double var_b = 0.0;
    double cov_ab = 0.0;
    double var_f = 0.0;
    double var_g = 0.0;
    double cov_fg = 0.0;

    // Noiseless response f + g*u and the pointwise bound in Eq. (34).
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

struct GPGlobalBound {
    bool valid = false;
    GPStatus status = GPStatus::NumericalFailure;
    double error_bound = 0.0;
    double sigma_bound = 0.0;
    double historical_error = 0.0;
};

struct GPModelBounds {
    bool valid = false;
    GPStatus status = GPStatus::NumericalFailure;
    double abs_f = 0.0;
    double g_min = 0.0;
    double g_max = 0.0;
};

// Independent of ROS and Torch. Copying preserves a complete estimator window
// and factorization; a caller can validate a candidate copy before committing it.
class OnlineGP {
public:
    explicit OnlineGP(const GPConfig& config = GPConfig());
    GPPrediction predict(const State& state, double f0, double g0) const;
    GPResponse response(const State& state, double f0, double g0, double u) const;
    GPInsertResult insert(const GPSample& sample);

    // A conservative uniform Eq. (34) envelope over ALL states and over inputs
    // with |u|<=abs_u, provided |f0(x)|<=abs_f0_bound and |g0(x)|<=abs_g0_bound
    // throughout that domain. A pointwise posterior sigma is not a uniform bound.
    GPGlobalBound globalBound(double abs_f0_bound, double abs_g0_bound,
                              double abs_u) const;
    // Certified box covering, not sampled point maxima. Each cell center is
    // inflated by SE RKHS/weight variation bounds to cover its whole cell;
    // convexity in h0 covers all |h0|<=abs_f0_bound+abs_g0_bound*abs_u.
    // Uses at most max_cells cells (1..256), and never exceeds globalBound.
    // A sufficiently small, observed domain can yield a contracting bound;
    // no such improvement is promised for broad or unobserved domains.
    GPGlobalBound domainBound(const State& lower, const State& upper,
                              double abs_f0_bound, double abs_g0_bound,
                              double abs_u, std::size_t max_cells = 8) const;
    // Eq. (34) over a predicted state set and the input interval [u_min,u_max].
    // f0/g0 hold the frozen prior at each state. At a fixed state the bound is
    // convex in h0=f0+g0*u, so the interval endpoints attain its maximum.
    GPGlobalBound predictedSetBound(const std::vector<State>& states,
                                    const std::vector<double>& f0,
                                    const std::vector<double>& g0,
                                    double u_min, double u_max) const;
    // Uniform bounds on posterior means, assuming the stated prior envelopes
    // hold throughout the domain. The returned gain interval is not floored.
    GPModelBounds modelBounds(double abs_f0_bound, double g0_min, double g0_max) const;

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
    Eigen::LLT<Eigen::MatrixXd> factor_;
    Eigen::VectorXd h0_;
    Eigen::VectorXd alpha_;
    Eigen::VectorXd errors_;
    // Cached coefficients for the absolute-weight global envelope.
    double global_error_a_ = 0.0;
    double global_error_b_ = 0.0;
};

}  // namespace uadl
#endif
