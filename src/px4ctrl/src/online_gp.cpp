#include "online_gp.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

namespace uadl {
namespace {

bool finite(double value) { return std::isfinite(value); }

// Remark 3: the frozen prior is accepted only for finite values with a raw
// gain g0 = 1/Theta2 >= 1e-6; failures are rejected, never clipped.
bool validPrior(double f0, double g0) {
    return finite(f0) && finite(g0) && g0 >= 1e-6;
}

// Clamp only roundoff-scale negative variances, not invalid covariance models.
bool nonnegativeVariance(double& value, double scale) {
    if (!finite(value) || !finite(scale)) return false;
    const double tolerance = 1e-10 * std::max(1.0, std::abs(scale));
    if (value < -tolerance) return false;
    value = std::max(0.0, value);
    return true;
}

struct Interval {
    double lower;
    double upper;
};

double down(double value) {
    return std::nextafter(value, -std::numeric_limits<double>::infinity());
}

double up(double value) {
    return std::nextafter(value, std::numeric_limits<double>::infinity());
}

bool validInterval(const Interval& value) {
    return finite(value.lower) && finite(value.upper) && value.lower <= value.upper;
}

Interval add(const Interval& lhs, const Interval& rhs) {
    return {down(lhs.lower + rhs.lower), up(lhs.upper + rhs.upper)};
}

Interval scale(const Interval& value, double coefficient) {
    if (coefficient >= 0.0)
        return {down(coefficient * value.lower), up(coefficient * value.upper)};
    return {down(coefficient * value.upper), up(coefficient * value.lower)};
}

Interval multiply(const Interval& lhs, const Interval& rhs) {
    const double products[] = {lhs.lower * rhs.lower, lhs.lower * rhs.upper,
                               lhs.upper * rhs.lower, lhs.upper * rhs.upper};
    return {down(*std::min_element(std::begin(products), std::end(products))),
            up(*std::max_element(std::begin(products), std::end(products)))};
}

double maxAbs(const Interval& value) {
    return std::max(std::abs(value.lower), std::abs(value.upper));
}

// The SE kernel is monotone in squared distance. Distances to a box attain
// their minima by coordinate projection and their maxima at farthest corners.
Interval kernelOnBox(const State& location, const GPStateRegion& region,
                     double lengthscale) {
    double squared_min = 0.0, squared_max = 0.0;
    for (int axis = 0; axis < 6; ++axis) {
        const Interval difference{down(location(axis) - region.state_upper(axis)),
                                  up(location(axis) - region.state_lower(axis))};
        double nearest = 0.0;
        if (difference.lower > 0.0) nearest = difference.lower;
        else if (difference.upper < 0.0) nearest = -difference.upper;
        const double farthest = maxAbs(difference);
        const double normalized_min = std::max(0.0, down(nearest / lengthscale));
        const double normalized_max = up(farthest / lengthscale);
        squared_min = std::max(0.0, down(squared_min + down(normalized_min * normalized_min)));
        squared_max = up(squared_max + up(normalized_max * normalized_max));
    }
    return {std::max(0.0, down(std::exp(down(-0.5 * squared_max)))),
            std::min(1.0, up(std::exp(up(-0.5 * squared_min))))};
}

}  // namespace

const char* gpStatusName(GPStatus status) {
    switch (status) {
    case GPStatus::Ok: return "ok";
    case GPStatus::InvalidConfiguration: return "invalid_configuration";
    case GPStatus::InvalidState: return "invalid_state";
    case GPStatus::InvalidPrior: return "invalid_prior";
    case GPStatus::InvalidObservation: return "invalid_observation";
    case GPStatus::InvalidTimestamp: return "invalid_timestamp";
    case GPStatus::NonIncreasingTimestamp: return "non_increasing_timestamp";
    case GPStatus::RateLimited: return "rate_limited";
    case GPStatus::ResidualRejected: return "residual_rejected";
    case GPStatus::NumericalFailure: return "numerical_failure";
    }
    return "unknown";
}

GPResponse GPPrediction::response(double u) const {
    GPResponse result;
    result.status = status;
    if (!valid) return result;
    if (!finite(u)) {
        result.status = GPStatus::InvalidObservation;
        return result;
    }
    // Eq. (26): h0 + q_hat = f_hat + u g_hat and
    // sigma_q^2 = sigma_a^2 + 2 h0 c_ab + h0^2 sigma_b^2
    //           = sigma_f^2 + 2 u c_fg + u^2 sigma_g^2.
    const double h0 = f0_ + g0_ * u;
    result.mean = f + g * u;
    result.variance = var_a + 2.0 * h0 * cov_ab + h0 * h0 * var_b;
    const double variance_scale = var_a + 2.0 * std::abs(h0 * cov_ab)
                                + h0 * h0 * var_b;
    if (!finite(h0) || !finite(result.mean) ||
        !nonnegativeVariance(result.variance, variance_scale)) {
        result.status = GPStatus::NumericalFailure;
        return result;
    }
    result.sigma = std::sqrt(result.variance);
    if (errors_.size() > 0) {
        // w = K_ab^{-1} k_q with k_q = k_a + h0 H0 k_b (Theorem 2).
        const Eigen::VectorXd weights = weights_a_ + h0 * weights_b_;
        if (!weights.allFinite()) {
            result.status = GPStatus::NumericalFailure;
            return result;
        }
        result.historical_error = weights.cwiseAbs().dot(errors_);
    }
    // Eq. (40): E_q = B sigma_q + sum_j |w_j| e_bar_j.
    result.error_bound = rkhs_bound_ * result.sigma + result.historical_error;
    if (!finite(result.error_bound)) {
        result.status = GPStatus::NumericalFailure;
        return result;
    }
    result.valid = true;
    result.status = GPStatus::Ok;
    return result;
}

OnlineGP::OnlineGP(const GPConfig& config) : config_(config) {
    const double lengthscale_squared = config_.lengthscale * config_.lengthscale;
    config_valid_ = finite(config_.lengthscale) && config_.lengthscale > 0.0 &&
        finite(lengthscale_squared) && lengthscale_squared > 0.0 &&
        finite(config_.variance_a) && config_.variance_a > 0.0 &&
        finite(config_.variance_b) && config_.variance_b > 0.0 &&
        finite(config_.noise_variance) && config_.noise_variance > 0.0 &&
        finite(config_.minimum_sample_interval) && config_.minimum_sample_interval >= 0.01 &&
        finite(config_.rkhs_bound) && config_.rkhs_bound >= 0.0 &&
        config_.max_samples > 0;
}

// Unit squared-exponential kernel; k_a and k_b scale it by their signal
// variances.
double OnlineGP::baseKernel(const State& lhs, const State& rhs) const {
    return std::exp(-0.5 * ((lhs - rhs) / config_.lengthscale).squaredNorm());
}

GPPrediction OnlineGP::predict(const State& state, double f0, double g0) const {
    GPPrediction result;
    if (!config_valid_) {
        result.status = GPStatus::InvalidConfiguration;
        return result;
    }
    if (!state.allFinite()) {
        result.status = GPStatus::InvalidState;
        return result;
    }
    if (!validPrior(f0, g0)) {
        result.status = GPStatus::InvalidPrior;
        return result;
    }

    result.f0_ = f0;
    result.g0_ = g0;
    result.rkhs_bound_ = config_.rkhs_bound;
    result.errors_ = errors_;
    result.var_a = config_.variance_a;
    result.var_b = config_.variance_b;

    const Eigen::Index count = static_cast<Eigen::Index>(samples_.size());
    if (count > 0) {
        Eigen::VectorXd ka(count), h0_kb(count);
        for (Eigen::Index i = 0; i < count; ++i) {
            const double kernel = baseKernel(samples_[static_cast<std::size_t>(i)].state, state);
            ka(i) = config_.variance_a * kernel;
            h0_kb(i) = h0_(i) * config_.variance_b * kernel;
        }
        result.weights_a_ = factor_.solve(ka);
        result.weights_b_ = factor_.solve(h0_kb);
        if (!result.weights_a_.allFinite() || !result.weights_b_.allFinite()) return result;
        // Eq. (22): a_hat = k_a' K_ab^{-1} r, b_hat = k_b' H0' K_ab^{-1} r.
        result.a = ka.dot(alpha_);
        result.b = h0_kb.dot(alpha_);
        // Eq. (24): latent posterior variances and cross-covariance.
        result.var_a -= ka.dot(result.weights_a_);
        result.var_b -= h0_kb.dot(result.weights_b_);
        result.cov_ab = -ka.dot(result.weights_b_);
    }
    if (!nonnegativeVariance(result.var_a, config_.variance_a) ||
        !nonnegativeVariance(result.var_b, config_.variance_b) ||
        !finite(result.a) || !finite(result.b) || !finite(result.cov_ab)) return result;

    const double max_cov = std::sqrt(result.var_a) * std::sqrt(result.var_b);
    const double cov_tolerance = 1e-10 * std::max(1.0,
        std::sqrt(config_.variance_a) * std::sqrt(config_.variance_b));
    if (std::abs(result.cov_ab) > max_cov + cov_tolerance) return result;
    result.cov_ab = std::max(-max_cov, std::min(max_cov, result.cov_ab));

    // Eq. (23): f_hat = f0 + a_hat + f0 b_hat, g_hat = g0 (1 + b_hat).
    result.f = f0 + result.a + f0 * result.b;
    // Keep the raw finite posterior gain, including zero or negative values.
    // The protected inverse of Eq. (28) belongs to the controller, not GP
    // conditioning.
    result.g = g0 * (1.0 + result.b);
    // Eq. (25) and the posterior cross-covariance c_fg.
    result.var_f = result.var_a + 2.0 * f0 * result.cov_ab + f0 * f0 * result.var_b;
    result.var_g = g0 * g0 * result.var_b;
    result.cov_fg = g0 * result.cov_ab + f0 * g0 * result.var_b;
    if (!finite(result.f) || !finite(result.g) || !finite(result.cov_fg) ||
        !nonnegativeVariance(result.var_f, result.var_a +
            2.0 * std::abs(f0 * result.cov_ab) + f0 * f0 * result.var_b) ||
        !nonnegativeVariance(result.var_g, g0 * g0 * result.var_b)) return result;

    result.valid = true;
    result.status = GPStatus::Ok;
    return result;
}

GPResponse OnlineGP::response(const State& state, double f0, double g0, double u) const {
    return predict(state, f0, g0).response(u);
}

bool OnlineGP::rebuild() {
    // Eqs. (18)-(21): r = H - h0, K_ab = K_a + H0 K_b H0' + Sigma_e.
    const Eigen::Index count = static_cast<Eigen::Index>(samples_.size());
    Eigen::MatrixXd gram(count, count);
    Eigen::VectorXd residual(count);
    h0_.resize(count);
    errors_.resize(count);
    for (Eigen::Index i = 0; i < count; ++i) {
        const GPSample& sample = samples_[static_cast<std::size_t>(i)];
        h0_(i) = sample.f0 + sample.g0 * sample.u_ex;
        residual(i) = sample.y - h0_(i);
        errors_(i) = sample.error_bound;
    }
    for (Eigen::Index i = 0; i < count; ++i) {
        for (Eigen::Index j = 0; j <= i; ++j) {
            const double kernel = baseKernel(samples_[static_cast<std::size_t>(i)].state,
                                             samples_[static_cast<std::size_t>(j)].state);
            double value = (config_.variance_a + h0_(i) * config_.variance_b * h0_(j)) * kernel;
            if (i == j) value += config_.noise_variance;
            gram(i, j) = value;
            gram(j, i) = value;
        }
    }
    if (!gram.allFinite() || !residual.allFinite()) return false;
    factor_.compute(gram);
    if (factor_.info() != Eigen::Success) return false;
    alpha_ = factor_.solve(residual);
    return alpha_.allFinite();
}

GPInsertResult OnlineGP::insert(const GPSample& sample) {
    GPInsertResult result;
    if (!config_valid_) {
        result.status = GPStatus::InvalidConfiguration;
        return result;
    }
    if (!sample.state.allFinite()) {
        result.status = GPStatus::InvalidState;
        return result;
    }
    if (!validPrior(sample.f0, sample.g0)) {
        result.status = GPStatus::InvalidPrior;
        return result;
    }
    if (!finite(sample.u_ex) || !finite(sample.y) || !finite(sample.error_bound) ||
        sample.error_bound < 0.0 || !finite(sample.f0 + sample.g0 * sample.u_ex)) {
        result.status = GPStatus::InvalidObservation;
        return result;
    }
    if (!finite(sample.timestamp) || sample.timestamp < 0.0) {
        result.status = GPStatus::InvalidTimestamp;
        return result;
    }
    if (!samples_.empty()) {
        const double interval = sample.timestamp - samples_.back().timestamp;
        if (interval <= 0.0) {
            result.status = GPStatus::NonIncreasingTimestamp;
            return result;
        }
        // Accommodate subtraction roundoff in epoch timestamps without
        // admitting a materially higher sampling rate than 100 Hz.
        const double tolerance = 4.0 * std::numeric_limits<double>::epsilon() *
            std::max(1.0, std::abs(sample.timestamp));
        if (interval + tolerance < config_.minimum_sample_interval) {
            result.status = GPStatus::RateLimited;
            return result;
        }
    }

    // Gate against the OLD posterior: an observation cannot validate itself.
    const GPResponse prior_response = response(sample.state, sample.f0, sample.g0, sample.u_ex);
    if (!prior_response.valid) {
        result.status = prior_response.status;
        return result;
    }
    result.residual = std::abs(sample.y - prior_response.mean);
    result.allowed_residual = prior_response.error_bound + sample.error_bound;
    if (!finite(result.residual) || !finite(result.allowed_residual)) return result;
    if (result.residual > result.allowed_residual) {
        result.status = GPStatus::ResidualRejected;
        return result;
    }

    OnlineGP candidate(*this);
    candidate.samples_.push_back(sample);
    if (candidate.samples_.size() > config_.max_samples) candidate.samples_.erase(candidate.samples_.begin());
    if (!candidate.rebuild() ||
        !candidate.predict(sample.state, sample.f0, sample.g0).response(sample.u_ex).valid) return result;
    *this = std::move(candidate);
    result.accepted = true;
    result.status = GPStatus::Ok;
    return result;
}

GPSetBound OnlineGP::predictedSetBound(const std::vector<State>& states,
                                       const std::vector<double>& f0,
                                       const std::vector<double>& g0,
                                       double u_min, double u_max) const {
    GPSetBound result;
    if (!config_valid_) {
        result.status = GPStatus::InvalidConfiguration;
        return result;
    }
    if (states.empty() || f0.size() != states.size() || g0.size() != states.size() ||
        !finite(u_min) || !finite(u_max) || u_min > u_max) {
        result.status = GPStatus::InvalidState;
        return result;
    }
    result.status = GPStatus::Ok;
    for (std::size_t i = 0; i < states.size(); ++i) {
        const GPPrediction prediction = predict(states[i], f0[i], g0[i]);
        if (!prediction.valid) {
            result.status = prediction.status;
            return result;
        }
        if (i == 0) {
            result.f_lower = result.f_upper = prediction.f;
            result.g_lower = result.g_upper = prediction.g;
        } else {
            result.f_lower = std::min(result.f_lower, prediction.f);
            result.f_upper = std::max(result.f_upper, prediction.f);
            result.g_lower = std::min(result.g_lower, prediction.g);
            result.g_upper = std::max(result.g_upper, prediction.g);
        }
        for (double u : {u_min, u_max}) {
            const GPResponse response = prediction.response(u);
            if (!response.valid) {
                result.status = response.status;
                return result;
            }
            result.sigma_bound = std::max(result.sigma_bound, response.sigma);
            result.historical_error = std::max(result.historical_error, response.historical_error);
            result.error_bound = std::max(result.error_bound, response.error_bound);
        }
    }
    result.valid = true;
    return result;
}

GPSetBound OnlineGP::predictedRegionBound(const GPStateRegion& region,
                                         double u_min, double u_max) const {
    GPSetBound result;
    if (!config_valid_) {
        result.status = GPStatus::InvalidConfiguration;
        return result;
    }
    if (!region.state_lower.allFinite() || !region.state_upper.allFinite() ||
        (region.state_lower.array() > region.state_upper.array()).any() ||
        !finite(u_min) || !finite(u_max) || u_min > u_max) {
        result.status = GPStatus::InvalidState;
        return result;
    }
    const Interval f0{region.f0_lower, region.f0_upper};
    const Interval g0{region.g0_lower, region.g0_upper};
    if (!validInterval(f0) || !validInterval(g0) || g0.lower < 1e-6) {
        result.status = GPStatus::InvalidPrior;
        return result;
    }

    const Eigen::Index count = static_cast<Eigen::Index>(samples_.size());
    std::vector<Interval> kernels(static_cast<std::size_t>(count));
    Interval a{0.0, 0.0}, b{0.0, 0.0};
    for (Eigen::Index i = 0; i < count; ++i) {
        kernels[static_cast<std::size_t>(i)] =
            kernelOnBox(samples_[static_cast<std::size_t>(i)].state, region, config_.lengthscale);
        const Interval& kernel = kernels[static_cast<std::size_t>(i)];
        a = add(a, scale(scale(kernel, config_.variance_a), alpha_(i)));
        b = add(b, scale(scale(scale(kernel, config_.variance_b), h0_(i)), alpha_(i)));
    }
    // Posterior mean enclosure in the policy-error coordinates, including the
    // dependence of both f and g on b. No online gain floor enters these maps.
    const Interval one_plus_b = add({1.0, 1.0}, b);
    const Interval f = add(a, multiply(f0, one_plus_b));
    const Interval g = multiply(g0, one_plus_b);
    if (!validInterval(f) || !validInterval(g)) return result;
    result.f_lower = f.lower;
    result.f_upper = f.upper;
    result.g_lower = g.lower;
    result.g_upper = g.upper;

    Eigen::MatrixXd lower;
    if (count > 0) lower = factor_.matrixL();
    // For each fixed x, E_q(x,u) is a sum of norms of affine functions of u:
    // B sqrt([1,h0] Cov(a,b) [1,h0]') + ||diag(e_bar) w||_1.
    // It is convex in u, hence bounding both endpoints covers every input.
    for (double u : {u_min, u_max}) {
        const Interval h = add(f0, scale(g0, u));
        if (!validInterval(h)) return result;
        const double h_abs = maxAbs(h);
        const double prior_variance = up(config_.variance_a +
            up(config_.variance_b * up(h_abs * h_abs)));
        std::vector<Interval> whitened(static_cast<std::size_t>(count));
        std::vector<Interval> weights(static_cast<std::size_t>(count));
        double reduction_lower = 0.0;
        for (Eigen::Index i = 0; i < count; ++i) {
            // k_q,i = SE(x_i,x) [s_a^2 + h0_i s_b^2 h0(x,u)].
            const Interval coefficient = add({config_.variance_a, config_.variance_a},
                scale(scale(h, config_.variance_b), h0_(i)));
            Interval value = multiply(kernels[static_cast<std::size_t>(i)], coefficient);
            // Interval forward solve z = L^-1 k_q. Lower-bound ||z||^2
            // without subtracting an invalid sampled posterior variance.
            for (Eigen::Index j = 0; j < i; ++j)
                value = add(value, scale(whitened[static_cast<std::size_t>(j)], -lower(i, j)));
            if (!finite(lower(i, i)) || lower(i, i) <= 0.0) return result;
            const double diagonal = lower(i, i);
            value = {down(value.lower / diagonal), up(value.upper / diagonal)};
            if (!validInterval(value)) return result;
            whitened[static_cast<std::size_t>(i)] = value;
            const double nearest = value.lower > 0.0 ? value.lower :
                (value.upper < 0.0 ? -value.upper : 0.0);
            reduction_lower = std::max(0.0, down(reduction_lower + down(nearest * nearest)));
        }
        double historical_upper = 0.0;
        for (Eigen::Index i = count; i-- > 0;) {
            // Interval backward solve w = L^-T z = K_ab^-1 k_q.
            Interval value = whitened[static_cast<std::size_t>(i)];
            for (Eigen::Index j = i + 1; j < count; ++j)
                value = add(value, scale(weights[static_cast<std::size_t>(j)], -lower(j, i)));
            value = {down(value.lower / lower(i, i)), up(value.upper / lower(i, i))};
            if (!validInterval(value)) return result;
            weights[static_cast<std::size_t>(i)] = value;
            historical_upper = up(historical_upper + up(maxAbs(value) * errors_(i)));
        }
        double variance_upper = up(prior_variance - reduction_lower);
        if (!nonnegativeVariance(variance_upper, prior_variance)) return result;
        const double sigma_upper = up(std::sqrt(variance_upper));
        const double error_upper = up(up(config_.rkhs_bound * sigma_upper) + historical_upper);
        if (!finite(error_upper)) return result;
        result.sigma_bound = std::max(result.sigma_bound, sigma_upper);
        result.historical_error = std::max(result.historical_error, historical_upper);
        result.error_bound = std::max(result.error_bound, error_upper);
    }
    result.valid = true;
    result.status = GPStatus::Ok;
    return result;
}

void OnlineGP::reset() {
    samples_.clear();
    factor_ = Eigen::LLT<Eigen::MatrixXd>();
    h0_.resize(0);
    alpha_.resize(0);
    errors_.resize(0);
}

}  // namespace uadl
