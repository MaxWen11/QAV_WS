#include "online_gp.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace uadl {
namespace {

bool finite(double value) { return std::isfinite(value); }

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

// For 0 <= k_j <= variance, |sum c_j*k_j| is bounded by variance
// times the greater of the positive and negative coefficient sums.
double positiveKernelWeightBound(const Eigen::VectorXd& coefficients) {
    return std::max(coefficients.cwiseMax(0.0).sum(),
                    -coefficients.cwiseMin(0.0).sum());
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
        const Eigen::VectorXd weights = weights_a_ + h0 * weights_b_;
        if (!weights.allFinite()) {
            result.status = GPStatus::NumericalFailure;
            return result;
        }
        result.historical_error = weights.cwiseAbs().dot(errors_);
    }
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
        result.a = ka.dot(alpha_);
        result.b = h0_kb.dot(alpha_);
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

    result.f = f0 + result.a + f0 * result.b;
    // Keep the raw finite posterior gain, including zero or negative values.
    // The protected inverse belongs to the controller, not GP conditioning.
    result.g = g0 * (1.0 + result.b);
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
    if (!alpha_.allFinite()) return false;

    global_error_a_ = 0.0;
    global_error_b_ = 0.0;
    for (Eigen::Index i = 0; i < count; ++i) {
        if (errors_(i) == 0.0) continue;
        // Symmetry makes this solved column the i-th row of the precision
        // operator. No explicit inverse is formed or used for prediction.
        const Eigen::VectorXd row = factor_.solve(Eigen::VectorXd::Unit(count, i));
        if (!row.allFinite()) return false;
        global_error_a_ += errors_(i) * config_.variance_a * positiveKernelWeightBound(row);
        global_error_b_ += errors_(i) * config_.variance_b *
            positiveKernelWeightBound(row.cwiseProduct(h0_));
    }
    return finite(global_error_a_) && finite(global_error_b_);
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

GPGlobalBound OnlineGP::globalBound(double abs_f0_bound, double abs_g0_bound,
                                   double abs_u) const {
    GPGlobalBound result;
    if (!config_valid_) {
        result.status = GPStatus::InvalidConfiguration;
        return result;
    }
    if (!finite(abs_f0_bound) || !finite(abs_g0_bound) || !finite(abs_u) ||
        abs_f0_bound < 0.0 || abs_g0_bound < 0.0 || abs_u < 0.0) {
        result.status = GPStatus::InvalidPrior;
        return result;
    }
    const double h0_bound = abs_f0_bound + abs_g0_bound * abs_u;
    // Conditioning cannot increase latent variance; this prior bound is
    // uniform for the supplied prior/input envelope, including unseen states.
    const double variance_bound = config_.variance_a + h0_bound * h0_bound * config_.variance_b;
    result.sigma_bound = std::sqrt(variance_bound);
    result.historical_error = global_error_a_ + h0_bound * global_error_b_;
    result.error_bound = config_.rkhs_bound * result.sigma_bound + result.historical_error;
    if (!finite(h0_bound) || !finite(result.error_bound)) return result;
    result.valid = true;
    result.status = GPStatus::Ok;
    return result;
}

GPGlobalBound OnlineGP::domainBound(const State& lower, const State& upper,
                                   double abs_f0_bound, double abs_g0_bound,
                                   double abs_u, std::size_t max_cells) const {
    GPGlobalBound bound = globalBound(abs_f0_bound, abs_g0_bound, abs_u);
    if (!bound.valid) return bound;
    if (!lower.allFinite() || !upper.allFinite() || (lower.array() > upper.array()).any()) {
        bound.valid = false;
        bound.status = GPStatus::InvalidState;
        return bound;
    }
    if (max_cells == 0 || max_cells > 256) {
        bound.valid = false;
        bound.status = GPStatus::InvalidConfiguration;
        return bound;
    }
    if (samples_.empty()) return bound; // Exactly the prior bound, no artificial inflation.

    struct Cell { State lower; State upper; };
    std::vector<Cell> cells;
    cells.reserve(max_cells);
    cells.push_back({lower, upper});
    // A scalar SE lengthscale makes longest physical and normalized dimensions
    // identical. Bisect the widest remaining cell; the children cover it fully.
    while (cells.size() < max_cells) {
        std::size_t selected = 0;
        Eigen::Index dimension = 0;
        double longest = 0.0;
        for (std::size_t i = 0; i < cells.size(); ++i) {
            const State width = cells[i].upper - cells[i].lower;
            if (!width.allFinite()) return bound;
            Eigen::Index candidate_dimension;
            const double candidate_width = width.maxCoeff(&candidate_dimension);
            if (candidate_width > longest) {
                longest = candidate_width;
                selected = i;
                dimension = candidate_dimension;
            }
        }
        if (longest <= 0.0) break;
        const double middle = cells[selected].lower(dimension) + 0.5 * longest;
        if (middle <= cells[selected].lower(dimension) || middle >= cells[selected].upper(dimension)) break;
        Cell right = cells[selected];
        cells[selected].upper(dimension) = middle;
        right.lower(dimension) = middle;
        cells.push_back(std::move(right));
    }

    const double h_max = abs_f0_bound + abs_g0_bound * abs_u;
    const double prior_variance = config_.variance_a + h_max * h_max * config_.variance_b;
    GPGlobalBound covered;
    covered.status = GPStatus::Ok;
    for (const Cell& cell : cells) {
        const State width = cell.upper - cell.lower;
        const State center = cell.lower + 0.5 * width;
        const double radius = 0.5 * width.norm();
        if (!center.allFinite() || !finite(radius)) return bound;
        const double normalized_radius = radius / config_.lengthscale;
        const double feature_distance = std::sqrt(2.0 * prior_variance *
            -std::expm1(-0.5 * normalized_radius * normalized_radius));
        // ||grad exp(-||x-x_i||^2/(2l^2))|| <= 1/(l*sqrt(e)).
        // The cached positive/negative coefficient envelope is >= half its
        // L1 norm, hence 2*global_error_{a,b} bounds the weight Lipschitz sums.
        const double kernel_difference = std::min(1.0, normalized_radius / std::sqrt(std::exp(1.0)));
        const double history_increase = 2.0 * kernel_difference *
            (global_error_a_ + h_max * global_error_b_);

        // The query prior (0,1) is used solely to parameterize h0=u here.
        // Covariance and historical weights depend on query h0, not its
        // separate f0/g0 values. At fixed x the norm + weighted absolute
        // affine function is convex in h0, so interval endpoints suffice.
        const auto prediction = predict(center, 0.0, 1.0);
        if (!prediction.valid) return bound;
        for (double h : {-h_max, h_max}) {
            const auto at_center = prediction.response(h);
            if (!at_center.valid) return bound;
            const double sigma = at_center.sigma + feature_distance;
            const double history = at_center.historical_error + history_increase;
            const double error = config_.rkhs_bound * sigma + history;
            if (!finite(error) || !finite(sigma) || !finite(history)) return bound;
            covered.sigma_bound = std::max(covered.sigma_bound, sigma);
            covered.historical_error = std::max(covered.historical_error, history);
            covered.error_bound = std::max(covered.error_bound, error);
        }
    }
    covered.valid = true;
    // Both certificates hold uniformly, so their minimum is also a certificate.
    return covered.error_bound < bound.error_bound ? covered : bound;
}

GPGlobalBound OnlineGP::predictedSetBound(const std::vector<State>& states,
                                          const std::vector<double>& f0,
                                          const std::vector<double>& g0,
                                          double u_min, double u_max) const {
    GPGlobalBound result;
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

GPModelBounds OnlineGP::modelBounds(double abs_f0_bound, double g0_min,
                                   double g0_max) const {
    GPModelBounds result;
    if (!config_valid_) {
        result.status = GPStatus::InvalidConfiguration;
        return result;
    }
    if (!finite(abs_f0_bound) || !finite(g0_min) || !finite(g0_max) ||
        abs_f0_bound < 0.0 || g0_min < 1e-6 || g0_max < g0_min) {
        result.status = GPStatus::InvalidPrior;
        return result;
    }
    const double abs_a = config_.variance_a * alpha_.cwiseAbs().sum();
    const double abs_b = config_.variance_b * h0_.cwiseProduct(alpha_).cwiseAbs().sum();
    result.abs_f = abs_f0_bound * (1.0 + abs_b) + abs_a;
    const double products[] = {g0_min * (1.0 - abs_b), g0_min * (1.0 + abs_b),
                               g0_max * (1.0 - abs_b), g0_max * (1.0 + abs_b)};
    result.g_min = *std::min_element(products, products + 4);
    result.g_max = *std::max_element(products, products + 4);
    if (!finite(result.abs_f) || !finite(result.g_min) || !finite(result.g_max)) return result;
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
    global_error_a_ = 0.0;
    global_error_b_ = 0.0;
}

}  // namespace uadl
