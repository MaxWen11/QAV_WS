#include "online_gp.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& description) {
    if (!condition) {
        std::cerr << "FAIL: " << description << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void near(double actual, double expected, const std::string& description, double tolerance = 1e-9) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance * (1.0 + std::abs(expected))) {
        std::cerr << "FAIL: " << description << ": got " << actual << ", expected " << expected << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void testEmptyPosteriorAndBounds() {
    uadl::GPConfig config;
    config.variance_a = 1.3;
    config.variance_b = 0.7;
    config.rkhs_bound = 2.4;
    uadl::OnlineGP gp(config);
    const double f0 = 0.7, g0 = 1.3, u = -0.4;
    const double h0 = f0 + g0 * u;
    const auto prediction = gp.predict(uadl::State::Zero(), f0, g0);
    require(prediction.valid, "empty posterior is valid");
    near(prediction.a, 0.0, "empty a mean");
    near(prediction.b, 0.0, "empty b mean");
    near(prediction.f, f0, "empty f is prior");
    near(prediction.g, g0, "empty g is prior");
    near(prediction.var_f, config.variance_a + f0 * f0 * config.variance_b, "induced drift prior variance");
    near(prediction.cov_fg, f0 * g0 * config.variance_b, "nonzero induced drift-gain prior covariance");
    const auto response = prediction.response(u);
    require(response.valid, "empty response is valid");
    near(response.mean, h0, "empty response mean");
    near(response.variance, config.variance_a + h0 * h0 * config.variance_b, "empty composite variance");
    // Empty window: the sum in Eq. (40) is zero and sigma_q^2 = k_q(xi, xi).
    near(response.error_bound, config.rkhs_bound * response.sigma, "empty Eq. (40) bound");
    near(response.historical_error, 0.0, "empty historical contamination");
}

void testAnalyticSingleObservation() {
    uadl::GPConfig config;
    config.variance_a = 1.0;
    config.variance_b = 2.0;
    config.noise_variance = 0.25;
    config.rkhs_bound = 1.7;
    uadl::OnlineGP gp(config);
    uadl::GPSample sample;
    sample.f0 = 2.0;
    sample.g0 = 3.0;
    sample.u_ex = 1.0;
    sample.y = 5.4;
    sample.error_bound = 0.2;
    sample.timestamp = 1.0;
    require(gp.insert(sample).accepted, "single observation accepted");

    // Scalar closed-form conditioning is an independent oracle for every
    // covariance term and makes the required H0 rather than U scaling visible.
    const double h0 = 5.0, residual = 0.4;
    const double ka = config.variance_a, hkb = h0 * config.variance_b;
    const double gram = ka + h0 * hkb + config.noise_variance;
    const auto p = gp.predict(sample.state, sample.f0, sample.g0);
    require(p.valid, "single observation posterior valid");
    near(p.a, ka * residual / gram, "a posterior mean");
    near(p.b, hkb * residual / gram, "b posterior mean uses prior response");
    near(p.var_a, ka - ka * ka / gram, "a posterior variance");
    near(p.var_b, config.variance_b - hkb * hkb / gram, "b posterior variance");
    near(p.cov_ab, -ka * hkb / gram, "a-b posterior cross covariance");
    require(p.cov_ab < 0.0, "conditioning induces negative task cross covariance");
    near(p.f, sample.f0 + p.a + sample.f0 * p.b, "f reconstruction");
    near(p.g, sample.g0 * (1.0 + p.b), "g reconstruction");
    near(p.var_f, p.var_a + 2.0 * sample.f0 * p.cov_ab + sample.f0 * sample.f0 * p.var_b,
         "f covariance transformation");
    near(p.var_g, sample.g0 * sample.g0 * p.var_b, "g covariance transformation");
    near(p.cov_fg, sample.g0 * p.cov_ab + sample.f0 * sample.g0 * p.var_b,
         "f-g covariance transformation");

    for (double u : {-1.0, 0.0, 1.0, 2.0}) {
        const double query_h0 = sample.f0 + sample.g0 * u;
        const double kq = ka + query_h0 * hkb;
        const double prior_variance = ka + query_h0 * query_h0 * config.variance_b;
        const auto r = p.response(u);
        require(r.valid, "queried response valid");
        near(r.mean, query_h0 + kq * residual / gram, "response posterior scalar oracle");
        near(r.mean, p.f + p.g * u, "posterior response identity");
        near(r.variance, prior_variance - kq * kq / gram, "response variance scalar oracle");
        near(r.variance, p.var_f + 2.0 * u * p.cov_fg + u * u * p.var_g,
             "response variance includes cross covariance");
        near(r.historical_error, std::abs(kq / gram) * sample.error_bound,
             "Eq. (40) absolute historical weight");
        near(r.error_bound, config.rkhs_bound * r.sigma + r.historical_error, "full Eq. (40) bound");
    }
}

void testGateTimestampsWindowAndCopy() {
    uadl::GPConfig config;
    config.rkhs_bound = 0.1;
    config.max_samples = 2;
    uadl::OnlineGP gp(config);
    uadl::GPSample sample;
    sample.timestamp = 10.0;
    sample.y = 0.3;
    const auto rejected = gp.insert(sample);
    require(!rejected.accepted && rejected.status == uadl::GPStatus::ResidualRejected,
            "outlier rejected against empty pre-insertion posterior");
    near(rejected.allowed_residual, 0.1, "pre-insertion residual allowance");
    require(gp.size() == 0, "rejection leaves old window unchanged");
    sample.y = 0.0;
    require(gp.insert(sample).accepted, "valid sample after rejected proposal accepted");
    require(gp.insert(sample).status == uadl::GPStatus::NonIncreasingTimestamp, "duplicate timestamp rejected");
    sample.timestamp = 9.9;
    require(gp.insert(sample).status == uadl::GPStatus::NonIncreasingTimestamp, "backward timestamp rejected");
    sample.timestamp = 10.005;
    require(gp.insert(sample).status == uadl::GPStatus::RateLimited, "200 Hz proposal rejected");
    sample.timestamp = 10.01;
    require(gp.insert(sample).accepted, "100 Hz insertion accepted");

    const auto snapshot = gp.predict(sample.state, 0.0, 1.0);
    const auto snapshot_response = snapshot.response(0.2);
    uadl::OnlineGP candidate = gp;
    sample.timestamp = 10.02;
    sample.state(0) = 0.3;
    require(candidate.insert(sample).accepted, "candidate copy can insert");
    require(candidate.size() == 2 && gp.size() == 2, "window capacity respected");
    near(gp.predict(uadl::State::Zero(), 0.0, 1.0).var_a, snapshot.var_a,
         "candidate mutation does not change original factorization");
    gp.reset();
    require(gp.size() == 0 && gp.valid(), "reset clears window and preserves configuration");
    near(snapshot.response(0.2).error_bound, snapshot_response.error_bound,
         "prediction owns its bound data across GP reset");
    near(gp.predict(uadl::State::Zero(), 0.0, 1.0).var_a, config.variance_a,
         "reset restores prior variance");
    sample.timestamp = 0.0;
    require(gp.insert(sample).accepted, "reset permits a new timestamp epoch");
}

void testFullStateAndRawGain() {
    uadl::GPConfig config;
    config.rkhs_bound = 10.0;
    uadl::OnlineGP gp(config);
    uadl::GPSample sample;
    sample.y = 1.0;
    sample.timestamp = 1.0;
    require(gp.insert(sample).accepted, "state sensitivity sample accepted");
    const double at_origin = gp.predict(sample.state, 0.0, 1.0).a;
    for (int i = 0; i < 6; ++i) {
        uadl::State query = uadl::State::Zero();
        query(i) = config.lengthscale;
        near(gp.predict(query, 0.0, 1.0).a, at_origin * std::exp(-0.5),
             "all six state coordinates enter kernel");
    }

    gp.reset();
    sample.u_ex = 2.0;
    sample.y = -4.0;
    require(gp.insert(sample).accepted, "finite gain-sign-changing observation accepted");
    const auto prediction = gp.predict(sample.state, 0.0, 1.0);
    require(prediction.valid && prediction.g < 0.0, "raw negative posterior gain retained");
    require(prediction.response(2.0).valid, "raw negative gain still yields valid GP response");
}

void testPredictedSetBoundAndValidation() {
    uadl::GPConfig config;
    config.rkhs_bound = 3.0;
    uadl::OnlineGP gp(config);
    for (int i = 0; i < 5; ++i) {
        uadl::GPSample sample;
        sample.state(i) = 0.12 * i;
        sample.f0 = -0.5 + 0.2 * i;
        sample.g0 = 0.7 + 0.1 * i;
        sample.u_ex = -1.2 + 0.5 * i;
        sample.y = sample.f0 + sample.g0 * sample.u_ex + 0.02 * (i + 1);
        sample.error_bound = 0.04;
        sample.timestamp = 1.0 + 0.02 * i;
        require(gp.insert(sample).accepted, "set-bound fixture insertion accepted");
    }
    // Eq. (40) over a predicted state set and an input interval dominates
    // every pointwise bound inside the set: at a fixed state the bound is
    // convex in h0 = f0 + g0 u, so the interval endpoints attain the maximum.
    std::vector<uadl::State> states;
    std::vector<double> f0, g0;
    for (int i = 0; i < 7; ++i) {
        uadl::State state;
        for (int j = 0; j < 6; ++j) state(j) = 0.2 * std::sin(0.37 * i + 0.29 * j);
        states.push_back(state);
        f0.push_back(0.8 * std::sin(0.31 * i));
        g0.push_back(1.0 + 0.5 * std::cos(0.47 * i));
    }
    const auto bound = gp.predictedSetBound(states, f0, g0, -1.5, 2.0);
    require(bound.valid, "predicted-set bound valid");
    for (std::size_t i = 0; i < states.size(); ++i) {
        const auto prediction = gp.predict(states[i], f0[i], g0[i]);
        require(prediction.valid, "set query posterior valid");
        for (int k = 0; k <= 20; ++k) {
            const double u = -1.5 + 3.5 * k / 20.0;
            const auto response = prediction.response(u);
            require(response.valid && response.error_bound <= bound.error_bound + 1e-10,
                    "predicted-set Eq. (40) bound dominates interior inputs");
        }
    }
    require(!gp.predictedSetBound(states, f0, g0, 2.0, -1.5).valid, "reversed input interval rejected");
    require(!gp.predictedSetBound({}, {}, {}, -1.0, 1.0).valid, "empty predicted set rejected");

    const double nan = std::numeric_limits<double>::quiet_NaN();
    require(!gp.predict(uadl::State::Constant(nan), 0.0, 1.0).valid, "NaN state rejected");
    require(gp.predict(uadl::State::Zero(), 0.0, -1.0).status == uadl::GPStatus::InvalidPrior,
            "nonpositive offline prior gain rejected");
    require(!gp.predict(uadl::State::Zero(), 0.0, 0.5e-6).valid, "raw gain below 1e-6 rejected (Remark 3)");
    require(!gp.response(uadl::State::Zero(), 0.0, 1.0, nan).valid, "NaN query input rejected");

    uadl::GPSample invalid;
    invalid.timestamp = 3.0;
    invalid.error_bound = -0.1;
    const auto count_before = gp.size();
    require(gp.insert(invalid).status == uadl::GPStatus::InvalidObservation && gp.size() == count_before,
            "invalid error bound leaves estimator unchanged");
    config.lengthscale = 0.0;
    uadl::OnlineGP invalid_gp(config);
    require(!invalid_gp.valid() && !invalid_gp.predict(uadl::State::Zero(), 0.0, 1.0).valid,
            "invalid kernel configuration rejected");
    config.lengthscale = 0.5;
    config.minimum_sample_interval = 0.005;
    require(!uadl::OnlineGP(config).valid(), "configuration cannot exceed 100 Hz");
}

void testContinuousRegionBound() {
    uadl::GPConfig config;
    config.rkhs_bound = 4.0;
    config.noise_variance = 0.2;
    uadl::OnlineGP gp(config);
    uadl::GPSample sample;
    sample.f0 = 0.4;
    sample.g0 = 1.2;
    sample.u_ex = -0.7;
    sample.y = -0.31;
    sample.error_bound = 0.12;
    sample.timestamp = 1.0;
    require(gp.insert(sample).accepted, "continuous-region observation accepted");

    // A degenerate box reduces to the independent scalar posterior oracle
    // exercised above; interval solving must preserve this limiting case.
    uadl::GPStateRegion point;
    point.f0_lower = point.f0_upper = sample.f0;
    point.g0_lower = point.g0_upper = sample.g0;
    const auto region_point = gp.predictedRegionBound(point, -0.9, 0.8);
    const auto discrete_point = gp.predictedSetBound({sample.state}, {sample.f0}, {sample.g0}, -0.9, 0.8);
    require(region_point.valid && discrete_point.valid, "degenerate region has a valid bound");
    near(region_point.error_bound, discrete_point.error_bound, "degenerate region matches Eq. (40)");
    near(region_point.f_lower, discrete_point.f_lower, "degenerate drift lower bound");
    near(region_point.f_upper, discrete_point.f_upper, "degenerate drift upper bound");
    near(region_point.g_lower, discrete_point.g_lower, "degenerate gain lower bound");
    near(region_point.g_upper, discrete_point.g_upper, "degenerate gain upper bound");

    // The prior varies over this entire box. Interior states and inputs are
    // not used by the bounding implementation, which operates analytically
    // on the box and the two input endpoints.
    uadl::GPStateRegion region;
    region.state_lower = uadl::State::Constant(-0.25);
    region.state_upper = uadl::State::Constant(0.3);
    region.f0_lower = 0.1;
    region.f0_upper = 0.7;
    region.g0_lower = 0.9;
    region.g0_upper = 1.4;
    const auto bound = gp.predictedRegionBound(region, -1.1, 0.8);
    require(bound.valid, "continuous state-input box bound valid");
    for (int i = 0; i <= 20; ++i) {
        uadl::State state;
        for (int j = 0; j < 6; ++j)
            state(j) = -0.25 + 0.55 * (0.5 + 0.5 * std::sin(0.37 * i + 0.29 * j));
        const double f0 = 0.4 + 0.3 * std::sin(0.47 * i);
        const double g0 = 1.15 + 0.25 * std::cos(0.53 * i);
        const auto prediction = gp.predict(state, f0, g0);
        require(prediction.valid && prediction.f >= bound.f_lower - 1e-10 &&
                prediction.f <= bound.f_upper + 1e-10 &&
                prediction.g >= bound.g_lower - 1e-10 &&
                prediction.g <= bound.g_upper + 1e-10, "raw posterior enclosed throughout state box");
        for (int j = 0; j <= 10; ++j) {
            const auto response = prediction.response(-1.1 + 1.9 * j / 10.0);
            require(response.valid && response.sigma <= bound.sigma_bound + 1e-10 &&
                    response.historical_error <= bound.historical_error + 1e-10 &&
                    response.error_bound <= bound.error_bound + 1e-10,
                    "continuous region encloses pointwise uncertainty and historical errors");
        }
    }
    region.state_lower(0) = region.state_upper(0) + 1.0;
    require(!gp.predictedRegionBound(region, -1.0, 1.0).valid, "reversed state region rejected");
    region.state_lower(0) = -0.25;
    region.g0_lower = -0.1;
    require(gp.predictedRegionBound(region, -1.0, 1.0).status == uadl::GPStatus::InvalidPrior,
            "region crossing invalid raw prior gain rejected");
}

}  // namespace

int main() {
    testEmptyPosteriorAndBounds();
    testAnalyticSingleObservation();
    testGateTimestampsWindowAndCopy();
    testFullStateAndRawGain();
    testPredictedSetBoundAndValidation();
    testContinuousRegionBound();
    std::cout << "All online GP tests passed.\n";
    return EXIT_SUCCESS;
}
