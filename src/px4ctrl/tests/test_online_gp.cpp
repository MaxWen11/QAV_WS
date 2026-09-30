#include "online_gp.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <string>

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
    near(response.error_bound, config.rkhs_bound * response.sigma, "empty Eq.34 bound");
    near(response.historical_error, 0.0, "empty historical contamination");
    const auto bound = gp.globalBound(0.7, 1.3, 0.4);
    require(bound.valid && bound.error_bound >= response.error_bound, "empty uniform bound covers response");
    const auto model = gp.modelBounds(0.7, 0.9, 1.3);
    require(model.valid, "empty posterior model envelope");
    near(model.abs_f, 0.7, "empty f envelope");
    near(model.g_min, 0.9, "empty lower g envelope");
    near(model.g_max, 1.3, "empty upper g envelope");
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
             "Eq.34 absolute historical weight");
        near(r.error_bound, config.rkhs_bound * r.sigma + r.historical_error, "full Eq.34 bound");
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
    const auto model = gp.modelBounds(0.2, 0.8, 1.2);
    require(model.valid && model.g_min < 0.0 && model.g_min <= prediction.g,
            "global gain envelope is not floored");
}

void testUniformEnvelopesAndValidation() {
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
        require(gp.insert(sample).accepted, "envelope fixture insertion accepted");
    }
    const auto bound = gp.globalBound(0.8, 1.5, 2.0);
    const auto model = gp.modelBounds(0.8, 0.5, 1.5);
    require(bound.valid && model.valid, "uniform envelopes valid");
    for (int i = 0; i < 21; ++i) {
        uadl::State state;
        for (int j = 0; j < 6; ++j) state(j) = 0.2 * std::sin(0.37 * i + 0.29 * j);
        const double f0 = 0.8 * std::sin(0.31 * i);
        const double g0 = 1.0 + 0.5 * std::cos(0.47 * i);
        const auto prediction = gp.predict(state, f0, g0);
        require(prediction.valid, "envelope query posterior valid");
        require(std::abs(prediction.f) <= model.abs_f + 1e-10 &&
                prediction.g >= model.g_min - 1e-10 && prediction.g <= model.g_max + 1e-10,
                "uniform model envelope covers varying prior values");
        for (double u : {-2.0, -1.0, 0.0, 1.0, 2.0}) {
            const auto response = prediction.response(u);
            require(response.valid && response.error_bound <= bound.error_bound + 1e-10,
                    "uniform Eq.34 envelope dominates query bounds");
        }
    }
    // A distant query keeps prior uncertainty: the uniform envelope cannot
    // be replaced by the much smaller posterior variance at training data.
    const auto far = gp.response(uadl::State::Constant(100.0), 0.8, 1.5, 2.0);
    require(far.valid && far.error_bound <= bound.error_bound, "uniform bound covers unseen states");

    const double nan = std::numeric_limits<double>::quiet_NaN();
    require(!gp.predict(uadl::State::Constant(nan), 0.0, 1.0).valid, "NaN state rejected");
    require(gp.predict(uadl::State::Zero(), 0.0, -1.0).status == uadl::GPStatus::InvalidPrior,
            "nonpositive offline prior gain rejected");
    require(!gp.predict(uadl::State::Zero(), 0.0, 0.5e-6).valid, "near-singular offline prior rejected");
    require(!gp.response(uadl::State::Zero(), 0.0, 1.0, nan).valid, "NaN query input rejected");
    require(!gp.globalBound(-1.0, 1.0, 1.0).valid, "negative global absolute bound rejected");
    require(!gp.modelBounds(1.0, 2.0, 1.0).valid, "reversed prior interval rejected");

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

void testCertifiedDomainCovering() {
    uadl::GPConfig config;
    config.variance_a = 1.3;
    config.variance_b = 0.7;
    config.noise_variance = 0.01;
    config.rkhs_bound = 2.0;
    uadl::OnlineGP gp(config);
    const uadl::State lower = uadl::State::Constant(-0.01);
    const uadl::State upper = uadl::State::Constant(0.01);
    const double abs_f = 0.2, abs_g = 1.1, abs_u = 2.0;
    const double H = abs_f + abs_g * abs_u;
    const auto empty = gp.domainBound(lower, upper, abs_f, abs_g, abs_u, 8);
    near(empty.error_bound, gp.globalBound(abs_f, abs_g, abs_u).error_bound,
         "empty domain bound equals exact prior envelope");
    require(empty.valid, "empty domain certificate valid");

    std::mt19937 random(1729);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    std::vector<uadl::GPSample> samples;
    for (int i = 0; i < 16; ++i) {
        uadl::GPSample sample;
        for (int j = 0; j < 6; ++j) sample.state(j) = 0.009 * unit(random);
        sample.f0 = abs_f;
        sample.g0 = abs_g;
        sample.u_ex = i % 2 ? abs_u : -abs_u;
        sample.y = sample.f0 + sample.g0 * sample.u_ex;
        sample.error_bound = 0.0001 * (1 + i % 3);
        sample.timestamp = 1.0 + 0.02 * i;
        require(gp.insert(sample).accepted, "domain fixture observation accepted");
        samples.push_back(sample);
    }
    const auto certified = gp.domainBound(lower, upper, abs_f, abs_g, abs_u, 8);
    require(certified.valid && certified.error_bound < 0.8 * empty.error_bound,
            "covered small domain admits genuine contraction below empty-window bound");
    require(certified.error_bound <= gp.globalBound(abs_f, abs_g, abs_u).error_bound,
            "domain certificate never exceeds global certificate");

    // Independent dense Gaussian conditioning oracle (not predict()) checks
    // thousands of random state/h0 queries plus every state-box corner.
    const Eigen::Index count = static_cast<Eigen::Index>(samples.size());
    Eigen::MatrixXd gram(count, count);
    Eigen::VectorXd hi(count), errors(count);
    for (Eigen::Index i = 0; i < count; ++i) {
        hi(i) = samples[i].f0 + samples[i].g0 * samples[i].u_ex;
        errors(i) = samples[i].error_bound;
    }
    for (Eigen::Index i = 0; i < count; ++i)
        for (Eigen::Index j = 0; j < count; ++j) {
            const double distance = (samples[i].state - samples[j].state).squaredNorm();
            gram(i, j) = (config.variance_a + hi(i) * hi(j) * config.variance_b) *
                         std::exp(-distance / (2 * config.lengthscale * config.lengthscale));
            if (i == j) gram(i, j) += config.noise_variance;
        }
    const Eigen::LDLT<Eigen::MatrixXd> independent_factor(gram);
    auto oracle = [&](const uadl::State& state, double h) {
        Eigen::VectorXd k(count);
        for (Eigen::Index i = 0; i < count; ++i)
            k(i) = (config.variance_a + h * hi(i) * config.variance_b) *
                   std::exp(-(samples[i].state - state).squaredNorm() /
                            (2 * config.lengthscale * config.lengthscale));
        const Eigen::VectorXd w = independent_factor.solve(k);
        const double variance = config.variance_a + h*h*config.variance_b - k.dot(w);
        return config.rkhs_bound * std::sqrt(std::max(0.0, variance)) + w.cwiseAbs().dot(errors);
    };
    for (int i = 0; i < 2000; ++i) {
        uadl::State state;
        for (int j = 0; j < 6; ++j) state(j) = 0.01 * unit(random);
        const double h = H * unit(random);
        require(oracle(state, h) <= certified.error_bound + 1e-9,
                "cell-cover bound dominates independent random-query Eq.34 oracle");
    }
    for (int mask = 0; mask < 64; ++mask) {
        uadl::State state;
        for (int j = 0; j < 6; ++j) state(j) = (mask & (1 << j)) ? upper(j) : lower(j);
        for (double h : {-H, 0.0, H})
            require(oracle(state, h) <= certified.error_bound + 1e-9,
                    "cell-cover bound includes all box corners and h0 extremes");
    }
    const auto singleton = gp.domainBound(uadl::State::Zero(), uadl::State::Zero(), abs_f, abs_g, abs_u, 8);
    near(singleton.error_bound, std::max(oracle(uadl::State::Zero(), -H), oracle(uadl::State::Zero(), H)),
         "zero-radius covering reduces to convex h0 endpoint bound", 1e-7);
    const auto wide = gp.domainBound(uadl::State::Constant(-100.), uadl::State::Constant(100.), abs_f, abs_g, abs_u, 1);
    near(wide.error_bound, gp.globalBound(abs_f, abs_g, abs_u).error_bound,
         "broad unseen domain safely falls back to global bound");
    require(!gp.domainBound(upper, lower, abs_f, abs_g, abs_u).valid, "reversed box rejected");
    require(!gp.domainBound(lower, upper, abs_f, abs_g, abs_u, 0).valid, "zero covering budget rejected");
    require(!gp.domainBound(lower, upper, abs_f, abs_g, abs_u, 257).valid, "excessive covering budget rejected");
    require(!gp.domainBound(uadl::State::Constant(std::numeric_limits<double>::quiet_NaN()), upper,
                            abs_f, abs_g, abs_u).valid, "nonfinite covering domain rejected");
    std::cout << "Certified tight-domain bound: " << empty.error_bound << " -> " << certified.error_bound << '\n';
}

}  // namespace

int main() {
    testEmptyPosteriorAndBounds();
    testAnalyticSingleObservation();
    testGateTimestampsWindowAndCopy();
    testFullStateAndRawGain();
    testUniformEnvelopesAndValidation();
    testCertifiedDomainCovering();
    std::cout << "All online GP tests passed.\n";
    return EXIT_SUCCESS;
}
