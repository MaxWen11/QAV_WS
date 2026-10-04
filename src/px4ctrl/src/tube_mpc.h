#ifndef UADL_TUBE_MPC_H
#define UADL_TUBE_MPC_H

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace uadl {

struct MPCConfig {
    double dt = 0.01;
    int horizon = 20;
    Eigen::Vector2d Q_diag = Eigen::Vector2d(10.0, 1.0);
    double R = 0.1;
    Eigen::Vector2d Q_anc_diag = Eigen::Vector2d(20.0, 2.0);
    double R_anc = 0.01;
    // Tracking-error limits E (Section VI-C): position and velocity error.
    Eigen::Vector2d state_limits = Eigen::Vector2d(2.0, 2.0);
    // Largest residual envelope accepted by the common terminal set. The
    // terminal set and constraint tightening are fixed for this value.
    double max_envelope = 0.0;
    // Smallest admissible residual envelope.
    double min_envelope = 0.0;
    // Common correction-input set V (Section V), shared by all model updates.
    Eigen::Vector2d correction_domain = Eigen::Vector2d(-3.0, 3.0);
    // Common bound used when building the fixed terminal set; individual
    // state-information boxes supplied to solve must not exceed it.
    Eigen::Vector2d max_estimation_error = Eigen::Vector2d::Zero();
    // Base set Z0 of the homothetic tube, Eq. (47), for a unit residual
    // envelope: the finite-sum invariant-set approximation of Rakovic et al.
    // (2005), Z0 = c * (W0 (+) A_K W0 (+) ... (+) A_K^(s-1) W0), with the unit
    // disturbance box W0 = {|w_p| <= T_s^2/2, |w_v| <= T_s} of Eq. (49) and
    // A_K^s W0 contained in alpha*W0.
    int tube_terms = 500;        // s
    double tube_alpha = 1.0e-4;  // alpha
    // lambda in A_K Z0 (+) W0 <= lambda Z0. lambda = 1 gives c = 1/(1-alpha),
    // the RPI finite-sum set; lambda < 1 scales the same set by the smallest
    // c that makes it lambda-contractive, so that the envelope may decrease by
    // lambda per sample while Remark 8(i) holds.
    double tube_contraction = 0.999;
    // Positive linear and quadratic slack penalties of the soft
    // tracking-performance constraints, Eq. (54).
    double slack_linear_weight = 1.0e3;
    double slack_quadratic_weight = 1.0e4;
    int max_working_set_recalculations = 200;
    double feasibility_tolerance = 1e-7;
};

// Hard safety limits of Remark 8 on the physical state [p, v] of one axis.
// The reference prediction holds [p_ref, v_ref] at the H+1 prediction instants;
// the nominal error z_i must satisfy lower + rho <= r_i + z_i <= upper - rho,
// where rho is the tube projection radius, so the true state stays inside.
struct SafetyLimits {
    std::vector<Eigen::Vector2d> reference;
    Eigen::Vector2d lower = Eigen::Vector2d::Constant(-1.0e9);
    Eigen::Vector2d upper = Eigen::Vector2d::Constant(1.0e9);
    // Componentwise error bound on the state estimate. This box enters the
    // initial tube constraint and the ancillary-input tightening, Remark 8.
    Eigen::Vector2d estimation_error = Eigen::Vector2d::Zero();
    // Envelope of every future [p_ref, v_ref] during backup continuation,
    // including the constant terminal tail of the compatible reference.
    Eigen::Vector2d tail_reference_lower = Eigen::Vector2d::Zero();
    Eigen::Vector2d tail_reference_upper = Eigen::Vector2d::Zero();
    bool have_tail_bounds = false;
    bool active() const { return !reference.empty(); }
};

struct MPCResult {
    bool valid = false;
    bool used_backup = false;
    double correction = 0.0;
    double envelope = 0.0;
    Eigen::Vector2d nominal_state = Eigen::Vector2d::Zero();
    Eigen::Vector2d tube_radii = Eigen::Vector2d::Zero();
    double input_reserve = 0.0;
    // Largest relaxation of each tracking-performance constraint [position,
    // velocity]. Initial containment, terminal, safety and input remain hard.
    Eigen::Vector2d slack = Eigen::Vector2d::Zero();
    std::string status;
};

// Axis-wise homothetic tube MPC, Eqs. (45)-(48). Value semantics: an updated
// model/envelope is evaluated on a COPY and committed by the caller only after
// all axes and the mapped command are accepted. An invalid result never
// changes the stored plan.
class TubeMPC {
public:
    explicit TubeMPC(const MPCConfig& config);
    MPCResult solve(const Eigen::Vector2d& error, double envelope,
                    const Eigen::Vector2d& correction_bounds,
                    const SafetyLimits& safety, double time_budget_seconds);
    MPCResult solve(const Eigen::Vector2d& error, double envelope,
                    const Eigen::Vector2d& correction_bounds,
                    double time_budget_seconds) {
        return solve(error, envelope, correction_bounds, SafetyLimits(), time_budget_seconds);
    }
    void reset();
    bool configured() const { return configured_; }
    bool has_backup() const { return has_plan_; }
    const std::string& configuration_status() const { return config_status_; }
    const Eigen::Matrix2d& A() const { return A_; }
    const Eigen::Vector2d& B() const { return B_; }
    const Eigen::RowVector2d& ancillary_gain() const { return K_; }
    const Eigen::RowVector2d& terminal_gain() const { return Kf_; }
    const Eigen::Matrix2d& terminal_weight() const { return P_; }
    // rho = sup |[1 0] delta| and sup |[0 1] delta| over the tube Z_kappa.
    Eigen::Vector2d tube_radii(double envelope) const;
    // r_v = sup |K_anc delta| over the tube Z_kappa.
    double input_reserve(double envelope) const;
    int tube_face_count() const { return static_cast<int>(tube_.normals.rows()); }
    // Scale c of the base set and alpha_req = min{a : A_K^s W0 <= a W0}.
    double tube_scale() const { return tube_.scale; }
    double tube_alpha_required() const { return tube_.alpha_required; }
    bool tube_contains(const Eigen::Vector2d& delta, double envelope) const;
    bool terminal_contains(const Eigen::Vector2d& state) const;
    // Envelope floor for both transition containment and continued backup
    // under the common envelope. Includes the next estimator-information box.
    double min_next_envelope() const;
    double committed_envelope() const { return has_plan_ ? envelope_ : 0.0; }

private:
    // Centrally symmetric zonotope c * sum_k [-g_k, g_k] with its exact
    // half-space representation n_i' x <= offset_i (unit envelope).
    struct Zonotope {
        std::vector<Eigen::Vector2d> generators;
        Eigen::MatrixXd normals;
        Eigen::VectorXd offsets;
        double scale = 1.0;
        double alpha_required = 0.0;
        double support(const Eigen::Vector2d& direction) const;
        bool contains(const Eigen::Vector2d& point, double envelope, double tolerance) const;
    };
    struct Polytope {
        Eigen::MatrixXd normals;
        std::vector<Eigen::Vector2d> vertices;
        double radius = 0.0;
        double support(const Eigen::Vector2d& direction) const;
        bool contains(const Eigen::Vector2d& point, double scale,
                      double tolerance) const;
    };
    struct Plan {
        std::vector<Eigen::Vector2d> states;
        std::vector<double> inputs;
        // Per-instant tracking-performance slacks [position, velocity].
        std::vector<Eigen::Vector2d> slack;
    };

    MPCConfig config_;
    Eigen::Matrix2d A_ = Eigen::Matrix2d::Identity();
    Eigen::Vector2d B_ = Eigen::Vector2d::Zero();
    Eigen::Matrix2d P_ = Eigen::Matrix2d::Identity();
    Eigen::RowVector2d K_ = Eigen::RowVector2d::Zero();
    Eigen::RowVector2d Kf_ = Eigen::RowVector2d::Zero();
    Zonotope tube_;
    Polytope terminal_;
    Eigen::Vector2d unit_tube_radii_ = Eigen::Vector2d::Zero();
    double unit_input_reserve_ = 0.0;
    Eigen::VectorXd propagated_face_support_, disturbance_face_support_;
    Eigen::VectorXd information_face_support_;
    double minimum_information_envelope_ = 0.0;
    bool configured_ = false;
    std::string config_status_;
    bool has_plan_ = false;
    Plan plan_;
    double envelope_ = 0.0;

    static bool dare(const Eigen::Matrix2d& A, const Eigen::Vector2d& B,
                     const Eigen::Matrix2d& Q, double R,
                     Eigen::Matrix2d& P, Eigen::RowVector2d& K);
    static bool contracting_polytope(const Eigen::Matrix2d& closed_loop,
                                     const Eigen::Matrix2d& metric,
                                     Polytope& result);
    bool build_base_tube(const Eigen::Matrix2d& closed_loop);
    bool information_propagation_valid(double current_envelope, double next_envelope) const;
    bool transition_valid(double next_envelope) const;
    double input_reserve(double envelope, const SafetyLimits& safety) const;
    bool context_valid(double envelope, const Eigen::Vector2d& bounds,
                       const SafetyLimits& safety) const;
    Plan shifted(const Plan& plan) const;
    void assign_slack(Plan& plan, double envelope) const;
    bool plan_valid(const Plan& plan, const Eigen::Vector2d& error, double envelope,
                    const Eigen::Vector2d& bounds, const SafetyLimits& safety) const;
    MPCResult make_result(const Plan& plan, const Eigen::Vector2d& error,
                          double envelope, bool backup,
                          const SafetyLimits& safety, const std::string& status) const;
};

} // namespace uadl
#endif
