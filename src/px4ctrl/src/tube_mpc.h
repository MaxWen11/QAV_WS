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
    Eigen::Vector2d state_limits = Eigen::Vector2d(2.0, 2.0);
    // Largest residual envelope accepted by the common terminal set. The
    // terminal set and constraint tightening are fixed for this value.
    double max_envelope = 0.0;
    // Lower clamp of the envelope. Must be positive when the estimation radius
    // is positive, so the fixed estimator terms scale into the base tube.
    double min_envelope = 0.0;
    // Euclidean radius of the [position, velocity] estimation error used in
    // initial tube containment and constraint tightening (Remark 8).
    double state_estimation_bound = 0.0;
    // Common correction-input set V (Section V), shared by all model updates.
    Eigen::Vector2d correction_domain = Eigen::Vector2d(-3.0, 3.0);
    // Base tube contraction: A_K*Z0 (+) W0 <= lambda*Z0. The envelope may
    // decrease by this factor per sample while Remark 8(i) holds.
    double tube_contraction = 0.999;
    // Face budget of the base tube polygon; bounds the QP size.
    int max_tube_faces = 64;
    // Exact-penalty weights of the soft tracking constraints (Remark 9).
    double slack_linear_weight = 1.0e3;
    double slack_quadratic_weight = 1.0e4;
    int max_working_set_recalculations = 200;
    double feasibility_tolerance = 1e-7;
};

struct MPCResult {
    bool valid = false;
    bool used_backup = false;
    double correction = 0.0;
    double envelope = 0.0;
    Eigen::Vector2d nominal_state = Eigen::Vector2d::Zero();
    Eigen::Vector2d tube_radii = Eigen::Vector2d::Zero();
    double input_reserve = 0.0;
    // Soft-constraint relaxation: position, velocity and terminal slack.
    Eigen::Vector3d slack = Eigen::Vector3d::Zero();
    std::string status;
};

// Axis-wise homothetic tube MPC, Eqs. (43)-(46). Value semantics: an updated
// model/envelope is evaluated on a COPY and committed by the caller only after
// all axes and the mapped command are accepted. An invalid result never
// changes the stored plan.
class TubeMPC {
public:
    explicit TubeMPC(const MPCConfig& config);
    MPCResult solve(const Eigen::Vector2d& error, double envelope,
                    const Eigen::Vector2d& correction_bounds,
                    double time_budget_seconds);
    void reset();
    bool configured() const { return configured_; }
    bool has_backup() const { return has_plan_; }
    const std::string& configuration_status() const { return config_status_; }
    const Eigen::Matrix2d& A() const { return A_; }
    const Eigen::Vector2d& B() const { return B_; }
    const Eigen::RowVector2d& ancillary_gain() const { return K_; }
    const Eigen::RowVector2d& terminal_gain() const { return Kf_; }
    const Eigen::Matrix2d& terminal_weight() const { return P_; }
    Eigen::Vector2d tube_radii(double envelope) const;
    double input_reserve(double envelope) const;
    int tube_face_count() const { return static_cast<int>(tube_.normals.rows()); }
    bool tube_contains(const Eigen::Vector2d& delta, double envelope) const;
    bool terminal_contains(const Eigen::Vector2d& state) const;
    // Smallest envelope that preserves A_K*Z_k (+) W_k <= Z_{k+1}.
    double min_next_envelope() const { return has_plan_ ? config_.tube_contraction * envelope_ : 0.0; }
    double committed_envelope() const { return has_plan_ ? envelope_ : 0.0; }

private:
    struct Polytope {
        Eigen::MatrixXd normals;
        std::vector<Eigen::Vector2d> vertices;
        double radius = 0.0;
        double contraction = 0.0;
        double support(const Eigen::Vector2d& direction) const;
        bool contains(const Eigen::Vector2d& point, double scale,
                      double tolerance) const;
    };
    struct Plan {
        std::vector<Eigen::Vector2d> states;
        std::vector<double> inputs;
        Eigen::Vector3d slack = Eigen::Vector3d::Zero();
    };

    MPCConfig config_;
    Eigen::Matrix2d A_ = Eigen::Matrix2d::Identity();
    Eigen::Vector2d B_ = Eigen::Vector2d::Zero();
    Eigen::Matrix2d P_ = Eigen::Matrix2d::Identity();
    Eigen::RowVector2d K_ = Eigen::RowVector2d::Zero();
    Eigen::RowVector2d Kf_ = Eigen::RowVector2d::Zero();
    Polytope tube_, terminal_;
    Eigen::Vector2d unit_tube_radii_ = Eigen::Vector2d::Zero();
    double unit_input_reserve_ = 0.0;
    Eigen::VectorXd propagated_face_support_, disturbance_face_support_;
    bool configured_ = false;
    std::string config_status_;
    bool has_plan_ = false;
    Plan plan_;
    double envelope_ = 0.0;
    Eigen::Vector2d bounds_ = Eigen::Vector2d::Zero();

    static bool dare(const Eigen::Matrix2d& A, const Eigen::Vector2d& B,
                     const Eigen::Matrix2d& Q, double R,
                     Eigen::Matrix2d& P, Eigen::RowVector2d& K);
    static bool contracting_polytope(const Eigen::Matrix2d& closed_loop,
                                     const Eigen::Matrix2d& metric,
                                     Polytope& result);
    bool build_rpi_polytope(const Eigen::Matrix2d& closed_loop,
                           const Eigen::Matrix2d& metric);
    double disturbance_support(const Eigen::Vector2d& direction) const;
    bool transition_valid(double next_envelope) const;
    bool context_valid(double envelope, const Eigen::Vector2d& bounds) const;
    Plan shifted(const Plan& plan, const Eigen::Vector2d& input_interval) const;
    Eigen::Vector3d required_slack(const Plan& plan, double envelope) const;
    bool plan_valid(const Plan& plan, const Eigen::Vector2d& error,
                    double envelope, const Eigen::Vector2d& bounds) const;
    MPCResult make_result(const Plan& plan, const Eigen::Vector2d& error,
                          double envelope, bool backup,
                          const std::string& status) const;
};

} // namespace uadl
#endif
