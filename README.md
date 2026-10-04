# Uncertainty-Aware Dynamics Learning

ROS/catkin workspace for learning-based quadrotor control with offline
generative priors, online Bayesian refinement and robust tube MPC.

**Keywords:** generative adversarial networks · affine feedback-linearizing
policy · finite-difference policy gradients · Gaussian process regression ·
composite task-error kernel · RKHS error bounds · interval bound propagation ·
protected dynamic inversion · homothetic tube MPC · robust positive invariance ·
recursive feasibility · backup control · PX4 offboard control · UWB/EKF2
localization

## Method-to-code correspondence

| Method component | Code |
| --- | --- |
| Section IV-A, Eq. (10): affine policy | `utils/offline_GANs_Train.py::AffinePolicy` |
| Eqs. (12)–(14): paired samples, relative objective, discriminator ascent and measured-response finite-difference generator descent | `loss_per_sample`, `fd_policy_step`, `PhysicalResponseBackend` |
| Section VI-B: real response acquisition and command reconstruction | `utils/physical_fd_collector.py`, `utils/export_flight_records.py` |
| Eq. (15), Remark 3: frozen prior and raw gain acceptance | `ExportedPrior`, `check_raw_gain`, `Controller::priors` |
| Section IV-B, Eqs. (16)–(21): task errors, composite kernel and observation covariance | `src/px4ctrl/src/online_gp.{h,cpp}` |
| Eqs. (22)–(27): posterior means, variances and cross-covariances | `OnlineGP::predict`, `GPPrediction::response` |
| Eq. (28): positive protected denominator | `Controller::evaluate` |
| Assumption 4, Eqs. (39)–(41): RKHS and historical-error bounds | `Controller::historicalError`, `GPPrediction::response` |
| Eq. (46): uniform residual envelope | `ExportedPrior::interval_bounds`, `OnlineGP::predictedRegionBound`, `Controller::targetEnvelopes` |
| Section V, Eqs. (45), (47)–(50): ancillary feedback, homothetic tube, OCP and terminal conditions | `src/px4ctrl/src/tube_mpc.{h,cpp}` |
| Finite-sum invariant tube, DARE gains and support-function tightening | `TubeMPC::build_base_tube`, `TubeMPC::dare` |
| Remark 8: accepted updates, tracking slacks, hard safety/input/terminal constraints and backup | `TubeMPC::solve`, `context_valid`, `transition_valid` |
| Remark 8: compatible references with terminal continuation | `src/px4ctrl/src/reference_continuation.h` |
| Shared solver cutoff and independent backup publication | `Controller::evaluate`, `beginCycle`, `commitPublished`, `px4ctrl_node.cpp` |
| Sections VI-B/VI-C: physical command and IMU response coordinates | `src/px4ctrl/src/control_geometry.{h,cpp}` |

## Offline generative prior

Each axis has a `6 → 64 → 2` policy with one Tanh hidden layer and two linear
output heads. Its input is `[px, py, pz, vx, vy, vz]`; normalization is fitted
on the training block and embedded in the exported model. The discriminator
has `2 → 64 → 1` layers, with Tanh only in its hidden layer.

The policy and training pairs are

```text
u(x, eta) = Theta1(x) + Theta2(x) eta
real       = (eta, eta)
generated  = (eta, measured h(x, u))
J          = mean((D(real) - D(generated) - tau)^2)
```

The discriminator ascends the written objective. The generator descends it,
using the measured central difference
`[h(x,u+epsilon)-h(x,u-epsilon)]/(2 epsilon)` and the exact derivative of
its action with respect to its parameters. All three responses are acquired
on the physical system. Requests and receipts carry full executed commands,
state and sensor timestamps, probe order, environment and unique identifiers.
Clipped probes, mismatched states, stale measurements and invalid receipts
are rejected. Recorded flight data supplies training states and context;
it does not substitute an interpolated response for a new physical measurement.

The fixed method settings are Adam learning rate `2e-4`, `beta1=0.5`,
batch size `1024`, `2000` epochs and score margin `tau=0.5`. The perturbation
step and virtual-input range are explicit acquisition inputs. Training data
uses fixed chronological blocks with boundary exclusions.
Mean/scale computation is part of network preprocessing.

Export uses `f0=-Theta1/Theta2`, `g0=1/Theta2`. Nonfinite values or raw gains
below `1e-6` are rejected without clipping. Exported TorchScript files expose
`forward(float32[N,6]) → (f0[N,1],g0[N,1])` and an `interval_bounds` method
for the same frozen network. No discriminator is deployed online.
The complete acquisition protocol and command-line interfaces are in
[`models/README.md`](src/px4ctrl/models/README.md).

## Online model and complete residual bound

The fixed prior defines `h0=f0+g0*u_ex`. Independent latent GP priors are assigned
to `a=f+g*Theta1` and `b=g*Theta2-1`, inducing

```text
q = a + h0*b
K_ab = K_a + H0*K_b*H0' + Sigma_e
r = measured_acceleration - h0
f_hat = f0 + a_hat + f0*b_hat
g_hat = g0*(1+b_hat)
```

The implementation retains `cov(a,b)` and the induced `cov(f,g)`. Response
variance includes the cross term, `sigma_f² + 2*u_ex*cov(f,g) + u_ex²*sigma_g²`.
The online gain floor does not modify this posterior.

Each axis retains at most 50 samples. The state timestamp must advance, and
insertion is limited to 100 Hz. Labels use the synchronized IMU specific force
rotated with odometry into the inertial frame, minus gravity. Each label is
paired with its preceding executed command, including the configured actuation
delay. The retained posterior checks a measurement before insertion. A failed
label or model update leaves the accepted model available for control.

The GP bound is `E_q = B*sigma_q + sum(abs(w_j)*e_bar_j)`, with
`w=K_ab^-1*k_q` and
`e_bar_j=d_bar+epsilon_bar+(L_f+abs(u_j)*L_g)*state_error+sync_error`.
The working Gaussian noise covariance and these bounded physical errors have
separate roles. Hyperparameters and the offline networks remain fixed.

Continuous-domain bounds use interval propagation through the frozen network
and analytic squared-exponential kernel intervals over the entire configured
state box and physical input interval. The controller does not treat values at
a finite number of prediction points as a uniform bound. The envelope adds
command modification, disturbance, state-error, hold and reference-discretization
terms. The mapped command is checked against the same budget through
`c=f_hat+g_hat*u_ex-eta`.

## Tube MPC and backup

Three axis problems use the sampled double integrator, `Ts=0.01 s`, `H=20`,
DARE terminal weights and ancillary feedback. The horizontal cost is
`Q=diag(10,1), R=0.1`; the vertical cost is `Q=diag(15,2), R=0.5`.
Ancillary design uses `Q_xy=diag(20,2)`, `Q_z=diag(30,5)`, `R_anc=0.01`.
The base tube uses 500 finite-sum terms and `alpha=1e-4`; its scale is the
complete residual envelope. Support functions implement the state and input
Pontryagin differences and tube-propagation checks.

The OCP chooses the initial nominal error and nominal correction sequence.
The command is `v=vbar_0+K_anc*(estimated_error-z0)`, `eta=v+reference_acceleration`,
then `u_cmd=(eta-f_hat)/max(g_hat,0.5)`. The correction-input interval is restricted
so that this inverse respects the physical command box over the full state
region and allowed references. Final attitude/thrust mapping reconstructs `u_ex`.

Only tracking-performance inequalities receive positive linear and quadratic
slacks. The terminal set, initial tube containment, physical safety and actuator
constraints are hard. State-information boxes enter containment and tightening;
future information-box propagation is checked explicitly. Bounds that cannot
support a common invariant tube or terminal region are rejected.

An accepted update must preserve tube propagation and feasibility of the shifted
sequence with the exact terminal policy `K_f*z_H`. Normal commands also preserve
their continuation. A reference continuation matches the current position,
velocity and acceleration, brakes within the acceleration limits, and has a
stationary infinite tail. Analytic curve bounds check that the terminal tube
remains inside the physical domain. Equal vehicle velocities/accelerations
preserve a constant formation offset during this continuation.

The controller prepares a retained-model backup before solving a candidate.
QP work runs on an isolated snapshot; the publication thread stops waiting at
8.50 ms from the scheduled release and can publish the backup independently of
an overdue solve. All axes share the cutoff, and late results are discarded.
Models, plans and executed-input history are committed only after publication.
The publication deadline is 10 ms. Recovery requires renewed backup eligibility.

## Workspace and configuration

- `src/px4ctrl`: the method, flight state machine, parameters and vehicle launch.
- `src/quadrotor_msgs`: ROS message definitions, including controller telemetry.
- `src/nlink_parser`, `src/uwb_bridge`: UWB input and localization bridge.
- `src/uav_utils`, `src/cmake_utils`: geometry and workspace support.
- `utils`: physical acquisition, training-record preparation and prior training.
- `src/px4ctrl/tests`, `utils/tests`: unit tests.

[`ctrl_param_fpv.yaml`](src/px4ctrl/config/ctrl_param_fpv.yaml) holds the controller
settings, the operating domain and the error-budget inputs used by the interval
and feasibility checks. Training produces the three model files.

`run_ctrl.launch` selects the vehicle namespace and the three frozen prior paths.
The controller consumes `position_cmd`, MAVROS odometry and IMU data, and publishes
MAVROS attitude/thrust commands. `debugPx4ctrl` publishes the current controller state. `reset_online` clears the
retained observations, plans and command history while preserving the frozen
networks and kernels.
