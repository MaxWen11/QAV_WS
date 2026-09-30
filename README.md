# UADL-QAV250: Uncertainty-Aware Dynamics Learning for Quadrotor Control

This workspace implements the uncertainty-aware dynamics learning framework of the
manuscript on a QAV250 quadrotor (PX4 Pixhawk 6C mini, Raspberry Pi 4B, UWB
localization):

1. **Offline generative prior** (Section IV-A): an affine linearizing policy
   `u = Θ1(x) + Θ2(x)·η` is trained as the generator of a relative GAN and converted
   into the prior `f0 = −Θ1/Θ2`, `g0 = 1/Θ2`.
2. **Online Bayesian refinement** (Section IV-B): a task-error GP on the policy errors
   `a, b` with kernel `k_q = k_a + h0·k_b·h0'` refines the drift and gain online and
   supplies the posterior error bound of Theorem 2.
3. **Uncertainty-aware tube MPC** (Section V): the residual envelope sizes a homothetic
   tube; three axis-wise MPC problems (qpOASES, 10 ms, H = 20) run at 100 Hz with a
   maintained feasible backup.

## Repository layout

| Path | Content |
| --- | --- |
| `src/px4ctrl/src/online_gp.{h,cpp}` | Task-error GP: posterior, covariances, Eq. (34) bounds, windowed insertion |
| `src/px4ctrl/src/tube_mpc.{h,cpp}` | Homothetic tube MPC: DARE, contractive RPI tube, terminal set, OCP, backup |
| `src/px4ctrl/src/control_geometry.{h,cpp}` | Net-acceleration ↔ attitude/thrust mapping, IMU label conversion |
| `src/px4ctrl/src/controller.{h,cpp}` | Per-cycle controller: priors, samples, envelopes, MPC, inverse, commit |
| `src/px4ctrl/src/PX4CtrlFSM.*`, `px4ctrl_node.cpp`, `input.*` | ROS node, flight state machine (takeoff, hover, command, land), inputs |
| `src/px4ctrl/config/ctrl_param_fpv.yaml` | Complete controller configuration |
| `src/px4ctrl/launch/run_ctrl.launch` | Per-vehicle launch file (namespace, configuration A–D, prior paths) |
| `src/px4ctrl/models/` | Exported TorchScript priors `generator_prior_{X,Y,Z}.pt` |
| `src/px4ctrl/tests/` | C++ unit tests and controller integration test |
| `src/quadrotor_msgs/` | `PositionCommand`, `TakeoffLand`, `Px4ctrlDebug` messages |
| `src/nlink_parser/`, `src/uwb_bridge/` | LinkTrack UWB driver and UWB/compass bridge to the PX4 EKF2 vision input |
| `utils/offline_GANs_Train.py` | Offline prior extraction (GAN and MLP–MSE baseline) |
| `utils/export_flight_records.py` | Flight logs → synchronized training records |
| `utils/Trajectory_Generation.py` | Synchronized dual-vehicle figure-eight trial and metrics |
| `utils/req_verification.m`, `utils/{x,y,z}_data.txt` | Frequency-response validation of the offline linearization (Fig. 4) |
| `utils/extras.txt` | PX4 SD-card MAVLink stream configuration (`ATTITUDE`, `HIGHRES_IMU`, `LOCAL_POSITION_NED`) |

## Method-to-code map

| Manuscript | Implementation |
| --- | --- |
| Eq. (12) affine policy, Eqs. (14)–(16) relative GAN with finite-difference generator gradient | `AffinePolicy`, `RelativeDiscriminator`, `fd_policy_step` in `offline_GANs_Train.py` |
| Eq. (17) prior extraction, Remark 3 gain check `g0 ≥ 1e-6` | `ExportedPrior`, `check_raw_gain` |
| Eqs. (18)–(21) task-error kernel and Gram matrix | `OnlineGP::rebuild` |
| Eqs. (22)–(23), `c_ab`, `c_fg`, response posterior | `OnlineGP::predict`, `GPPrediction::response` |
| Protected inverse with gain floor 0.5 | `Controller::evaluate` |
| Eq. (34) posterior error bound | `GPPrediction::response`, `OnlineGP::predictedSetBound` |
| Assumption 4 per-sample error `ē_j` | `Controller::historicalError` |
| Eq. (44) residual envelope | `Controller::targetEnvelopes`, `Controller::residualMargin` |
| Eqs. (43), (45), (46) homothetic tube OCP | `TubeMPC::solve` |
| Theorem 3 terminal conditions | `TubeMPC` constructor (DARE terminal weight, invariant terminal polygon) |
| Remark 8 tube/model updates | contractive base tube, `TubeMPC::min_next_envelope`, candidate evaluation in `Controller::update` |
| Remark 9 soft tracking constraints | slack variables in `TubeMPC::solve` |
| Remark 10 backup activation and recovery | shifted plan with terminal continuation, `Controller::continuedReference` |

## Online controller

Each 100 Hz cycle (`Controller::update`):

1. **State and label.** The state `[p, v]` comes from `mavros/local_position/odom`. The
   response label is the IMU specific force rotated by the odometry attitude plus the
   gravity vector.
2. **Sample insertion (C, D).** A sample is proposed when the state timestamp advances
   and the IMU label is synchronized (20 ms). It is paired with the command published
   before the measurement (plus `input_delay`). The prediction residual is checked
   against the retained posterior before insertion; insertion is limited to 100 Hz and
   a 50-sample window per axis. Learning runs in `AUTO_HOVER` and `CMD_CTRL` once
   OFFBOARD is confirmed.
3. **Residual envelope.** Eq. (34) is evaluated over the predicted state set (reference
   continuation over the horizon) and the input interval around the last command;
   Eq. (44) adds command modification, disturbance, state-error and hold terms. The
   envelope scales the base tube (Eq. 45) and may decrease by at most the tube
   contraction factor per cycle (Remark 8). A model update that would exceed
   `max_envelope` is rejected and the retained model is used.
4. **Backup.** The shifted plan of the retained model with a compatible reference
   continuation is prepared first (Remark 10).
5. **Tube MPC.** The three axis OCPs share an 8.5 ms solver cutoff measured from the
   cycle start. The OCP decides `z0` and the nominal corrections; the applied correction
   is `v = v̄0 + K_anc(e − z0)`. Initial tube containment and the correction-input set
   are hard constraints; tracking and terminal constraints are soft (Remark 9). After
   a reference switch the OCP is re-solved from the measured error.
6. **Inverse and mapping.** `η = v + a_ref`, `u = (η − f̂)/max(ĝ, 0.5)` per axis; `u` is
   mapped to attitude and normalized thrust and the executed input is reconstructed
   from the final command.
7. **Commit.** The command is published before the 10 ms deadline; the candidate model,
   plan and command history are committed only after publication.

Configurations (`controller/method`, Table II):

| Method | Offline prior | Online learning | Controller |
| --- | --- | --- | --- |
| A | GAN | none | LMPC (zero tube) |
| B | GAN | none | Tube MPC, fixed envelope (1.5, 1.5, 1.2) m/s² |
| C | nominal `f0 = 0`, `g0 = 1` | task-error GP | adaptive tube MPC |
| D (proposed) | GAN | task-error GP | adaptive tube MPC |

## Configuration

`src/px4ctrl/config/ctrl_param_fpv.yaml` contains the complete configuration.

| Group | Values |
| --- | --- |
| MPC | `dt = 0.01 s`, `H = 20`; `Q_xy = diag(10, 1)`, `R_xy = 0.1`; `Q_z = diag(15, 2)`, `R_z = 0.5`; DARE terminal weights |
| Ancillary LQR | `Q_anc,xy = diag(20, 2)`, `Q_anc,z = diag(30, 5)`, `R_anc = 0.01` |
| Error limits | horizontal 2 m, 2 m/s; vertical 1.5 m, 1 m/s |
| Physical input `U` / correction set `V` | horizontal [−3, 3] m/s², vertical [−2, 5] m/s² |
| Tube | contraction λ = 0.999, 64-face base polygon, slack weights 1e3 (linear), 1e4 (quadratic) |
| GP | window 50, lengthscale 0.5, signal variances 1.0, working noise variance 0.01, gain floor 0.5 |
| Residual budget (x/y) | B = 0.72 (GAN prior), 0.95 (nominal prior); d = 0.25; ε = 0.10; e_x = 0.03; sync 0.02; c = 0.10; hold 0.05; L_f = 0.8; L_g = 0.1; envelope ∈ [0.5, 1.85] |
| Residual budget (z) | B = 0.40 / 0.60; d = 0.20; ε = 0.08; e_x = 0.03; sync 0.02; c = 0.08; hold 0.04; envelope ∈ [0.4, 1.45] |
| Timing | 100 Hz, solver cutoff 8.5 ms, command deadline 10 ms |
| Analysis domain | `p ∈ [−5, 5] × [−5, 5] × [−0.5, 3.5] m`, `v ∈ [−3, 3]² × [−2, 2] m/s` |

## Build

Requirements: Ubuntu 20.04, ROS Noetic with MAVROS, Eigen ≥ 3.3, qpOASES 3.2,
CPU LibTorch (C++11 ABI) for configurations A, B and D, Python ≥ 3.8 with NumPy,
PyTorch and pytest.

```bash
# qpOASES (once)
git clone https://github.com/coin-or/qpOASES.git && cd qpOASES
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON
sudo cmake --build build --target install -j4

# Workspace
source /opt/ros/noetic/setup.bash
cd ~/QAV_WS
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$HOME/libtorch;/opt/ros/noetic"
source devel/setup.bash
```

A build for configuration C only can omit LibTorch with `-DUADL_ENABLE_TORCH=OFF`.

## Workflow

### 1. Flight records

Record the px4ctrl debug topic during flights under automatic control and export
the synchronized records (state, executed net acceleration, measured net
acceleration, virtual input). Cycles without a valid automatic command are skipped.

```bash
rosbag record /drone1/debugPx4ctrl -O flight_01.bag
python3 utils/export_flight_records.py flight_01.bag flight_02.bag \
    --topic /drone1/debugPx4ctrl --output data/offline_records.csv
```

### 2. Offline prior

```bash
python3 utils/offline_GANs_Train.py --records data/offline_records.csv
```

The script splits the records into contiguous train/validation/test time blocks,
trains one policy per axis (Adam, learning rate 2e-4, β1 = 0.5, batch size 1024,
2000 epochs, margin τ = 0.5), reports validation/test response RMSE, and writes
`generator_prior_{X,Y,Z}.pt` and `manifest.json` to `src/px4ctrl/models/`.
`--criterion mse` trains the MLP–MSE baseline of Table I with the same policy
architecture; `--backend module:factory` measures the finite-difference responses
on the vehicle instead of the logged-response model.

Frequency-response validation of the frozen policy (Fig. 4) uses
`utils/req_verification.m` with the white-noise excitation logs `utils/{x,y,z}_data.txt`.

### 3. Flight

Each vehicle runs MAVROS, the UWB bridge and the controller in its own namespace:

```bash
ROS_NAMESPACE=drone1 roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
rosrun uwb_bridge uwb_bridge_node _drone_namespace:=drone1
roslaunch px4ctrl run_ctrl.launch drone_namespace:=drone1 method:=D
```

Take off automatically with
`rostopic pub -1 /drone1/px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"`
(`2` lands), or take off manually and switch the RC hover channel to enter
`AUTO_HOVER`. The RC command channel enables tracking of `/drone1/position_cmd`
(`CMD_CTRL`). The node reports valid automatic hover on
`/drone1/px4ctrl/controller_ready`.

### 4. Synchronized dual-vehicle trial

Once both vehicles report `controller_ready` with the RC command channel enabled:

```bash
python3 utils/Trajectory_Generation.py
rosservice call /dual_figure8_commander/start_trial
```

Both vehicles follow one shared clock with formation offset `r_des = [0, 1.5, 0]` m.
After a 10 s warm-up, both controllers reset their GP windows at a common reserved
epoch and a 60 s scoring interval starts. The node writes `summary.json` (startup
0–1.5 s and full-trial synchronization RMSE, peak synchronization error, individual
tracking RMSE, coverage) and `paired_metrics.jsonl` to the trial directory; it then
brakes smoothly and holds.

## Topics and services (per vehicle)

| Name | Type | Role |
| --- | --- | --- |
| `/droneN/position_cmd` | `quadrotor_msgs/PositionCommand` | reference trajectory |
| `/droneN/px4ctrl/takeoff_land` | `quadrotor_msgs/TakeoffLand` | takeoff (1) / land (2) |
| `/droneN/px4ctrl/controller_ready` | `std_msgs/Bool` | valid automatic hover |
| `/droneN/px4ctrl/reset_online` | `std_srvs/Trigger` | reset GP windows, plans and history (immediately or at `~reset_epoch`) |
| `/droneN/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | attitude and normalized thrust command |
| `/droneN/debugPx4ctrl` | `quadrotor_msgs/Px4ctrlDebug` | references, measured/executed net acceleration, η, GP posterior (`f̂`, `ĝ`, `c_fg`, σ_q, E_q), envelopes, tube radii, input reserve, slack, backup and insertion flags, cycle time |

## Tests

```bash
# Numerical core and controller integration (no ROS or LibTorch required)
cmake -S src/px4ctrl -B /tmp/uadl-core -DUADL_STANDALONE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/uadl-core -j4
ctest --test-dir /tmp/uadl-core --output-on-failure

# Python tools
python3 -m pytest utils/tests -q
```

`online_gp` checks the posterior against scalar closed forms, the Eq. (34) bound and
window/gate behavior; `tube_mpc` checks the DARE solutions, contractive tube and
terminal invariance, backup continuation, admissible tube updates and soft
constraints, and prints the unit tube radii, input reserve and solve time;
`control_geometry` checks the attitude/thrust mapping; `controller_integration`
compiles the real controller with message shims and checks input pairing,
commit-after-publication, reset epochs, reference switching, solver cutoff and the
command deadline. The Python tests cover policy/discriminator updates, prior export,
record splitting and export, end-to-end training, and the trial protocol and metrics.
If Eigen or qpOASES are installed in non-standard locations, pass `Eigen3_DIR`,
`qpOASES_INCLUDE_DIR` and `qpOASES_LIBRARY` to CMake.
