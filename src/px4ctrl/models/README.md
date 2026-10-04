# Offline generative priors

`utils/offline_GANs_Train.py` implements the affine generator and relative
adversarial objective in manuscript Sections IV-A and VI-B. Each axis has a
`6 → 64 Tanh → 2` policy with linear `Theta1, Theta2` heads, and a
`2 → 64 Tanh → 1` discriminator with a linear score. The physical policy is
`u = Theta1(x) + Theta2(x) eta`. The two discriminator pairs are `(eta, eta)`
and `(eta, measured_response)`.

The value is `J = mean((D(eta,eta) - D(eta,h) - tau)^2)`. The discriminator
**ascends** this value; the generator **descends** it. Both use Adam with
learning rate `2e-4`, `beta1=0.5`, `beta2=0.999`; defaults are batch size 1024,
2,000 epochs, and `tau=0.5`. The output heads and hidden weights are trained.
Assumption 3's fixed-feature structural-loss result is a conditional theorem,
and is not claimed for the full adversarial optimization.

The generator gradient uses three actual measurements at `u-eps`, `u`, and
`u+eps` under matched states and physical conditions. The discriminator is
updated with the nominal response, then its derivative at that response is
combined with the measured plant finite difference:

```
dJ/dtheta = dJ/dh * (h_plus - h_minus)/(2 eps) * du/dtheta
```

The logged-neighbor interpolation backend has been removed. No learned plant
surrogate or recorded neighboring action substitutes for these measurements.
Out-of-range policy probes or modified commands reject the update; commands
are not clipped and differentiated as though their original values executed.
The FD step and independent uniform target range are required CLI arguments.
Their numerical values, response delay, state tolerances, and probe duration
are acquisition settings that the paper does not specify.

## Flight records and normalization

`utils/export_flight_records.py` reads ROS1 bags supplied with
`--flight BAG:ENVIRONMENT`, where the physical label is `fans_off`, `fan_a`,
or `fans_on`. `--namespace`, `--hover-thrust`, `--mass`, and `--gravity` select
the vehicle and its hover-calibrated thrust mapping. The default mass is
0.9 kg and gravity is 9.81 m/s². This exporter uses the hover mapping; its
recorded flights must use that same mapping. The live collector below also
supports the configured voltage-dependent thrust model.

Each CSV row contains `timestamp, flight, environment`, inertial position and
velocity `[p_x,p_y,p_z,v_x,v_y,v_z]`, executed net-acceleration coordinates
`u_x,u_y,u_z`, IMU net acceleration `a_x,a_y,a_z`, and a fixed `split`.
MAVROS body-frame velocity is rotated with odometry. Labels use
`R_odom * specific_force_body - g*e3`. The command is reconstructed as
`(F_T,cmd/m0) R_cmd e3 - g*e3`, including the inverse of the published
`q_sp = q_imu q_odom^-1 q_cmd` frame mapping. IMU and odometry are aligned by
nearest timestamps; the same causal setpoint must cover both sensor stamps.

Within each environment the exporter fixes chronological blocks at
approximately 70/15/15 percent. When a boundary lies inside one flight, it
removes a symmetric 2 s interval around that boundary. The CSV itself is the
split manifest. Normalization is fitted only to rows marked `train` and is
embedded in the exported modules. Training reads that fixed split, and rejects
unordered splits or missing within-flight exclusions. The records provide a
fixed state distribution and other-axis command contexts. New physical probes
are collected during training; an ordinary flight CSV alone cannot supply
policy-conditioned responses after the policy changes.

## Physical acquisition

The implemented ROS1 endpoint is `utils/physical_fd_collector.py`. It consumes
request files and acquires actual IMU/odometry responses. It does not arm the
vehicle or change its mode. Use a dedicated acquisition session with an armed
OFFBOARD vehicle, a continuous baseline controller, and the correct fan
configuration. The collector waits for each requested state to recur within
tolerance, and fails explicitly when that does not happen before the deadline.
It does not synthesize a reset or manufacture a matched-state response.

The collector is the sole publisher to
`<namespace>/mavros/setpoint_raw/attitude`. Route the baseline controller's
attitude outputs to `<namespace>/offline_fd/baseline_attitude`; the collector
forwards that stream at 100 Hz and temporarily substitutes each probe. A
`std_msgs/String` on `<namespace>/offline_fd/environment` supplies the actual
fan configuration. The collector subscribes to MAVROS odometry, IMU, flight
state, and battery voltage. It uses the controller YAML for mass, gravity,
physical input bounds, maximum tilt, and thrust calibration. It rejects
saturation, stale measurements, stale baseline commands, changed environments,
and mismatched action/state receipts; after each probe or failure it resumes
the baseline stream. The normal controller must not also publish directly to
the same MAVROS attitude topic during this acquisition session.

From the workspace root, the acquisition endpoint command is:

```sh
python3 utils/physical_fd_collector.py \
  --settings utils/physical_fd_settings.example.json \
  --controller-config src/px4ctrl/config/ctrl_param_fpv.yaml
```

The supplied JSON is a complete **implementation configuration example**, not
reported experimental calibration. Set its state tolerances, timing, namespace,
and exchange directory for the physical session. The trainer uses the same
file. With `FLIGHT_RECORDS` set to the exported CSV path, its invocation is:

```sh
python3 utils/offline_GANs_Train.py \
  --records "$FLIGHT_RECORDS" \
  --backend exchange \
  --backend-config utils/physical_fd_settings.example.json \
  --fd-epsilon 0.05 --eta-range 1.0
```

The values `0.05` and `1.0` are explicit example acquisition choices. The
physical input box must admit every nominal and perturbed action. Uniform
`eta` is sampled independently of state, with variance `eta_range²/3`.
Nonzero tolerances and finite probe spacing do not make responses exactly
coincident in state/time; the acquisition error must satisfy the disturbance
and observation-error assumptions used in the manuscript.

## Request and receipt interface

The protocol is `uadl.physical_fd.v1`. The trainer atomically writes
`EXCHANGE/requests/UUID.json`. The collector atomically writes
`EXCHANGE/responses/UUID.json`; a timeout or interruption produces
`EXCHANGE/cancellations/UUID.json`, checked between probes. Files are retained
as the acquisition record. An error receipt contains `protocol`, `request_id`,
and `error`, and stops the training update with that reason.

For a batch of `B` states, a request contains:

| Field | Meaning |
| --- | --- |
| `protocol`, `request_id` | Protocol version and unique query UUID |
| `measurement_source` | `physical_imu` |
| `axis` | The single trained axis, `X`, `Y`, or `Z` |
| `probe_order` | `minus, nominal, plus` |
| `state_features` | `p_x,p_y,p_z,v_x,v_y,v_z`, all in SI units |
| `state` | Requested states, shape `[B,6]` |
| `eta` | Independent virtual targets, shape `[B]` |
| `commands` | Full physical commands, `[B,3 probes,3 axes]` |
| `environment`, `context_flight` | Physical environment and source flight per row |
| `state_tolerance`, `action_tolerance` | Allowed deviations from the requested probes |
| `max_sensor_skew`, `max_response_delay`, `max_probe_span` | Sensor, command, and triplet timing bounds in seconds |

A successful receipt echoes `protocol`, `request_id`, `measurement_source`,
`probe_order`, and `environment`, and supplies:

| Field | Shape and meaning |
| --- | --- |
| `state` | `[B,3,6]`, measured inertial state for every probe |
| `executed_commands` | `[B,3,3]`, mapped command actually published, including wire thrust precision |
| `net_acceleration` | `[B,3,3]`, measured inertial IMU response |
| `command_stamp` | `[B,3]`, actual published command stamp |
| `response_stamp` | `[B,3]`, aligned odometry stamp used for the response |
| `imu_stamp`, `odom_stamp` | `[B,3]`, sensor stamps used to form the response |

The action perturbation changes only the trained axis. The other two command
coordinates remain fixed to the context command throughout the triplet.
All arrays must be finite; each response stamp must be unique and newer than
the preceding query's stamps. The trainer validates every receipt before
updating either network. For a different physical acquisition transport,
`--backend module:factory` loads `factory(settings)`, which must return a
callable `measure(request) -> receipt` with exactly the same validated protocol.
The supplied ROS collector provides a complete implementation of that protocol.

## Deployment files and interval bounds

| File | Interface |
| --- | --- |
| `generator_prior_X.pt` | CPU `float32[N,6] -> (f0[N,1],g0[N,1])` for x |
| `generator_prior_Y.pt` | Same interface for y |
| `generator_prior_Z.pt` | Same interface for z |
| `manifest.json` | Method, interfaces, fixed-split source, acquisition protocol, and training configuration |

Here `f0=-Theta1/Theta2`, `g0=1/Theta2`. Raw gains below `1e-6` or nonfinite
outputs are rejected, without clipping. Checking recorded states is a pointwise
acceptance check and does not establish Remark 3 over the complete domain.

Each model additionally exports TorchScript
`interval_bounds(lower[6], upper[6]) -> (f_min,f_max,g_min,g_max)`. It accepts
float64 SI state-box endpoints and returns four scalar float64 tensors. The
method propagates intervals through the actual frozen normalization, affine
layers, monotone Tanh, and reciprocal/quotient. Bounds include float32 forward
roundoff padding. A box whose `Theta2` enclosure crosses zero, whose raw gain
floor cannot be established, or whose outputs may overflow is rejected.
These bounds cover the full input box and supply the controller's prior
contribution to its continuous-domain residual envelope. They are not inferred
from a finite set of predicted states. `run_ctrl.launch` loads the three files
from this directory; no discriminator is deployed.
