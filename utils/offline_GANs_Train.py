#!/usr/bin/env python3
"""Offline generative prior extraction, manuscript Sections IV-A and VI-B.

u = Theta1(x) + Theta2(x) eta; real/generated pairs are (eta,eta) and
(eta,measured h(x,u)). D ascends J=E[(D(real)-D(generated)-tau)^2]; G descends
J using finite differences of NEW measured responses to its action probes.
A flight CSV supplies states and other-axis commands, never counterfactual
responses. Physical acquisition uses the protocol in models/README.md.

Generator 6->64 Tanh->2 linear heads; discriminator 2->64 Tanh->1 linear score.
Adam lr=2e-4, beta1=.5, beta2=.999; batch=1024, epochs=2000, tau=.5.
Hidden features are trained. Assumption 3's conditional curvature result for
fixed features and structural square loss is not an optimizer guarantee.
Normalization uses only the CSV's fixed training split. eta is drawn
independently of state; its range and the physical FD step are explicit
acquisition settings, neither specified numerically by the manuscript.
"""
import argparse
import copy
import csv
import importlib
import json
import math
import os
from pathlib import Path
from dataclasses import asdict, dataclass
import random
import time
from typing import Tuple
import uuid

import numpy as np
import torch
from torch import nn

STATE_FEATURES = ['p_x', 'p_y', 'p_z', 'v_x', 'v_y', 'v_z']
RECORD_COLUMNS = (['timestamp', 'flight', 'environment'] + STATE_FEATURES +
                  ['u_x', 'u_y', 'u_z', 'a_x', 'a_y', 'a_z', 'split'])
ENVIRONMENTS = ('fans_off', 'fan_a', 'fans_on')
AXES = ('X', 'Y', 'Z')
RAW_GAIN_MIN = 1e-6
INPUT_LIMITS = {'X': (-3.0, 3.0), 'Y': (-3.0, 3.0), 'Z': (-2.0, 5.0)}
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / 'src' / 'px4ctrl' / 'models'
MEASUREMENT_PROTOCOL = 'uadl.physical_fd.v1'


@dataclass(frozen=True)
class TrainingConfig:
    epochs: int = 2000
    batch_size: int = 1024
    lr: float = 2e-4
    beta1: float = 0.5
    tau: float = 0.5
    fd_epsilon: float = 0.05
    eta_range: float = 1.0
    seed: int = 42

    def validate(self):
        if self.epochs < 1 or self.batch_size < 1:
            raise ValueError('epochs and batch_size must be positive')
        for name in ('lr', 'tau', 'fd_epsilon', 'eta_range'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(name + ' must be finite and positive')
        if not 0 <= self.beta1 < 1:
            raise ValueError('beta1 must be in [0,1)')
        if any(2*self.fd_epsilon >= hi-lo for lo, hi in INPUT_LIMITS.values()):
            raise ValueError('FD probes must fit inside the physical input intervals')


class AffinePolicy(nn.Module):
    """Two linear output heads on psi(x)=[tanh(hidden(x));1]."""
    def __init__(self, mean=None, scale=None):
        super().__init__()
        mean = torch.zeros(6) if mean is None else torch.as_tensor(mean, dtype=torch.float32)
        scale = torch.ones(6) if scale is None else torch.as_tensor(scale, dtype=torch.float32)
        if mean.shape != (6,) or scale.shape != (6,) or not torch.isfinite(mean).all() or \
                not torch.isfinite(scale).all() or (scale <= 0).any():
            raise ValueError('Normalization requires six finite means and positive finite scales')
        self.register_buffer('mean', mean)
        self.register_buffer('scale', scale)
        self.hidden = nn.Linear(6, 64)
        self.output = nn.Linear(64, 2)
        nn.init.zeros_(self.output.weight)
        with torch.no_grad():
            self.output.bias.copy_(torch.tensor([0.0, 1.0]))

    def forward(self, state):
        coefficients = self.output(torch.tanh(self.hidden((state-self.mean)/self.scale)))
        return coefficients[:, :1], coefficients[:, 1:2]

    def action(self, state, eta):
        theta1, theta2 = self(state)
        return theta1 + theta2*eta


class RelativeDiscriminator(nn.Module):
    """One 64-unit Tanh hidden layer and a scalar linear score."""
    def __init__(self):
        super().__init__()
        self.network = nn.Sequential(nn.Linear(2, 64), nn.Tanh(), nn.Linear(64, 1))

    def forward(self, pair):
        return self.network(pair)


class ExportedPrior(nn.Module):
    """f0=-Theta1/Theta2, g0=1/Theta2 and state-box interval bounds."""
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        torch._assert(state.dim() == 2 and state.size(1) == 6, 'Expected [N,6] state tensor')
        theta1, theta2 = self.policy(state)
        return -theta1/theta2, 1.0/theta2

    def _affine_bounds(self, lower: torch.Tensor, upper: torch.Tensor,
                       weight: torch.Tensor, bias: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        weight, bias = weight.double(), bias.double()
        positive, negative = torch.clamp_min(weight, 0.0), torch.clamp_max(weight, 0.0)
        lo = torch.mv(positive, lower) + torch.mv(negative, upper) + bias
        hi = torch.mv(positive, upper) + torch.mv(negative, lower) + bias
        # Enclose float32 accumulation in forward; interval arithmetic is float64.
        eps = 1.1920928955078125e-7
        count = 2.0*float(weight.size(1)) + 2.0
        gamma = count*eps/(1.0-count*eps)
        pad = gamma*(torch.mv(torch.abs(weight), torch.maximum(torch.abs(lower), torch.abs(upper)))
                     + torch.abs(bias)) + 1e-30
        return lo-pad, hi+pad

    @torch.jit.export
    def interval_bounds(self, lower: torch.Tensor, upper: torch.Tensor
                        ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Scalar (f_min,f_max,g_min,g_max) over an entire six-state box.

        Propagate the actual normalization, affine layers, monotone Tanh and
        quotient. Reject boxes whose Theta2 interval cannot establish positivity.
        """
        torch._assert(lower.dim() == 1 and lower.numel() == 6, 'Expected lower[6]')
        torch._assert(upper.dim() == 1 and upper.numel() == 6, 'Expected upper[6]')
        lower, upper = lower.double(), upper.double()
        torch._assert(bool(torch.isfinite(lower).all() and torch.isfinite(upper).all()), 'Nonfinite state box')
        torch._assert(bool((lower <= upper).all()), 'Invalid state box')
        mean, scale = self.policy.mean.double(), self.policy.scale.double()
        torch._assert(bool((scale > 0).all()), 'Invalid normalization')
        eps = 1.1920928955078125e-7
        pad = 4.0*eps*(torch.maximum(torch.abs(lower), torch.abs(upper))+torch.abs(mean))/scale + 1e-30
        lo, hi = (lower-mean)/scale-pad, (upper-mean)/scale+pad
        lo, hi = self._affine_bounds(lo, hi, self.policy.hidden.weight, self.policy.hidden.bias)
        lo = torch.clamp(torch.tanh(lo)-4.0*eps, min=-1.0, max=1.0)
        hi = torch.clamp(torch.tanh(hi)+4.0*eps, min=-1.0, max=1.0)
        lo, hi = self._affine_bounds(lo, hi, self.policy.output.weight, self.policy.output.bias)
        torch._assert(bool(torch.isfinite(lo).all() and torch.isfinite(hi).all()), 'Nonfinite coefficient interval')
        torch._assert(bool(lo[1] > 0.0), 'State box does not establish strictly positive Theta2')
        torch._assert(bool(hi[1] <= 1e6), 'State box does not establish raw g0 >= 1e-6')
        ratios = torch.stack((-lo[0]/lo[1], -lo[0]/hi[1], -hi[0]/lo[1], -hi[0]/hi[1]))
        f_lo, f_hi = torch.min(ratios), torch.max(ratios)
        g_lo, g_hi = 1.0/hi[1], 1.0/lo[1]
        f_pad = 2.0*eps*torch.maximum(torch.abs(f_lo), torch.abs(f_hi)) + 1e-30
        g_min, g_max = g_lo*(1.0-2.0*eps)-1e-30, g_hi*(1.0+2.0*eps)+1e-30
        bounds = torch.stack((f_lo-f_pad, f_hi+f_pad, g_min, g_max))
        torch._assert(bool(torch.isfinite(bounds).all() and (torch.abs(bounds) < 3.402823466e38).all()),
                      'State box contains nonfinite float32 prior outputs')
        torch._assert(bool(g_min >= 1e-6), 'State box cannot establish the raw gain floor')
        return bounds[0], bounds[1], bounds[2], bounds[3]


def read_records(path):
    """Load the fixed split; no resplitting or silent dropping of invalid rows."""
    records, seen = [], set()
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        missing = set(RECORD_COLUMNS)-set(reader.fieldnames or [])
        if missing:
            raise ValueError('Flight record CSV is missing columns: '+','.join(sorted(missing)))
        for number, row in enumerate(reader, start=2):
            values = [float(row[column]) for column in ['timestamp']+RECORD_COLUMNS[3:-1]]
            env, flight, split = row['environment'].strip(), row['flight'].strip(), row['split'].strip()
            if env not in ENVIRONMENTS or not flight or split not in ('train', 'validation', 'test'):
                raise ValueError(f'Invalid flight, environment or split at CSV row {number}')
            if not all(map(math.isfinite, values)):
                raise ValueError(f'Nonfinite measurement at CSV row {number}')
            key = (flight, values[0])
            if key in seen:
                raise ValueError(f'Duplicate flight/timestamp at CSV row {number}')
            seen.add(key)
            records.append((env, flight, values, split))
    if not records:
        raise ValueError('Flight record CSV contains no measurements')
    rank = {'train': 0, 'validation': 1, 'test': 2}
    for environment in ENVIRONMENTS:
        members = sorted((row for row in records if row[0] == environment), key=lambda row: row[2][0])
        if not members:
            continue
        if {row[3] for row in members} != set(rank):
            raise ValueError('Each included environment requires fixed train, validation and test blocks')
        for before, after in zip(members, members[1:]):
            if rank[before[3]] > rank[after[3]]:
                raise ValueError('The split must consist of chronological contiguous blocks')
            if before[3] != after[3] and before[1] == after[1] and after[2][0]-before[2][0] < 2.0:
                raise ValueError('Adjacent subsets in the same flight require a 2 s exclusion interval')
    return records


def training_block(records, config=None):
    selected = [i for i, record in enumerate(records) if record[3] == 'train']
    if not selected:
        raise ValueError('The fixed split contains no training records')
    return selected


def make_targets(count, config):
    """Fixed empirical pairs from independent uniform draws; Var=range^2/3."""
    return np.random.default_rng(config.seed).uniform(-config.eta_range, config.eta_range, (count, 3))


class FileMeasurementExchange:
    """Atomic requests/<id>.json -> responses/<id>.json physical acquisition.

    Files stay as acquisition records. A timeout/interruption writes
    cancellations/<id>.json; the measurement process checks it between probes.
    """
    def __init__(self, settings):
        self.directory = Path(settings['exchange_directory']).expanduser().resolve()
        self.timeout = float(settings['timeout_seconds'])
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('timeout_seconds must be positive')
        for name in ('requests', 'responses', 'cancellations'):
            (self.directory/name).mkdir(parents=True, exist_ok=True)

    def __call__(self, request):
        request_id = request['request_id']
        path = self.directory/'requests'/(request_id+'.json')
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(request, allow_nan=False))
        os.replace(temporary, path)
        receipt = self.directory/'responses'/(request_id+'.json')
        deadline = time.monotonic()+self.timeout
        try:
            while not receipt.exists():
                if time.monotonic() >= deadline:
                    raise TimeoutError('Physical acquisition timed out: '+request_id)
                time.sleep(0.1)
            return json.loads(receipt.read_text())
        except BaseException:
            (self.directory/'cancellations'/(request_id+'.json')).write_text(json.dumps(
                {'protocol': MEASUREMENT_PROTOCOL, 'request_id': request_id, 'cancelled': True}))
            raise


class PhysicalResponseBackend:
    """Validate measured triplets at the requested state and physical actions.

    Probe columns are minus/nominal/plus on ONE axis, not X/Y/Z. Every probe
    carries a full three-axis command; other axes keep the context command.
    """
    def __init__(self, data, axis_index, measure, settings):
        self.data, self.axis_index, self.measure = data, axis_index, measure
        self.state_tolerance = np.asarray(settings['state_tolerance'], dtype=np.float64)
        self.action_tolerance = float(settings['action_tolerance'])
        self.max_sensor_skew = float(settings['max_sensor_skew'])
        self.max_response_delay = float(settings['max_response_delay'])
        self.max_probe_span = float(settings['max_probe_span'])
        if self.state_tolerance.shape != (6,) or not np.isfinite(self.state_tolerance).all() or \
                (self.state_tolerance < 0).any():
            raise ValueError('state_tolerance requires six finite nonnegative SI values')
        for name in ('action_tolerance', 'max_sensor_skew', 'max_response_delay', 'max_probe_span'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(name+' must be finite and positive')
        self.last_response_stamp = -math.inf

    def responses(self, index, actions):
        index, actions = np.asarray(index, dtype=np.int64), np.asarray(actions, dtype=np.float64)
        if actions.shape != (len(index), 3) or not np.isfinite(actions).all():
            raise ValueError('Each row requires finite minus/nominal/plus action probes')
        if not np.all(np.diff(actions, axis=1) > 0):
            raise ValueError('Probes must be ordered minus, nominal, plus')
        if 2*self.action_tolerance >= np.min(np.diff(actions, axis=1)):
            raise ValueError('action_tolerance must be less than half the probe spacing')
        commands = np.repeat(self.data['u'][index, None, :], 3, axis=1)
        commands[:, :, self.axis_index] = actions
        for j, axis in enumerate(AXES):
            if (commands[:, :, j] < INPUT_LIMITS[axis][0]).any() or \
                    (commands[:, :, j] > INPUT_LIMITS[axis][1]).any():
                raise ValueError('Requested physical command lies outside the input box')
        request_id = uuid.uuid4().hex
        request = {
            'protocol': MEASUREMENT_PROTOCOL, 'request_id': request_id,
            'axis': AXES[self.axis_index], 'probe_order': ['minus', 'nominal', 'plus'],
            'measurement_source': 'physical_imu', 'state_features': STATE_FEATURES,
            'state': self.data['state'][index].tolist(),
            'eta': self.data['eta'][index, self.axis_index].tolist(),
            'commands': commands.tolist(), 'environment': [self.data['environment'][i] for i in index],
            'context_flight': [self.data['flight'][i] for i in index],
            'state_tolerance': self.state_tolerance.tolist(), 'action_tolerance': self.action_tolerance,
            'max_sensor_skew': self.max_sensor_skew, 'max_response_delay': self.max_response_delay,
            'max_probe_span': self.max_probe_span,
        }
        receipt = self.measure(request)
        if isinstance(receipt, dict) and receipt.get('request_id') == request_id and 'error' in receipt:
            raise ValueError('Physical acquisition rejected the request: '+str(receipt['error']))
        if not isinstance(receipt, dict) or receipt.get('protocol') != MEASUREMENT_PROTOCOL or \
                receipt.get('request_id') != request_id or receipt.get('measurement_source') != 'physical_imu':
            raise ValueError('Physical receipt has wrong protocol, request ID or measurement source')
        if receipt.get('environment') != request['environment'] or receipt.get('probe_order') != request['probe_order']:
            raise ValueError('Physical receipt has mismatched environments or probe order')

        def array(name, shape):
            value = np.asarray(receipt[name], dtype=np.float64)
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError('Invalid measured array: '+name)
            return value

        count = len(index)
        state = array('state', (count, 3, 6))
        executed = array('executed_commands', (count, 3, 3))
        response = array('net_acceleration', (count, 3, 3))
        command_stamp = array('command_stamp', (count, 3))
        response_stamp = array('response_stamp', (count, 3))
        imu_stamp = array('imu_stamp', (count, 3))
        odom_stamp = array('odom_stamp', (count, 3))
        if (np.abs(state-self.data['state'][index, None, :]) > self.state_tolerance).any():
            raise ValueError('Probe states do not match the requested repeated-state condition')
        if (np.abs(executed-commands) > self.action_tolerance).any():
            raise ValueError('Vehicle did not execute the requested probes; clipped-action gradients are invalid')
        delay = response_stamp-command_stamp
        if (delay < 0).any() or (delay > self.max_response_delay).any():
            raise ValueError('Response is not paired with a current executed command')
        if (np.abs(imu_stamp-response_stamp) > self.max_sensor_skew).any() or \
                (np.abs(odom_stamp-response_stamp) > self.max_sensor_skew).any():
            raise ValueError('Unsynchronized physical-response timestamps')
        if (np.ptp(response_stamp, axis=1) > self.max_probe_span).any():
            raise ValueError('A probe triplet exceeds the repeated-condition time span')
        if (response_stamp <= self.last_response_stamp).any() or len(np.unique(response_stamp)) != response_stamp.size:
            raise ValueError('Reused response or acquisition outside the fresh measurement interval')
        self.last_response_stamp = float(response_stamp.max())
        return response[:, :, self.axis_index]


def make_backend_factory(args, config):
    settings = json.loads(Path(args.backend_config).read_text())
    if args.backend == 'exchange':
        measure = FileMeasurementExchange(settings)
    else:
        if ':' not in args.backend:
            raise ValueError('--backend must be exchange or module:factory; interpolation is not measurement')
        module_name, factory_name = args.backend.split(':', 1)
        measure = getattr(importlib.import_module(module_name), factory_name)(settings)
        if not callable(measure):
            raise ValueError('Acquisition factory must return callable(request)->receipt')
    return lambda data, axis_index: PhysicalResponseBackend(data, axis_index, measure, settings)


def check_raw_gain(policy, states):
    """Pointwise acceptance of raw g0>=1e-6, without clipping."""
    with torch.no_grad():
        theta1, theta2 = policy(states)
        gain, drift = 1.0/theta2, -theta1/theta2
    if not all(torch.isfinite(value).all() for value in (theta1, theta2, gain, drift)):
        raise ValueError('Rejecting nonfinite prior coefficients')
    if (theta2 <= 0).any() or (gain < RAW_GAIN_MIN).any():
        raise ValueError('Rejecting raw gain below 1e-6')


def loss_per_sample(discriminator, eta, response, config):
    real = discriminator(torch.cat((eta, eta), dim=1))
    generated = discriminator(torch.cat((eta, response), dim=1))
    return (real-generated-config.tau).square()


def fd_policy_step(policy, discriminator, optimizer_g, optimizer_d, states, eta,
                   backend, index, config, limits):
    """D ascent, then G descent with the updated D and FD through the plant.

    dJ/dtheta = (dJ/dh at measured nominal h) *
               (h(u+eps)-h(u-eps))/(2eps) * du/dtheta.
    """
    with torch.no_grad():
        action = policy.action(states, eta).cpu().numpy()[:, 0]
    if not np.isfinite(action).all() or (action-config.fd_epsilon < limits[0]).any() or \
            (action+config.fd_epsilon > limits[1]).any():
        raise ValueError('Policy-action probes leave the physical input interval; rejecting update')
    actions = np.stack((action-config.fd_epsilon, action, action+config.fd_epsilon), axis=1)
    measured = torch.as_tensor(backend.responses(index, actions), dtype=states.dtype, device=states.device)
    if measured.shape != (states.shape[0], 3) or not torch.isfinite(measured).all():
        raise ValueError('Invalid measured physical responses')
    minus, nominal, plus = measured[:, :1], measured[:, 1:2], measured[:, 2:3]
    optimizer_d.zero_grad()
    objective_d = loss_per_sample(discriminator, eta, nominal.detach(), config).mean()
    if not torch.isfinite(objective_d):
        raise ValueError('Nonfinite discriminator objective')
    (-objective_d).backward()
    optimizer_d.step()
    observed = nominal.detach().requires_grad_(True)
    objective_g = loss_per_sample(discriminator, eta, observed, config)
    response_gradient, = torch.autograd.grad(objective_g.sum(), observed)
    action_gradient = (response_gradient*(plus-minus)/(2.0*config.fd_epsilon)).detach()
    if not torch.isfinite(action_gradient).all():
        raise ValueError('Nonfinite measured-response generator gradient')
    optimizer_g.zero_grad()
    (policy.action(states, eta)*action_gradient).mean().backward()
    optimizer_g.step()
    return float(objective_g.detach().mean())


def train_axis(axis, data, config, device, backend_factory, log=print):
    axis_index = AXES.index(axis)
    torch.manual_seed(config.seed+axis_index)
    mean, scale = data['state'].mean(axis=0), data['state'].std(axis=0)
    scale[scale < 1e-6] = 1.0
    policy, discriminator = AffinePolicy(mean, scale).to(device), RelativeDiscriminator().to(device)
    optimizer_g = torch.optim.Adam(policy.parameters(), lr=config.lr, betas=(config.beta1, .999))
    optimizer_d = torch.optim.Adam(discriminator.parameters(), lr=config.lr, betas=(config.beta1, .999))
    backend = backend_factory(data, axis_index)
    states = torch.as_tensor(data['state'], dtype=torch.float32, device=device)
    eta = torch.as_tensor(data['eta'][:, axis_index:axis_index+1], dtype=torch.float32, device=device)
    generator = np.random.default_rng(config.seed+axis_index)
    for epoch in range(config.epochs):
        order = generator.permutation(len(states))
        for start in range(0, len(states), config.batch_size):
            index = order[start:start+config.batch_size]
            selected = torch.as_tensor(index, device=device)
            fd_policy_step(policy, discriminator, optimizer_g, optimizer_d, states[selected],
                           eta[selected], backend, index, config, INPUT_LIMITS[axis])
        if epoch == 0 or (epoch+1) % 100 == 0 or epoch+1 == config.epochs:
            log(f'[{axis}] completed epoch {epoch+1}/{config.epochs}')
    return policy.eval()


def export_prior(policy, path, verification_states):
    cpu_policy = copy.deepcopy(policy).cpu().eval()
    states = verification_states.detach().cpu().float()
    check_raw_gain(cpu_policy, states)
    module = torch.jit.script(ExportedPrior(cpu_policy).eval())
    with torch.no_grad():
        for before, after in zip(ExportedPrior(cpu_policy)(states), module(states)):
            torch.testing.assert_close(before, after)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    module.save(str(path))


def train(args, log=print):
    config = TrainingConfig(epochs=args.epochs, batch_size=args.batch_size, fd_epsilon=args.fd_epsilon,
                            eta_range=args.eta_range, seed=args.seed)
    config.validate()
    records = read_records(args.records)
    block = training_block(records)
    values = np.asarray([records[i][2] for i in block], dtype=np.float64)
    data = {'state': values[:, 1:7], 'u': values[:, 7:10], 'eta': make_targets(len(block), config),
            'environment': [records[i][0] for i in block], 'flight': [records[i][1] for i in block]}
    np.random.seed(config.seed)
    random.seed(config.seed)
    device = torch.device(args.device)
    backend_factory = make_backend_factory(args, config)
    policies = {axis: train_axis(axis, data, config, device, backend_factory, log) for axis in AXES}
    all_states = torch.as_tensor(np.asarray([record[2][1:7] for record in records]), dtype=torch.float32)
    for policy in policies.values():
        check_raw_gain(policy, all_states.to(device))
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    for axis, policy in policies.items():
        export_prior(policy, output/f'generator_prior_{axis}.pt', all_states)
    manifest = {'method': 'relative GAN with measured physical-response finite differences',
                'state_features': STATE_FEATURES, 'input_shape': ['N', 6],
                'output': ['f0[N,1]', 'g0[N,1]'],
                'interval_bounds': '[lower[6],upper[6]] -> (f0_min,f0_max,g0_min,g0_max)',
                'input_limits': INPUT_LIMITS, 'records': str(Path(args.records).resolve()),
                'split': 'fixed CSV split column', 'backend': args.backend,
                'measurement_protocol': MEASUREMENT_PROTOCOL,
                'backend_config': str(Path(args.backend_config).resolve()),
                'config': asdict(config), 'torch_version': torch.__version__}
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    log(f'Exported generator_prior_{{X,Y,Z}}.pt and manifest.json to {output}')
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--records', required=True, help='CSV with fixed chronological split')
    parser.add_argument('--output', default=str(DEFAULT_OUTPUT))
    parser.add_argument('--backend', default='exchange', help='exchange or module:factory for physical acquisition')
    parser.add_argument('--backend-config', required=True, help='physical acquisition settings JSON')
    parser.add_argument('--epochs', type=int, default=2000)
    parser.add_argument('--batch-size', type=int, default=1024)
    parser.add_argument('--fd-epsilon', type=float, required=True, help='physical action perturbation, m/s^2')
    parser.add_argument('--eta-range', type=float, required=True, help='uniform virtual-input range, m/s^2')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args(argv)
    try:
        train(args)
    except (ValueError, OSError, KeyError, TypeError, RuntimeError) as exc:
        parser.exit(2, 'Training stopped: '+str(exc)+'\n')


if __name__ == '__main__':
    main()
