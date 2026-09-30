#!/usr/bin/env python3
"""Offline generative prior extraction (Section IV-A, Eqs. 12-17).

For each axis, the affine linearizing policy u = Theta1(x) + Theta2(x) * eta
(Eq. 12) is trained as the generator of a relative GAN (Eqs. 14-16). The
trained policy is exported as the generative prior
f0 = -Theta1/Theta2, g0 = 1/Theta2 (Eq. 17) for the online controller.

Flight records (CSV, one row per 100 Hz sample):
    timestamp                      s
    p_x, p_y, p_z, v_x, v_y, v_z   inertial state
    u_x, u_y, u_z                  executed net acceleration command
    a_x, a_y, a_z                  measured net acceleration (IMU rotated by odometry)
    eta_x, eta_y, eta_z            random-excitation targets
An optional ``split`` column (train/validation/test) selects the time blocks;
otherwise contiguous 70/15/15 time blocks are used. The records are produced
from recorded flights by ``utils/export_flight_records.py``.

Response measurement. The generator update uses central finite differences
of the measured response to the policy action (Eq. 16b). The default
``logged`` backend evaluates the measured response of record j at an action u
through the local input sensitivity of the recorded flight data,
    h(x_j, u) = a_j + beta_j (u - u_j),
where beta_j is a ridge-regularized, kernel-weighted least-squares slope of
the measured responses of the k nearest records (normalized state) of the same
split. Alternatively, ``--backend module:factory`` measures the responses on
the vehicle: ``factory(settings)`` returns a callable
``measure(states[B,6], eta[B], actions[B,3]) -> responses[B,3]``.

Outputs (default ``src/px4ctrl/models``): ``generator_prior_{X,Y,Z}.pt``
mapping float32[N,6] -> (f0[N,1], g0[N,1]) with the state normalization
embedded, and ``manifest.json`` with the configuration, split sizes and
validation/test response RMSE.

    python3 utils/offline_GANs_Train.py --records data/offline_records.csv
    python3 utils/offline_GANs_Train.py --records data/offline_records.csv \\
        --criterion mse --output data/mlp_mse_baseline
"""
import argparse
import copy
import csv
import importlib
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
import random
import time

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

STATE_FEATURES = ['p_x', 'p_y', 'p_z', 'v_x', 'v_y', 'v_z']
RECORD_COLUMNS = (['timestamp'] + STATE_FEATURES + ['u_x', 'u_y', 'u_z', 'a_x', 'a_y', 'a_z']
                  + ['eta_x', 'eta_y', 'eta_z'])
AXES = ('X', 'Y', 'Z')
SPLITS = ('train', 'validation', 'test')
RAW_GAIN_MIN = 1e-6
# Physical net-acceleration limits (Section VI-C).
INPUT_LIMITS = {'X': (-3.0, 3.0), 'Y': (-3.0, 3.0), 'Z': (-2.0, 5.0)}
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / 'src' / 'px4ctrl' / 'models'


@dataclass(frozen=True)
class TrainingConfig:
    epochs: int = 2000
    batch_size: int = 1024
    lr: float = 2e-4
    beta1: float = 0.5
    tau: float = 0.5
    fd_epsilon: float = 0.05
    neighbours: int = 32
    ridge: float = 1.0
    seed: int = 1011
    criterion: str = 'gan'
    train_fraction: float = 0.70
    validation_fraction: float = 0.15

    def validate(self):
        if self.epochs < 1 or self.batch_size < 1 or self.neighbours < 2:
            raise ValueError('epochs, batch_size and neighbours must be positive (neighbours >= 2)')
        if not (math.isfinite(self.fd_epsilon) and self.fd_epsilon > 0 and self.ridge >= 0):
            raise ValueError('fd_epsilon must be positive and ridge nonnegative')
        if self.criterion not in ('gan', 'mse'):
            raise ValueError('criterion must be gan or mse')
        if not (0 < self.train_fraction < 1 and 0 < self.validation_fraction < 1
                and self.train_fraction + self.validation_fraction < 1):
            raise ValueError('train/validation fractions must leave a test block')


class AffinePolicy(nn.Module):
    """Generator of Eq. (12): one 64-unit Tanh hidden layer, outputs Theta1, Theta2 > 0."""

    def __init__(self, mean=None, scale=None):
        super().__init__()
        self.register_buffer('mean', torch.zeros(6) if mean is None else torch.as_tensor(mean, dtype=torch.float32))
        self.register_buffer('scale', torch.ones(6) if scale is None else torch.as_tensor(scale, dtype=torch.float32))
        self.hidden = nn.Linear(6, 64)
        self.output = nn.Linear(64, 2)
        # Start from the nominal inverse u = eta (f0 = 0, g0 = 1).
        nn.init.zeros_(self.output.weight)
        with torch.no_grad():
            self.output.bias.copy_(torch.tensor([0.0, math.log(math.expm1(1.0))]))

    def forward(self, state):
        features = torch.tanh(self.hidden((state - self.mean) / self.scale))
        coefficients = self.output(features)
        return coefficients[:, :1], F.softplus(coefficients[:, 1:2])

    def action(self, state, eta):
        theta1, theta2 = self(state)
        return theta1 + theta2 * eta


class RelativeDiscriminator(nn.Module):
    """Discriminator of Eq. (15): one 64-unit Tanh hidden layer. The bounded
    score keeps the inner maximization of the squared relative margin finite."""

    def __init__(self):
        super().__init__()
        self.network = nn.Sequential(nn.Linear(2, 64), nn.Tanh(), nn.Linear(64, 1), nn.Tanh())

    def forward(self, pair):
        return self.network(pair)


class ExportedPrior(nn.Module):
    """Eq. (17): f0 = -Theta1/Theta2, g0 = 1/Theta2."""

    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, state: torch.Tensor):
        torch._assert(state.dim() == 2, 'Expected [N,6] state tensor')
        torch._assert(state.size(1) == 6, 'Expected p_x,p_y,p_z,v_x,v_y,v_z')
        theta1, theta2 = self.policy(state)
        return -theta1 / theta2, 1.0 / theta2


def read_records(path, config):
    """Load flight records and split them into ordered, disjoint time blocks."""
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        missing = [column for column in RECORD_COLUMNS if column not in (reader.fieldnames or [])]
        if missing:
            raise ValueError('Flight record CSV is missing columns: ' + ','.join(missing))
        rows, labels = [], []
        for row in reader:
            values = [float(row[column]) for column in RECORD_COLUMNS]
            if all(map(math.isfinite, values)):
                rows.append(values)
                labels.append((row.get('split') or '').strip())
    if len(rows) < 30:
        raise ValueError('At least 30 valid flight records are required')
    data = np.asarray(rows, dtype=np.float64)
    order = np.argsort(data[:, 0], kind='stable')
    data = data[order]
    labels = np.asarray([labels[i] for i in order])
    if not all(label in SPLITS for label in labels):
        count = len(data)
        first = int(round(config.train_fraction * count))
        second = int(round((config.train_fraction + config.validation_fraction) * count))
        labels = np.asarray(['train'] * first + ['validation'] * (second - first) + ['test'] * (count - second))
    groups = {}
    for split in SPLITS:
        block = data[labels == split]
        if len(block) < 10:
            raise ValueError(f'The {split} block needs at least 10 records')
        groups[split] = {'timestamp': block[:, 0], 'state': block[:, 1:7], 'u': block[:, 7:10],
                         'a': block[:, 10:13], 'eta': block[:, 13:16]}
    for left, right in zip(SPLITS, SPLITS[1:]):
        if groups[left]['timestamp'].max() >= groups[right]['timestamp'].min():
            raise ValueError('train, validation and test blocks overlap in time')
    return groups


def local_input_sensitivity(state, u, a, neighbours, ridge, chunk=512):
    """Kernel-weighted least-squares slope of the measured response versus the
    executed input over the k nearest records, regularized toward the nominal
    net-acceleration gain 1."""
    count = len(state)
    k = min(neighbours, count)
    scale = state.std(axis=0)
    scale[scale < 1e-9] = 1.0
    z = (state - state.mean(axis=0)) / scale
    squared = (z * z).sum(axis=1)
    slope = np.empty(count)
    for start in range(0, count, chunk):
        stop = min(count, start + chunk)
        distance = np.maximum(squared[start:stop, None] - 2.0 * z[start:stop] @ z.T + squared[None, :], 0.0)
        index = np.argpartition(distance, k - 1, axis=1)[:, :k]
        nearest = np.take_along_axis(distance, index, axis=1)
        bandwidth = np.maximum(np.median(nearest, axis=1, keepdims=True), 1e-12)
        weight = np.exp(-0.5 * nearest / bandwidth)
        inputs, responses = u[index], a[index]
        total = weight.sum(axis=1, keepdims=True)
        input_mean = (weight * inputs).sum(axis=1, keepdims=True) / total
        response_mean = (weight * responses).sum(axis=1, keepdims=True) / total
        covariance = (weight * (inputs - input_mean) * (responses - response_mean)).sum(axis=1)
        variance = (weight * (inputs - input_mean) ** 2).sum(axis=1)
        slope[start:stop] = (covariance + ridge) / (variance + ridge)
    return slope


class LoggedResponseBackend:
    """Measured responses of logged records at policy actions (first order in u)."""

    def __init__(self, group, axis_index, config):
        self.u = group['u'][:, axis_index]
        self.a = group['a'][:, axis_index]
        self.slope = local_input_sensitivity(group['state'], self.u, self.a, config.neighbours, config.ridge)

    def responses(self, index, actions):
        return self.a[index, None] + self.slope[index, None] * (actions - self.u[index, None])


class PluginResponseBackend:
    """On-vehicle measurement through a user module (module:factory)."""

    def __init__(self, group, axis_index, measure):
        self.state = group['state']
        self.eta = group['eta'][:, axis_index]
        self.measure = measure

    def responses(self, index, actions):
        measured = np.asarray(self.measure(self.state[index], self.eta[index], actions), dtype=np.float64)
        if measured.shape != actions.shape or not np.isfinite(measured).all():
            raise ValueError('Measurement backend must return finite responses shaped like the actions')
        return measured


def make_backend_factory(args, config):
    if args.backend == 'logged':
        return lambda group, axis_index: LoggedResponseBackend(group, axis_index, config)
    if ':' not in args.backend:
        raise ValueError("--backend must be 'logged' or module:factory")
    module_name, factory_name = args.backend.split(':', 1)
    settings = json.loads(Path(args.backend_config).read_text()) if args.backend_config else {}
    measure = getattr(importlib.import_module(module_name), factory_name)(settings)
    return lambda group, axis_index: PluginResponseBackend(group, axis_index, measure)


def check_raw_gain(policy, states):
    """Remark 3: accept raw gains g0 = 1/Theta2 >= 1e-6 without clipping."""
    with torch.no_grad():
        theta1, theta2 = policy(states)
        gain = 1.0 / theta2
        drift = -theta1 / theta2
    if not all(torch.isfinite(value).all() for value in (theta1, theta2, gain, drift)):
        raise ValueError('Rejecting non-finite prior coefficients')
    if (theta2 <= 0).any() or (gain < RAW_GAIN_MIN).any():
        raise ValueError('Rejecting raw gain below 1e-6')
    return gain.min().item(), gain.max().item(), drift.abs().max().item()


def loss_per_sample(discriminator, eta, response, config):
    """Eq. (15) with real pair (eta, eta) and generated pair (eta, h)."""
    if config.criterion == 'mse':
        return (response - eta).square()
    real = discriminator(torch.cat((eta, eta), dim=1))
    generated = discriminator(torch.cat((eta, response), dim=1))
    return (real - generated - config.tau).square()


def fd_policy_step(policy, discriminator, optimizer_g, optimizer_d, states, eta,
                   backend, index, config, limits):
    """One alternating update: discriminator ascent (Eq. 16a), then generator
    descent with the finite-difference response gradient (Eq. 16b)."""
    with torch.no_grad():
        action = policy.action(states, eta).cpu().numpy()[:, 0]
    action = np.clip(action, limits[0] + config.fd_epsilon, limits[1] - config.fd_epsilon)
    actions = np.stack((action - config.fd_epsilon, action, action + config.fd_epsilon), axis=1)
    measured = torch.as_tensor(backend.responses(index, actions), dtype=states.dtype, device=states.device)
    minus, nominal, plus = measured[:, :1], measured[:, 1:2], measured[:, 2:3]
    if config.criterion == 'gan':
        optimizer_d.zero_grad()
        objective_d = loss_per_sample(discriminator, eta, nominal, config).mean()
        (-objective_d).backward()
        optimizer_d.step()
    with torch.no_grad():
        slope = (loss_per_sample(discriminator, eta, plus, config) -
                 loss_per_sample(discriminator, eta, minus, config)) / (2.0 * config.fd_epsilon)
        objective_g = loss_per_sample(discriminator, eta, nominal, config).mean()
    optimizer_g.zero_grad()
    (policy.action(states, eta) * slope).mean().backward()
    optimizer_g.step()
    return float(objective_g)


def train_axis(axis, group, config, device, backend_factory, log=print):
    axis_index = AXES.index(axis)
    mean = group['state'].mean(axis=0)
    scale = group['state'].std(axis=0)
    scale[scale < 1e-6] = 1.0
    policy = AffinePolicy(mean, scale).to(device)
    discriminator = RelativeDiscriminator().to(device)
    optimizer_g = torch.optim.Adam(policy.parameters(), lr=config.lr, betas=(config.beta1, 0.999))
    optimizer_d = torch.optim.Adam(discriminator.parameters(), lr=config.lr, betas=(config.beta1, 0.999))
    backend = backend_factory(group, axis_index)
    states = torch.as_tensor(group['state'], dtype=torch.float32, device=device)
    eta = torch.as_tensor(group['eta'][:, axis_index:axis_index + 1], dtype=torch.float32, device=device)
    generator = np.random.default_rng(config.seed + axis_index)
    count = len(states)
    started = time.time()
    for epoch in range(config.epochs):
        order = generator.permutation(count)
        objective = 0.0
        for start in range(0, count, config.batch_size):
            index = order[start:start + config.batch_size]
            selected = torch.as_tensor(index, device=device)
            objective = fd_policy_step(policy, discriminator, optimizer_g, optimizer_d, states[selected],
                                       eta[selected], backend, index, config, INPUT_LIMITS[axis])
        if epoch == 0 or (epoch + 1) % 100 == 0 or epoch + 1 == config.epochs:
            log(f'[{axis}] epoch {epoch + 1}/{config.epochs}  objective {objective:.5f}  '
                f'({time.time() - started:.0f} s)')
    policy.eval()
    return policy


def evaluate(policy, group, axis, backend, device):
    """Response RMSE of the frozen policy against eta (validation/test blocks)."""
    axis_index = AXES.index(axis)
    states = torch.as_tensor(group['state'], dtype=torch.float32, device=device)
    eta = group['eta'][:, axis_index]
    with torch.no_grad():
        action = policy.action(states, torch.as_tensor(eta[:, None], dtype=torch.float32, device=device))
    low, high = INPUT_LIMITS[axis]
    action = np.clip(action.cpu().numpy()[:, 0], low, high)
    index = np.arange(len(eta))
    response = backend.responses(index, action[:, None])[:, 0]
    nominal = backend.responses(index, np.clip(eta, low, high)[:, None])[:, 0]
    gain_min, gain_max, drift_max = check_raw_gain(policy, states)
    return {'samples': int(len(eta)),
            'response_rmse': float(np.sqrt(np.mean((response - eta) ** 2))),
            'nominal_inverse_response_rmse': float(np.sqrt(np.mean((nominal - eta) ** 2))),
            'g0_min': gain_min, 'g0_max': gain_max, 'f0_abs_max': drift_max}


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
    loaded = torch.jit.load(str(path), map_location='cpu')
    with torch.no_grad():
        for value in loaded(states[:1]):
            if value.shape != (1, 1) or value.device.type != 'cpu':
                raise ValueError('Exported prior must return CPU (f0[N,1], g0[N,1])')


def train(args, log=print):
    config = TrainingConfig(epochs=args.epochs, batch_size=args.batch_size, fd_epsilon=args.fd_epsilon,
                            neighbours=args.neighbours, seed=args.seed, criterion=args.criterion)
    config.validate()
    groups = read_records(args.records, config)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(config.seed)
    np.random.seed(config.seed)
    random.seed(config.seed)
    device = torch.device(args.device)
    backend_factory = make_backend_factory(args, config)
    log('Records: ' + ', '.join(f'{split} {len(groups[split]["timestamp"])}' for split in SPLITS) +
        f'  | criterion {config.criterion}  | backend {args.backend}')
    manifest = {'method': 'relative GAN (Eqs. 14-16)' if config.criterion == 'gan' else 'MLP-MSE baseline',
                'state_features': STATE_FEATURES, 'input_shape': ['N', 6],
                'output': ['f0[N,1]', 'g0[N,1]'], 'input_limits': INPUT_LIMITS,
                'records': str(Path(args.records).resolve()), 'backend': args.backend,
                'config': asdict(config), 'torch_version': torch.__version__,
                'split_sizes': {split: int(len(groups[split]['timestamp'])) for split in SPLITS},
                'split_time_bounds': {split: [float(groups[split]['timestamp'][0]),
                                              float(groups[split]['timestamp'][-1])] for split in SPLITS},
                'results': {}}
    policies = {}
    for axis in AXES:
        policies[axis] = train_axis(axis, groups['train'], config, device, backend_factory, log)
        manifest['results'][axis] = {split: evaluate(policies[axis], groups[split], axis,
                                                     backend_factory(groups[split], AXES.index(axis)), device)
                                     for split in ('validation', 'test')}
        test = manifest['results'][axis]['test']
        log(f'[{axis}] test response RMSE {test["response_rmse"]:.3f} m/s^2 '
            f'(nominal inverse {test["nominal_inverse_response_rmse"]:.3f})')
    all_states = torch.as_tensor(np.concatenate([groups[split]['state'] for split in SPLITS]),
                                 dtype=torch.float32)
    for axis, policy in policies.items():
        export_prior(policy, output / f'generator_prior_{axis}.pt', all_states.to(device))
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    log(f'Exported generator_prior_{{X,Y,Z}}.pt and manifest.json to {output}')
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--records', required=True, help='synchronized flight-record CSV')
    parser.add_argument('--output', default=str(DEFAULT_OUTPUT), help='directory for the exported priors')
    parser.add_argument('--backend', default='logged', help="'logged' or module:factory for on-vehicle measurement")
    parser.add_argument('--backend-config', help='JSON settings passed to a module:factory backend')
    parser.add_argument('--criterion', choices=('gan', 'mse'), default='gan')
    parser.add_argument('--epochs', type=int, default=2000)
    parser.add_argument('--batch-size', type=int, default=1024)
    parser.add_argument('--fd-epsilon', type=float, default=0.05, help='finite-difference action step, m/s^2')
    parser.add_argument('--neighbours', type=int, default=32, help='records per local sensitivity fit')
    parser.add_argument('--seed', type=int, default=1011)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args(argv)
    try:
        train(args)
    except (ValueError, OSError) as exc:
        parser.exit(2, 'Training stopped: ' + str(exc) + '\n')


if __name__ == '__main__':
    main()
