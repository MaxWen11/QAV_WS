"""Unit tests for the offline prior extraction; the plants are test fixtures."""
import csv
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import torch

UTILS = Path(__file__).resolve().parents[1]


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, UTILS / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


m = load('offline_training_under_test', 'offline_GANs_Train.py')
exporter = load('flight_record_export_under_test', 'export_flight_records.py')


class AffineBackend:
    def __init__(self, drift=1.0, gain=2.0):
        self.drift, self.gain = drift, gain

    def responses(self, index, actions):
        return self.drift + self.gain * actions


def write_records(path, count=240, gain=1.4, noise=0.02, seed=3, split=None):
    rng = np.random.default_rng(seed)
    with path.open('w', newline='') as stream:
        columns = m.RECORD_COLUMNS + (['split'] if split else [])
        writer = csv.writer(stream)
        writer.writerow(columns)
        for k in range(count):
            state = rng.normal(0.0, [0.5, 0.5, 0.2, 0.3, 0.3, 0.1])
            u = rng.uniform(-1.0, 1.0, 3)
            a = 0.2 * state[3:6] + gain * u + noise * rng.normal(size=3)
            eta = rng.uniform(-1.0, 1.0, 3)
            row = [0.01 * k, *state, *u, *a, *eta]
            if split:
                row.append(split(k))
            writer.writerow(row)


def test_fd_policy_gradient_matches_true_action_gradient():
    torch.manual_seed(7)
    policy = m.AffinePolicy()
    discriminator = m.RelativeDiscriminator()
    state = torch.zeros(2, 6)
    eta = torch.tensor([[0.3], [0.5]])
    action = policy.action(state, eta).detach().flatten().tolist()
    config = m.TrainingConfig(criterion='mse')
    optimizer = torch.optim.SGD(policy.parameters(), lr=0.001)
    m.fd_policy_step(policy, discriminator, optimizer, None, state, eta, AffineBackend(),
                     np.arange(2), config, (-100.0, 100.0))
    expected = np.mean([2 * (1 + 2 * u - target) * 2 for u, target in zip(action, eta.flatten().tolist())])
    assert policy.output.bias.grad[0].item() == pytest.approx(expected, rel=1e-4)
    assert policy.action(state, eta).mean() < sum(action) / len(action)


def test_discriminator_ascends_bounded_squared_margin():
    torch.manual_seed(11)
    policy, discriminator = m.AffinePolicy(), m.RelativeDiscriminator()
    state, eta = torch.zeros(4, 6), torch.ones(4, 1) * 0.5
    config = m.TrainingConfig()
    response = torch.full((4, 1), 2.0)
    before = m.loss_per_sample(discriminator, eta, response, config).mean().item()
    opt_g = torch.optim.SGD(policy.parameters(), lr=0.0)
    opt_d = torch.optim.SGD(discriminator.parameters(), lr=1e-2)
    backend = AffineBackend(drift=2.0, gain=0.0)
    m.fd_policy_step(policy, discriminator, opt_g, opt_d, state, eta, backend, np.arange(4), config, (-3.0, 3.0))
    after = m.loss_per_sample(discriminator, eta, response, config).mean().item()
    assert after > before
    assert (discriminator(torch.randn(64, 2) * 100).abs() <= 1.0).all()
    assert policy.hidden.in_features == 6 and policy.hidden.out_features == 64
    assert len([layer for layer in discriminator.modules() if isinstance(layer, torch.nn.Linear)]) == 2


def test_raw_gain_below_threshold_is_rejected():
    policy = m.AffinePolicy()
    with torch.no_grad():
        policy.output.bias[1] = 2e6
    with pytest.raises(ValueError, match='raw gain'):
        m.check_raw_gain(policy, torch.zeros(1, 6))


def test_export_is_cpu_tuple_with_embedded_normalization(tmp_path):
    path = tmp_path / 'generator_prior_X.pt'
    states = torch.randn(5, 6)
    policy = m.AffinePolicy(mean=np.ones(6), scale=2.0 * np.ones(6))
    m.export_prior(policy, path, states)
    loaded = torch.jit.load(str(path), map_location='cpu')
    f, g = loaded(states)
    assert f.shape == g.shape == (5, 1)
    assert f.device.type == g.device.type == 'cpu'
    torch.testing.assert_close(g, torch.ones_like(g))
    torch.testing.assert_close(loaded.policy.mean, torch.ones(6))
    with pytest.raises((torch.jit.Error, RuntimeError)):
        loaded(torch.zeros(1, 1))


def test_records_form_ordered_time_blocks(tmp_path):
    path = tmp_path / 'records.csv'
    write_records(path, count=100)
    groups = m.read_records(path, m.TrainingConfig())
    assert [len(groups[s]['timestamp']) for s in m.SPLITS] == [70, 15, 15]
    assert groups['train']['timestamp'].max() < groups['validation']['timestamp'].min()
    write_records(path, count=100, split=lambda k: 'train' if k % 2 else 'test')
    with pytest.raises(ValueError):
        m.read_records(path, m.TrainingConfig())
    path.write_text('timestamp,p_x\n0,0\n')
    with pytest.raises(ValueError, match='missing columns'):
        m.read_records(path, m.TrainingConfig())
    with pytest.raises(SystemExit) as error:
        m.main([])
    assert error.value.code == 2


def test_local_sensitivity_recovers_input_gain():
    rng = np.random.default_rng(5)
    state = rng.normal(size=(400, 6))
    u = rng.uniform(-1.5, 1.5, 400)
    a = 0.3 + 1.7 * u + 0.01 * rng.normal(size=400)
    slope = m.local_input_sensitivity(state, u, a, neighbours=32, ridge=1e-3)
    assert np.median(slope) == pytest.approx(1.7, abs=0.02)


def test_end_to_end_training_exports_priors(tmp_path):
    records = tmp_path / 'records.csv'
    write_records(records, count=240)
    args = SimpleNamespace(records=str(records), output=str(tmp_path / 'models'), backend='logged',
                           backend_config=None, criterion='gan', epochs=3, batch_size=64,
                           fd_epsilon=0.05, neighbours=16, seed=1, device='cpu')
    manifest = m.train(args, log=lambda *_: None)
    for axis in m.AXES:
        loaded = torch.jit.load(str(tmp_path / 'models' / f'generator_prior_{axis}.pt'))
        f, g = loaded(torch.zeros(3, 6))
        assert torch.isfinite(f).all() and (g > 0).all()
        assert manifest['results'][axis]['test']['samples'] == 36
    saved = json.loads((tmp_path / 'models' / 'manifest.json').read_text())
    assert saved['split_sizes'] == {'train': 168, 'validation': 36, 'test': 36}


def test_flight_record_export_pairs_command_with_next_response(tmp_path):
    def message(k, valid=True):
        return SimpleNamespace(controller_valid=valid, real_x=k, real_y=0.0, real_z=1.0,
                               real_vx=0.1, real_vy=0.0, real_vz=0.0,
                               des_a_x=0.1 * k, des_a_y=0.0, des_a_z=0.0,
                               fb_a_x=10.0 + k, fb_a_y=0.0, fb_a_z=0.0, eta=(0.2 * k, 0.0, 0.0))
    messages = [(100.00, message(0)), (100.01, message(1)), (100.02, message(2, valid=False)),
                (100.05, message(3)), (100.06, message(4))]
    rows = list(exporter.records_from_messages(messages))
    assert [row[0] for row in rows] == [100.00, 100.05]
    assert rows[0][7] == pytest.approx(0.0) and rows[0][10] == pytest.approx(11.0)
    assert rows[1][13] == pytest.approx(0.6)
    path = tmp_path / 'records.csv'
    exporter.write_records(rows, path)
    assert path.read_text().splitlines()[0].split(',') == m.RECORD_COLUMNS
