"""Method/interface checks. Synthetic plants below are unit-test fixtures only."""
import copy
import csv
import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest
import torch

UTILS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(UTILS))


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, UTILS/filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


m = load('offline_training_under_test', 'offline_GANs_Train.py')
exporter = load('flight_export_under_test', 'export_flight_records.py')
collector = load('physical_collector_under_test', 'physical_fd_collector.py')


class AffinePlantFixture:
    def __init__(self, drift=1.0, gain=2.0):
        self.drift, self.gain = drift, gain
        self.calls = []

    def responses(self, index, actions):
        self.calls.append(actions.copy())
        return self.drift+self.gain*actions


def test_fd_chain_rule_and_discriminator_ascent():
    torch.manual_seed(7)
    policy, discriminator = m.AffinePolicy(), m.RelativeDiscriminator()
    state, eta = torch.zeros(2, 6), torch.tensor([[.3], [.5]])
    config, plant = m.TrainingConfig(), AffinePlantFixture()
    action = policy.action(state, eta).detach().requires_grad_(True)
    m.loss_per_sample(discriminator, eta, 1+2*action, config).sum().backward()
    expected = action.grad.mean().item()
    frozen_d = torch.optim.SGD(discriminator.parameters(), lr=0)
    opt_g = torch.optim.SGD(policy.parameters(), lr=0)
    m.fd_policy_step(policy, discriminator, opt_g, frozen_d, state, eta, plant, np.arange(2), config, (-3, 3))
    assert policy.output.bias.grad[0].item() == pytest.approx(expected, rel=1e-4, abs=1e-6)
    assert plant.calls[0] == pytest.approx(np.array([[.25, .3, .35], [.45, .5, .55]]))
    before = m.loss_per_sample(discriminator, eta, 1+2*eta, config).mean().item()
    opt_d = torch.optim.SGD(discriminator.parameters(), lr=1e-3)
    m.fd_policy_step(policy, discriminator, opt_g, opt_d, state, eta, plant, np.arange(2), config, (-3, 3))
    assert m.loss_per_sample(discriminator, eta, 1+2*eta, config).mean().item() > before
    assert sum(p.numel() for p in policy.parameters()) == 578
    assert sum(p.numel() for p in discriminator.parameters()) == 257
    assert isinstance(discriminator.network[-1], torch.nn.Linear)


def test_out_of_range_action_is_rejected_without_query_or_clipping():
    policy, discriminator, plant = m.AffinePolicy(), m.RelativeDiscriminator(), AffinePlantFixture()
    with pytest.raises(ValueError, match='physical input interval'):
        m.fd_policy_step(policy, discriminator, torch.optim.SGD(policy.parameters(), lr=0),
                         torch.optim.SGD(discriminator.parameters(), lr=0), torch.zeros(1, 6),
                         torch.tensor([[3.0]]), plant, np.array([0]), m.TrainingConfig(), (-3, 3))
    assert plant.calls == []


def test_raw_gain_and_interval_reject_nonpositive_theta2():
    policy = m.AffinePolicy()
    with torch.no_grad():
        policy.output.bias[1] = -.5
    with pytest.raises(ValueError, match='raw gain'):
        m.check_raw_gain(policy, torch.zeros(1, 6))
    with pytest.raises(AssertionError, match='positive Theta2'):
        m.ExportedPrior(policy).interval_bounds(-torch.ones(6), torch.ones(6))
    with torch.no_grad():
        policy.output.bias[1] = 2e6
    with pytest.raises(ValueError, match='raw gain'):
        m.check_raw_gain(policy, torch.zeros(1, 6))


def test_export_matches_cpp_forward_and_interval_contract(tmp_path):
    torch.manual_seed(9)
    policy = m.AffinePolicy(mean=np.ones(6), scale=2*np.ones(6))
    with torch.no_grad():
        policy.output.weight.normal_(std=.002)
        policy.output.bias.copy_(torch.tensor([.2, 1.0]))
    states = torch.tensor([[-1., 0, 1, .2, -.2, .3], [1., -1, 0, 0, .4, -.3]])
    path = tmp_path/'generator_prior_X.pt'
    m.export_prior(policy, path, states)
    prior = torch.jit.load(str(path), map_location='cpu')
    f, g = prior(states)
    assert f.shape == g.shape == (2, 1)
    assert f.dtype == g.dtype == torch.float32
    assert f.device.type == g.device.type == 'cpu'
    bounds = prior.interval_bounds(-torch.ones(6, dtype=torch.float64), torch.ones(6, dtype=torch.float64))
    assert all(value.ndim == 0 and value.dtype == torch.float64 for value in bounds)
    assert (f >= bounds[0]).all() and (f <= bounds[1]).all()
    assert (g >= bounds[2]).all() and (g <= bounds[3]).all()
    assert bounds[2] >= m.RAW_GAIN_MIN
    torch.testing.assert_close(prior.policy.mean, torch.ones(6))
    with pytest.raises((torch.jit.Error, RuntimeError)):
        prior(torch.zeros(1, 1))


def test_fixed_chronological_split_has_two_second_exclusions(tmp_path):
    rows = [[.1*i, 'flight', 'fans_off', *([0.]*12)] for i in range(400)]
    split = exporter.partition_records(rows)
    path = tmp_path/'records.csv'
    exporter.write_records(split, path)
    records = m.read_records(path)
    assert [records[i][3] for i in m.training_block(records)] == ['train']*len(m.training_block(records))
    for left, right in (('train', 'validation'), ('validation', 'test')):
        last = max(row[0] for row in split if row[-1] == left)
        first = min(row[0] for row in split if row[-1] == right)
        assert first-last >= 2.0
    assert path.read_text().splitlines()[0].split(',') == m.RECORD_COLUMNS
    split[0][-1] = 'test'
    exporter.write_records(split, path)
    with pytest.raises(ValueError, match='chronological'):
        m.read_records(path)


def physical_fixture():
    data = {'state': np.zeros((1, 6)), 'u': np.zeros((1, 3)), 'eta': np.full((1, 3), .3),
            'environment': ['fans_off'], 'flight': ['unit_fixture']}
    settings = {'state_tolerance': [1e-3]*6, 'action_tolerance': 1e-5,
                'max_sensor_skew': .01, 'max_response_delay': .03, 'max_probe_span': .1}

    def receipt(request):
        # Fabricated receipt used only to exercise validation, never training.
        stamps = np.array([[10.01, 10.03, 10.05]])
        return {**{key: copy.deepcopy(request[key]) for key in ('protocol', 'request_id', 'measurement_source',
                                                               'environment', 'probe_order')},
                'state': np.zeros((1, 3, 6)).tolist(), 'executed_commands': request['commands'],
                'net_acceleration': (1+2*np.asarray(request['commands'])).tolist(),
                'command_stamp': (stamps-.01).tolist(), 'response_stamp': stamps.tolist(),
                'imu_stamp': stamps.tolist(), 'odom_stamp': stamps.tolist()}
    return data, settings, receipt


def test_physical_protocol_rejects_wrong_action_state_and_reuse():
    data, settings, receipt = physical_fixture()
    actions = np.array([[.25, .3, .35]])
    backend = m.PhysicalResponseBackend(data, 0, receipt, settings)
    assert backend.responses([0], actions) == pytest.approx(1+2*actions)
    with pytest.raises(ValueError, match='Reused'):
        backend.responses([0], actions)
    for field, match in (('executed_commands', 'requested probes'), ('state', 'repeated-state')):
        def invalid(request):
            result = receipt(request)
            result[field][0][0][0] += 1
            return result
        with pytest.raises(ValueError, match=match):
            m.PhysicalResponseBackend(data, 0, invalid, settings).responses([0], actions)


def test_sync_and_thrust_mapping_use_net_acceleration_coordinates():
    identity = (1., 0., 0., 0.)
    odom = [(10., (0., 0., 1.), (0., 0., 0.), identity)]
    imu = [(10.002, (0., 0., 9.81), identity)]
    command = [(9.995, identity, .5)]
    rows = list(exporter.synchronize(odom, imu, command, 'flight', 'fans_off', .5))
    assert rows[0][9:15] == pytest.approx([0.]*6, abs=1e-12)
    # A command transition between aligned sensors prevents a valid pairing.
    assert list(exporter.synchronize(odom, imu, command+[(10.001, identity, .6)], 'flight', 'fans_off', .5)) == []
    config = {'gra': 9.81, 'mass': .9, 'max_angle': 80.,
              'thrust_model': {'accurate_thrust_model': False, 'hover_percentage': .5},
              'rtmpc': {'xy': {'limit_u_min': -3., 'limit_u_max': 3.},
                        'z': {'limit_u_min': -2., 'limit_u_max': 5.}}}
    for command in ([0., 0., 0.], [1., -.5, 2.]):
        q, thrust, achieved = collector.map_command(command, .2, float('nan'), config)
        reconstructed = exporter.executed_net_acceleration(q, thrust, identity, identity, .5)
        assert achieved == pytest.approx(command, abs=2e-6)
        assert reconstructed == pytest.approx(achieved, abs=1e-10)
    with pytest.raises(ValueError, match='input interval'):
        collector.map_command([4., 0., 0.], 0., float('nan'), config)
