import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'Trajectory_Generation.py'
spec = importlib.util.spec_from_file_location('trajectory_under_test', SCRIPT)
m = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = m
spec.loader.exec_module(m)


def protocol():
    trial = m.TrialProtocol(m.TrialConfig())
    trial.start(100., [(0., .75, 1.), (0., -.75, 1.)])
    return trial


def test_analytic_derivatives_match_central_differences():
    config = m.TrialConfig()
    step = 1e-5
    for t in (.0, .317, 1.7, 4.99):
        derivatives = m.figure8(t, config)
        left, right = m.figure8(t-step, config), m.figure8(t+step, config)
        for order in range(3):
            for axis in range(3):
                estimate = (right[order][axis]-left[order][axis])/(2*step)
                assert derivatives[order+1][axis] == pytest.approx(estimate, abs=1e-6)


def test_shared_epoch_exact_formation_and_reset_gate():
    trial = protocol()
    assert trial.reference(100.)[0][0] == (0., .75, 1.)
    assert trial.reference(100.)[0][1:] == ((0., 0., 0.),)*3
    assert not trial.reset_due(108.99)
    assert trial.reset_due(109.)
    trial.begin_reset(109., 110.)
    assert trial.scoring_start is None
    assert trial.acknowledge_resets(109.025, [True, True])
    assert not trial.activate_scoring(109.9)
    assert trial.scoring_start is None
    assert trial.activate_scoring(110.005)
    assert trial.scoring_start == 110.
    for stamp in (110.025, 123.7, 170.):
        first, second = trial.reference(stamp)
        assert tuple(a-b for a,b in zip(first[0], second[0])) == pytest.approx((0., 1.5, 0.))
        assert first[1:] == second[1:]
    failed = protocol()
    failed.begin_reset(109., 110.)
    assert not failed.acknowledge_resets(110., [True, False])
    assert failed.scoring_start is None and failed.state == 'aborted'


def test_start_and_stop_are_smooth_through_acceleration_and_jerk():
    trial = protocol()
    trial.begin_reset(109., 110.)
    trial.acknowledge_resets(109.01, [True, True])
    trial.activate_scoring(110.)
    for boundary in (105., 170., 173.):
        left = trial.reference(boundary-1e-6)[0]
        right = trial.reference(boundary+1e-6)[0]
        for order in range(4):
            assert left[order] == pytest.approx(right[order], abs=2e-3)
    held = trial.reference(200.)[0]
    assert held[1:] == ((0., 0., 0.),)*3
    assert trial.reference(201.) == trial.reference(200.)


def measured(trial, stamp, vehicle, displacement=(0., 0., 0.)):
    ref = trial.reference(stamp)[vehicle]
    return m.OdomSample(stamp, tuple(a+b for a,b in zip(ref[0], displacement)), ref[1])


def test_metrics_reject_missing_stale_duplicate_and_unsynchronized_coverage():
    trial = protocol()
    trial.begin_reset(109., 110.)
    trial.acknowledge_resets(109.01, [True, True])
    trial.activate_scoring(110.)
    metrics = m.TrialMetrics(trial, .01)
    assert metrics.add(measured(trial, 110., 0), measured(trial, 110., 1))
    assert not metrics.add(measured(trial, 110., 0), measured(trial, 110., 1))
    assert not metrics.add(measured(trial, 110.02, 0), measured(trial, 110.04, 1))
    assert not metrics.add(measured(trial, 109.999, 0), measured(trial, 110.001, 1))
    report = metrics.report(170.)
    assert report['coverage'] == pytest.approx(1/6000)
    assert report['full_trial_sync_rmse'] is None
    assert report['full_trial_tracking_rmse'] == [None, None]
    assert report['observed_sync_rmse'] == pytest.approx(0.)


def test_full_trial_and_startup_rmse_include_backup_observations():
    trial = protocol()
    trial.begin_reset(109., 110.)
    trial.acknowledge_resets(109.01, [True, True])
    trial.activate_scoring(110.)
    metrics = m.TrialMetrics(trial, .01)
    for k in range(6000):
        stamp = 110. + k*.01
        if k == 100:
            trial.abort('Synthetic command interruption; sensing continues during backup')
        assert metrics.add(measured(trial, stamp, 0, (0.1, 0., 0.)), measured(trial, stamp, 1))
    report = metrics.report(170.)
    assert report['complete_metrics'] and report['coverage'] == 1.
    assert report['full_trial_sync_rmse'] == pytest.approx(.1)
    assert report['startup_sync_rmse'] == pytest.approx(.1)
    assert report['full_trial_tracking_rmse'] == pytest.approx([.1, 0.])
    assert report['trial_interrupted']


def test_pairing_chooses_next_unused_nearest_sample():
    trial = protocol()
    trial.begin_reset(109., 110.)
    trial.acknowledge_resets(109.01, [True, True])
    trial.activate_scoring(110.)
    adapter = object.__new__(m.DualDroneManager)
    adapter.protocol = trial
    adapter.metrics = m.TrialMetrics(trial, .01)
    adapter.max_skew, adapter.odom_timeout, adapter.frame = .01, .1, 'map'
    adapter.odom = [[measured(trial, 110.+t, 0) for t in (0., .010)],
                    [measured(trial, 110.+t, 1) for t in (.006, .016)]]
    adapter._collect_metrics(110.04)
    assert len(adapter.metrics.records) == 2


def test_adapter_refuses_stale_or_unready_start_without_ros():
    adapter = object.__new__(m.DualDroneManager)
    adapter.frame = 'map'
    adapter.odom_timeout, adapter.state_timeout, adapter.ready_timeout = .1, 1., .5
    adapter.hover_window, adapter.hover_speed = 1., .15
    adapter.odom = [[m.OdomSample(t, (0., 0., 1.), (0., 0., 0.)) for t in (99., 99.5, 100.)] for _ in range(2)]
    adapter.fcu = [(100., True, True, 'OFFBOARD')]*2
    adapter.ready = [(True, 100.)]*2
    assert adapter._valid(100., require_hover=True)[0]
    assert not adapter._valid(100.2)[0]
    adapter.ready[0] = (False, 100.)
    assert not adapter._valid(100., require_hover=True)[0]
    assert adapter._valid(100., require_hover=False)[0]  # CMD_CTRL needn't remain AUTO_HOVER


def test_late_reset_ack_aborts_without_scoring_or_retry():
    trial = protocol()
    with pytest.raises(ValueError, match='reservation window'):
        trial.begin_reset(108., 110.)
    trial.begin_reset(109., 110.)
    assert not trial.acknowledge_resets(110., [True, True])
    assert trial.state == 'aborted' and trial.scoring_start is None
    assert not trial.activate_scoring(112.)
    with pytest.raises(ValueError):
        trial.begin_reset(112., 113.)


def test_stale_odometry_stops_both_streams_and_never_auto_resumes():
    import threading
    adapter = object.__new__(m.DualDroneManager)
    adapter.protocol = protocol()
    adapter.lock = threading.RLock()
    clock = [101.]
    adapter.rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: clock[0])),
                                    logerr=lambda *args: None)
    adapter.last_tick = 100.5
    adapter.frame, adapter.odom_timeout, adapter.state_timeout = 'map', .1, 1.
    adapter.odom = [[m.OdomSample(100., (0., 0., 1.), (0., 0., 0.))] for _ in range(2)]
    adapter.fcu = [(101., True, True, 'OFFBOARD')]*2
    adapter._collect_metrics = lambda now: None
    adapter._save_summary = lambda now: None
    publications = []
    adapter._publish = lambda *args: publications.append(args)
    adapter._tick(None)
    assert adapter.protocol.state == 'aborted' and publications == []
    clock[0] = 102.
    adapter.odom = [[m.OdomSample(102., (0., 0., 1.), (0., 0., 0.))] for _ in range(2)]
    adapter.fcu = [(102., True, True, 'OFFBOARD')]*2
    adapter._tick(None)
    assert publications == []


def test_reset_service_is_not_called_after_wait_crosses_reserved_epoch():
    import threading
    adapter = object.__new__(m.DualDroneManager)
    adapter.protocol = protocol()
    adapter.protocol.begin_reset(109., 110.)
    adapter.lock = threading.RLock()
    adapter.names, adapter.reset_timeout = ['drone1', 'drone2'], 3.
    adapter.reset_results = [None, None]
    adapter.Trigger = object
    clock, calls = [109.1], []
    def delayed_service_wait(*args, **kwargs):
        clock[0] = 110.01
    adapter.rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: clock[0])),
                                    wait_for_service=delayed_service_wait,
                                    ServiceProxy=lambda *args: calls.append(args))
    adapter._request_reset(0)
    assert calls == []
    assert adapter.reset_results[0][0] is False


def test_configurable_lead_only_changes_reservation_time_not_scoring_epoch():
    trial = protocol()
    assert not trial.reset_due(107.99, 2.)
    assert trial.reset_due(108., 2.)
    trial.begin_reset(108.03, 110., 2.)
    assert trial.acknowledge_resets(108.9, [True, True])
    assert not trial.activate_scoring(109.999)
    assert trial.activate_scoring(110.004)
    assert trial.scoring_start - trial.start_time == 10.
    for lead in (0., -1., 10.01, float('nan')):
        with pytest.raises(ValueError):
            protocol().reset_due(109., lead)
    with pytest.raises(ValueError, match='end of the 10 s warm-up'):
        protocol().begin_reset(109., 111.)


def test_timer_schedules_async_before_ten_seconds_and_aborts_missed_epoch(monkeypatch):
    import threading
    pending_workers = []
    class QueuedWorker:
        def __init__(self, target, daemon):
            self.target = target
            assert daemon
        def start(self):
            pending_workers.append(self.target)  # Simulates scheduling, never a service call.
    monkeypatch.setattr(m.threading, 'Thread', QueuedWorker)
    adapter = object.__new__(m.DualDroneManager)
    adapter.protocol, adapter.config = protocol(), m.TrialConfig()
    adapter.lock = threading.RLock()
    clock = [109.03]  # Timer jitter must not move the fixed scoring epoch.
    adapter.rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: clock[0])),
                                    loginfo=lambda *args: None, logerr=lambda *args: None)
    adapter.last_tick, adapter.reset_lead, adapter.reset_timeout = 108.99, 1., 3.
    adapter.reset_results = [None, None]
    adapter._collect_metrics = lambda now: None
    adapter._valid = lambda now: (True, '')
    adapter._reserve_resets = lambda: pytest.fail('Services must not run inside the timer callback')
    publications = []
    adapter._publish = lambda *args: publications.append(args)
    adapter._tick(None)
    assert len(pending_workers) == 1 and len(publications) == 1
    assert adapter.protocol.reset_epoch == 110.
    adapter.reset_results = [(True, 'reserved', 109.3), (True, 'reserved', 109.4)]
    clock[0] = 109.5
    adapter._tick(None)
    assert adapter.protocol.scoring_start is None
    clock[0] = 110.007
    adapter._tick(None)
    assert adapter.protocol.scoring_start == 110.
    assert len(publications) == 3
    # Missing the reservation window aborts instead of moving the scoring epoch.
    adapter.protocol = protocol()
    adapter.reset_results = [None, None]
    clock[0] = 110.1
    adapter._tick(None)
    assert adapter.protocol.state == 'aborted' and adapter.protocol.scoring_start is None
    assert len(publications) == 3 and len(pending_workers) == 1
    # Timely acknowledgements are judged by receipt time, not a jittered tick.
    adapter.protocol = protocol()
    adapter.protocol.begin_reset(109., 110.)
    adapter.last_tick = 109.99
    adapter.reset_results = [(True, 'reserved', 109.995), (True, 'reserved', 109.996)]
    clock[0] = 110.007
    adapter._tick(None)
    assert adapter.protocol.scoring_start == 110. and len(publications) == 4
    adapter.protocol = protocol()
    adapter.protocol.begin_reset(109., 110.)
    adapter.last_tick = 109.99
    adapter.reset_results = [(True, 'reserved', 109.995), (True, 'late', 110.001)]
    adapter._tick(None)
    assert adapter.protocol.state == 'aborted' and len(publications) == 4
