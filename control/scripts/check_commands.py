"""Offline contract checks for managed authority; never imports the hardware SDK."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT.parent / 'src' if (ROOT.parent / 'src/embodied_contracts').is_dir() else ROOT / 'shared')]
from adapters.commands import CommandEngine, MotionPlanner
from arx5_vr.types import RobotState, ControlCommand, Mode
from arx5_vr.settings import load_settings


class CommandsTests(unittest.TestCase):
    def setUp(self):
        self.config = load_settings().robot
        self.engine = CommandEngine('run', self.config, 'model')
        self.now, self.sequence = 10., 0
        self.state = RobotState(np.asarray(self.config.park_q), np.zeros(6), np.array([0,0,0,0,0,0,1.]), .04, self.now)
        self.lease = self.request('acquire', source='cli')
        self.engine.next(self.state, self.now)

    def request(self, action, **fields):
        self.sequence += 1
        lease = getattr(self, 'lease', {})
        return self.engine.request({'version': 1, 'runtime_id': 'run', 'action': action,
            'sent_monotonic_ns': round(self.now*1e9), 'sequence': self.sequence,
            'token': lease.get('token'), 'epoch': lease.get('epoch'), **fields}, self.state, self.now)

    def test_single_owner_sequence_and_epoch(self):
        with self.assertRaisesRegex(ValueError, 'already owned'):
            self.request('acquire', source='phone')
        self.request('heartbeat')
        with self.assertRaisesRegex(ValueError, 'sequence'):
            self.request('heartbeat', sequence=1)
        self.request('release')
        self.lease = self.request('acquire', source='phone')
        with self.assertRaisesRegex(ValueError, 'epoch'):
            self.request('joint', q=self.state.q.tolist(), epoch=self.lease['epoch']-2)
        command = self.engine.next(self.state, self.now)
        self.assertTrue(command.hold and command.reset_reference)
        self.assertEqual(command.mode, Mode.COMMAND_FOLLOWING)

    def test_target_age_not_refreshed_by_heartbeat_or_queue(self):
        self.request('joint', q=(self.state.q+.001).tolist(), ttl_ms=100)
        self.now += .08
        self.request('heartbeat')
        self.assertIsNotNone(self.engine.next(self.state, self.now).joint_target)
        self.now += .03
        self.request('heartbeat')
        self.assertTrue(self.engine.next(self.state, self.now).hold)
        with self.assertRaisesRegex(ValueError, 'expired'):
            self.request('joint', q=self.state.q.tolist(), sent_monotonic_ns=round((self.now-.11)*1e9))
        with self.assertRaisesRegex(ValueError, 'source sequence'):
            self.request('joint', q=self.state.q.tolist(), source_sequence=2**64)

    def test_lease_expiry_cancel_and_stop_hold_without_home(self):
        self.request('joint', q=self.state.q.tolist())
        self.now += .51
        command = self.engine.next(self.state, self.now)
        self.assertIsNone(self.engine.owner)
        self.assertTrue(command.hold and command.reset_reference)
        self.assertEqual(command.mode, Mode.COMMAND_FOLLOWING)
        self.request('stop')
        self.assertTrue(self.engine.stopped)
        with self.assertRaisesRegex(ValueError, 'latched'):
            self.request('acquire', source='cli')

    def test_execute_identity_start_state_idempotence_and_cancel(self):
        planner = MotionPlanner(self.config, 'model')
        snapshot = self.engine.status(self.state, self.now)
        plan = planner.plan({'q': (self.state.q + [.04,0,0,0,0,0]).tolist()}, snapshot, self.now)
        self.request('execute', _plan=plan)
        self.state.q = self.state.q + [.04,0,0,0,0,0]
        self.assertEqual(self.request('execute', _plan=plan)['status'], 'running')
        self.request('cancel')
        self.assertEqual(self.engine.operations[plan['plan_id']]['status'], 'cancelled')
        self.assertTrue(self.engine.next(self.state, self.now).reset_reference)
        bad = {**plan, 'model_id': 'other'}
        with self.assertRaisesRegex(ValueError, 'owner/model'):
            self.request('execute', _plan=bad)
        bad = {**plan, 'plan_id': 'new'}
        with self.assertRaisesRegex(ValueError, 'moved'):
            self.request('execute', _plan=bad)

    def test_explicit_ingress_epoch_deadline_and_fault_are_source_independent(self):
        from adapters.ingress import Candidate
        self.request('release')
        self.engine.ingress_channels = {'xr-test'}
        self.lease = self.request('acquire', source='arbitrary_label', ingress='xr-test', purpose='teleop')
        self.engine.next(self.state, self.now)
        command = ControlCommand(Mode.COMMAND_FOLLOWING, pose_target=self.state.pose)
        def candidate(**fields):
            return {'xr-test': Candidate(**dict(epoch=self.engine.epoch, command=command,
                                                expires_at=self.now+.1, phase='teleop') | fields)}
        self.assertIs(self.engine.next(self.state, self.now, candidate()), command)
        self.assertEqual(self.engine.phase, 'teleop')
        self.assertTrue(self.engine.next(self.state, self.now, candidate(epoch=self.engine.epoch-1)).hold)
        self.assertTrue(self.engine.next(self.state, self.now, candidate(expires_at=self.now)).hold)
        with self.assertRaisesRegex(ValueError, 'installed ingress'):
            self.request('joint', q=self.state.q.tolist())
        self.request('cancel')
        self.assertTrue(self.engine.next(self.state, self.now, candidate()).hold)
        self.engine.next(self.state, self.now, candidate(fault='router exited'))
        self.assertIsNone(self.engine.owner)
        self.assertEqual(self.engine.last_revocation['reason'], 'router exited')
        self.lease = self.request('acquire', source='phone', purpose='teleop')
        self.request('joint', q=self.state.q.tolist())
        self.engine.next(self.state, self.now, candidate(fault='router exited'))
        self.assertEqual(self.engine.owner['source'], 'phone')
        self.assertIsNotNone(self.engine.next(self.state, self.now, candidate(fault='router exited')).joint_target)

    def test_joint_and_linear_plans_obey_sampled_derivative_bounds(self):
        planner = MotionPlanner(self.config, 'model')
        q = np.array([0., .6, 1., 0., .2, 0.])
        snapshot = {**self.engine.status(self.state, self.now), 'q': q.tolist()}
        target = planner.model.fk(q + [.002,0,0,0,0,0])
        for request in ({'kind':'movej','q':(q+.02).tolist()}, {'kind':'movel','pose':target.tolist()}):
            plan = planner.plan(request, snapshot, self.now)
            v = np.gradient(plan['q'], plan['times'], axis=0)
            a = np.gradient(v, plan['times'], axis=0)
            j = np.gradient(a, plan['times'], axis=0)
            self.assertLessEqual(np.max(np.abs(v)), .1515)
            self.assertLessEqual(np.max(np.abs(a)), .505)
            self.assertLessEqual(np.max(np.abs(j)), 2.02)


class NativeGateTests(unittest.TestCase):
    def setUp(self):
        from adapters.sources import NativeIngress
        with patch('adapters.sources.RobotBridge'), patch('adapters.sources.create_node'), patch('adapters.sources.open_service'), patch('adapters.sources.make_subscriber'):
            self.source = NativeIngress('test', ['input','coordinator','arm','gripper'])
        self.state = SimpleNamespace(gripper_width=.04)
        self.command = ControlCommand(Mode.COMMAND_FOLLOWING)
        self.source.bridge.poll_commands.return_value = self.command
        self.now, self.sequence = 10., 0

    def frame(self, grip, valid=True, epoch=1, selected=True, mask=0, buttons=0, **overrides):
        from adapters.native_health import DependencyHealth, TeleopGate, TeleopRouteHealth
        self.sequence += 1
        self.now = round(self.now + .01, 6)
        samples = {
            'input': SimpleNamespace(sequence=self.sequence, timestamp_us=round(self.now*1e6), connection_state=1,
                                     tracking_valid=valid, buttons_bitset=buttons, grip=grip),
            'permit': TeleopGate(round(self.now*1e6), self.sequence, 1, 1, 1),
            'arm': TeleopRouteHealth(round(self.now*1e6), self.sequence, 1),
            'gripper': TeleopRouteHealth(round(self.now*1e6), self.sequence, 1),
            'dependencies': DependencyHealth(round(self.now*1e9), self.sequence, mask),
        }
        samples.update(overrides)
        with patch('adapters.sources.time.monotonic', return_value=self.now), patch('adapters.sources.time.time', return_value=self.now), patch('adapters.sources.drain_latest', side_effect=list(samples.values())):
            return self.source.poll(self.state, self.now, epoch, selected)

    def test_handover_requires_new_release_and_tracking_failure_withdraws(self):
        self.assertTrue(self.frame(.9).command.hold)
        self.assertTrue(self.frame(0).command.hold)
        self.assertIs(self.frame(.9).command, self.command)
        self.assertIn('tracking', self.frame(.9, False).fault)
        self.assertTrue(self.frame(.9, epoch=2).command.hold)
        self.frame(0, epoch=2)
        self.assertIs(self.frame(.9, epoch=2).command, self.command)
        self.assertIn('coordinator', self.frame(.9, epoch=2, mask=2).fault)

    def test_native_permit_and_unsolicited_disabled_cannot_bypass_hold(self):
        from adapters.native_health import TeleopGate
        self.frame(0)
        self.frame(.9)
        gate = TeleopGate(round((self.now+.01)*1e6), self.sequence+1, 0, 0, 0)
        self.assertTrue(self.frame(.9, permit=gate).command.hold)
        self.source.bridge.poll_commands.return_value = ControlCommand(Mode.DISABLED)
        self.assertTrue(self.frame(.9).command.hold)

    def test_explicit_home_can_resynchronize_native_mode_after_reacquisition(self):
        self.source.bridge.poll_commands.return_value = ControlCommand(Mode.DISABLED)
        self.assertTrue(self.frame(0).command.hold)
        self.assertTrue(self.frame(.9).command.hold)
        self.assertEqual(self.frame(0, buttons=12).command.mode, Mode.DISABLED)
        # A new lease cannot inherit the previous operator's Home authorization.
        self.assertTrue(self.frame(0, epoch=2).command.hold)
        self.source.bridge.poll_commands.return_value = self.command
        self.frame(0, epoch=2, buttons=12)
        # The coordinator's mode response can arrive after the short chord ends.
        self.source.bridge.poll_commands.return_value = ControlCommand(Mode.DISABLED)
        self.assertEqual(self.frame(0, epoch=2).command.mode, Mode.DISABLED)

    def test_duplicate_regressed_and_old_producer_stamps_never_renew_health(self):
        from adapters.sources import FreshSample
        tracker = FreshSample(.1)
        self.assertTrue(tracker.update(SimpleNamespace(sequence=2,timestamp_us=10_000_000),10.,10.))
        for seq, stamp in [(2,10_050_000),(1,10_050_000),(3,9_000_000),(3,11_000_000)]:
            self.assertFalse(tracker.update(SimpleNamespace(sequence=seq,timestamp_us=stamp),10.05,10.05))
        self.assertFalse(tracker.fresh(10.11))

    def test_native_health_abi(self):
        import ctypes
        from adapters.native_health import TeleopGate, TeleopRouteHealth
        self.assertEqual(ctypes.sizeof(TeleopGate),32)
        self.assertEqual(TeleopGate.action.offset,20)
        self.assertEqual(ctypes.sizeof(TeleopRouteHealth),24)

    def test_managed_rpc_only_worker_still_publishes_readonly_feedback(self):
        import json, os, subprocess, tempfile, time, uuid
        from arx5_vr.ipc.services import create_node, open_service, make_subscriber, drain_latest
        from arx5_vr.ipc.types import JointVector15
        prefix = 'arx5-feedback-' + uuid.uuid4().hex[:10]
        node = create_node()
        subscriber = make_subscriber(open_service(node, prefix+'/robot/arm/states/joint_position', JointVector15))
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory/'runtime.json').write_text(json.dumps({'input':'idle','native':None}))
            with (directory/'worker.log').open('w') as log:
                worker = subprocess.Popen([sys.executable,'-B','-m','arx5_vr','_worker','--managed',
                    '--backend','sim','--prefix',prefix,'--directory',str(directory)], cwd=ROOT,
                    stdout=log,stderr=subprocess.STDOUT,
                    env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'})
                try:
                    deadline = time.monotonic()+5
                    stamps=[]
                    while time.monotonic()<deadline and len(stamps)<3:
                        self.assertIsNone(worker.poll(),(directory/'worker.log').read_text())
                        sample=drain_latest(subscriber)
                        if sample is not None:
                            self.assertEqual(sample.len,6)
                            stamps.append(sample.timestamp_us)
                        time.sleep(.015)
                    self.assertEqual(len(stamps),3)
                    self.assertTrue(all(b>a for a,b in zip(stamps,stamps[1:])))
                finally:
                    worker.terminate()
                    worker.wait(timeout=10)
                self.assertEqual(worker.returncode,0,(directory/'worker.log').read_text())
                self.assertIsNone(json.loads((directory/'report.json').read_text())['error'])

    def test_simulation_uses_existing_controller_injection(self):
        from arx5_vr.sim import MujocoBackend
        from arx5_vr.model import JointTargetController
        with patch('arx5_vr.sim.JointTargetController', wraps=JointTargetController) as factory:
            backend = MujocoBackend(load_settings().robot, controller_factory=factory)
            self.addCleanup(backend.close)
            factory.assert_called_once_with(backend.config, backend.kinematics)
            state = backend.read_state(10.)
            backend.step(ControlCommand(Mode.COMMAND_FOLLOWING,hold=True),10.)
            np.testing.assert_allclose(backend.controller.q_target, state.q)


if __name__ == '__main__':
    unittest.main(verbosity=2)
