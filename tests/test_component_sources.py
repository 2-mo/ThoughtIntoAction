"""Real UDP/codec/mapping with an injected command boundary; no SDK or devices."""
from dataclasses import replace
import importlib.util
from pathlib import Path
import socket
import time
import unittest
from unittest.mock import patch

from embodied.devices.inputs.phone import PhoneController
from embodied.devices.phone.protocol.golden import GOLDEN_POSE
from embodied.devices.phone.protocol.packet import encode_packet
from embodied.devices.phone.core.models import Pose, Vector3, Quaternion, TrackingQuality
from embodied.applications.workstation.workbench import profile


class FakeSession:
    parameters = {'effective_robot_config': {'gripper_max_width': .082}}
    def __init__(self, *args, **kwargs):
        self.actions = []
    def __enter__(self): return self
    def close(self): self.actions.append(('release', {}))
    def command(self, action, **fields):
        self.actions.append((action, fields)); return {'status':'applied'}
    request = command
    def status(self):
        return {'feedback_monotonic_ns':time.monotonic_ns(),'pose':[.3,0,.2,0,0,0,1],
                'q':[0]*6,'dq':[0]*6,'gripper_width':.04,'operations':[]}


class PhoneTests(unittest.TestCase):
    def setUp(self):
        with patch('embodied.devices.inputs.phone.ControlSession', FakeSession):
            self.phone = PhoneController('unused', port=0)
        self.phone.start()
        self.addCleanup(self.phone.stop)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.sock.close)
        self.seq = 0

    def packet(self, **fields):
        self.seq += 1
        return replace(GOLDEN_POSE, sequence=self.seq, session_id=1, client_time_ns=time.monotonic_ns(),
            pose=Pose(Vector3(0,0,0), Quaternion(0,0,0,1)), recenter_requested=False, **fields)

    def send(self, packet):
        self.sock.sendto(encode_packet(packet), self.phone.transport.local_address)
        return self.phone.latest()

    def test_release_rearm_mapping_gripper_and_duplicate_cannot_refresh(self):
        self.assertIsNone(self.send(self.packet(clutch_active=True)))
        self.send(self.packet(clutch_active=False, gripper=.25))
        self.assertEqual(self.phone.client.actions, [])
        self.send(self.packet(clutch_active=True, gripper=.25))
        self.assertAlmostEqual(self.phone.client.actions[-1][1]['gripper_width'], .0205)
        packet = replace(self.packet(clutch_active=True), pose=Pose(Vector3(.01,.02,.03),Quaternion(0,0,0,1)))
        result = self.send(packet)
        for actual, expected in zip(result['pose'][:3], [.32,-.01,.23]):
            self.assertAlmostEqual(actual, expected, places=6)
        count = len(self.phone.client.actions)
        self.assertIsNone(self.send(packet))
        self.assertEqual(len(self.phone.client.actions), count)
        time.sleep(.11)
        self.phone.latest()
        self.assertEqual(self.phone.client.actions[-1][0], 'hold')
        self.assertIsNone(self.send(self.packet(clutch_active=True)))
        self.send(self.packet(clutch_active=False))
        self.assertIsNotNone(self.send(self.packet(clutch_active=True)))

    def test_tracking_and_session_change_rearm(self):
        self.send(self.packet(clutch_active=False))
        self.send(self.packet(clutch_active=True))
        self.assertIsNone(self.send(self.packet(clutch_active=False, tracking_quality=TrackingQuality.UNAVAILABLE)))
        self.assertIsNone(self.send(self.packet(clutch_active=True)))
        self.send(self.packet(clutch_active=False))
        self.assertIsNotNone(self.send(self.packet(clutch_active=True)))
        packet = replace(self.packet(clutch_active=True), session_id=2)
        self.assertIsNone(self.send(packet))

    def test_estop_is_not_discarded_when_newer_pose_in_same_batch(self):
        client = self.phone.client
        for packet in (self.packet(estop_requested=True), self.packet()):
            self.sock.sendto(encode_packet(packet), self.phone.transport.local_address)
        with self.assertRaisesRegex(RuntimeError, '停止已锁存'):
            self.phone.latest()
        self.assertEqual(client.actions, [('release', {})])
        self.assertTrue(self.phone.ghost)
        self.assertTrue(self.phone.estop)

    def test_target_lead_does_not_accumulate_and_reversing_responds_immediately(self):
        self.send(self.packet(clutch_active=False))
        self.send(self.packet(clutch_active=True))
        far = replace(self.packet(clutch_active=True), pose=Pose(Vector3(0,.5,0), Quaternion(0,0,0,1)))
        self.assertAlmostEqual(self.send(far)['pose'][0], .35)
        self.assertTrue(self.phone.policy.limited)
        reverse = replace(self.packet(clutch_active=True), pose=Pose(Vector3(0,.49,0), Quaternion(0,0,0,1)))
        self.assertAlmostEqual(self.send(reverse)['pose'][0], .34)
        self.send(self.packet(clutch_active=False))
        # Releasing permits re-positioning the phone without moving the robot.
        rearmed = replace(self.packet(clutch_active=True), pose=far.pose)
        self.assertAlmostEqual(self.send(rearmed)['pose'][0], .3)

    def test_cross_language_vectors(self):
        path = Path(__file__).parent / 'golden/test_golden_vectors.py'
        spec = importlib.util.spec_from_file_location('golden_checks', path)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        for name in dir(module):
            if name.startswith('test_'):
                getattr(module, name)()

    def test_profiles_resolve_without_opening_hardware(self):
        for name in ('sim-manual','sim-phone','sim-quest','real-manual','real-phone','real-quest'):
            config, robot = profile(name)
            self.assertEqual(robot.hardware, name.startswith('real'))


class CompositionTests(unittest.TestCase):
    def configured(self, config):
        import json, tempfile
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'profile.json'
            path.write_text(json.dumps(config))
            return profile(path)

    def test_manual_and_phone_need_no_native_plugins(self):
        from embodied.contracts.registry import Registry
        registry = Registry()
        for key in ('vr', 'routing', 'ingress'):
            registry.entries.pop(key)
        with patch('embodied.applications.workstation.workbench.Registry', return_value=registry):
            for source in ('cli','phone.arkit'):
                config,_ = self.configured({'version':1,'robot':'arx.sim','source':source})
                self.assertNotIn('native',config)

    def test_binding_uses_capabilities_and_explicit_channel_not_source_name(self):
        from embodied.contracts.registry import Registry
        registry = Registry()
        registry.entries['source']['operator_b'] = registry.entries['source']['quest']
        config={'version':1,'robot':'arx.sim','source':'operator_b',
                'native':{'device':'arx.demo','routing':'arx.native','ingress':'arx.native','channel':'xr_demo'}}
        with patch('embodied.applications.workstation.workbench.Registry', return_value=registry):
            self.assertEqual(self.configured(config)[0]['source_options']['ingress'],'xr_demo')
            with self.assertRaisesRegex(ValueError,'explicit native'):
                self.configured({k:v for k,v in config.items() if k!='native'})
            with self.assertRaisesRegex(ValueError,'does not match'):
                self.configured({**config,'source_options':{'ingress':'other'}})
            with self.assertRaisesRegex(ValueError,'simulation only'):
                self.configured({**config,'robot':'arx.real'})


if __name__ == '__main__':
    unittest.main()
