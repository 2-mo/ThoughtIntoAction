from mcap_fixture import write_records
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from robot_core.model import RobotModel
from quest_teleop.replay import RecordedEpisode, WebReplay, RobotReplay
from quest_teleop.runtime.session import TeleopSession


class ReplayTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.root = Path(folder.name)
        self.meta = {'schema_version': 1, 'kind': 'quest_teleop_episode', 'id': 'previous',
                     'lifecycle': 'complete', 'outcome': 'unknown', 'robot': RobotModel(eef='eef_link').metadata()}
        (self.root / 'episode.json').write_text(json.dumps(self.meta))
        self.rows = [{'host_monotonic_ns': 1_000_000_000 + i * 1_000_000_000,
                      'actual_pos': [i * .1, 0, 0, 0, 0, 0], 'gripper_actual_pos': .04 - i * .01} for i in range(2)]
        write_records(self.root, states=self.rows)
        frames = []
        for i in range(2):
            cv2.imwrite(str(self.root / f'{i}.png'), np.full((16, 16, 3), i * 200, dtype=np.uint8))
            frames.append({'host_received_monotonic_ns': 1_000_000_000 + i * 1_000_000_000, 'image': f'{i}.png'})
        write_records(self.root, frames=frames)

    def test_readonly_review_video_and_joints_follow_same_clock(self):
        data = RecordedEpisode(self.root)
        with patch('quest_teleop.replay.time.monotonic', return_value=10) as clock:
            replay = WebReplay(data)
            clock.return_value = 10.5
            first = replay.frame()
            self.assertEqual(first['sample_index'], 0)
            self.assertTrue(first['image_base64'])
            replay.control('pause')
            clock.return_value = 20
            self.assertEqual(replay.frame()['position_s'], .5)
            replay.control('seek', 1.)
            last = replay.frame()
            self.assertEqual(last['sample_index'], 1)
            self.assertNotEqual(last['image_base64'], first['image_base64'])
            replay.control('play')
            self.assertEqual(replay.frame()['position_s'], 0.)

    def test_mp4_review_seek_keeps_image_and_robot_sample_together(self):
        from quest_teleop.video import RGBVideoWriter
        writer = RGBVideoWriter(self.root / "wrist_rgb.mp4", 16, 16, 1, 20)
        try:
            for value in (0, 200):
                writer.write(np.full((16, 16, 3), value, dtype=np.uint8))
        finally:
            writer.close()
        frames = [{"host_received_monotonic_ns": row["host_monotonic_ns"],
                   "video": "wrist_rgb.mp4", "video_frame_index": i, "video_pts_s": float(i)}
                  for i, row in enumerate(self.rows)]
        write_records(self.root, frames=frames)
        replay = WebReplay(RecordedEpisode(self.root))
        replay.control("pause")
        replay.control("seek", 0.)
        first = replay.frame()
        replay.control("seek", 1.)
        last = replay.frame()
        self.assertEqual(first["sample_index"], 0)
        self.assertEqual(last["sample_index"], 1)
        self.assertTrue(first["image_base64"])
        self.assertNotEqual(first["image_base64"], last["image_base64"])

    def test_discarded_incomplete_wrong_model_and_invalid_times_rejected(self):
        for field, value in [('outcome', 'discarded'), ('lifecycle', 'failed')]:
            (self.root / 'episode.json').write_text(json.dumps({**self.meta, field: value}))
            with self.assertRaises(ValueError): RecordedEpisode(self.root)
        (self.root / 'episode.json').write_text(json.dumps(self.meta))
        self.rows[1]['host_monotonic_ns'] = self.rows[0]['host_monotonic_ns']
        write_records(self.root, states=self.rows)
        with self.assertRaises(ValueError): RecordedEpisode(self.root)

    def test_session_replays_only_saved_previous_and_blocks_active_recording(self):
        session = TeleopSession.__new__(TeleopSession)
        session.operations = threading.Lock()
        session.closed = False
        session.recording = None
        session.last_episode = {'path': str(self.root)}
        session.latest_state = None
        session._web_replay = None
        self.assertEqual(session.dispatch({'command': 'collection.replay_previous'})['episode'], 'previous')
        self.assertIsNotNone(session.dispatch({'command': 'replay.frame'})['replay'])
        session.recording = object()
        with self.assertRaises(ValueError): session.load_previous_episode()

    def test_web_api_loads_video_and_pose_but_cannot_start_hardware_replay(self):
        import urllib.request
        from robot_core.local_server import LocalServer
        from robot_simulation.scene import Scene
        session = TeleopSession.__new__(TeleopSession)
        session.operations = threading.Lock()
        session.closed = False
        session.recording = None
        session.last_episode = {'path': str(self.root)}
        session.latest_state = None
        session._web_replay = None
        socket = self.root / 'session.sock'
        server = LocalServer(socket, session.dispatch)
        self.addCleanup(server.close)
        scene = Scene(RobotModel(), port=0)
        self.addCleanup(scene.close)
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        def post(action, origin=None):
            request = urllib.request.Request(scene.url + 'api/replay',
                data=json.dumps({'action': action}).encode(), headers={'Content-Type': 'application/json', **({'Origin': origin} if origin else {})})
            with opener.open(request) as response: return json.load(response)
        with patch('robot_core.project.load_config', return_value={'runtime': {'socket': str(socket)}}):
            self.assertTrue(post('load_last')['ok'])
            self.assertTrue(post('load_last', scene.url.rstrip('/').replace('127.0.0.1', 'localhost'))['ok'])
            import urllib.error
            with self.assertRaises(urllib.error.HTTPError) as rejected:
                post('load_last', 'http://untrusted.example')
            self.assertEqual(rejected.exception.code, 403)
            with opener.open(scene.url + 'api/replay') as response: payload = json.load(response)
            self.assertNotIn('error', payload)
            frame = payload['replay']
            self.assertTrue(frame['image_base64'])
            self.assertEqual(frame['image_mime'], 'image/png')
            self.assertTrue(frame['pose']['meshes'])
            self.assertIn('error', post('robot_replay'))

    def test_robot_replay_runs_through_virtual_service_with_table_limit(self):
        import time
        from arm_service_fixture import ServiceFixture
        from robot_control.client import ArmClient
        fixture = ServiceFixture(table_safety=True)
        self.addCleanup(fixture.close)
        client = ArmClient(fixture.robot, path=fixture.path, hardware=False, actuate=True,
                           speed=1., motion={"max_command_lead_rad": 0.})
        self.addCleanup(client.close)
        start = client.snapshot()['actual_pos']
        for i, row in enumerate(self.rows):
            row['actual_pos'] = list(start)
            row['actual_pos'][0] += i * .03
        write_records(self.root, states=self.rows)
        replay = RobotReplay(RecordedEpisode(self.root))
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if not replay.tick(client, time.monotonic()): break
            time.sleep(.02)
        self.assertEqual(replay.phase, 'finished')
        state = client.snapshot()
        np.testing.assert_allclose(state['actual_pos'], self.rows[-1]['actual_pos'], atol=.03)
        self.assertFalse(state['table_safety']['blocked'])

    def test_robot_replay_positions_first_and_waits_for_final_feedback(self):
        class Arm:
            q = [.2, 0, 0, 0, 0, 0]
            blocked = False
            grip = .04
            def __init__(self): self.commands = []
            def snapshot(self):
                return {'actual_pos': self.q, 'actual_vel': [0]*6, 'gripper_actual_pos': self.grip,
                        'trajectory': {'finished': True, 'stopping': False},
                        'table_safety': {'blocked': self.blocked}}
            def set_joint_positions(self, q, profile): self.q = list(q); self.commands.append((self.q, profile))
            def set_catch_pos(self, grip): self.grip = grip
        arm = Arm()
        player = RobotReplay(RecordedEpisode(self.root), now=0)
        self.assertTrue(player.tick(arm, 0))
        self.assertEqual(arm.commands[-1][1], 'preset')
        self.assertTrue(player.tick(arm, .1))
        self.assertEqual(player.phase, 'playing')
        self.assertTrue(player.tick(arm, 1.2))
        self.assertEqual(arm.q, self.rows[-1]['actual_pos'])
        self.assertAlmostEqual(arm.grip, .03)
        self.assertFalse(player.tick(arm, 1.3))
        blocked = RobotReplay(RecordedEpisode(self.root), now=0)
        blocked.tick(arm, 0)
        arm.blocked = True
        with self.assertRaisesRegex(RuntimeError, '安全边界'): blocked.tick(arm, .02)
