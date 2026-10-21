"""URDF-to-browser transport, resource boundaries and scene lifecycle."""

import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from urllib.error import HTTPError, URLError
from urllib.request import ProxyHandler, Request, build_opener
import xml.etree.ElementTree as ET

import numpy as np

from embodied.core.model import RobotModel
from embodied.core.transforms import from_rt
from embodied.core.plugins import Registry
from embodied.monitor.scene import Scene, WEB_ROOT


class SceneTests(unittest.TestCase):
    def setUp(self):
        self.robot = RobotModel()
        self.scene = Scene(self.robot)
        self.addCleanup(self.scene.close)
        self.q = np.array([0., 1.5, 1.45, -1.52, 0., 0.])
        self.http = build_opener(ProxyHandler({}))  # Loopback probes never use a shell proxy.

    def read(self, path, scene=None):
        with self.http.open((scene or self.scene).url + path, timeout=2) as response:
            self.assertEqual(response.headers['Cache-Control'], 'no-store')
            return response.read()

    def test_feedback_velocity_matches_fk_displacement_and_missing_stays_missing(self):
        velocity = np.array([.1, -.2, .1, .05, -.1, .2])
        dt = 1e-6
        measured = self.scene._velocity(self.q, velocity)
        expected = (self.robot.fk(self.q + velocity * dt)[:3, 3] - self.robot.fk(self.q)[:3, 3]) / dt
        np.testing.assert_allclose(measured['linear'], expected, atol=1e-7)
        self.assertIsNone(self.scene._velocity(self.q, None))
        self.assertEqual(self.scene._velocity(self.q, np.zeros(6))['speed'], 0.)

    def test_fixed_port_restarts_at_same_url_and_rejects_second_owner(self):
        url = self.scene.url
        port = self.scene._server.server_port
        with self.assertRaises(OSError):
            Scene(self.robot, port=port)
        self.scene.close()
        replacement = Scene(self.robot, port=port)
        self.addCleanup(replacement.close)
        self.assertEqual(replacement.url, url)
        self.assertIn("robot", json.loads(self.read("api/model", replacement)))

    def test_visual_transforms_and_eef_follow_model_without_modifying_host_buffers(self):
        for offset in (0., .02, -.03):
            q = self.q + offset
            expected_eef = self.robot.fk(q)
            self.scene.update(q, q + .01)
            state = json.loads(self.read('api/state'))
            np.testing.assert_allclose(np.array(state['actual']['eef']).reshape(4, 4, order='F'), expected_eef)
            # Scene target FK must not change the host's current actual frame placements.
            np.testing.assert_allclose(self.robot.data.oMf[self.robot.frame].homogeneous, expected_eef)
            for mesh, matrix in zip(self.scene._manifest['meshes'], state['actual']['meshes']):
                frame = self.robot.model.getFrameId(mesh['link'])
                np.testing.assert_allclose(np.array(matrix).reshape(4, 4, order='F'),
                                           self.robot.data.oMf[frame].homogeneous, atol=1e-12)
        self.assertTrue(state['target_visible'])
        self.scene.update(self.q)
        self.assertFalse(json.loads(self.read('api/state'))['target_visible'])
        self.scene.update(self.q, self.q + .0003)
        self.assertFalse(json.loads(self.read('api/state'))['target_visible'])

    def test_custom_urdf_visual_origin_scale_and_eef_are_respected(self):
        # Catch accidental reuse of the Desktop model's hardcoded chain or visual offsets.
        tree = ET.parse(self.robot.urdf)
        for mesh in tree.findall('.//mesh'):
            mesh.set('filename', str((self.robot.urdf.parent / mesh.get('filename')).resolve()))
        visual = tree.find("./link[@name='link2']/visual")
        visual.find('origin').set('xyz', '.01 -.02 .03')
        visual.find('geometry/mesh').set('scale', '.75 .8 1.2')
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'variant.urdf'
            tree.write(path)
            robot = RobotModel(path, eef='link4')
            scene = Scene(robot)
            try:
                expected_eef = robot.fk(self.q)
                scene.update(self.q)
                state = json.loads(self.read('api/state', scene))
                manifest = json.loads(self.read('api/model', scene))
                self.assertEqual(manifest['robot']['eef_frame'], 'link4')
                index = next(i for i, mesh in enumerate(manifest['meshes']) if mesh['link'] == 'link2')
                self.assertEqual(manifest['meshes'][index]['scale'], [.75, .8, 1.2])
                expected = robot.data.oMf[robot.model.getFrameId('link2')].homogeneous @ from_rt(np.eye(3), [.01, -.02, .03])
                np.testing.assert_allclose(np.array(state['actual']['meshes'][index]).reshape(4, 4, order='F'), expected)
                np.testing.assert_allclose(np.array(state['actual']['eef']).reshape(4, 4, order='F'), expected_eef)
            finally:
                scene.close()

    def test_paths_keep_xyz_rows_and_history_is_bounded(self):
        self.scene.update(self.q)
        for count in (0, 3, 4, 10):
            points = np.arange(count * 3).reshape(count, 3).astype(float)
            self.scene.path('reference', points, 0x2299FF)
            state = json.loads(self.read('api/state'))
            self.assertEqual(state['paths']['reference']['points'], points.tolist())
        with self.assertRaises(ValueError):
            self.scene.path('invalid', [[1, 2]], 0)
        for step in range(505):
            self.scene.update(self.q + step * .0001)
        self.assertEqual(len(json.loads(self.read('api/state'))['paths']['playback_path']['points']), 500)
        with self.assertRaises(ValueError):
            self.scene.update([float('nan')] * 6)

    def test_static_assets_are_local_and_http_has_no_command_or_arbitrary_file_access(self):
        self.scene.update(self.q)
        manifest = json.loads(self.read('api/model'))
        self.assertEqual(manifest['robot'], self.robot.metadata())
        np.testing.assert_allclose(manifest['torque_limits'][:6], self.robot.model.effortLimit)
        self.assertEqual(manifest['torque_limits'][6], 1.5)
        np.testing.assert_allclose(manifest['joint_limits']['lower'], self.robot.lower)
        np.testing.assert_allclose(manifest['joint_limits']['upper'], self.robot.upper)
        self.assertIn(b'robot-canvas', self.read(''))
        for spec in manifest['meshes']:
            self.assertGreater(len(self.read(spec['url'].lstrip('/'))), 1000)
        for path in ('../configs/local.toml', 'scene.py', 'meshes/999.stl'):
            with self.assertRaises(HTTPError) as error:
                self.read(path)
            self.assertEqual(error.exception.code, 404)
        with self.assertRaises(HTTPError) as error:
            self.http.open(Request(self.scene.url + 'api/state', data=b'{}', method='POST'), timeout=2)
        self.assertEqual(error.exception.code, 404)
        provenance = json.loads((WEB_ROOT / 'provenance.json').read_text())
        for path, digest in provenance['sha256'].items():
            self.assertEqual(hashlib.sha256((WEB_ROOT / path).read_bytes()).hexdigest(), digest, path)

    def test_close_releases_port_and_old_meshcat_selector_is_removed(self):
        self.scene.close()
        self.scene.close()
        self.assertFalse(self.scene._thread.is_alive())
        with self.assertRaises(URLError):
            self.read('api/model')
        with self.assertRaisesRegex(ValueError, 'Unknown viewer'):
            Registry().load('viewer', 'meshcat')
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            self.scene.update(self.q)
