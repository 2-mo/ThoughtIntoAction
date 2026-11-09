"""Depth geometry at the boundary to remote grasp prediction."""
import importlib.util
import hashlib
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('graspnet_capture', Path(__file__).parents[1] / 'scripts/graspnet/capture.py')
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)

remote_spec = importlib.util.spec_from_file_location('graspnet_remote', Path(__file__).parents[1] / 'scripts/graspnet/remote.py')
remote = importlib.util.module_from_spec(remote_spec)
remote_spec.loader.exec_module(remote)


def test_metric_depth_and_optical_axes():
    camera = {'height': 2, 'width': 2, 'matrix': [[100, 0, 0], [0, 100, 0], [0, 0, 1]],
              'distortion_model': 'none'}
    xyz = capture.point_cloud(np.array([[1000, 1000], [1000, 0]], dtype=np.uint16), .001, camera)
    np.testing.assert_allclose(xyz[0, 0], [0, 0, 1])
    np.testing.assert_allclose(xyz[0, 1], [.01, 0, 1])
    np.testing.assert_allclose(xyz[1, 0], [0, .01, 1])
    np.testing.assert_allclose(xyz[1, 1], [0, 0, 0])
    with pytest.raises(ValueError):
        capture.point_cloud(np.ones((2, 2)), .001, camera)
    with pytest.raises(ValueError):
        capture.point_cloud(np.ones((2, 2), dtype=np.uint16), float('nan'), camera)


def test_remote_output_is_bound_to_input_and_valid_rotation(tmp_path):
    source = tmp_path / 'input.npz'
    np.savez(source, points=np.ones((100, 3)))
    report = {'input_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'frame': 'camera_color_optical'}
    row = np.r_[.8, .03, .02, .01, np.eye(3).ravel(), [0, 0, .4], -1][None]
    result = tmp_path / 'grasps.npz'
    np.savez(result, grasps=row, raw_grasps=row)
    remote.validate_result(result, report, source)
    with pytest.raises(ValueError, match='match'):
        remote.validate_result(result, {**report, 'input_sha256': 'wrong'}, source)
    row[0, 4] = -1
    np.savez(result, grasps=row, raw_grasps=row)
    with pytest.raises(ValueError, match='rotation'):
        remote.validate_result(result, report, source)
