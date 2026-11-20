"""XR-to-base basis: vendor profiles may replace the ARX X5 rotation."""
import pathlib
import tempfile
import tomllib
import unittest

from embodied.devices.inputs.quest.native_config import write_native_configs


class NativeBasisTests(unittest.TestCase):
    def test_vendor_basis_is_used_verbatim(self):
        with tempfile.TemporaryDirectory() as folder:
            paths = write_native_configs(
                pathlib.Path(folder), "robot-test", .07, "/x/y.so",
                controller="left", input_prefix="robot-test", feedback_hz=50,
                xr_basis=[[0, 0, 1], [1, 0, 0], [0, 1, 0]])
            xr = tomllib.loads(paths["xrobot"].read_text())
            arm = tomllib.loads(paths["arm"].read_text())
        self.assertEqual(xr["xr_basis"], [0., 0., 1., 1., 0., 0., 0., 1., 0.])
        self.assertEqual(arm["control_rate_hz"], 250.0)
        self.assertEqual(arm["leader_channel_id"], "xr.left_controller")

    def test_default_keeps_the_x5_rotation(self):
        with tempfile.TemporaryDirectory() as folder:
            paths = write_native_configs(pathlib.Path(folder), "robot-test", .07, "/x/y.so",
                                         base_yaw_deg=180., controller="right")
            xr = tomllib.loads(paths["xrobot"].read_text())
        self.assertEqual(xr["xr_basis"], [0., 0., 1., 1., 0., 0., 0., 1., 0.])


if __name__ == "__main__":
    unittest.main()
