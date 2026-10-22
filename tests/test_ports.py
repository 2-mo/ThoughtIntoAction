"""`./run.sh check ports`: registry, owners and conflicts from ss output (no sockets opened)."""
import unittest
from unittest.mock import patch

from embodied.applications.workstation import ports

CONFIG = {"ports": {"console": 8000, "phone_udp": 5005, "vision": 8765}, "viewer_service": {"port": 18081},
          "telemetry": {"endpoint": "tcp://127.0.0.1:5567"}, "camera_service": {"socket": "outputs/runtime/camera.sock"}}


class PortTests(unittest.TestCase):
    def table(self, network, commands, config=CONFIG):
        with patch.object(ports, "listeners", return_value=(network, {})), \
             patch.object(ports, "command_line", side_effect=lambda pid: commands[pid]), \
             patch.object(ports, "live_runs", return_value=[]):
            return {row["name"]: row for row in ports.rows(config)}

    def test_owner_classification(self):
        table = self.table({("tcp", 18081): [11], ("tcp", 8000): [12], ("udp", 5005): [13]},
                           {11: f"python -m embodied.applications.monitor.cli", 12: "nginx: worker", 13: f"{ports.PROJECT_ROOT}/.venv/bin/python -m embodied.devices.inputs.phone"})
        self.assertEqual(table["monitor"]["state"], "本项目")
        self.assertEqual(table["console"]["state"], "其他程序")
        self.assertEqual(table["phone_udp"]["state"], "本项目")
        self.assertEqual(table["vision"]["state"], "空闲")
        self.assertEqual(table["telemetry"]["address"], "tcp 5567")
        self.assertEqual(table["camera_service"]["address"], "unix outputs/runtime/camera.sock")

    def test_two_names_on_one_port_is_a_registry_conflict(self):
        config = {**CONFIG, "ports": {**CONFIG["ports"], "vision": 8000}}
        table = self.table({}, {}, config)
        self.assertTrue(table["vision"]["state"].startswith("登记冲突"))
        self.assertTrue(table["console"]["state"].startswith("登记冲突"))


if __name__ == "__main__":
    unittest.main()
