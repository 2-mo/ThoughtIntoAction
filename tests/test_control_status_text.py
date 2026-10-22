"""Input-specific terminal hints; no runtime or device is started."""

from contextlib import redirect_stdout
import io
import unittest

from embodied.control.app import component_exit_message, print_ready


class ControlStatusTextTests(unittest.TestCase):
    def ready_text(self, device, routing='arx.native'):
        output = io.StringIO()
        with redirect_stdout(output):
            print_ready(device, routing, .08)
        return output.getvalue()

    def test_manual_mode_never_advertises_quest_controls(self):
        message = self.ready_text(None, 'arx.workbench')
        self.assertIn('等待网页或命令控制', message)
        for hint in ('Quest', 'Grip', 'Trigger', 'A+B', '左手柄'):
            self.assertNotIn(hint, message)

    def test_managed_quest_uses_the_selected_plugin_for_hints(self):
        message = self.ready_text('arx.quest', 'arx.workbench')
        self.assertIn('Quest', message)
        self.assertIn('Grip', message)
        self.assertIn('左手柄', message)
        self.assertNotIn('左手柄', self.ready_text('arx.quest'))

    def test_demo_and_other_inputs_do_not_claim_quest_controls(self):
        self.assertIn('自动运动', self.ready_text('arx.demo'))
        for device in ('arx.demo', 'custom.input'):
            with self.subTest(device=device):
                self.assertNotIn('Grip', self.ready_text(device))
                self.assertNotIn('Quest', self.ready_text(device))

    def test_viewer_failure_does_not_imply_robot_disconnected(self):
        message = component_exit_message('viewer', 1)
        self.assertIn('异常退出', message)
        self.assertIn('退出码 1', message)
        self.assertIn('robot 仍在运行', message)
        self.assertIn('viewer.log', message)

    def test_observer_exit_explicitly_reports_lost_monitor_feedback(self):
        message = component_exit_message('observer', 0)
        self.assertIn('监看反馈已中断', message)
        self.assertIn('robot 仍在运行', message)
        self.assertIn('observer.log', message)


if __name__ == '__main__':
    unittest.main()
