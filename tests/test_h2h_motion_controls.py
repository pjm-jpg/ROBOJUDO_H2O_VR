"""Keep balancing active when H2H reference playback pauses."""
from queue import Queue
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from robojudo.controller.utils.motion_gui import MotionGUI
from robojudo.pipeline.rl_pipeline import RlPipeline


class MotionControlsTest(unittest.TestCase):
    def pipeline(self, policy_type):
        pipeline = RlPipeline.__new__(RlPipeline)
        pipeline.cfg = SimpleNamespace(policy=SimpleNamespace(policy_type=policy_type),
            ctrl=[SimpleNamespace(ctrl_type='MotionH2HCtrl')], debug=SimpleNamespace(log_obs=False))
        pipeline.policy = SimpleNamespace(post_step_callback=Mock())
        pipeline.ctrl_manager = SimpleNamespace(post_step_callback=Mock())
        pipeline.visualizer = None
        pipeline.do_safety_check = False
        pipeline.timestep = 0
        pipeline._blend_out_active = False
        pipeline._blend_out_step = 0
        pipeline._user_fade_out = False
        pipeline._pending_blend_in = False
        return pipeline

    def test_h2h_pause_and_resume_never_blend_actor_to_fixed_joints(self):
        pipeline = self.pipeline('H2HStudentPolicy')
        for command in ['[MOTION_FADE_OUT]', '[MOTION_FADE_IN]'] * 2:
            pipeline.post_step_callback(None, {'COMMANDS': [command]}, {}, None)
            self.assertFalse(pipeline._blend_out_active)
            self.assertFalse(pipeline._pending_blend_in)
            self.assertFalse(pipeline._user_fade_out)
        self.assertEqual(pipeline.policy.post_step_callback.call_count, 4)
        self.assertEqual(pipeline.ctrl_manager.post_step_callback.call_count, 4)

    def test_other_policy_keeps_its_original_fade_behavior(self):
        pipeline = self.pipeline('OtherPolicy')
        pipeline.post_step_callback(None, {'COMMANDS': ['[MOTION_FADE_OUT]']}, {}, None)
        self.assertTrue(pipeline._blend_out_active)
        pipeline.post_step_callback(None, {'COMMANDS': ['[MOTION_FADE_IN]']}, {}, None)
        self.assertTrue(pipeline._pending_blend_in)

    def test_confirm_current_motion_cannot_disengage_policy(self):
        gui = MotionGUI.__new__(MotionGUI)
        gui.motion_ctrl = SimpleNamespace(motion_id=0, gui_commands=Queue())
        gui.motion_input = Mock()
        gui.motion_input.get.return_value = '0'
        gui.switch_pending = False
        gui.root = Mock()
        gui.confirm_button = Mock()
        gui.switch_motion()
        self.assertTrue(gui.motion_ctrl.gui_commands.empty())
        gui.root.after.assert_not_called()
        self.assertFalse(gui.switch_pending)


if __name__ == '__main__':
    unittest.main()
