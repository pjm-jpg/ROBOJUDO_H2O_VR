"""Quest mapping and actual H1 policy/physics integration, without ADB or a display."""
import copy
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from robojudo.config.config_manager import ConfigManager
from robojudo.controller.quest_h2h_ctrl import QuestAlignment, QuestH2HCtrl, load_reader_class
from robojudo.pipeline.quest_h2h_pipeline import QuestH2HPipeline


HUMAN = np.array([[0.3, 0.3, 1.2], [0.3, -0.3, 1.2], [0, 0, 1.7]])


def frame(sequence=0, **updates):
    result = dict(version=1, session_id=1, sequence=sequence,
                  sample_time_ns=1_000_000_000 + sequence * 250_000_000,
                  space_epoch=0, reference_space='stage', axes='flu', focused=True,
                  points={name: dict(valid=True, tracked=True, position_m=pos.tolist(),
                                     orientation_valid=True, orientation_tracked=True,
                                     orientation_xyzw=[0., 0., 0., 1.], linear_velocity_m_s=None)
                          for name, pos in zip(('left', 'right', 'head'), HUMAN)})
    result.update(updates)
    return result


class FakeReader:
    def __init__(self, **kwargs):
        self.frame = None
        self.closed = False

    def start(self):
        pass

    def get_latest(self, **kwargs):
        return copy.deepcopy(self.frame)

    def close(self):
        self.closed = True


def quest_config():
    cfg = ConfigManager('h1_h2h_quest').get_cfg()
    return next(ctrl for ctrl in cfg.ctrl if ctrl.ctrl_type == 'QuestH2HCtrl')


def calibrate(control):
    control.reader.frame = frame()
    control.get_data()
    control.command('[QUEST_CALIBRATE]')
    for sequence in range(5):
        control.reader.frame = frame(sequence)
        control.get_data()


class AlignmentTest(unittest.TestCase):
    def test_yaw_offsets_and_motion(self):
        robot = HUMAN + [2, 3, -.2]
        align = QuestAlignment(HUMAN, robot, math.pi / 2, 0)
        np.testing.assert_allclose(align.map(HUMAN), robot)
        moved = HUMAN + [0, .1, 0]
        np.testing.assert_allclose(align.map(moved), robot + [.1, 0, 0], atol=1e-12)
        moved[0] += [0, .2, .1]
        expected = robot + [.1, 0, 0]
        expected[0] += [.2, 0, .1]
        np.testing.assert_allclose(align.map(moved), expected, atol=1e-12)

    def test_separate_head_and_relative_hand_scales(self):
        align = QuestAlignment(HUMAN, HUMAN, 0, 0, head_scale=.5, hand_scale=.25)
        moved = HUMAN + [0, 0, -.2]
        moved[0, 0] += .4
        expected = HUMAN + [0, 0, -.1]
        expected[0, 0] += .1
        np.testing.assert_allclose(align.map(moved), expected)


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.cfg = quest_config()
        fk = {entry['parent_name']: dict(pos=np.array([0., 0., 1.]), quat=np.array([0., 0., 0., 1.]))
              for entry in self.cfg.extend_config}
        env = SimpleNamespace(kinematics=True, fk_info=fk, base_quat=[0, 0, 0, 1])
        with patch('robojudo.controller.quest_h2h_ctrl.load_reader_class', return_value=FakeReader):
            self.control = QuestH2HCtrl(self.cfg, env=env)

    def tearDown(self):
        self.control.close()

    def test_duplicate_frames_cannot_complete_calibration(self):
        self.control.reader.frame = frame()
        self.control.get_data()
        self.control.command('[QUEST_CALIBRATE]')
        for _ in range(100):
            self.control.get_data()
        self.assertIsNone(self.control.alignment)
        self.assertEqual(len(self.control.samples), 1)

    def test_calibration_command_reports_missing_data(self):
        with self.assertLogs('robojudo.controller.quest_h2h_ctrl', level='INFO') as logs:
            self.control.command('[QUEST_CALIBRATE]')
        self.assertIn('C received', logs.output[0])
        self.assertIn('cannot calibrate', logs.output[1])
        self.assertFalse(self.control.calibrating)
        self.assertIsNone(self.control.alignment)

    def test_calibration_preview_enable_and_velocity_limit(self):
        control = self.control
        calibrate(control)
        self.assertIsNotNone(control.alignment)
        self.assertFalse(control.enabled)
        np.testing.assert_allclose(control.get_data()['ref_body_pos_subset'], control.robot_points())
        control.reader.frame = frame(5)
        control.reader.frame['points']['left']['position_m'][0] += .5
        preview = control.get_data()['ref_body_pos_subset']
        self.assertAlmostEqual(preview[0, 0] - control.robot_points()[0, 0], .5)
        control.command('[QUEST_ENABLE]')
        data = control.get_data()
        self.assertTrue(data['enabled'])
        self.assertLessEqual(np.linalg.norm(data['ref_body_vel_subset'], axis=1).max(), self.cfg.max_speed + 1e-12)
        np.testing.assert_allclose(data['ref_body_vel_subset'] / self.cfg.frequency,
                                   data['ref_body_pos_subset'] - control.robot_points())
        control.command('[QUEST_PAUSE]')
        self.assertFalse(control.get_data()['enabled'])

    def test_stale_data_pauses_without_automatic_resume(self):
        calibrate(self.control)
        self.control.command('[QUEST_ENABLE]')
        self.control.reader.frame = None
        self.assertFalse(self.control.get_data()['enabled'])
        self.control.reader.frame = frame(7)
        self.assertFalse(self.control.get_data()['enabled'])
        self.assertIsNone(self.control.alignment)

    def test_reference_and_session_changes_require_calibration(self):
        for changed in (dict(space_epoch=1), dict(session_id=2)):
            self.control.reset()
            calibrate(self.control)
            self.control.command('[QUEST_ENABLE]')
            self.control.reader.frame = frame(7, **changed)
            data = self.control.get_data()
            self.assertFalse(data['enabled'])
            self.assertFalse(data['calibrated'])
            self.control.command('[QUEST_ENABLE]')
            self.assertFalse(self.control.enabled)

    def test_focus_loss_requires_calibration(self):
        calibrate(self.control)
        self.control.reader.frame = frame(6, focused=False)
        self.assertFalse(self.control.get_data()['calibrated'])

    def test_local_space_and_inferred_calibration_rejected(self):
        self.control.reader.frame = frame(reference_space='local')
        self.control.get_data()
        self.control.command('[QUEST_ENABLE]')
        self.assertFalse(self.control.enabled)
        self.control.command('[QUEST_CALIBRATE]')
        for i in range(6):
            self.control.reader.frame = frame(i)
            self.control.reader.frame['points']['left']['tracked'] = False
            self.control.get_data()
        self.assertIsNone(self.control.alignment)

    def test_runtime_inferred_option_and_invalid_tracking(self):
        calibrate(self.control)
        self.control.command('[QUEST_ENABLE]')
        self.control.reader.frame = frame(7)
        self.control.reader.frame['points']['left']['tracked'] = False
        self.assertTrue(self.control.get_data()['enabled'])
        self.cfg.require_tracked = True
        self.assertFalse(self.control.get_data()['enabled'])
        self.cfg.require_tracked = False
        self.control.reader.frame['points']['left']['valid'] = False
        self.assertFalse(self.control.get_data()['enabled'])

    def test_large_jump_invalidates_calibration(self):
        calibrate(self.control)
        self.control.reader.frame = frame(7)
        self.control.reader.frame['points']['head']['position_m'][2] += 2
        self.assertFalse(self.control.get_data()['calibrated'])

    def test_calibration_restarts_on_head_turn(self):
        self.control.reader.frame = frame()
        self.control.get_data()
        self.control.command('[QUEST_CALIBRATE]')
        for i in range(5):
            self.control.reader.frame = frame(i)
            angle = i * .2
            self.control.reader.frame['points']['head']['orientation_xyzw'] = [
                0, 0, math.sin(angle/2), math.cos(angle/2)]
            self.control.get_data()
        self.assertIsNone(self.control.alignment)

    def test_shared_receiver_loads_without_legacy_adb_dependency(self):
        self.assertEqual(load_reader_class(self.cfg.reader_root).__name__, 'ThreePointReader')


class RealPolicyIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.cfg = ConfigManager('h1_h2h_quest').get_cfg()
        self.cfg.device = 'cpu'
        self.cfg.env.headless = True
        self.cfg.ctrl = [ctrl for ctrl in self.cfg.ctrl if ctrl.ctrl_type == 'QuestH2HCtrl']
        with patch('robojudo.controller.quest_h2h_ctrl.load_reader_class', return_value=FakeReader):
            self.pipeline = QuestH2HPipeline(self.cfg)

    def tearDown(self):
        self.pipeline.close()

    def test_model_contract_and_legacy_motion_interface(self):
        p = self.pipeline
        self.assertEqual(p.policy.cfg_policy.policy_name, 'h2h_release')
        data = p.env.get_data()
        controls = p.ctrl_manager.get_ctrl_data(data)
        obs, _ = p.policy.get_observation(data, controls)
        self.assertEqual(obs.shape, (1665,))
        self.assertEqual(p.policy.get_pd_target(obs).shape, (19,))
        motion = controls['QuestH2HCtrl']
        legacy = p.policy._get_commands({'MotionH2HCtrl': motion})
        np.testing.assert_array_equal(legacy[0], motion['ref_body_pos_subset'])

    def test_physics_freeze_active_step_and_disconnect(self):
        p = self.pipeline
        qpos = p.env.data.qpos.copy()
        time_before = p.env.data.time
        for _ in range(3):
            p.step()
        np.testing.assert_array_equal(qpos, p.env.data.qpos)
        self.assertEqual(p.env.data.time, time_before)
        calibrate(p.quest)
        p._commands(['[QUEST_ENABLE]'])
        for _ in range(10):
            p.step()
        self.assertAlmostEqual(p.env.data.time - time_before, 10 * p.dt)
        self.assertTrue(np.isfinite(p.env.data.qpos).all())
        p.quest.reader.frame = None
        time_before = p.env.data.time
        p.step()
        self.assertFalse(p.quest.enabled)
        self.assertEqual(time_before, p.env.data.time)
        p.quest.reader.frame = frame(10)
        p.step()
        self.assertEqual(time_before, p.env.data.time)
        p._commands(['[SIM_REBORN]'])
        self.assertIsNone(p.quest.alignment)

    def test_preview_and_pause_commands_override_enable(self):
        p = self.pipeline
        calibrate(p.quest)
        self.cfg.preview_only = True
        p._commands(['[QUEST_ENABLE]'])
        self.assertFalse(p.quest.enabled)
        self.cfg.preview_only = False
        for commands in (['[QUEST_ENABLE]', '[QUEST_PAUSE]'], ['[QUEST_PAUSE]', '[QUEST_ENABLE]']):
            p._commands(commands)
            self.assertFalse(p.quest.enabled)
        p._commands(['[QUEST_ENABLE]', '[QUEST_CALIBRATE]'])
        self.assertFalse(p.quest.enabled)

    def test_reject_real_hardware_before_initializing_environment(self):
        self.cfg.env.is_sim = False
        with self.assertRaisesRegex(ValueError, 'simulation only'):
            QuestH2HPipeline(self.cfg)


if __name__ == '__main__':
    unittest.main()
