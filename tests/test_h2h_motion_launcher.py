"""The original motion launcher can select an Actor without starting an environment."""
from pathlib import Path
import unittest
from unittest.mock import patch

from scripts import run_pipeline as launcher
from robojudo.config.config_manager import ConfigManager


class MotionLauncherTest(unittest.TestCase):
    def test_original_config_and_controls_are_preserved(self):
        baseline = ConfigManager('h1_h2h').get_cfg()
        cfg = launcher.make_cfg(launcher.parse_args(['--config', 'h1_h2h']))
        self.assertEqual(cfg.model_dump(), baseline.model_dump())
        self.assertTrue(cfg.env.is_sim)
        self.assertEqual([ctrl.ctrl_type for ctrl in cfg.ctrl], ['KeyboardCtrl', 'MotionH2HCtrl'])

    def test_override_changes_model_and_its_imu_profile(self):
        baseline = launcher.make_cfg(launcher.parse_args(['-c', 'h1_h2h']))
        for name in ('model_30000_jit', 'model_30000_jit.pt'):
            cfg = launcher.make_cfg(launcher.parse_args(['-c', 'h1_h2h', '--policy-name', name]))
            self.assertEqual(Path(cfg.policy.policy_file).name, 'model_30000_jit.pt')
            expected = baseline.model_dump()
            expected['policy']['policy_name'] = 'model_30000_jit'
            expected['policy']['use_imu_torso'] = False
            expected['policy']['action_beta'] = 1.0
            self.assertEqual(cfg.model_dump(), expected)

    def test_bad_models_fail_during_configuration(self):
        for name, exception in [('model_30000', ValueError), ('missing_actor_for_test', FileNotFoundError)]:
            with self.assertRaises(exception):
                launcher.make_cfg(launcher.parse_args(['-c', 'h1_h2h', '--policy-name', name]))
        with patch.object(launcher.ConfigManager, 'get_cfg', return_value=ConfigManager('h1').get_cfg()):
            with self.assertRaisesRegex(ValueError, 'only supported for H2H'):
                launcher.make_cfg(launcher.parse_args(['-c', 'h1', '--policy-name', 'model_30000_jit']))


if __name__ == '__main__':
    unittest.main()
