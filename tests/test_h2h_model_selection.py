"""Actor export and both Quest launchers, without ADB, DDS or robot commands."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import torch

from scripts.export_h2h_actor import METADATA_FILE, export_actor
from scripts import run_quest_h2h as simulation
from scripts import run_quest_h2h_real as deployment
from robojudo.config.h1.policy.h1_h2h_policy_cfg import H1H2HPolicyCfg
from robojudo.policy.utils.h2h_model import MODEL_OBSERVATION_PROFILES, policy_name, select_h2h_model


class ExportTest(unittest.TestCase):
    def test_export_is_deterministic_and_preserves_checkpoint(self):
        actor = torch.nn.Sequential(torch.nn.Linear(1665, 8), torch.nn.ELU(),
                                    torch.nn.Linear(8, 19)).eval()
        checkpoint = dict(iter=30000, training_config=dict(policy=dict(
            activation='elu', actor_hidden_dims=[8])), model_state_dict={
                **{'actor.'+key: value for key, value in actor.state_dict().items()},
                'critic.0.weight': torch.ones(1, 2), 'std': torch.ones(19)})
        with tempfile.TemporaryDirectory() as directory:
            source, target = Path(directory)/'training.pt', Path(directory)/'actor.pt'
            torch.save(checkpoint, source)
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            result = export_actor(source, target)
            metadata = {METADATA_FILE: ''}
            loaded = torch.jit.load(str(target), _extra_files=metadata).eval()
            payload = json.loads(metadata[METADATA_FILE])
            self.assertEqual(payload['source_sha256'], digest)
            self.assertEqual(payload['iteration'], 30000)
            self.assertEqual(result['equivalence_max_abs_error'], 0)
            probe = torch.randn(4, 1665)
            with torch.no_grad():
                torch.testing.assert_close(actor(probe), loaded(probe))
            self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), digest)
            self.assertFalse(any('critic' in key or key == 'std' for key in loaded.state_dict()))
            with self.assertRaises(FileExistsError):
                export_actor(source, target)
            with self.assertRaises(ValueError):
                export_actor(source, source)

    def test_export_refuses_missing_activation_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'training.pt'
            torch.save(dict(model_state_dict={}), source)
            with self.assertRaisesRegex(ValueError, 'activation'):
                export_actor(source, Path(directory)/'actor.pt')


class SelectionTest(unittest.TestCase):
    def test_name_normalization_and_path_rejection(self):
        self.assertEqual(policy_name('model_30000_jit.pt'), 'model_30000_jit')
        for value in ('../training', '/tmp/policy', '', '.pt'):
            with self.assertRaises(argparse.ArgumentTypeError):
                policy_name(value)

    def test_raw_training_checkpoint_cannot_be_selected(self):
        with self.assertRaisesRegex(ValueError, 'training checkpoint'):
            select_h2h_model(H1H2HPolicyCfg(), 'model_30000')

    def test_missing_model_fails_before_starting_environment(self):
        with self.assertRaises(FileNotFoundError):
            select_h2h_model(H1H2HPolicyCfg(), 'nonexistent_model_for_test')

    def test_profile_matches_embedded_checkpoint_identity_not_filename(self):
        class LocalPolicyCfg(H1H2HPolicyCfg):
            directory: str

            @property
            def policy_file(self):
                return str(Path(self.directory) / (self.policy_name + '.pt'))

        source_hash = next(iter(MODEL_OBSERVATION_PROFILES))[1]
        actor = torch.jit.script(torch.nn.Linear(1665, 19))
        with tempfile.TemporaryDirectory() as directory:
            cfg = LocalPolicyCfg(directory=directory)
            for name, metadata, expected_torso in (
                    ('renamed_actor', {'source_sha256': source_hash}, False),
                    ('model_30000_jit', {'source_sha256': 'unrelated_checkpoint'}, True),
                    ('legacy_actor', None, True)):
                extra = {METADATA_FILE: json.dumps(metadata)} if metadata else {}
                torch.jit.save(actor, str(Path(directory) / (name + '.pt')), _extra_files=extra)
                selected = select_h2h_model(cfg, name)
                self.assertEqual(selected.use_imu_torso, expected_torso)
                self.assertEqual(selected.action_dof.model_dump(), cfg.action_dof.model_dump())
                self.assertEqual(selected.action_beta, 1.0 if not expected_torso else cfg.action_beta)
                self.assertEqual(selected.action_scale, cfg.action_scale)
            self.assertTrue(cfg.use_imu_torso)

    def test_both_launchers_select_the_same_model_and_keep_defaults(self):
        for launcher in (simulation, deployment):
            for requested in (None, 'model_30000_jit', 'model_30000_jit.pt'):
                argv = ['launcher', '--dds-sim', '--no-keyboard']
                if requested:
                    argv += ['--policy-name', requested]
                with patch.object(sys, 'argv', argv):
                    args = launcher.parse_args()
                config = launcher.make_cfg(args)
                expected = 'model_30000_jit' if requested else 'h2h_release'
                self.assertEqual(config.policy.policy_name, expected)
                self.assertEqual(Path(config.policy.policy_file).name, expected+'.pt')
                self.assertEqual(config.policy.action_scale, .25)
                self.assertEqual(config.policy.action_beta, 1.0 if requested else .8)
                self.assertEqual(config.policy.history_length, 25)
                self.assertEqual(config.policy.use_imu_torso, not bool(requested))


if __name__ == '__main__':
    unittest.main()
