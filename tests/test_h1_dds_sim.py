"""Real DDS packets between processes, with a headless MuJoCo H1 endpoint."""
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from robojudo.config.config_manager import ConfigManager
from robojudo.config.h1.env.h1_dds_sim_env_cfg import H1DDSSimEnvCfg
from robojudo.environment.h1_dds_bridge import command_arrays
from robojudo.environment.unitree_dds_sim_env import UnitreeDDSSimEnv
from robojudo.environment.utils.unitree_command import init_cmd_go
from robojudo.environment.utils.unitree_rotation import transform_imu_data
from robojudo.pipeline.quest_h2h_pipeline import QuestH2HPipeline

ROOT = Path(__file__).resolve().parents[1]


class ProtocolTest(unittest.TestCase):
    def test_torso_to_pelvis_imu_with_nonzero_waist(self):
        waist, speed = .6, .4
        base = Rotation.from_euler('xyz', [.1, -.2, .3])
        torso = base * Rotation.from_euler('z', waist)
        base_rate = np.array([.2, -.3, .5])
        gyro = Rotation.from_euler('z', -waist).apply(base_rate + [0, 0, speed])
        q, w = transform_imu_data(waist, speed, torso.as_quat(scalar_first=True), gyro)
        np.testing.assert_allclose(Rotation.from_quat(q, scalar_first=True).as_matrix(), base.as_matrix(), atol=1e-7)
        np.testing.assert_allclose(w, base_rate, atol=1e-7)

    def test_crc_stop_sentinels_velocity_field_and_invalid_commands(self):
        cfg = H1DDSSimEnvCfg()
        cmd = unitree_go_msg_dds__LowCmd_()
        init_cmd_go(cmd, cfg.weak_motor)
        cmd.crc = CRC().Crc(cmd)
        np.testing.assert_array_equal(command_arrays(cmd, cfg.joint2motor_idx)[:, 2:4], 0)
        motor = cmd.motor_cmd[cfg.joint2motor_idx[0]]
        motor.q, motor.dq, motor.kp, motor.kd = .1, .2, 3., 4.
        cmd.crc = CRC().Crc(cmd)
        np.testing.assert_allclose(command_arrays(cmd, cfg.joint2motor_idx)[0], [.1, .2, 3, 4, 0])
        bad = copy.deepcopy(cmd)
        bad.motor_cmd[0].tau = 1
        with self.assertRaisesRegex(ValueError, 'CRC'):
            command_arrays(bad, cfg.joint2motor_idx)
        motor.q = float('nan')
        cmd.crc = CRC().Crc(cmd)
        with self.assertRaisesRegex(ValueError, 'Non-finite'):
            command_arrays(cmd, cfg.joint2motor_idx)

    def test_simulation_config_rejects_hardware_interface_or_domain(self):
        for updates in ({'net_if': 'eno2'}, {'domain_id': 0}):
            cfg = H1DDSSimEnvCfg()
            values = cfg.model_dump()
            values['unitree'].update(updates)
            with self.assertRaises(ValueError):
                H1DDSSimEnvCfg.model_validate(values)


class DDSIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.log = tempfile.TemporaryFile(mode='w+')
        self.bridge = subprocess.Popen([sys.executable, '-u', str(ROOT / 'scripts/run_h1_dds_sim.py'),
                                        '--headless', '--duration', '60'], cwd=ROOT,
                                       stdout=self.log, stderr=subprocess.STDOUT)
        self.env = None
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            self.log.seek(0)
            if 'H1 DDS MuJoCo ready' in self.log.read():
                return
            if self.bridge.poll() is not None:
                break
            time.sleep(.1)
        self.log.seek(0)
        self.fail(self.log.read())

    def tearDown(self):
        if self.env:
            self.env.shutdown()
        self.bridge.terminate()
        self.bridge.wait(timeout=5)
        self.log.close()

    def test_motor_roundtrip_torque_imu_crc_pause_reset_and_state_loss(self):
        self.env = env = UnitreeDDSSimEnv(H1DDSSimEnvCfg())
        env.reborn()
        env.update()
        np.testing.assert_allclose(env.dof_pos, env.default_pos, atol=1e-6)
        self.assertEqual(env.low_state.crc, CRC().Crc(env.low_state))
        self.assertEqual(env.base_ang_vel.shape, (3,))
        start = env.dof_pos.copy()
        target = start.copy()
        target[13] += .12
        env.step(target)
        # Assert serialized fields and mapping before the physics response.
        motor = env.low_cmd.motor_cmd[env.cfg_env.joint2motor_idx[13]]
        self.assertAlmostEqual(motor.q, target[13])
        self.assertEqual(motor.dq, 0)
        env.set_sim_active(True)
        for _ in range(12):
            env.step(target)
            time.sleep(.02)
            env.update()
        self.assertGreater(abs(env.dof_pos[13] - start[13]), .02)
        self.assertGreater(env._sim_status['commands_received'], 5)
        self.assertEqual(env._sim_status['commands_rejected'], 0)
        env.set_sim_active(False)
        time.sleep(.05)
        env.update()
        frozen = env.dof_pos.copy()
        time.sleep(.1)
        env.update()
        np.testing.assert_allclose(env.dof_pos, frozen, atol=1e-7)
        env.reborn()
        time.sleep(.05)
        env.update()
        np.testing.assert_allclose(env.dof_pos, start, atol=1e-6)
        self.bridge.terminate()
        self.bridge.wait(timeout=5)
        time.sleep(.6)
        with self.assertRaisesRegex(TimeoutError, 'stale'):
            env.update()

    def test_policy_and_quest_controller_over_real_dds(self):
        from test_quest_h2h import FakeReader, frame
        cfg = ConfigManager('h1_h2h_quest').get_cfg()
        cfg.env = H1DDSSimEnvCfg()
        cfg.ctrl = [c for c in cfg.ctrl if c.ctrl_type == 'QuestH2HCtrl']
        with patch('robojudo.controller.quest_h2h_ctrl.load_reader_class', return_value=FakeReader):
            pipeline = QuestH2HPipeline(cfg)
        self.env = pipeline.env
        try:
            control = pipeline.quest
            control.reader.frame = frame()
            pipeline.step()
            control.command('[QUEST_CALIBRATE]')
            for seq in range(1, 8):
                control.reader.frame = frame(seq)
                pipeline.step()
                time.sleep(.02)
            self.assertIsNotNone(control.alignment)
            control.command('[QUEST_ENABLE]')
            for seq in range(8, 158):
                payload = frame(seq)
                payload['points']['left']['position_m'][0] += .04 * math.sin(seq / 10)
                control.reader.frame = payload
                pipeline.step()
                time.sleep(.02)
            self.assertTrue(control.enabled)
            self.assertGreater(pipeline.env._sim_status['simulation_time_s'], 2.0)
            self.assertGreater(pipeline.env._sim_status['root_height_m'], .5)
            self.assertTrue(pipeline.env._sim_status['targets_received'])
            goals = np.asarray(pipeline.env._sim_status['target_points_m'])
            self.assertEqual(goals.shape, (3, 3))
            self.assertTrue(np.isfinite(goals).all())
            self.assertTrue(np.isfinite(pipeline.env.dof_pos).all())
            # VR loss must pause the simulator and invalidate calibration.
            control.reader.frame = None
            pipeline.step()
            self.assertFalse(control.enabled)
            self.assertTrue(pipeline.env._sim_status['paused'])
        finally:
            pipeline.close()
            self.env = None

    def test_sender_changes_to_damping_when_policy_targets_stop(self):
        self.env = env = UnitreeDDSSimEnv(H1DDSSimEnvCfg())
        env.reborn()
        env.step(env.default_pos)
        time.sleep(.6)
        # Check the actual publication from the recurrent sender, not low_cmd:
        # timeout fallback must not overwrite the operator's stored command.
        with patch.object(env.lowcmd_publisher_, 'Write') as write:
            env.send_cmd()
            sent = write.call_args.args[0]
            self.assertTrue(all(m.kp == 0 and m.kd == 8 for m in sent.motor_cmd))
            self.assertEqual(sent.crc, CRC().Crc(sent))
        self.assertGreater(env.low_cmd.motor_cmd[env.cfg_env.joint2motor_idx[0]].kp, 0)

    def test_bad_packet_and_command_loss_require_explicit_resume(self):
        self.env = env = UnitreeDDSSimEnv(H1DDSSimEnvCfg())
        env.reborn()
        env.step(env.default_pos)
        env.set_sim_active(True)
        # Stop only the sender; the controller's state subscriber stays alive.
        env.lowcmd_send_thread.Wait(1)
        bad = copy.deepcopy(env.low_cmd)
        bad.crc ^= 1
        env.lowcmd_publisher_.Write(bad)
        time.sleep(.4)
        self.assertEqual(env._sim_status['commands_rejected'], 1)
        self.assertEqual(env._sim_status['command_timeout_count'], 1)
        self.assertTrue(env._sim_status['paused'])
        self.assertIsNotNone(env.consume_sim_fault())
        self.assertIsNone(env.consume_sim_fault())
        # A new valid command alone must not restart physics.
        env.step(env.default_pos)
        time.sleep(.06)
        self.assertTrue(env._sim_status['paused'])
        env.set_sim_active(True)
        self.assertFalse(env._sim_status['paused'])


if __name__ == '__main__':
    unittest.main()
