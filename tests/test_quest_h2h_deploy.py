"""Staged hardware path rehearsed over real local DDS; never binds a physical NIC."""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
from unitree_sdk2py.utils.crc import CRC

from robojudo.environment.h1_quest_hardware_env import H1QuestHardwareEnv
from robojudo.environment.unitree_env import UnitreeEnv
from robojudo.environment.utils.h1_state_check import valid_h1_state
from robojudo.pipeline.quest_h2h_deploy_pipeline import QuestH2HDeployPipeline
from scripts.run_quest_h2h_real import make_cfg

ROOT = Path(__file__).resolve().parents[1]


class HardwareGateTest(unittest.TestCase):
    def test_wait_never_publishes_and_remote_select_latches(self):
        env = H1QuestHardwareEnv.__new__(H1QuestHardwareEnv)
        env.armed, env.emergency, env.rejected_states = False, False, 0
        env._cmd_lock = threading.RLock()
        with patch.object(UnitreeEnv, 'send_cmd') as send:
            env.send_cmd()
            send.assert_not_called()
        state = unitree_go_msg_dds__LowState_()
        state.head, state.imu_state.quaternion = [0xFE, 0xEF], [1., 0., 0., 0.]
        state.wireless_remote[2] = 1 << 3
        state.crc = CRC().Crc(state)
        self.assertTrue(valid_h1_state(state))
        env.LowStateGoHandler(state)
        self.assertTrue(env.emergency)
        state.wireless_remote[2] = 0
        state.crc = CRC().Crc(state)
        env.LowStateGoHandler(state)
        self.assertTrue(env.emergency)
        state.crc ^= 1
        env.LowStateGoHandler(state)
        self.assertEqual(env.rejected_states, 1)
        env.armed, env.low_cmd = True, unitree_go_msg_dds__LowCmd_()
        with patch.object(UnitreeEnv, 'send_cmd') as send:
            env.send_cmd()
            sent = send.call_args.args[0]
            self.assertTrue(all(m.kp == 0 and m.kd == 8 and m.tau == 0 for m in sent.motor_cmd))

    def test_failed_motion_mode_check_cannot_arm_or_publish(self):
        env = H1QuestHardwareEnv.__new__(H1QuestHardwareEnv)
        env.armed, env.emergency = False, False
        with patch.object(env, 'update'), patch(
                'robojudo.environment.h1_quest_hardware_env.MotionSwitcherClient') as switcher:
            switcher.return_value.CheckMode.return_value = (0, {'name': 'normal'})
            with self.assertRaisesRegex(RuntimeError, 'still active'):
                env.arm()
            self.assertFalse(env.armed)
            switcher.return_value.ReleaseMode.assert_not_called()

    def test_shutdown_latches_damping_before_closing_transport(self):
        env = H1QuestHardwareEnv.__new__(H1QuestHardwareEnv)
        env.armed, env.emergency = True, False
        with patch.object(env, 'set_damping_mode') as damping, patch.object(
                UnitreeEnv, 'shutdown') as close, patch('time.sleep'):
            env.shutdown()
            damping.assert_called_once()
            close.assert_called_once()
        self.assertTrue(env.emergency)
        self.assertFalse(env.armed)


class DeployDDSIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.log = open(Path(self.temp.name) / 'bridge.log', 'w+')
        self.report = Path(self.temp.name) / 'report.json'
        self.process = subprocess.Popen([sys.executable, '-u', 'scripts/run_h1_dds_sim.py',
                                         '--domain', '44', '--headless', '--duration', '90',
                                         '--report', str(self.report)], cwd=ROOT,
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.pipeline = None
        for _ in range(120):
            self.log.seek(0)
            if 'H1 DDS MuJoCo ready' in self.log.read():
                break
            if self.process.poll() is not None:
                self.fail('DDS simulator exited unexpectedly')
            time.sleep(.1)
        else:
            self.fail('DDS simulator startup timeout')

    def tearDown(self):
        if self.pipeline:
            self.pipeline.close()
        if self.process.poll() is None:
            self.process.send_signal(2)
            self.process.wait(timeout=6)
        self.log.close()
        self.temp.cleanup()

    def test_read_only_launcher_sends_no_commands(self):
        result = subprocess.run([sys.executable, '-u', 'scripts/run_quest_h2h_real.py',
                                 '--dds-sim', '--dds-domain', '44', '--duration', '1'],
                                cwd=ROOT, capture_output=True, text=True, timeout=12)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('"command_messages_sent": 0', result.stdout)
        self.process.send_signal(2)
        self.process.wait(timeout=6)
        self.assertEqual(json.loads(self.report.read_text())['commands_received'], 0)

    def test_prepare_stand_vr_loss_keeps_policy_running_and_never_auto_enables(self):
        from test_quest_h2h import FakeReader, frame
        args = SimpleNamespace(interface='enp3s0', dds_sim=True, dds_domain=44, ip=None,
                               serial=None, adb=None, reader_root=None, record=None, no_keyboard=True)
        cfg = make_cfg(args)
        cfg.prepare_seconds, cfg.return_seconds, cfg.blend_seconds = .15, .1, .1
        with patch('robojudo.controller.quest_h2h_ctrl.load_reader_class', return_value=FakeReader):
            self.pipeline = pipeline = QuestH2HDeployPipeline(cfg)
        pipeline.quest.reader.frame = frame()
        pipeline.step()
        self.assertEqual(pipeline.phase, 'WAIT')
        pipeline._commands({'[QUEST_ENABLE]'}, pipeline.env.get_data())
        self.assertEqual(pipeline.phase, 'WAIT')
        pipeline._commands({'[H1_PREPARE]'}, pipeline.env.get_data())
        for _ in range(12):
            pipeline.step()
            time.sleep(.02)
        self.assertEqual(pipeline.phase, 'READY')
        pipeline._commands({'[QUEST_PAUSE]'}, pipeline.env.get_data())
        self.assertEqual(pipeline.phase, 'HOLD')
        pipeline.step()
        pipeline._commands({'[H1_PREPARE]'}, pipeline.env.get_data())
        for _ in range(12):
            pipeline.step()
            time.sleep(.02)
        self.assertEqual(pipeline.phase, 'READY')
        pipeline._commands({'[H1_STAND]'}, pipeline.env.get_data())
        for seq in range(1, 12):
            pipeline.quest.reader.frame = frame(seq)
            pipeline.step()
            time.sleep(.02)
        self.assertEqual(pipeline.phase, 'STAND')
        pipeline._commands({'[QUEST_CALIBRATE]'}, pipeline.env.get_data())
        for seq in range(12, 20):
            pipeline.quest.reader.frame = frame(seq)
            pipeline.step()
            time.sleep(.02)
        pipeline._commands({'[QUEST_ENABLE]'}, pipeline.env.get_data())
        self.assertEqual(pipeline.phase, 'VR')
        for seq in range(20, 30):
            pipeline.quest.reader.frame = frame(seq)
            pipeline.step()
            time.sleep(.02)
        pipeline.quest.reader.frame = None
        steps_before = pipeline._policy_steps
        for _ in range(10):
            pipeline.step()
            time.sleep(.02)
        self.assertEqual(pipeline.phase, 'STAND')
        self.assertGreater(pipeline._policy_steps, steps_before)
        self.assertFalse(pipeline.env._sim_status['paused'])
        pipeline.quest.reader.frame = frame(31)
        pipeline.step()
        self.assertEqual(pipeline.phase, 'STAND')
        self.assertFalse(pipeline.quest.enabled)
        self.assertTrue(np.isfinite(pipeline.env.dof_pos).all())

    def test_sustained_landing_balance_and_vr_loss(self):
        """Real-time regression: landing, 30s balance, VR and fallback, not just message delivery."""
        from test_quest_h2h import FakeReader, frame
        args = SimpleNamespace(interface='enp3s0', dds_sim=True, dds_domain=44, ip=None,
                               serial=None, adb=None, reader_root=None, record=None, no_keyboard=True)
        cfg = make_cfg(args)  # Keep the deployment defaults, including 5s preparation.
        with patch('robojudo.controller.quest_h2h_ctrl.load_reader_class', return_value=FakeReader):
            self.pipeline = pipeline = QuestH2HDeployPipeline(cfg)
        samples = []
        pipeline.step()
        pipeline._commands({'[H1_PREPARE]'}, pipeline.env.get_data())
        for _ in range(270):
            start = time.monotonic()
            pipeline.step()
            time.sleep(max(0, pipeline.dt-(time.monotonic()-start)))
        self.assertEqual(pipeline.phase, 'READY')
        pipeline._commands({'[H1_STAND]'}, pipeline.env.get_data())
        began = time.monotonic()
        passed = False

        def tick():
            start = time.monotonic()
            pipeline.step()
            q = pipeline.env.base_quat
            tilt = float(np.arccos(np.clip(1-2*(q[0]**2+q[1]**2), -1, 1)))
            height = pipeline.env._sim_status['root_height_m']
            samples.append(dict(t=time.monotonic()-began, height=height, tilt=tilt))
            self.assertGreater(height, .65, 'Robot fell after the policy was enabled')
            self.assertLess(tilt, .5, 'Robot failed sustained upright balance')
            time.sleep(max(0, pipeline.dt-(time.monotonic()-start)))

        try:
            for _ in range(1500):
                tick()
            self.assertEqual(pipeline.phase, 'STAND')
            pipeline.quest.reader.frame = frame(1)
            tick()
            pipeline._commands({'[QUEST_CALIBRATE]'}, pipeline.env.get_data())
            for seq in range(2, 112):
                pipeline.quest.reader.frame = frame(seq)
                tick()
            pipeline._commands({'[QUEST_ENABLE]'}, pipeline.env.get_data())
            self.assertEqual(pipeline.phase, 'VR')
            for seq in range(112, 212):
                payload = frame(seq)
                payload['points']['left']['position_m'][0] += .03*math.sin((seq-112)/25)
                pipeline.quest.reader.frame = payload
                tick()
            pipeline.quest.reader.frame = None
            for _ in range(150):
                tick()
            self.assertEqual(pipeline.phase, 'STAND')
            self.assertFalse(pipeline.env._sim_status['paused'])
            self.assertFalse(pipeline.quest.enabled)
            self.assertEqual(pipeline.env._sim_status['command_timeout_count'], 0)
            passed = True
        finally:
            report = dict(passed=passed, scope='Local DDS MuJoCo only; synthetic VR, no hardware',
                          samples=len(samples), elapsed_s=time.monotonic()-began,
                          min_height_m=min((s['height'] for s in samples), default=None),
                          max_tilt_rad=max((s['tilt'] for s in samples), default=None),
                          simulator=pipeline.env._sim_status)
            Path('/tmp/quest-h1-balance-validation.json').write_text(json.dumps(report, indent=2))


if __name__ == '__main__':
    unittest.main()
