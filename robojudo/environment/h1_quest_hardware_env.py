"""Gated H1 command publisher. Constructing this environment sends no commands."""
import time

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

from robojudo.environment.unitree_env import UnitreeEnv
from robojudo.environment.utils.h1_state_check import valid_h1_state
from robojudo.environment.utils.unitree_command import create_damping_cmd


class H1QuestHardwareEnv(UnitreeEnv):
    def __init__(self, cfg_env, device='cpu'):
        if (cfg_env.is_sim or cfg_env.unitree.net_if == 'lo' or cfg_env.unitree.domain_id != 0
                or cfg_env.unitree.robot != 'h1' or cfg_env.unitree.msg_type != 'go'
                or cfg_env.unitree.hand_type != 'NONE' or not cfg_env.act or cfg_env.dof.num_dofs != 19):
            raise ValueError('Hardware environment requires H1 go protocol, hardware interface and domain 0')
        self.armed = False
        self.emergency = False
        self.rejected_states = 0
        try:
            super().__init__(cfg_env, device)
        except Exception:
            # Failure before arming must remain publication-free.
            if hasattr(self, 'lowcmd_send_thread'):
                self.lowcmd_send_thread.Wait(1)
            for name in ('lowstate_subscriber', 'lowcmd_publisher_'):
                if hasattr(self, name):
                    getattr(self, name).Close()
            raise

    def LowStateGoHandler(self, message):
        if not valid_h1_state(message):
            self.rejected_states += 1
            return
        # Same Select bit as the SDK's H1 remote layout. Latch on press,
        # independently of GUI key events and the main policy loop.
        if int.from_bytes(bytes(message.wireless_remote[2:4]), 'little') & (1 << 3):
            self.emergency = True
        super().LowStateGoHandler(message)

    def arm(self):
        if self.emergency:
            raise RuntimeError('Remote Select stop is latched; restart after checking robot')
        self.update()
        switcher = MotionSwitcherClient()
        switcher.SetTimeout(3)
        switcher.Init()
        code, mode = switcher.CheckMode()
        if code != 0 or not isinstance(mode, dict):
            raise RuntimeError('Cannot verify the robot motion service; arming refused')
        if mode.get('name'):
            raise RuntimeError(f'Robot motion service still active: {mode["name"]}; enter the supported debug mode first')
        with self._cmd_lock:
            self._last_target_received = time.monotonic()
            for j, motor in enumerate(self._dof_idx):
                self.set_cmd_i(motor, float(self.dof_pos[j]), kp=0, kd=0)
            self.armed = True

    def send_cmd(self, cmd=None):
        if not self.armed:
            return
        with self._cmd_lock:
            if self.emergency:
                create_damping_cmd(self.low_cmd)
                cmd = self.low_cmd
            super().send_cmd(cmd)

    def shutdown(self):
        try:
            if self.armed:
                self.emergency = True
                self.set_damping_mode()
                # Brief best-effort burst before closing DDS. This still cannot
                # guarantee delivery over a broken cable or a killed process.
                time.sleep(.05)
            super().shutdown()
        finally:
            self.armed = False
