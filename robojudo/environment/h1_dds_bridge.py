"""H1 go-protocol DDS robot endpoint backed by MuJoCo, restricted to localhost."""
import json
import threading
import time
import uuid

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from robojudo.config.h1.env.h1_real_env_cfg import H1RealEnvCfg
from robojudo.environment.utils.dds_sim import CONTROL_TOPIC, STATUS_TOPIC, TARGET_TOPIC, initialize_sim_dds


def command_arrays(command, mapping):
    if list(command.head) != [0xFE, 0xEF] or command.level_flag != 0xFF:
        raise ValueError('Invalid H1 lowcmd header')
    if command.crc != CRC().Crc(command):
        raise ValueError('Invalid lowcmd CRC')
    motors = [command.motor_cmd[i] for i in mapping]
    values = np.array([[m.q, m.dq, m.kp, m.kd, m.tau] for m in motors], dtype=float)
    if not np.isfinite(values).all() or np.any(values[:, 2:4] < 0):
        raise ValueError('Non-finite command or negative gain')
    if any(m.mode not in (0, 1, 0x0A) for m in motors):
        raise ValueError('Unsupported motor mode')
    # Official stop sentinels disable the corresponding position/velocity term.
    values[values[:, 0] >= 1e9, 2] = 0
    values[values[:, 1] >= 16000, 3] = 0
    values[[m.mode == 0 for m in motors], 2:] = 0
    return values


class H1DDSBridge:
    def __init__(self, domain=42, headless=False, command_timeout=.25):
        initialize_sim_dds(domain)
        self.cfg = H1RealEnvCfg()
        self.domain = domain
        self.command_timeout = command_timeout
        self.model = mujoco.MjModel.from_xml_path(self.cfg.xml)
        self.model.opt.timestep = .001
        self.data = mujoco.MjData(self.model)
        self.mapping = self.cfg.joint2motor_idx
        self.qadr, self.vadr, self.actuators = [], [], []
        for name in self.cfg.dof.joint_names:
            joint = self.model.joint(name)
            self.qadr.append(int(joint.qposadr[0]))
            self.vadr.append(int(joint.dofadr[0]))
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if aid < 0 or self.model.actuator_trnid[aid, 0] != joint.id:
                raise ValueError(f'Missing or mismatched actuator: {name}')
            self.actuators.append(aid)
        # XML uses <position> actuators. DDS already supplies the PD torque, so
        # remove their built-in position bias and use a unit torque gain.
        self.model.actuator_gainprm[self.actuators] = 0
        self.model.actuator_gainprm[self.actuators, 0] = 1
        self.model.actuator_biasprm[self.actuators] = 0
        self.limits = np.asarray(self.cfg.dof.torque_limits)
        self.model.actuator_ctrlrange[self.actuators] = np.column_stack((-self.limits, self.limits))
        self.lock = threading.Lock()
        self.command = None
        self.last_command = None
        self.pending = None
        self.paused = True
        self.server_id = uuid.uuid4().hex
        self.request_id = None
        self.reason = 'startup preview'
        self.received = self.rejected = self.state_count = self.timeout_count = 0
        self.tick = 0
        self.max_command_gap = 0
        self.last_rejection = None
        self.last_torque = np.zeros(19)
        self.targets = self.target_root = None
        self.targets_received = None
        self.reset()
        self.viewer = None
        if not headless:
            import mujoco_viewer
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data, width=1200, height=900,
                                                   hide_menus=True, diable_key_callbacks=True)
            self.viewer.cam.distance = 3
            self.viewer.cam.azimuth = 180
        self.state_pub = ChannelPublisher('rt/lowstate', LowState_)
        self.state_pub.Init()
        self.status_pub = ChannelPublisher(STATUS_TOPIC, String_)
        self.status_pub.Init()
        self.cmd_sub = ChannelSubscriber('rt/lowcmd', LowCmd_)
        self.cmd_sub.Init(self.on_command, 10)
        self.control_sub = ChannelSubscriber(CONTROL_TOPIC, String_)
        self.control_sub.Init(self.on_control, 10)
        self.target_sub = ChannelSubscriber(TARGET_TOPIC, String_)
        self.target_sub.Init(self.on_targets, 1)

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.paused = True
        self.reason = 'reset preview'
        with self.lock:
            self.targets = self.target_root = self.targets_received = None

    def on_targets(self, message):
        try:
            value = json.loads(message.data)
            if value['server_id'] != self.server_id or value['order'] != ['left', 'right', 'head']:
                return
            points = np.asarray(value['targets_m'], dtype=float)
            root = np.asarray(value['base_pos_m'], dtype=float)
            if points.shape != (3, 3) or root.shape != (3,) or not np.isfinite(points).all() or not np.isfinite(root).all():
                return
        except (ValueError, TypeError, KeyError):
            return
        with self.lock:
            self.targets, self.target_root, self.targets_received = points, root, time.monotonic()

    def draw_targets(self):
        with self.lock:
            visible = self.targets_received is not None and time.monotonic() - self.targets_received <= .5
            # DUMMY odometry has a fixed root. Project policy-relative goals to
            # the simulated root so the balls depict the actual policy task.
            points = (self.targets - self.target_root + self.data.qpos[:3]) if visible else np.zeros((3, 3))
        for i, label in enumerate(('LEFT', 'RIGHT', 'HEAD')):
            self.viewer.add_marker(id=1000 + i, pos=points[i], size=.04 if visible else 0,
                                   rgba=(0, 1, 0, 1 if visible else 0),
                                   type=mujoco.mjtGeom.mjGEOM_SPHERE, label=label if visible else '')

    def on_command(self, message):
        try:
            command = command_arrays(message, self.mapping)
        except (ValueError, OverflowError, TypeError) as exc:
            with self.lock:
                self.rejected += 1
                self.last_rejection = str(exc)
            return
        now = time.monotonic()
        with self.lock:
            if self.last_command is not None:
                self.max_command_gap = max(self.max_command_gap, now - self.last_command)
            self.command, self.last_command = command, now
            self.received += 1

    def on_control(self, message):
        try:
            request = json.loads(message.data)
            if request['server_id'] != self.server_id or request['action'] not in ('pause', 'resume', 'reset'):
                return
            with self.lock:
                self.pending = request
        except (ValueError, KeyError, TypeError):
            return

    def cycle(self):
        now = time.monotonic()
        with self.lock:
            command = None if self.command is None else self.command.copy()
            age = None if self.last_command is None else now - self.last_command
            request, self.pending = self.pending, None
        if request and request['request_id'] != self.request_id:
            action = request['action']
            if action == 'reset':
                self.reset()
            elif action == 'pause':
                self.paused, self.reason = True, 'operator pause'
            elif command is not None and age <= self.command_timeout:
                self.paused, self.reason = False, 'running'
            self.request_id = request['request_id']
        if not self.paused and (age is None or age > self.command_timeout):
            self.paused, self.reason = True, 'lowcmd timeout; explicit resume required'
            self.timeout_count += 1
        if not self.paused:
            q, dq, kp, kd, tau = command.T
            torque = kp * (q - self.data.qpos[self.qadr]) + kd * (dq - self.data.qvel[self.vadr]) + tau
            self.last_torque = np.clip(torque, -self.limits, self.limits)
            self.data.ctrl[self.actuators] = self.last_torque
            mujoco.mj_step(self.model, self.data)
        self.tick = self.tick % 0xFFFFFFFF + 1
        if self.tick % 2 == 0:
            self.publish_state()
        if self.tick % 50 == 0:
            self.status_pub.Write(String_(json.dumps(self.status())))

    def publish_state(self):
        state = unitree_go_msg_dds__LowState_()
        state.head, state.level_flag, state.tick = [0xFE, 0xEF], 0xFF, self.tick
        for j, motor in enumerate(self.mapping):
            item = state.motor_state[motor]
            item.mode = 1 if motor in self.cfg.weak_motor else 0x0A
            item.q = item.q_raw = float(self.data.qpos[self.qadr[j]])
            item.dq = item.dq_raw = float(self.data.qvel[self.vadr[j]])
            item.ddq = item.ddq_raw = float(self.data.qacc[self.vadr[j]])
            item.tau_est = float(self.data.qfrc_actuator[self.vadr[j]])
        # These XML sensors are physically attached to the H1 torso IMU site.
        quat = self.data.sensor('imu-torso-orientation').data.copy()
        state.imu_state.quaternion = quat.tolist()  # wxyz, as in the SDK
        state.imu_state.gyroscope = self.data.sensor('imu-torso-angular-velocity').data.tolist()
        state.imu_state.accelerometer = self.data.sensor('imu-linear-acceleration').data.tolist()
        state.imu_state.rpy = Rotation.from_quat(quat, scalar_first=True).as_euler('xyz').tolist()
        state.crc = CRC().Crc(state)
        self.state_pub.Write(state)
        self.state_count += 1

    def status(self):
        with self.lock:
            return dict(protocol='robojudo-mujoco-h1-v1', server_id=self.server_id,
                        request_id=self.request_id, paused=self.paused, reason=self.reason,
                        commands_received=self.received, commands_rejected=self.rejected,
                        last_rejection=self.last_rejection, states_sent=self.state_count,
                        command_timeout_count=self.timeout_count, max_command_gap_s=self.max_command_gap,
                        simulation_time_s=self.data.time, root_height_m=float(self.data.qpos[2]),
                        targets_received=self.targets_received is not None,
                        target_points_m=None if self.targets is None else self.targets.tolist(),
                        domain=self.domain, interface='lo')

    def run(self, duration=None):
        start, deadline = time.monotonic(), time.monotonic()
        render_at = start
        while duration is None or time.monotonic() - start < duration:
            self.cycle()
            now = time.monotonic()
            if self.viewer and now >= render_at:
                if not self.viewer.is_alive:
                    break
                self.viewer.cam.lookat = self.data.qpos[:3]
                self.draw_targets()
                self.viewer.render()
                render_at = now + 1 / 60
            deadline += self.model.opt.timestep
            if deadline < now - .1:
                deadline = now
            time.sleep(max(0, deadline - time.monotonic()))

    def close(self):
        self.cmd_sub.Close()
        self.control_sub.Close()
        self.target_sub.Close()
        self.state_pub.Close()
        self.status_pub.Close()
        if self.viewer:
            self.viewer.close()
