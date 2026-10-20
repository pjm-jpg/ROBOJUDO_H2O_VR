"""Quest fixed-space three-point references for the H2H deployment policy."""
import importlib.util
import logging
import math
import time
from pathlib import Path

import numpy as np

from robojudo.controller import Controller, ctrl_registry
from robojudo.controller.ctrl_cfgs import QuestH2HCtrlCfg
from robojudo.utils.util_func import my_quat_rotate_np

logger = logging.getLogger(__name__)
POINT_ORDER = ('left', 'right', 'head')


def load_reader_class(root):
    # Load the shared, stdlib-only receiver without importing the legacy package
    # initializer (which otherwise requires pure-python-adb in this environment).
    path = Path(root).expanduser() / 'oculus_reader/three_point_reader.py'
    if not path.is_file():
        raise FileNotFoundError(f'Quest receiver not found: {path}; configure reader_root')
    spec = importlib.util.spec_from_file_location('robojudo_quest_receiver', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.ThreePointReader


def yaw_xyzw(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q), 1, atol=0.01):
        raise ValueError('Invalid heading quaternion')
    x, y, z, w = q
    forward = np.array([1 - 2 * (y*y + z*z), 2 * (x*y + w*z)])
    if np.linalg.norm(forward) < 0.2:
        raise ValueError('Look forward horizontally to calibrate')
    return math.atan2(forward[1], forward[0])


def rotation_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


class QuestAlignment:
    """Fixed yaw alignment and offsets; output is in the robot world frame."""
    def __init__(self, human_points, robot_points, human_yaw, robot_yaw, head_scale=1, hand_scale=1):
        self.human = np.array(human_points, dtype=float, copy=True)
        self.robot = np.array(robot_points, dtype=float, copy=True)
        self.rotation = rotation_z(robot_yaw - human_yaw)
        self.head_scale, self.hand_scale = head_scale, hand_scale

    def map(self, points):
        points = np.asarray(points, dtype=float)
        head = self.robot[2] + self.head_scale * self.rotation @ (points[2] - self.human[2])
        hands = []
        for i in (0, 1):
            delta = (points[i] - points[2]) - (self.human[i] - self.human[2])
            hands.append(head + (self.robot[i] - self.robot[2]) + self.hand_scale * self.rotation @ delta)
        return np.array([*hands, head])


@ctrl_registry.register
class QuestH2HCtrl(Controller):
    """No motion library, actor inference or robot actuation in this controller."""
    def __init__(self, cfg_ctrl: QuestH2HCtrlCfg, env=None, device='cpu'):
        super().__init__(cfg_ctrl=cfg_ctrl, env=env, device=device)
        self.cfg_ctrl = cfg_ctrl
        if env is None or env.kinematics is None:
            raise ValueError('QuestH2HCtrl requires robot forward kinematics')
        self.names = [entry['joint_name'] for entry in cfg_ctrl.extend_config]
        if self.names != ['left_hand_link', 'right_hand_link', 'head_link']:
            raise ValueError('Expected extension order: left hand, right hand, head')
        self.reset()
        reader_class = load_reader_class(cfg_ctrl.reader_root)
        self.reader = reader_class(ip=cfg_ctrl.ip_address, port=cfg_ctrl.port, serial=cfg_ctrl.serial,
                                   adb=cfg_ctrl.adb, record=cfg_ctrl.record)
        try:
            self.reader.start()
        except Exception:
            self.reader.close()
            raise

    def reset(self):
        self.alignment = None
        self.enabled = False
        self.calibrating = False
        self.samples = []
        self.stream_key = None
        self.last_sample = None
        self.frame = None
        self.positions = None
        self.filtered = None
        self.reason = 'not calibrated; press C'
        self._last_logged_reason = None
        self._last_report = 0.0
        self._previous_problem = None
        self._last_health_report = time.monotonic()

    def close(self):
        if hasattr(self, 'reader'):
            self.reader.close()

    def _report_receiver_health(self, force=False):
        now = time.monotonic()
        get_status = getattr(self.reader, 'get_status', None)
        if get_status is None or (not force and now - self._last_health_report < 5):
            return
        status = get_status()
        age = status['last_frame_age_s']
        logger.warning('Quest receiver: logcat_running=%s, frames=%s, malformed=%s, last_frame_age=%s, error=%s',
                       status['logcat_running'], status['frames_received'], status['malformed_frames'],
                       'never received' if age is None else f'{age:.1f}s', status['last_error'])
        self._last_health_report = now

    def robot_points(self):
        fk = self.env.fk_info
        if fk is None:
            raise ValueError('Robot FK is unavailable')
        points = []
        for entry in self.cfg_ctrl.extend_config:
            body = fk[entry['parent_name']]
            offset = my_quat_rotate_np(np.asarray(body['quat']).reshape(1, 4),
                                       np.asarray(entry['pos']).reshape(1, 3))[0]
            points.append(np.asarray(body['pos']) + offset)
        return np.asarray(points)

    def invalidate(self, reason, recalibrate=False):
        self.enabled = False
        self.reason = reason
        self.samples = []
        if recalibrate:
            self.alignment = None
            self.calibrating = False
            self.filtered = None

    def _quality(self, frame, for_calibration=False):
        if frame is None:
            return 'no fresh data / disconnected'
        if frame['reference_space'] != 'stage':
            return 'floor space required (STAGE)'
        if not frame['focused']:
            return 'headset app not focused; recalibrate after resuming'
        for name in POINT_ORDER:
            point = frame['points'][name]
            if not point['valid']:
                return f'{name}: no valid tracking'
            if (self.cfg_ctrl.require_tracked or for_calibration) and not point['tracked']:
                return f'{name}: waiting for TRACKED'
        return None

    def command(self, command):
        if command == '[QUEST_PAUSE]':
            self.enabled = False
            self.calibrating = False
            self.reason = 'paused; E to resume, C to recalibrate'
        elif command == '[QUEST_CALIBRATE]':
            logger.info('Quest: C received / calibration requested')
            problem = self._quality(self.frame, for_calibration=True)
            if problem:
                self.invalidate('cannot calibrate: ' + problem, recalibrate=True)
                logger.warning('Quest: %s; restore tracking then press C again', self.reason)
                if self.frame is None:
                    self._report_receiver_health(force=True)
                return
            self.invalidate('calibrating: stand still and look forward', recalibrate=True)
            self.calibrating = True
            self.last_sample = None
        elif command == '[QUEST_ENABLE]':
            if self.enabled:
                return
            problem = self._quality(self.frame)
            if problem:
                self.invalidate(problem)
            elif self.alignment is None:
                self.reason = 'calibrate with C before enabling'
            else:
                self.enabled = True
                self.filtered = self.robot_points().copy()
                self.reason = 'enabled'

    def _calibrate(self, frame, positions, robot_points):
        problem = self._quality(frame, for_calibration=True)
        if problem:
            self.samples = []
            self.reason = 'calibration: ' + problem
            return
        head = frame['points']['head']
        if not head['orientation_valid'] or not head['orientation_tracked']:
            self.samples = []
            self.reason = 'calibration: head orientation must be TRACKED'
            return
        try:
            yaw = yaw_xyzw(head['orientation_xyzw'])
        except ValueError as error:
            self.samples = []
            self.reason = str(error)
            return
        if self.samples and frame['sample_time_ns'] <= self.samples[-1][0]:
            return  # Do not count the same received frame more than once.
        if self.samples:
            yaw_delta = math.atan2(math.sin(yaw - self.samples[0][2]), math.cos(yaw - self.samples[0][2]))
            if (np.linalg.norm(positions - self.samples[0][1], axis=1).max() > self.cfg_ctrl.calibration_tolerance
                    or abs(yaw_delta) > math.radians(10)):
                self.samples = []
        self.samples.append((frame['sample_time_ns'], positions.copy(), yaw))
        duration = (self.samples[-1][0] - self.samples[0][0]) * 1e-9
        self.reason = f'calibrating: keep still {duration:.1f}/{self.cfg_ctrl.calibration_seconds:.1f}s'
        if duration < self.cfg_ctrl.calibration_seconds or len(self.samples) < 5:
            return
        human = np.mean([sample[1] for sample in self.samples], axis=0)
        angles = np.array([sample[2] for sample in self.samples])
        human_yaw = math.atan2(np.sin(angles).mean(), np.cos(angles).mean())
        self.alignment = QuestAlignment(human, robot_points, human_yaw, yaw_xyzw(self.env.base_quat),
                                        self.cfg_ctrl.head_scale, self.cfg_ctrl.hand_scale)
        self.filtered = robot_points.copy()
        self.samples = []
        self.calibrating = False
        self.reason = 'calibrated / preview; E to enable, SPACE to pause'
        logger.info('Quest calibrated: human yaw %.1f deg; point order left/right/head', math.degrees(human_yaw))

    def get_data(self):
        robot = self.robot_points()
        frame = self.reader.get_latest(max_age=self.cfg_ctrl.max_age)
        self.frame = frame
        if frame is not None:
            key = (frame['session_id'], frame['space_epoch'])
            if self.stream_key is not None and key != self.stream_key:
                self.invalidate('reference space/session changed; press C to recalibrate', recalibrate=True)
                self.last_sample = None
            self.stream_key = key
        problem = self._quality(frame)
        velocity = np.zeros((3, 3))
        target = robot.copy() if self.filtered is None else self.filtered.copy()
        if problem:
            # A gap can hide a recenter or a headset removal. Do not reuse its
            # calibration just because the first frame after the gap is valid.
            self.invalidate(problem, recalibrate=(frame is None or not frame['focused']
                                                 or frame['reference_space'] != 'stage'))
        else:
            if self._previous_problem and not self.calibrating:
                self.reason = ('tracking restored; E to resume, C to recalibrate' if self.alignment is not None
                               else 'data ready; press C to calibrate')
            positions = np.array([frame['points'][name]['position_m'] for name in POINT_ORDER])
            stamp = (frame['session_id'], frame['sequence'])
            is_new = stamp != self.last_sample
            if self.calibrating and is_new:
                self._calibrate(frame, positions, robot)
            self.last_sample = stamp
            self.positions = positions
            if self.alignment is not None:
                desired = self.alignment.map(positions)
                if np.linalg.norm(desired - self.alignment.robot, axis=1).max() > self.cfg_ctrl.max_displacement:
                    self.invalidate('target outside calibrated range; C to recalibrate', recalibrate=True)
                else:
                    if self.enabled:
                        dt = 1.0 / self.cfg_ctrl.frequency
                        alpha = 1 - math.exp(-dt / self.cfg_ctrl.smoothing_seconds)
                        delta = alpha * (desired - self.filtered)
                        norm = np.linalg.norm(delta, axis=1, keepdims=True)
                        delta *= np.minimum(1.0, self.cfg_ctrl.max_speed * dt / np.maximum(norm, 1e-12))
                        self.filtered += delta
                        velocity = delta / dt
                        target = self.filtered.copy()
                    else:
                        target = desired  # Preview target balls without stepping physics.
        self._previous_problem = problem
        if frame is None:
            self._report_receiver_health()
        now = time.monotonic()
        if self.reason != self._last_logged_reason and (not self.calibrating or now - self._last_report >= 1):
            logger.info('Quest: %s', self.reason)
            self._last_logged_reason = self.reason
            self._last_report = now
        return {'ref_body_pos_subset': target, 'ref_body_vel_subset': velocity,
                'robot_body_pos_subset': robot, 'enabled': self.enabled,
                'calibrated': self.alignment is not None, 'status': self.reason,
                'stream_key': self.stream_key}
