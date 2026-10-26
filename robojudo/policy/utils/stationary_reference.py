"""Conservative reference governor; it does not constrain the Actor's balance output."""
import logging
import math
import time

import numpy as np
from pydantic import Field, model_validator

from robojudo.config import Config

logger = logging.getLogger(__name__)


class StationaryReferenceCfg(Config):
    enabled: bool = False
    forward_m: float = Field(default=.90, gt=0, allow_inf_nan=False)
    backward_m: float = Field(default=.25, gt=0, allow_inf_nan=False)
    lateral_m: float = Field(default=.90, gt=0, allow_inf_nan=False)
    upward_m: float = Field(default=.90, gt=0, allow_inf_nan=False)
    downward_m: float = Field(default=.65, gt=0, allow_inf_nan=False)
    shoulder_reach_m: float = Field(default=.90, gt=0, allow_inf_nan=False)
    hand_separation_m: float = Field(default=.12, gt=0, allow_inf_nan=False)
    soft_boundary: float = Field(default=.97, gt=0, lt=1, allow_inf_nan=False)
    max_speed_m_s: float = Field(default=.80, gt=0, allow_inf_nan=False)
    smoothing_seconds: float = Field(default=.05, gt=0, allow_inf_nan=False)
    tilt_soft_rad: float = Field(default=math.radians(10), gt=0, allow_inf_nan=False)
    tilt_hard_rad: float = Field(default=math.radians(20), gt=0, allow_inf_nan=False)
    drift_soft_m: float = Field(default=.06, gt=0, allow_inf_nan=False)
    drift_hard_m: float = Field(default=.12, gt=0, allow_inf_nan=False)
    foot_lift_soft_m: float = Field(default=.035, gt=0, allow_inf_nan=False)
    foot_lift_hard_m: float = Field(default=.07, gt=0, allow_inf_nan=False)
    tracking_soft_m: float = Field(default=.12, gt=0, allow_inf_nan=False)
    tracking_hard_m: float = Field(default=.25, gt=0, allow_inf_nan=False)
    tracking_delay_seconds: float = Field(default=.5, gt=0, allow_inf_nan=False)
    tracking_baseline_max_m: float = Field(default=.15, ge=0, allow_inf_nan=False)
    recovery_seconds: float = Field(default=1., gt=0, allow_inf_nan=False)
    feedback_warmup_seconds: float = Field(default=2., ge=0, allow_inf_nan=False)

    @model_validator(mode='after')
    def check_intervals(self):
        for prefix, unit in [('tilt', 'rad'), ('drift', 'm'), ('foot_lift', 'm'), ('tracking', 'm')]:
            if getattr(self, f'{prefix}_hard_{unit}') <= getattr(self, f'{prefix}_soft_{unit}'):
                raise ValueError(f'{prefix}: hard threshold must exceed soft threshold')
        return self


def heading(quat):
    x, y, z, w = np.asarray(quat, dtype=float)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


class StationaryReferenceGuard:
    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.anchor = None
        self.target = None
        self.tracking_baseline = np.zeros(2)
        self._active = False
        self.scale = self.ceiling = 1.
        self.bad_tracking_seconds = self.recovery_time = self.feedback_elapsed = 0.
        self._last_reason = None
        self._last_report = -float('inf')

    @staticmethod
    def points(value):
        value = np.asarray(value, dtype=float)
        if value.shape != (3, 3) or not np.isfinite(value).all():
            raise ValueError('Stationary references must be finite left/right/head positions, shape (3,3)')
        return value.copy()

    def _anchor(self, anchor, robot, data):
        self.anchor = anchor.copy()
        self.target = anchor.copy()
        self.tracking_baseline = np.zeros(2)
        yaw = heading(data['base_quat'])
        c, s = math.cos(yaw), math.sin(yaw)
        self.rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
        base = data.get('base_pos')
        self.base = None if base is None else np.asarray(base, dtype=float).copy()
        fk = data.get('fk_info') or {}
        # Freeze shoulder centers in the initial heading; never follow a lean.
        self.shoulders = []
        for side in ('left', 'right'):
            item = fk.get(f'{side}_shoulder_pitch_link')
            self.shoulders.append(None if item is None else
                np.asarray(item['pos'], dtype=float).copy())
        self.feet = {name: np.asarray(fk[name]['pos']).copy()
                     for name in ('left_ankle_link', 'right_ankle_link') if name in fk}
        self.scale = self.ceiling = 1.
        self.bad_tracking_seconds = self.recovery_time = self.feedback_elapsed = 0.
        self._active = False

    def _project_hand(self, point, index):
        cfg = self.cfg
        center = self.shoulders[index]
        center = self.anchor[index] if center is None else center
        delta = self.rotation.T @ (point-center)
        radii = np.array([cfg.forward_m if delta[0] >= 0 else cfg.backward_m,
                          cfg.lateral_m, cfg.upward_m if delta[2] >= 0 else cfg.downward_m])
        # Independent shoulder-relative axes let a hand rise AND move forward.
        # A neutral-hand-centered ellipsoid incorrectly suppresses that punch.
        normalized = np.abs(delta)/radii
        limited = bool(np.any(normalized > cfg.soft_boundary))
        outside = normalized > cfg.soft_boundary
        normalized[outside] = cfg.soft_boundary+(1-cfg.soft_boundary)*np.tanh(
            (normalized[outside]-cfg.soft_boundary)/(1-cfg.soft_boundary))
        point = center+self.rotation @ (np.sign(delta)*normalized*radii)
        local = self.rotation.T @ (point-self.anchor[2])
        side = 1. if index == 0 else -1.
        if side*local[1] < cfg.hand_separation_m/2:
            local[1] = side*cfg.hand_separation_m/2
            point = self.anchor[2]+self.rotation @ local
            limited = True
        # Radial projection about the shoulder preserves a raised arm. Scaling
        # the whole ray from a low resting hand would suppress its elevation.
        if self.shoulders[index] is not None:
            radius = max(cfg.shoulder_reach_m, float(np.linalg.norm(self.anchor[index]-center)))
            offset = point-center
            length = float(np.linalg.norm(offset))
            if length > radius:
                point = center+offset*(radius/length)
                limited = True
        return point, limited

    @staticmethod
    def _risk(value, soft, hard):
        return float(np.clip((hard-value)/(hard-soft), 0., 1.))

    def _feedback(self, robot, data, dt):
        cfg = self.cfg
        quat = np.asarray(data['base_quat'], dtype=float)
        tilt = math.acos(float(np.clip(1-2*(quat[0]**2+quat[1]**2), -1, 1)))
        limits = [('body tilt', self._risk(tilt, cfg.tilt_soft_rad, cfg.tilt_hard_rad))]
        if self.base is not None and data.get('base_pos') is not None:
            drift = np.linalg.norm(np.asarray(data['base_pos'])[:2]-self.base[:2])
            limits.append(('base drift', self._risk(drift, cfg.drift_soft_m, cfg.drift_hard_m)))
        fk = data.get('fk_info') or {}
        for name, initial in self.feet.items():
            if name not in fk:
                continue
            displacement = np.asarray(fk[name]['pos'])-initial
            limits.append(('foot displacement', self._risk(np.linalg.norm(displacement[:2]),
                                                          cfg.drift_soft_m, cfg.drift_hard_m)))
            limits.append(('foot lift', self._risk(displacement[2], cfg.foot_lift_soft_m,
                                                   cfg.foot_lift_hard_m)))
        residual = np.linalg.norm(self.target[:2]-robot[:2], axis=1)
        error = float(np.maximum(residual-self.tracking_baseline, 0.).max())
        self.bad_tracking_seconds = self.bad_tracking_seconds+dt if error > cfg.tracking_soft_m else 0.
        if self.bad_tracking_seconds >= cfg.tracking_delay_seconds:
            limits.append(('tracking error', self._risk(error, cfg.tracking_soft_m, cfg.tracking_hard_m)))
        return min(limits, key=lambda item: item[1])

    def apply(self, raw, robot, anchor, data, dt, active=True):
        raw, robot, anchor = self.points(raw), self.points(robot), self.points(anchor)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Stationary reference dt must be positive and finite')
        if self.anchor is None or not np.array_equal(anchor, self.anchor):
            self._anchor(anchor, robot, data)
        # Remove whole-body translation of the input; only head-relative arm
        # motion reaches the fixed world-space workspace.
        desired = raw.copy()
        desired[:2] += self.anchor[2]-raw[2]
        desired[2] = self.anchor[2]
        projected = self.anchor.copy()
        reasons = []
        for index, side in enumerate(('left', 'right')):
            projected[index], limited = self._project_hand(desired[index], index)
            if limited:
                reasons.append(side+' workspace')
        if np.linalg.norm(raw[2]-self.anchor[2]) > .02:
            reasons.append('head fixed')
        if active:
            if self.feedback_elapsed < self.cfg.feedback_warmup_seconds:
                # Capture support and actual shoulder centers during initial
                # settling only. Head/neutral references stay fixed throughout.
                base = data.get('base_pos')
                self.base = None if base is None else np.asarray(base, dtype=float).copy()
                fk = data.get('fk_info') or {}
                self.feet = {name: np.asarray(fk[name]['pos']).copy() for name in self.feet if name in fk}
                for i, side in enumerate(('left', 'right')):
                    item = fk.get(f'{side}_shoulder_pitch_link')
                    if item is not None:
                        self.shoulders[i] = np.asarray(item['pos'], dtype=float).copy()
                self.tracking_baseline = np.minimum(np.linalg.norm(self.target[:2]-robot[:2], axis=1),
                                                    self.cfg.tracking_baseline_max_m)
                self.feedback_elapsed += dt
                risk_reason, risk_scale = 'initial support settling', 1.
            else:
                risk_reason, risk_scale = self._feedback(robot, data, dt)
            if risk_scale < 1:
                self.ceiling = min(self.ceiling, risk_scale)
                self.recovery_time = 0.
                reasons.append(risk_reason)
            # Require the operator/reference to return near neutral before
            # releasing a retreat. Otherwise an unreachable held pose is retried.
            near_neutral = np.linalg.norm(desired[:2]-self.anchor[:2], axis=1).max() < .05
            if risk_scale >= 1 and near_neutral:
                self.recovery_time += dt
                if self.recovery_time >= self.cfg.recovery_seconds:
                    self.ceiling = 1.
            else:
                self.recovery_time = 0.
            tau = .15 if self.ceiling < self.scale else self.cfg.recovery_seconds
            self.scale += (1-math.exp(-dt/tau))*(self.ceiling-self.scale)
            if self.ceiling < .999:
                reasons.append('retreat latched; return hands near neutral')
        projected[:2] = self.anchor[:2]+self.scale*(projected[:2]-self.anchor[:2])
        previous = self.target.copy()
        if active:
            delta = (1-math.exp(-dt/self.cfg.smoothing_seconds))*(projected-previous)
            length = np.linalg.norm(delta, axis=1, keepdims=True)
            delta *= np.minimum(1., self.cfg.max_speed_m_s*dt/np.maximum(length, 1e-12))
            self.target = previous+delta
            self.target[2] = self.anchor[2]
            result = self.target.copy()
            velocity = (result-previous)/dt
        else:
            result, velocity = projected, np.zeros((3, 3))
        self._active = active
        status = dict(limited=bool(reasons), reasons=reasons, arm_scale=float(self.scale),
                      retreat_latched=self.ceiling < .999)
        reason = ', '.join(reasons) or 'inside workspace'
        now = time.monotonic()
        if reason != self._last_reason and now-self._last_report >= 1:
            logger.info('Stationary reference: %s (arm scale %.2f)', reason, self.scale)
            self._last_reason, self._last_report = reason, now
        return result.copy(), velocity, status
