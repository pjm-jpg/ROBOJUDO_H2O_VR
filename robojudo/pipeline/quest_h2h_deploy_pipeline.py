"""Staged H1 deployment: WAIT -> PREPARE -> READY -> STAND -> VR.

Unlike the simulation preview pipeline, loss of VR never freezes physics:
the policy keeps running with a smooth return to the fixed standing task.
"""
import logging
import time

import numpy as np
from box import Box

from robojudo.controller import CtrlManager
from robojudo.environment.h1_quest_hardware_env import H1QuestHardwareEnv
from robojudo.environment.unitree_dds_sim_env import UnitreeDDSSimEnv
from robojudo.pipeline import Pipeline
from robojudo.pipeline.quest_h2h_pipeline import QuestH2HPipeline
from robojudo.pipeline.rl_pipeline import PolicyWrapper

logger = logging.getLogger(__name__)


class QuestH2HDeployPipeline(Pipeline):
    _policy_env_data = QuestH2HPipeline._policy_env_data
    self_check = QuestH2HPipeline.self_check

    def __init__(self, cfg):
        self.hardware = cfg.env.env_type == 'H1QuestHardwareEnv' and not cfg.env.is_sim
        if not self.hardware and not (cfg.env.env_type == 'UnitreeDDSSimEnv' and cfg.env.is_sim):
            raise ValueError('Use the gated H1 hardware environment or local DDS simulator')
        super().__init__(cfg)
        self.env = self.ctrl_manager = None
        self.running, self.phase = True, 'WAIT'
        self._closed = False
        self._policy_steps = 0
        self._last_loop = None
        self._return_from = None
        try:
            cls = H1QuestHardwareEnv if self.hardware else UnitreeDDSSimEnv
            self.env = cls(cfg.env, self.device)
            self.policy = PolicyWrapper(cfg.policy, self.env.dof_cfg, self.device)
            self.env.update_dof_cfg(override_cfg=self.policy.cfg_action_dof)
            self.freq, self.dt = cfg.policy.freq, 1 / cfg.policy.freq
            self.kp, self.kd = self.env.stiffness.copy(), self.env.damping.copy()
            # Use named XML joint ranges; an old H1DoF table swaps the two
            # shoulder-roll limits. The actual robot limits still need checking.
            self.limits = np.asarray([self.env.kinematics.model.joint(n).range.copy()
                                      for n in self.env.joint_names])
            for control in cfg.ctrl:
                if control.ctrl_type == 'QuestH2HCtrl':
                    control.frequency = self.freq
            self.env.update()
            self.ctrl_manager = CtrlManager(cfg.ctrl, env=self.env, device=self.device)
            self.quest = self.ctrl_manager.controllers['QuestH2HCtrl'].inst
            self.self_check()
            logger.warning('WAIT: P=prepare under support; L=fixed-target policy; C=calibrate; E=VR; '
                           'SPACE=return to fixed target; X/ESC/remote Select=damping and exit')
        except Exception:
            self.close()
            raise

    def reset(self):
        raise RuntimeError('No physical reset/reborn; restart the supported preparation procedure')

    def prepare(self):
        return

    def _return_to_stand(self, reason):
        self.quest.command('[QUEST_PAUSE]')
        if self.phase in ('STAND', 'VR'):
            self._return_from = self._last_reference.copy()
            self._return_step = 0
            self.phase = 'STAND'
            logger.warning('VR disengaged: %s; policy continues, returning to fixed targets', reason)

    def _commands(self, commands, data):
        if '[SHUTDOWN]' in commands:
            self.running = False
            return
        if '[QUEST_PAUSE]' in commands:
            if self.phase in ('PREPARE', 'READY'):
                self._hold_target = data.dof_pos.copy()
                self.phase = 'HOLD'
                logger.warning('HOLD: preparation interrupted; keep robot supported; P to prepare again')
            else:
                self._return_to_stand('operator SPACE')
            return
        if '[H1_PREPARE]' in commands:
            if self.phase not in ('WAIT', 'HOLD'):
                logger.warning('P ignored: preparation is only allowed from WAIT/HOLD')
                return
            if self.hardware and not self.env.armed:
                self.env.arm()  # No publication before the operator presses P.
            self._prepare_from = data.dof_pos.copy()
            self._last_target = data.dof_pos.copy()
            self._prepare_started = time.monotonic()
            self._last_loop = self._prepare_started
            self.phase = 'PREPARE'
            logger.warning('PREPARE: interpolating to initial joint posture; keep robot supported')
            return
        if '[H1_STAND]' in commands:
            if self.phase == 'READY':
                self._stand_reference = self.quest.robot_points().copy()
                self._last_reference = self._stand_reference.copy()
                self._blend_from = data.dof_pos.copy()
                self._policy_steps = 0
                # WAIT may have observed an unprepared robot. Anchor the
                # stationary task only to the posture actually reached by P.
                self.policy.reset_reference_guard()
                self.policy.reset_temporal_state(self._policy_env_data(data))
                self.phase = 'STAND'
                logger.warning('STAND: policy enabled with fixed three-point targets; VR is not in control')
            elif self.phase in ('STAND', 'VR'):
                self._return_to_stand('operator L')
            else:
                logger.warning('L refused: complete P preparation first')
            return
        if '[QUEST_CALIBRATE]' in commands:
            if self.phase not in ('STAND', 'VR'):
                logger.warning('C refused: start fixed-target policy with L first')
                return
            self._return_to_stand('calibrating')
            self.quest.command('[QUEST_CALIBRATE]')
            return
        if '[QUEST_ENABLE]' in commands:
            if self.phase != 'STAND' or self._return_from is not None:
                logger.warning('E refused: enter STAND and finish returning to fixed targets first')
                return
            self.quest.command('[QUEST_ENABLE]')
            if self.quest.enabled:
                self.phase = 'VR'
                logger.warning('VR: operator targets enabled')

    def _send_target(self, target, limit_rate=True):
        if not np.isfinite(target).all():
            raise ValueError('Non-finite policy target; stopping')
        target = np.clip(target, self.limits[:, 0], self.limits[:, 1])
        if limit_rate:
            delta = self.cfg.max_target_speed * self.dt
            target = self._last_target + np.clip(target - self._last_target, -delta, delta)
        self.env.step(target)
        self._last_target = target.copy()

    def step(self, dry_run=False):
        if dry_run:
            raise ValueError('Deployment step does not support dry_run; use the subscriber-only launcher')
        now = time.monotonic()
        if self._last_loop is not None and self.phase != 'WAIT' and now - self._last_loop > .15:
            raise TimeoutError('Policy loop stalled for >150 ms; stop and inspect')
        self._last_loop = now
        self.env.update()
        data = self.env.get_data()
        if getattr(self.env, 'emergency', False):
            raise RuntimeError('Remote Select emergency stop is latched')
        quat = np.asarray(data.base_quat)
        if not np.isfinite(data.dof_pos).all() or not np.isfinite(quat).all():
            raise ValueError('Non-finite robot observation')
        tilt = np.arccos(np.clip(1 - 2*(quat[0]**2 + quat[1]**2), -1, 1))
        if self.phase != 'WAIT' and tilt > self.cfg.max_tilt_rad:
            raise RuntimeError(f'Robot tilt {tilt:.2f} rad exceeds deployment limit')
        if hasattr(self.env, 'consume_sim_fault') and self.env.consume_sim_fault():
            raise TimeoutError('DDS simulation command timeout; restart preparation')
        control = self.ctrl_manager.get_ctrl_data(data)
        self._commands(control.get('COMMANDS', []), data)
        if not self.running:
            return
        if self.phase == 'WAIT':
            self.timestep += 1
            return
        if self.phase == 'PREPARE':
            alpha = np.clip((now - self._prepare_started) / self.cfg.prepare_seconds, 0., 1.)
            self.env.set_gains(self.kp * alpha, self.kd * max(alpha, .1))
            self._send_target((1-alpha)*self._prepare_from + alpha*self.env.default_pos)
            if now - self._prepare_started > self.cfg.prepare_seconds + 5:
                raise TimeoutError('Initial joint posture was not reached; inspect supported robot')
            if alpha >= 1 and np.max(np.abs(data.dof_pos-self.env.default_pos)) < .15:
                self.phase = 'READY'
                logger.warning('READY: initial posture reached. Press L to start fixed-target policy')
        elif self.phase == 'READY':
            self._send_target(self.env.default_pos)
        elif self.phase == 'HOLD':
            self._send_target(self._hold_target)
        else:
            references = Box(self.quest.get_data())
            if self.phase == 'VR' and not references['enabled']:
                self._return_to_stand(self.quest.reason)
            old_reference = self._last_reference.copy()
            if self.phase == 'VR':
                key = 'raw_ref_body_pos_subset' if self.policy.reference_guard is not None else 'ref_body_pos_subset'
                reference = references.get(key, references['ref_body_pos_subset']).copy()
            elif self._return_from is not None:
                self._return_step += 1
                alpha = min(self._return_step * self.dt / self.cfg.return_seconds, 1.)
                # Smoothstep gives zero endpoint speed for the fallback task.
                alpha = alpha * alpha * (3-2*alpha)
                reference = (1-alpha)*self._return_from + alpha*self._stand_reference
                if alpha >= 1:
                    self._return_from = None
            else:
                reference = self._stand_reference.copy()
            references['ref_body_pos_subset'] = reference
            references['raw_ref_body_pos_subset'] = reference.copy()
            references['ref_body_vel_subset'] = (reference-old_reference)/self.dt
            control['QuestH2HCtrl'] = references
            obs, _ = self.policy.get_observation(data, control)
            references = control["QuestH2HCtrl"]
            reference = references["ref_body_pos_subset"].copy()
            target = self.policy.get_pd_target(obs)
            alpha = min((self._policy_steps+1)*self.dt / self.cfg.policy_start_blend_seconds, 1.)
            # Preserve the learned balance controller. Slewing its PD position
            # target is not a motor velocity limit and can prevent recovery.
            self._send_target((1-alpha)*self._blend_from + alpha*target, limit_rate=False)
            if hasattr(self.env, 'set_sim_active'):
                self.env.set_sim_active(True)
            if hasattr(self.env, 'publish_targets'):
                self.env.publish_targets(references, data.base_pos)
            self._last_reference = reference.copy()
            self._policy_steps += 1
        self.timestep += 1

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            if self.env is not None:
                self.env.shutdown()
        finally:
            if self.ctrl_manager is not None:
                for controller in self.ctrl_manager.controllers.values():
                    if hasattr(controller.inst, 'close'):
                        controller.inst.close()
