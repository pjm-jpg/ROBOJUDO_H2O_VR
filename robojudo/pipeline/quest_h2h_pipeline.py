"""Simulation-only, explicitly enabled Quest teleoperation with paused preview."""
import logging
import numpy as np
from box import Box

import robojudo.environment
from robojudo.controller import CtrlManager
from robojudo.pipeline import Pipeline, pipeline_registry
from robojudo.pipeline.rl_pipeline import PolicyWrapper

logger = logging.getLogger(__name__)


@pipeline_registry.register
class QuestH2HPipeline(Pipeline):
    def __init__(self, cfg):
        if not cfg.env.is_sim or cfg.env.env_type != 'MujocoEnv':
            raise ValueError('QuestH2HPipeline currently supports MuJoCo simulation only')
        super().__init__(cfg)
        self.ctrl_manager = None
        self.env = None
        self.running = True
        self._was_enabled = False
        self._blend_step = 0
        try:
            env_class = getattr(robojudo.environment, cfg.env.env_type)
            self.env = env_class(cfg_env=cfg.env, device=self.device)
            self.policy = PolicyWrapper(cfg.policy, self.env.dof_cfg, self.device)
            self.env.update_dof_cfg(override_cfg=self.policy.cfg_action_dof)
            self.freq, self.dt = cfg.policy.freq, 1.0 / cfg.policy.freq
            for control in cfg.ctrl:
                if control.ctrl_type == 'QuestH2HCtrl':
                    control.frequency = self.freq
            # Start from the existing H1 XML home pose, not a motion-file frame.
            self.env.reborn()
            self.env.reset()
            self.env.update()
            self.ctrl_manager = CtrlManager(cfg.ctrl, env=self.env, device=self.device)
            self.quest = self.ctrl_manager.controllers['QuestH2HCtrl'].inst
            self.self_check()
            logger.info('Quest preview: physics paused. C=calibrate, E=enable, SPACE=pause, R=reset, ESC=exit')
            if cfg.preview_only:
                logger.info('PREVIEW ONLY: robot stays frozen; C calibrates green target balls; E is disabled')
        except Exception:
            self.close()
            raise

    def self_check(self):
        self.env.update()
        data = self.env.get_data()
        control = self.ctrl_manager.get_ctrl_data(data)
        points = control['QuestH2HCtrl']['robot_body_pos_subset']
        control['QuestH2HCtrl']['ref_body_pos_subset'] = points.copy()
        control['QuestH2HCtrl']['ref_body_vel_subset'] = np.zeros_like(points)
        self.policy.reset_temporal_state(self._policy_env_data(data))
        obs, _ = self.policy.get_observation(data, control)
        action = self.policy.get_pd_target(obs)
        if obs.shape != (1665,) or action.shape != (19,) or not np.isfinite(action).all():
            raise ValueError(f'H1 H2H contract mismatch: observation={obs.shape}, action={action.shape}')
        self.policy.reset_temporal_state(self._policy_env_data(data))
        logger.info('Verified h2h_release: 1665 observation values -> 19 joint targets')

    def _policy_env_data(self, data):
        result = data.copy()
        result.dof_pos = self.policy.obs_adapter.fit(result.dof_pos)
        result.dof_vel = self.policy.obs_adapter.fit(result.dof_vel)
        return result

    def reset(self):
        self.env.reborn()
        self.env.update()
        self.quest.reset()
        self._was_enabled = False
        self.policy.reset_temporal_state(self._policy_env_data(self.env.get_data()))

    def prepare(self, *args, **kwargs):
        # Preview remains frozen until explicitly enabled by the operator.
        return

    def _commands(self, commands):
        # CtrlManager combines commands in a set. Explicit priorities ensure a
        # simultaneous enable can never override pause, reset or shutdown.
        if '[SHUTDOWN]' in commands:
            self.running = False
            self.quest.command('[QUEST_PAUSE]')
            return
        if '[SIM_REBORN]' in commands:
            self.reset()
            return
        if '[QUEST_PAUSE]' in commands:
            self.quest.command('[QUEST_PAUSE]')
            return
        if '[QUEST_CALIBRATE]' in commands:
            self.quest.command('[QUEST_CALIBRATE]')
            return
        for command in commands:
            if command == '[QUEST_ENABLE]' and self.cfg.preview_only:
                logger.info('Preview-only mode: enabling is disabled')
            else:
                self.quest.command(command)

    def _draw(self, data, references):
        if self.env.visualizer is not None:
            self.env.visualizer.update_rg_view(
                references['robot_body_pos_subset'], np.tile([0, 0, 0, 1], (3, 1)), 0)
            self.env.visualizer.update_rg_view(
                references['ref_body_pos_subset'], np.tile([0, 0, 0, 1], (3, 1)), 1)
            # Existing viewer markers persist by ID.
        viewer = self.env.viewer
        if viewer is not None and not self.quest.enabled:
            viewer.cam.lookat = self.env.data.qpos[:3]
            if viewer.is_alive:
                viewer.render()
            else:
                self.running = False

    def step(self, dry_run=False):
        self.env.update()
        data = self.env.get_data()
        control = self.ctrl_manager.get_ctrl_data(data)
        commands = control.get('COMMANDS', [])
        self._commands(commands)
        if commands:
            # Apply pause/enable before any action in the current step.
            self.env.update()
            data = self.env.get_data()
            control['QuestH2HCtrl'] = Box(self.quest.get_data())
        if not self.running:
            return
        references = control['QuestH2HCtrl']
        active = references['enabled'] and not self.cfg.preview_only
        if active and not self._was_enabled:
            self.policy.reset_temporal_state(self._policy_env_data(data))
            self._blend_from = data.dof_pos.copy()
            self._blend_step = 0
        self._draw(data, references)
        if active:
            obs, extras = self.policy.get_observation(data, control)
            target = self.policy.get_pd_target(obs)
            if not np.isfinite(target).all():
                self.quest.invalidate('non-finite policy output; recalibrate', recalibrate=True)
                active = False
            elif not dry_run:
                steps = max(int(self.cfg.blend_seconds * self.freq), 1)
                alpha = min((self._blend_step + 1) / steps, 1.0)
                target = (1 - alpha) * self._blend_from + alpha * target
                self.env.step(target, extras.get('hand_pose'))
                self._blend_step += 1
                if self.env.viewer is not None and not self.env.viewer.is_alive:
                    self.running = False
        self._was_enabled = active
        self.timestep += 1

    def close(self):
        try:
            if self.ctrl_manager is not None:
                for controller in self.ctrl_manager.controllers.values():
                    if hasattr(controller.inst, 'close'):
                        controller.inst.close()
        finally:
            if self.env is not None:
                self.env.shutdown()
