"""Standalone Quest H1 simulation configuration; never selects a real environment."""
import yaml
from pydantic import Field

from robojudo.config import THIRD_PARTY_DIR, cfg_registry
from robojudo.controller.ctrl_cfgs import KeyboardCtrlCfg, QuestH2HCtrlCfg
from robojudo.pipeline.pipeline_cfgs import RlPipelineCfg
from .env.h1_mujuco_env_cfg import H1MujocoEnvCfg
from .policy.h1_h2h_policy_cfg import H1H2HPolicyCfg


class QuestH2HPipelineCfg(RlPipelineCfg):
    pipeline_type: str = 'QuestH2HPipeline'
    preview_only: bool = False
    blend_seconds: float = Field(default=1.0, gt=0, allow_inf_nan=False)


@cfg_registry.register
class h1_h2h_quest(QuestH2HPipelineCfg):
    robot: str = 'h1'
    env: H1MujocoEnvCfg = H1MujocoEnvCfg(random_heading=False)
    policy: H1H2HPolicyCfg = H1H2HPolicyCfg(policy_name='h2h_release')
    ctrl: list = [
        KeyboardCtrlCfg(triggers={
            'c': '[QUEST_CALIBRATE]',
            'e': '[QUEST_ENABLE]',
            'Key.space': '[QUEST_PAUSE]',
            'r': '[SIM_REBORN]',
            'Key.esc': '[SHUTDOWN]',
        }),
        QuestH2HCtrlCfg(extend_config=yaml.safe_load(
            (THIRD_PARTY_DIR / 'phc/phc/data/cfg/robot/unitree_h1.yaml').read_text()
        )['extend_config']),
    ]
