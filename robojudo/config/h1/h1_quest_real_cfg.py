"""Staged H1 deployment; hardware binding is selected only by the real launcher."""
from pydantic import Field

from .h1_quest_cfg import h1_h2h_quest


class QuestH2HDeployCfg(h1_h2h_quest):
    prepare_seconds: float = Field(default=5., ge=.1, allow_inf_nan=False)
    return_seconds: float = Field(default=2., ge=.1, allow_inf_nan=False)
    # Position targets are the trained PD controller's commands, not physical
    # joint velocities. Do not slew-limit or attenuate the balance controller.
    policy_start_blend_seconds: float = Field(default=.02, gt=0, allow_inf_nan=False)
    max_target_speed: float = Field(default=6., gt=0, allow_inf_nan=False,
                                    description='Preparation/HOLD target slew only; not the balance policy output')
    max_tilt_rad: float = Field(default=.7, gt=0, lt=1.57, allow_inf_nan=False)
