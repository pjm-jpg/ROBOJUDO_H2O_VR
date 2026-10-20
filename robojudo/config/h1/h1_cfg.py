from robojudo.config import cfg_registry
from robojudo.controller.ctrl_cfgs import (
    JoystickCtrlCfg,  # noqa: F401
    KeyboardCtrlCfg,  # noqa: F401
    UnitreeCtrlCfg,  # noqa: F401
)
from robojudo.pipeline.pipeline_cfgs import (
    RlMultiPolicyPipelineCfg,  # noqa: F401
    RlPipelineCfg,  # noqa: F401
)

from .ctrl.h1_motion_ctrl_cfg import H1MotionH2HCtrlCfg  # noqa: F401
from .env.h1_dummy_env_cfg import H1DummyEnvCfg  # noqa: F401
from .env.h1_mujuco_env_cfg import H1MujocoEnvCfg  # noqa: F401
from .env.h1_real_env_cfg import H1RealEnvCfg, H1UnitreeCfg  # noqa: F401
from .policy.h1_h2h_policy_cfg import H1H2HPolicyCfg  # noqa: F401
from .policy.h1_smooth_policy_cfg import H1SmoothPolicyCfg  # noqa: F401
from .policy.h1_unitree_policy_cfg import H1UnitreePolicyCfg  # noqa: F401


# ======================== Basic Configs ======================== #
@cfg_registry.register
class h1(RlPipelineCfg):
    """
    Unitree H1 robot configuration, Unitree Policy, Sim2Sim.
    You can modify to play with other policies and controllers.
    """

    robot: str = "h1"
    env: H1MujocoEnvCfg = H1MujocoEnvCfg()

    ctrl: list[JoystickCtrlCfg | KeyboardCtrlCfg] = [
        JoystickCtrlCfg(),
        KeyboardCtrlCfg(),
    ]

    policy: H1UnitreePolicyCfg = H1UnitreePolicyCfg()
    # policy: H1SmoothPolicyCfg = H1SmoothPolicyCfg()


@cfg_registry.register
class h1_real(h1):
    """
    Unitree H1 robot, Unitree Policy, Sim2Real.
    To extend the sim2sim config to sim2real, just need to change the env to real env.
    """

    # env: H1DummyEnvCfg = H1DummyEnvCfg()
    env: H1RealEnvCfg = H1RealEnvCfg(
        env_type="UnitreeEnv",
        unitree=H1UnitreeCfg(
            net_if="eno2",  # note: change to your network interface
        ),
        update_with_fk=True,
    )

    ctrl: list[UnitreeCtrlCfg] = [
        UnitreeCtrlCfg(),
    ]

    do_safety_check: bool = True  # enable safety check for real robot


@cfg_registry.register
class h1_switch(RlMultiPolicyPipelineCfg):
    """
    Example of multi-policy pipeline configuration.
    """

    robot: str = "h1"
    env: H1MujocoEnvCfg = H1MujocoEnvCfg()

    ctrl: list[KeyboardCtrlCfg | JoystickCtrlCfg] = [
        KeyboardCtrlCfg(
            triggers_extra={
                "Key.tab": "[POLICY_TOGGLE]",
            }
        ),
        JoystickCtrlCfg(
            triggers_extra={
                "L1+Y": "[POLICY_TOGGLE]",
            }
        ),
    ]

    policies: list[H1UnitreePolicyCfg | H1SmoothPolicyCfg] = [
        H1UnitreePolicyCfg(),
        H1SmoothPolicyCfg(),
    ]


# ======================== Configs for supported Policy ======================== #


@cfg_registry.register
#继承自RlPipelineCfg类，定义了一个名为h1_h2h的配置类，用于配置H1机器人在H2H任务中的RL管道。
class h1_h2h(RlPipelineCfg):
    robot: str = "h1"
    #指定Mujoco环境的配置类为H1MujocoEnvCfg，用于模拟H1机器人的物理环境。
    env: H1MujocoEnvCfg = H1MujocoEnvCfg()
    #指定控制器的配置类为H1MotionH2HCtrlCfg，用于控制H1机器人在H2H任务中的动作。
    ctrl: list[KeyboardCtrlCfg | H1MotionH2HCtrlCfg] = [
        KeyboardCtrlCfg(),
        H1MotionH2HCtrlCfg(),
    ]
    #指定策略的配置类为H1H2HPolicyCfg，用于定义H1机器人在H2H任务中的强化学习策略。
    policy: H1H2HPolicyCfg = H1H2HPolicyCfg()
