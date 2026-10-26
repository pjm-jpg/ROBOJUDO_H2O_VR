# Fix OMP perfmance issue on ARM platform (Jetson)
import os
import platform

if platform.machine().startswith("aarch64"):
    os.environ["OMP_NUM_THREADS"] = "1"

import argparse
import logging
import time

import robojudo.pipeline
from robojudo.config.config_manager import ConfigManager
from robojudo.pipeline.pipeline_cfgs import RlPipelineCfg
from robojudo.pipeline.rl_pipeline import RlPipeline
from robojudo.policy.policy_cfgs import H2HPolicyCfg
from robojudo.policy.utils.h2h_model import policy_name, select_h2h_model

logger = logging.getLogger("robojudo")

#命令行参数解析，用于指定配置文件
def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        default="g1",
        help="Name of the config class to use",
    )
    parser.add_argument(
        "--policy-name", type=policy_name, default=None,
        help="Override the Actor model name for H2H policies (for example model_30000_jit)",
    )
    parser.add_argument("--full-body", action="store_true", help="Use original unrestricted full-body references")
    args = parser.parse_args(argv)
    return args


def make_cfg(args) -> RlPipelineCfg:
    config_manager = ConfigManager(config_name=args.config)
    cfg: RlPipelineCfg = config_manager.get_cfg()
    if args.policy_name is not None:
        if not isinstance(cfg.policy, H2HPolicyCfg):
            raise ValueError("--policy-name is only supported for H2H policies")
        cfg.policy = select_h2h_model(cfg.policy, args.policy_name)
    if isinstance(cfg.policy, H2HPolicyCfg) and getattr(args, "full_body", False):
        cfg.policy.stationary_reference.enabled = False
    return cfg


def main():
    args = parse_args()
    logger.info(f"Using config: {args.config}")
    cfg = make_cfg(args)
    if isinstance(cfg.policy, H2HPolicyCfg):
        logger.info("Using H2H Actor: %s", cfg.policy.policy_file)

    #根据配置文件中的pipeline_type字段，获取对应的pipeline类
    pipeline_type = cfg.pipeline_type
    pipeline_class: type[RlPipeline] = getattr(robojudo.pipeline, pipeline_type)
    logger.info(f"Using pipeline: {pipeline_type} -> {pipeline_class}")

    pipeline = pipeline_class(cfg=cfg)

    if not cfg.env.is_sim:
        pipeline.prepare()
    elif getattr(pipeline, "_has_default_pose_mode", False):
        pipeline._set_default_pose_mode(True)
        logger.warning("Sim mode — holding default pose, press R to start motion")

    while True:
        time_start = time.time()
        pipeline.step()#进行机器人的控制和环境交互
        time_end = time.time()
        time_diff = time_end - time_start

        # keep the pipeline running at the desired frequency
        if not cfg.run_fullspeed:
            time_diff = pipeline.dt - time_diff
            if time_diff > 0:
                time.sleep(time_diff)
            else:
                if not cfg.env.is_sim:
                    logger.error(f"Warning: frame drop -> {time_diff}")
                    if time_diff < -0.2:
                        logger.critical("Exiting due to excessive frame drop")
                        pipeline.env.shutdown()
                        time.sleep(10)
                        break


if __name__ == "__main__":
    main()
