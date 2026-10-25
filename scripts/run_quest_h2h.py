"""Run Quest three-point teleoperation in H1 MuJoCo using h2h_release.pt."""
import argparse
import logging
import time

from robojudo.config.config_manager import ConfigManager
from robojudo.pipeline.quest_h2h_pipeline import QuestH2HPipeline


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    connection = parser.add_mutually_exclusive_group()
    connection.add_argument('--ip', help='Quest Wi-Fi address; omit for USB')
    connection.add_argument('--serial', help='USB ADB device serial')
    parser.add_argument('--port', type=int, default=5555)
    parser.add_argument('--adb', help='Path to the same modern adb used to authorize the headset')
    parser.add_argument('--reader-root', help='Path to oculus_reader-main')
    parser.add_argument('--record', help='Record original received frames to a JSONL file')
    parser.add_argument('--preview-only', action='store_true', help='Show targets; E cannot enable physics')
    parser.add_argument('--headless', action='store_true', help='Disable the MuJoCo viewer')
    parser.add_argument('--dds-sim', action='store_true', help='Use real Unitree DDS transport to the local MuJoCo bridge')
    parser.add_argument('--dds-domain', type=int, default=42, help='Local simulation DDS domain (1..100)')
    parser.add_argument('--no-keyboard', action='store_true', help='Disable keyboard listener (passive monitoring)')
    parser.add_argument('--require-tracked', action='store_true', help='Pause on inferred controller positions')
    parser.add_argument('--head-scale', type=float, default=1.0)
    parser.add_argument('--hand-scale', type=float, default=1.0)
    parser.add_argument('--steps', type=int, help='Exit after this many loop iterations')
    args = parser.parse_args()
    if args.steps is not None and args.steps <= 0:
        parser.error('--steps must be positive')
    return args


def main():
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s', force=True)
    cfg = ConfigManager('h1_h2h_quest').get_cfg()
    cfg.device = 'cpu'
    cfg.preview_only = args.preview_only
    if args.dds_sim:
        from robojudo.config.h1.env.h1_dds_sim_env_cfg import H1DDSSimEnvCfg
        from robojudo.config.h1.env.h1_real_env_cfg import H1UnitreeCfg
        cfg.env = H1DDSSimEnvCfg(unitree=H1UnitreeCfg(net_if='lo', domain_id=args.dds_domain))
    else:
        cfg.env.headless = args.headless
    if args.no_keyboard:
        cfg.ctrl = [ctrl for ctrl in cfg.ctrl if ctrl.ctrl_type != 'KeyboardCtrl']
    quest = next(ctrl for ctrl in cfg.ctrl if ctrl.ctrl_type == 'QuestH2HCtrl')
    overrides = dict(ip_address=args.ip, serial=args.serial, port=args.port,
                     require_tracked=args.require_tracked,
                     head_scale=args.head_scale, hand_scale=args.hand_scale)
    for name in ('adb', 'reader_root', 'record'):
        if getattr(args, name) is not None:
            overrides[name] = getattr(args, name)
    # Re-validate CLI values rather than bypassing Pydantic with assignment.
    cfg.ctrl = [type(ctrl).model_validate({**ctrl.model_dump(), **overrides})
                if ctrl is quest else ctrl for ctrl in cfg.ctrl]
    pipeline = None
    try:
        pipeline = QuestH2HPipeline(cfg)
        while pipeline.running and (args.steps is None or pipeline.timestep < args.steps):
            start = time.monotonic()
            pipeline.step()
            remaining = pipeline.dt - (time.monotonic() - start)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        if pipeline is not None:
            pipeline.close()


if __name__ == '__main__':
    main()
