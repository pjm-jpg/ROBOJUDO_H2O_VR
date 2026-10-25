"""H1 deployment launcher. Default is subscriber-only; --actuate enables staged control."""
import argparse
import logging
import math
import socket
import time

from robojudo.environment.utils.h1_state_check import inspect_h1_state


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='enp3s0', help='H1-facing Ethernet interface; no robot IP required')
    parser.add_argument('--actuate', action='store_true', help='Allow commands AFTER P; use only with robot supported')
    parser.add_argument('--duration', type=float, default=10, help='Subscriber-only check duration, seconds')
    parser.add_argument('--dds-sim', action='store_true', help='Rehearse the same staged pipeline on local DDS MuJoCo')
    parser.add_argument('--dds-domain', type=int, default=42)
    connection = parser.add_mutually_exclusive_group()
    connection.add_argument('--ip', help='Quest Wi-Fi IP, not the robot IP')
    connection.add_argument('--serial', help='Quest USB serial')
    parser.add_argument('--adb')
    parser.add_argument('--reader-root')
    parser.add_argument('--record')
    parser.add_argument('--no-keyboard', action='store_true', help='Passive WAIT only; cannot initiate motion')
    parser.add_argument('--steps', type=int)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be finite and positive')
    if args.steps is not None and args.steps <= 0:
        parser.error('--steps must be positive')
    if not 1 <= args.dds_domain <= 100:
        parser.error('--dds-domain must be 1..100')
    if not args.dds_sim:
        if args.interface == 'lo':
            parser.error('Hardware mode cannot use lo; use --dds-sim for simulation')
        try:
            socket.if_nametoindex(args.interface)
        except OSError:
            parser.error(f'Network interface does not exist: {args.interface}')
    return args


def make_cfg(args):
    from robojudo.config.h1.h1_quest_real_cfg import QuestH2HDeployCfg
    from robojudo.config.h1.env.h1_real_env_cfg import H1RealEnvCfg, H1UnitreeCfg
    from robojudo.controller.ctrl_cfgs import KeyboardCtrlCfg

    cfg = QuestH2HDeployCfg(device='cpu')
    unitree = H1UnitreeCfg(net_if=args.interface, control_dt=.0025, state_timeout=.1,
                         command_timeout=.15, connect_timeout=10)
    if args.dds_sim:
        from robojudo.config.h1.env.h1_dds_sim_env_cfg import H1DDSSimEnvCfg
        unitree.net_if, unitree.domain_id = 'lo', args.dds_domain
        cfg.env = H1DDSSimEnvCfg(unitree=unitree)
    else:
        cfg.env = H1RealEnvCfg(env_type='H1QuestHardwareEnv', born_place_align=False, unitree=unitree)
    quest = next(ctrl for ctrl in cfg.ctrl if ctrl.ctrl_type == 'QuestH2HCtrl')
    updates = dict(ip_address=args.ip, serial=args.serial, require_tracked=True)
    for key in ('adb', 'reader_root', 'record'):
        if getattr(args, key) is not None:
            updates[key] = getattr(args, key)
    quest = type(quest).model_validate({**quest.model_dump(), **updates})
    cfg.ctrl = [quest]
    if not args.no_keyboard:
        cfg.ctrl.insert(0, KeyboardCtrlCfg(triggers={
            'p': '[H1_PREPARE]', 'l': '[H1_STAND]', 'c': '[QUEST_CALIBRATE]',
            'e': '[QUEST_ENABLE]', 'Key.space': '[QUEST_PAUSE]',
            'x': '[SHUTDOWN]', 'Key.esc': '[SHUTDOWN]',
        }))
    return cfg


def main():
    args = parse_args()
    if not args.actuate:
        if args.dds_sim:
            from robojudo.environment.utils.dds_sim import initialize_sim_dds
            initialize_sim_dds(args.dds_domain)
        print('READ ONLY: no command publisher, no motor commands, no motion service changes', flush=True)
        report = inspect_h1_state('lo' if args.dds_sim else args.interface, args.duration,
                                  args.dds_domain if args.dds_sim else 0)
        return 0 if report['passed'] else 2
    from robojudo.pipeline.quest_h2h_deploy_pipeline import QuestH2HDeployPipeline
    logging.basicConfig(level=logging.INFO, force=True)
    cfg = make_cfg(args)
    print('STAGED CONTROL: starts in WAIT. Hardware sends nothing until P. '
          'Support robot before P; X/ESC/Select/exit causes damping, not standing.', flush=True)
    pipeline = None
    try:
        pipeline = QuestH2HDeployPipeline(cfg)
        while pipeline.running and (args.steps is None or pipeline.timestep < args.steps):
            start = time.monotonic()
            pipeline.step()
            time.sleep(max(0, pipeline.dt - (time.monotonic()-start)))
    except KeyboardInterrupt:
        pass
    finally:
        if pipeline is not None:
            pipeline.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
