"""Run the local Unitree H1 DDS endpoint in MuJoCo; never binds a hardware NIC."""
import argparse
import json
from pathlib import Path

from robojudo.environment.h1_dds_bridge import H1DDSBridge


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=42, help='DDS domain 1..100, default 42')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error('--duration must be positive')
    bridge = H1DDSBridge(domain=args.domain, headless=args.headless)
    print(f'H1 DDS MuJoCo ready: lo, domain={args.domain}; rt/lowcmd -> physics -> rt/lowstate', flush=True)
    print('Starts paused; run run_quest_h2h.py --dds-sim in a second terminal.', flush=True)
    try:
        bridge.run(args.duration)
    except KeyboardInterrupt:
        pass
    finally:
        report = json.dumps(bridge.status(), indent=2)
        print(report, flush=True)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(report + '\n')
        bridge.close()


if __name__ == '__main__':
    main()
