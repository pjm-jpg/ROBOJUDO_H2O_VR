#!/usr/bin/env python3
"""Commit main-project files and saved dependency changes from the project root."""
import argparse
import subprocess

from sync_submodule_patches import ROOT, git, snapshot_submodules, stage_saved_patches


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-m', '--message', help='Git commit message')
    parser.add_argument('--stage-only', action='store_true', help='Prepare all changes for review without committing')
    args = parser.parse_args()
    if not args.message and not args.stage_only:
        parser.error('-m/--message is required unless --stage-only is used')
    try:
        snapshot_submodules()
        git(ROOT, 'add', '--all')
        stage_saved_patches()
        if args.stage_only:
            subprocess.run(['git', 'diff', '--cached', '--stat'], cwd=ROOT, check=True)
        else:
            subprocess.run(['git', 'commit', '-m', args.message], cwd=ROOT, check=True)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f'Unified commit failed: {exc}\n')


if __name__ == '__main__':
    main()
