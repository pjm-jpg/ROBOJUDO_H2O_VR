#!/usr/bin/env python3
"""Initialize dependencies and apply saved patches without discarding local work."""
import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
CONFIG_FILE = ROOT / 'submodule_cfg.yaml'


def run(command, cwd=ROOT):
    print(f'Running: {command}', flush=True)
    return subprocess.run(command, cwd=cwd, check=True)


def apply_patch(path, patch_path):
    patch_path = Path(patch_path).resolve()
    if not patch_path.is_file():
        raise FileNotFoundError(f'Missing dependency patch: {patch_path}')
    if not patch_path.read_bytes():
        return
    # Full snapshots include additions, deletions and binary files. Reverse
    # checking makes repeated installs safe without resetting the source tree.
    reverse = subprocess.run(['git', 'apply', '--reverse', '--check', str(patch_path)],
                             cwd=path, capture_output=True)
    if reverse.returncode == 0:
        print(f'Already applied: {patch_path.name}')
        return
    forward = subprocess.run(['git', 'apply', '--check', str(patch_path)],
                             cwd=path, capture_output=True, text=True)
    if forward.returncode:
        raise RuntimeError(f'Patch conflicts with local changes; source preserved: {path}\n'
                           f'{forward.stderr}\nRun scripts/sync_submodule_patches.py before reinstalling, '
                           'or resolve the conflicting source changes.')
    run(['git', 'apply', str(patch_path)], cwd=path)


def load_config():
    with CONFIG_FILE.open() as stream:
        return yaml.safe_load(stream)


def install_submodules(selected=None, source_only=False):
    config = load_config()
    unknown = set(selected or ()) - config.keys()
    if unknown:
        raise ValueError(f'Unknown modules: {sorted(unknown)}')
    for name, info in config.items():
        if (selected is None and not info.get('install', False)) or (selected is not None and name not in selected):
            continue
        print(f'\n----- Installing submodule: {name} -----')
        path = ROOT / info['path']
        if not (path / '.git').exists():
            if path.exists() and any(path.iterdir()):
                raise RuntimeError(f'Uninitialized dependency contains files; source preserved: {path}')
            run(['git', 'submodule', 'update', '--init', '--', info['path']])
        for patch in info.get('patches', []):
            apply_patch(path, path / patch)
        for addon in info.get('addons', []):
            addon_path = path / addon
            if not addon_path.is_dir():
                raise FileNotFoundError(f'Missing addon: {addon_path}')
            shutil.copytree(addon_path, path, dirs_exist_ok=True)
        if not source_only:
            packages = [path, *(ROOT / p for p in info.get('extra_packages', []))]
            for package in packages:
                run([sys.executable, '-m', 'pip', 'install', '-e', str(package)])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('modules', nargs='*', help='Override modules selected in submodule_cfg.yaml')
    parser.add_argument('--source-only', action='store_true', help='Apply saved sources without pip installation')
    args = parser.parse_args()
    try:
        install_submodules(args.modules or None, source_only=args.source_only)
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f'Dependency installation failed: {exc}\n')
