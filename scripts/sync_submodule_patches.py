#!/usr/bin/env python3
"""Keep local dependency changes in patches owned by the main repository."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = Path('third_party/patches/local_sources.json')


def git(repo, *args, env=None):
    return subprocess.check_output(['git', *args], cwd=repo, env=env)


def modules(root):
    return json.loads((root / MANIFEST).read_text())['modules']


def snapshot_submodules(root=ROOT, output_root=None, stage=False):
    """Use private indexes: include local commits and untracked, nonignored files."""
    root = Path(root)
    output_root = root if output_root is None else Path(output_root)
    entries = modules(root)
    pending = {}
    for name, info in entries.items():
        source = root / info['path']
        if not (source / '.git').exists():
            if source.exists() and any(source.iterdir()):
                raise RuntimeError(f'{name}: source exists but is not an initialized submodule')
            print(f'{name}: not initialized; retaining the saved patch')
            continue
        base = info['base']
        # Hooks export repository-local Git variables. Do not let those point
        # dependency commands at the main repository's index or object store.
        source_env = dict(os.environ)
        for variable in git(root, 'rev-parse', '--local-env-vars').decode().splitlines():
            source_env.pop(variable, None)
        git(source, 'cat-file', '-e', base + '^{commit}', env=source_env)
        # Never stage or discard anything in the dependency's real index.
        with tempfile.TemporaryDirectory(prefix='robojudo-index-') as directory:
            env = dict(source_env, GIT_INDEX_FILE=str(Path(directory) / 'index'))
            objects = Path(os.fsdecode(git(source, 'rev-parse', '--git-path', 'objects', env=source_env)).strip())
            objects = objects if objects.is_absolute() else source / objects
            temporary_objects = Path(directory) / 'objects'
            temporary_objects.mkdir()
            env['GIT_OBJECT_DIRECTORY'] = str(temporary_objects)
            env['GIT_ALTERNATE_OBJECT_DIRECTORIES'] = str(objects.resolve())
            git(source, 'read-tree', base, env=env)
            git(source, 'add', '--all', '--', '.', env=env)
            patch = git(source, 'diff', '--cached', '--binary', '--full-index',
                        '--no-ext-diff', '--no-textconv', base, '--', env=env)
            pending[info['patch']] = patch
    # Compute every patch first, so a failed source does not leave partial output.
    for relative, content in pending.items():
        target = output_root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists() or target.read_bytes() != content:
            fd, temporary = tempfile.mkstemp(prefix='.patch-', dir=target.parent)
            try:
                with os.fdopen(fd, 'wb') as stream:
                    stream.write(content)
                os.chmod(temporary, 0o644)
                os.replace(temporary, target)
            finally:
                if os.path.exists(temporary):
                    os.unlink(temporary)
        print(f'{relative}: {len(content)} bytes')
    if stage:
        if output_root != root:
            raise ValueError('Cannot stage patches written to a different root')
        stage_saved_patches(root)
    return pending


def stage_saved_patches(root=ROOT):
    root = Path(root)
    entries = modules(root)
    # Keep public upstream pins, even when git add -A staged a local commit.
    for info in entries.values():
        git(root, 'update-index', '--add', '--cacheinfo',
            f"160000,{info['base']},{info['path']}")
    saved = [info['patch'] for info in entries.values()
             if (root / info['patch']).exists()]
    if saved:
        git(root, 'add', '--', *saved)


def install_hook(root=ROOT):
    root = Path(root)
    current = subprocess.run(['git', 'config', '--get', 'core.hooksPath'],
                             cwd=root, capture_output=True, text=True)
    if current.returncode not in (0, 1):
        raise RuntimeError(current.stderr.strip())
    if current.returncode == 0 and current.stdout.strip() != '.githooks':
        raise RuntimeError('An existing core.hooksPath is configured; integrate the sync command there first')
    if current.returncode == 1:
        hook = Path(os.fsdecode(git(root, 'rev-parse', '--git-path', 'hooks/pre-commit')).strip())
        hook = hook if hook.is_absolute() else root / hook
        if hook.exists():
            raise RuntimeError(f'Existing hook preserved: {hook}; integrate the sync command there first')
    git(root, 'config', '--local', 'core.hooksPath', '.githooks')
    print('Enabled .githooks/pre-commit for this clone')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', action='store_true', help='Stage saved patches and upstream pins')
    parser.add_argument('--install-hook', action='store_true', help='Enable automatic sync on normal git commit')
    args = parser.parse_args()
    try:
        if args.install_hook:
            install_hook()
        else:
            snapshot_submodules(stage=args.stage)
    except (RuntimeError, subprocess.CalledProcessError, OSError, ValueError) as exc:
        parser.exit(1, f'Submodule patch sync failed: {exc}\n')


if __name__ == '__main__':
    main()
