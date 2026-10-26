"""Reproduce a root-only commit and restore dependencies using local Git fixtures."""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sync = load('patch_sync_test', ROOT / 'scripts/sync_submodule_patches.py')
installer = load('patch_install_test', ROOT / 'submodule_install.py')


class SubmodulePatchTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='robojudo-patch-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / 'main'
        self.root.mkdir()
        self.initialize(self.root)
        self.source = self.root / 'deps/demo'
        self.source.mkdir(parents=True)
        self.initialize(self.source)
        (self.source / 'source.py').write_text('original\n')
        (self.source / 'removed.txt').write_text('remove me\n')
        (self.source / '.gitignore').write_text('ignored.txt\n')
        self.git(self.source, 'add', '--all')
        self.git(self.source, 'commit', '-qm', 'Upstream')
        self.base = self.git(self.source, 'rev-parse', 'HEAD').strip()
        self.entry = dict(path='deps/demo', base=self.base, patch='third_party/patches/demo.patch')
        (self.root / '.gitmodules').write_text(
            '[submodule "deps/demo"]\n\tpath = deps/demo\n\turl = ./demo\n\tignore = all\n')
        (self.root / sync.MANIFEST).parent.mkdir(parents=True)
        (self.root / sync.MANIFEST).write_text(json.dumps(dict(modules=dict(demo=self.entry))))
        (self.root / self.entry['patch']).write_bytes(b'')
        for relative in ('scripts/sync_submodule_patches.py', 'scripts/commit_all.py', '.githooks/pre-commit'):
            target = self.root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / relative, target)
        self.git(self.root, 'add', '--all')
        self.git(self.root, 'commit', '-qm', 'Main project')

    def git(self, root, *args):
        return subprocess.check_output(['git', *args], cwd=root, stderr=subprocess.PIPE, text=True)

    def initialize(self, root):
        self.git(root, 'init', '-q')
        self.git(root, 'config', 'user.name', 'Local Test')
        self.git(root, 'config', 'user.email', 'test@example.invalid')
        self.git(root, 'config', 'commit.gpgsign', 'false')
        self.git(root, 'config', 'core.hooksPath', '.no-hooks')

    def changes(self):
        (self.source / 'source.py').write_text('local commit\n')
        self.git(self.source, 'add', '--all')
        self.git(self.source, 'commit', '-qm', 'Not pushed to upstream')
        (self.source / 'source.py').write_text('uncommitted final state\n')
        (self.source / 'removed.txt').unlink()
        (self.source / 'new file.bin').write_bytes(bytes(range(256)))
        (self.source / 'ignored.txt').write_text('not part of source\n')

    def test_snapshot_restores_local_commits_untracked_binary_and_deletions(self):
        self.changes()
        index = (self.source / '.git/index').read_bytes()
        before = self.git(self.source, 'status', '--porcelain')
        sync.snapshot_submodules(self.root)
        first = (self.root / self.entry['patch']).read_bytes()
        sync.snapshot_submodules(self.root)
        self.assertEqual(first, (self.root / self.entry['patch']).read_bytes())
        self.assertEqual(index, (self.source / '.git/index').read_bytes())
        self.assertEqual(before, self.git(self.source, 'status', '--porcelain'))
        restored = Path(self.temporary.name) / 'restored'
        self.git(self.root, 'clone', '-q', '--no-hardlinks', str(self.source), str(restored))
        self.git(restored, 'checkout', '-q', self.base)
        installer.apply_patch(restored, self.root / self.entry['patch'])
        installer.apply_patch(restored, self.root / self.entry['patch'])
        self.assertEqual((restored / 'source.py').read_bytes(), (self.source / 'source.py').read_bytes())
        self.assertEqual((restored / 'new file.bin').read_bytes(), bytes(range(256)))
        self.assertFalse((restored / 'removed.txt').exists())
        self.assertFalse((restored / 'ignored.txt').exists())

    def test_normal_root_commit_saves_patches_and_keeps_public_pin(self):
        self.changes()
        self.git(self.root, 'config', '--unset', 'core.hooksPath')
        sync.install_hook(self.root)
        (self.root / 'README.md').write_text('A staged main-project change\n')
        self.git(self.root, 'add', '--all')
        self.git(self.root, 'commit', '-qm', 'Everything from project root')
        self.assertIn(self.base, self.git(self.root, 'ls-tree', 'HEAD', 'deps/demo'))
        saved = self.git(self.root, 'show', 'HEAD:third_party/patches/demo.patch')
        self.assertIn('uncommitted final state', saved)
        self.assertIn('GIT binary patch', saved)
        self.assertIn('new file.bin', saved)

    def test_unified_commit_captures_only_dependency_working_tree_changes(self):
        self.git(self.root, 'config', '--unset', 'core.hooksPath')
        sync.install_hook(self.root)
        (self.source / 'source.py').write_text('only a working-tree change\n')
        # An empty main index is a Git pre-commit edge case. The root helper
        # generates patches before Git checks whether there is anything to commit.
        subprocess.run(['python3', 'scripts/commit_all.py', '-m', 'Root-only save'],
                       cwd=self.root, capture_output=True, check=True)
        self.assertIn('only a working-tree change', self.git(self.root, 'show',
                      'HEAD:third_party/patches/demo.patch'))

    def test_install_conflict_preserves_local_files(self):
        self.changes()
        sync.snapshot_submodules(self.root)
        (self.source / 'source.py').write_text('additional work to preserve\n')
        before = self.git(self.source, 'status', '--porcelain')
        with self.assertRaisesRegex(RuntimeError, 'source preserved'):
            installer.apply_patch(self.source, self.root / self.entry['patch'])
        self.assertEqual((self.source / 'source.py').read_text(), 'additional work to preserve\n')
        self.assertEqual(before, self.git(self.source, 'status', '--porcelain'))

    def test_hook_install_preserves_existing_hook(self):
        self.git(self.root, 'config', '--unset', 'core.hooksPath')
        existing = self.root / '.git/hooks/pre-commit'
        existing.write_text('# existing user hook\n')
        with self.assertRaisesRegex(RuntimeError, 'Existing hook preserved'):
            sync.install_hook(self.root)
        self.assertEqual(existing.read_text(), '# existing user hook\n')


if __name__ == '__main__':
    unittest.main()
