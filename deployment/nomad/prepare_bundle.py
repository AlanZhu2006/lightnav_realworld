"""Build a bounded, hash-recorded NoMaD bring-up bundle from local sources.

No network, system installation or hardware operations are performed. Only
the listed upstream files, local deployment code and supplied checkpoint are
included. Existing destination directories are never overwritten.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile


NOMAD_COMMIT = 'dca79815b704e5aa9c6bdc3082351f9e3b2848c2'
DIFFUSION_COMMIT = '5ba07ac6661db573af695b419a7947ecb704690f'
GEM_COMMIT = 'fd23b5d4006226e7a76b2c3f0bf03a8ec16e3436'
CHECKPOINT_SHA = '70f79b8262527e20e56ced64a3e3d7ef91855bc9e7c3fa348d78edcb83c6a333'


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def check_revision(path: Path, expected: str) -> None:
    revision = subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != expected:
        raise ValueError(f'{path}: expected {expected}, found {revision}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', type=Path, required=True)
    parser.add_argument('--diffusion-policy', type=Path, required=True)
    parser.add_argument('--gem', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--all-deployment', action='store_true', help='include ImageGoal, sensors, operator, Go2, LightNav and bringup source; exclude private env/assets')
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[2]
    output = args.output.expanduser().resolve()
    archive = output.with_suffix('.tar.gz')
    if output.exists() or archive.exists():
        parser.error('output directory/archive already exists')
    for path, revision in [(args.upstream, NOMAD_COMMIT),
                           (args.diffusion_policy, DIFFUSION_COMMIT), (args.gem, GEM_COMMIT)]:
        check_revision(path, revision)
    if sha256(args.checkpoint) != CHECKPOINT_SHA:
        raise ValueError('checkpoint differs from the reviewed official asset')
    output.mkdir(parents=True)
    records = []

    def copy_file(source: Path, relative: str, origin: str):
        destination = output / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        records.append({'path': relative, 'sha256': sha256(destination),
                        'bytes': destination.stat().st_size, 'origin': origin})

    source_root = root / ('deployment' if args.all_deployment else 'deployment/nomad')
    for path in sorted(source_root.rglob('*')):
        relative = path.relative_to(source_root)
        private = any(part.startswith('.') or part == '__pycache__' or part == 'node_modules' for part in relative.parts)
        if path.is_file() and not private and path.suffix not in {'.pyc', '.pth', '.safetensors'}:
            copy_file(path, path.relative_to(root).as_posix(), 'DejaView deployment worktree')
    nomad_files = [
        'LICENSE', 'train/config/nomad.yaml', 'train/vint_train/data/data_config.yaml',
        'train/vint_train/models/nomad/nomad.py',
        'train/vint_train/models/nomad/nomad_vint.py',
        'train/vint_train/models/vint/self_attention.py',
        'train/vint_train/training/train_utils.py',
        'train/vint_train/visualizing/visualize_utils.py',
        'deployment/src/utils.py', 'deployment/src/pd_controller.py', 'deployment/src/navigate.py',
    ]
    for relative in nomad_files:
        copy_file(args.upstream / relative, f'third_party/visualnav_transformer/{relative}',
                  f'robodhruv/visualnav-transformer@{NOMAD_COMMIT}')
    for relative in ['LICENSE',
                     'diffusion_policy/model/diffusion/conditional_unet1d.py',
                     'diffusion_policy/model/diffusion/conv1d_components.py',
                     'diffusion_policy/model/diffusion/positional_embedding.py']:
        copy_file(args.diffusion_policy / relative, f'third_party/diffusion_policy/{relative}',
                  f'real-stanford/diffusion_policy@{DIFFUSION_COMMIT}')
    for name in ['go2_cmd_bridge.py', 'go2_battery_monitor.py',
                 'trajectory_control.py', 'latency_motion_guard.py']:
        copy_file(args.gem / 'deployment/go2' / name, f'third_party/gem_runtime/{name}',
                  f'AlanZhu2006/GEM-RealWorld@{GEM_COMMIT}; staged only, never started')
    copy_file(args.checkpoint, 'assets/nomad/nomad.pth', 'reviewed official NoMaD checkpoint')
    manifest = {'schema_version': 'nomad.bringup_bundle.v1', 'files': records,
                'scope': ('complete deployment source; default observer/shadow profile; physical execution requires explicit hardware preflight'
                          if args.all_deployment else 'model inference and shadow tooling; no robot launch entry point'),
                'robot_workspace': '/home/unitree/work/lightnav_realworld'}
    (output / 'SOURCE_MANIFEST.json').write_text(json.dumps(manifest, indent=2) + '\n')
    with tarfile.open(archive, 'w:gz', compresslevel=1) as stream:
        for path in sorted(output.rglob('*')):
            if path.is_file():
                stream.add(path, arcname=path.relative_to(output).as_posix(), recursive=False)
    print(json.dumps({'directory': str(output), 'archive': str(archive),
                      'archive_sha256': sha256(archive), 'files': len(records)}, indent=2))


if __name__ == '__main__':
    main()
