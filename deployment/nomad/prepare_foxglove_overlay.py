"""Unpack pinned ROS Foxglove packages into the user's work directory.

Uses the configured signed APT package metadata, verifies each SHA256, and
extracts files without dpkg installation or maintainer scripts. The system ROS
installation and package database are unchanged. Intended for Jammy/Humble.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess


PACKAGES = {
    'ros-humble-foxglove-bridge': '3.6.0-1jammy.20260929.215014',
    'ros-humble-rosx-introspection': '2.3.0-1jammy.20260910.002055',
    'rapidjson-dev': '1.1.0+dfsg2-7',
}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True, help='new user-owned overlay directory')
    args = parser.parse_args(argv)
    root = args.root.expanduser().resolve()
    if root.exists():
        parser.error('overlay directory already exists; choose a new versioned directory')
    setup = Path('/opt/ros/humble/setup.bash')
    if not setup.is_file():
        parser.error('the host ROS 2 Humble installation is required')
    root.mkdir(parents=True)
    cache, unpack = root / 'debs', root / 'unpacked'
    cache.mkdir()
    unpack.mkdir()
    records = []
    for name, version in PACKAGES.items():
        spec = f'{name}={version}'
        text = subprocess.check_output(['apt-cache', 'show', spec], text=True)
        metadata = dict(line.split(': ', 1) for line in text.split('\n\n')[0].splitlines()
                        if ': ' in line and not line.startswith(' '))
        if metadata.get('Package') != name or metadata.get('Version') != version:
            raise RuntimeError(f'APT did not return the pinned package {spec}')
        expected = metadata['SHA256']
        filename = Path(metadata['Filename']).name
        subprocess.run(['apt-get', 'download', spec], cwd=cache, check=True)
        package = cache / filename
        actual = hashlib.sha256(package.read_bytes()).hexdigest()
        if actual != expected:
            raise RuntimeError(f'package hash mismatch: {package}')
        subprocess.run(['dpkg-deb', '-x', str(package), str(unpack)], check=True)
        records.append({'package': name, 'version': version, 'sha256': actual,
                        'filename': filename})
    prefix = unpack / 'opt/ros/humble'
    env = root / 'setup.bash'
    quoted = shlex.quote(str(prefix))
    env.write_text(
        '# User-space Foxglove overlay; no system installation.\n'
        'source /opt/ros/humble/setup.bash\n'
        f'export AMENT_PREFIX_PATH={quoted}:"${{AMENT_PREFIX_PATH:-}}"\n'
        f'export LD_LIBRARY_PATH={quoted}/lib:"${{LD_LIBRARY_PATH:-}}"\n'
        f'export PATH={quoted}/bin:"$PATH"\n')
    manifest = {'packages': records, 'prefix': str(prefix), 'setup': str(env),
                'system_install_performed': False, 'services_started': False}
    (root / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
