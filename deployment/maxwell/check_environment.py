#!/usr/bin/env python3
"""Download locked official wheels or verify the migrated environment on CPU only."""
from __future__ import annotations
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import urllib.request


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def read_lock(path):
    packages = []
    for line in Path(path).read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        match = re.fullmatch(r'([A-Za-z0-9_.-]+)==([A-Za-z0-9_.+!-]+)', line)
        if match is None:
            raise ValueError(f'Only exact version pins are accepted: {line}')
        packages.append(match.groups())
    return packages


def download_wheels(lock, destination, output):
    from packaging.tags import sys_tags
    from packaging.utils import parse_wheel_filename
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    priorities = {tag: index for index, tag in enumerate(sys_tags())}

    def download(item):
        name, version = item
        with urllib.request.urlopen(f'https://pypi.org/pypi/{name}/{version}/json', timeout=45) as response:
            release = json.load(response)
        choices = []
        for asset in release['urls']:
            if not asset['filename'].endswith('.whl') or asset.get('yanked'):
                continue
            _, _, _, tags = parse_wheel_filename(asset['filename'])
            rank = min((priorities[tag] for tag in tags if tag in priorities), default=None)
            if rank is not None:
                choices.append((rank, asset['filename'], asset))
        if not choices:
            raise RuntimeError(f'No compatible official wheel for {name}=={version}')
        asset = min(choices)[2]
        path = destination / asset['filename']
        expected = asset['digests']['sha256']
        if not (path.is_file() and path.stat().st_size == asset['size'] and sha256(path) == expected):
            partial = path.with_suffix(path.suffix + '.part')
            for attempt in range(5):
                try:
                    offset = partial.stat().st_size if partial.exists() else 0
                    if offset >= asset['size']:
                        if offset == asset['size'] and sha256(partial) == expected:
                            partial.replace(path)
                            break
                        partial.unlink()
                        offset = 0
                    headers = {'Range': f'bytes={offset}-'} if offset else {}
                    request = urllib.request.Request(asset['url'], headers=headers)
                    with urllib.request.urlopen(request, timeout=60) as response:
                        resumed = bool(offset and response.status == 206)
                        if resumed and not response.headers.get('Content-Range', '').startswith(f'bytes {offset}-'):
                            raise RuntimeError('Unexpected resume Content-Range')
                        with partial.open('ab' if resumed else 'wb') as stream:
                            shutil.copyfileobj(response, stream, length=1 << 20)
                    if partial.stat().st_size != asset['size'] or sha256(partial) != expected:
                        raise RuntimeError(f'Official PyPI wheel verification failed: {asset["filename"]}')
                    partial.replace(path)
                    break
                except Exception:
                    if attempt == 4:
                        raise
                    time.sleep(min(2 ** attempt, 8))
        result = {'name': name, 'version': version, 'filename': path.name,
                  'url': asset['url'], 'bytes': path.stat().st_size,
                  'sha256': expected, 'verified': True}
        print(f'VERIFIED {name}=={version} {result["bytes"]} bytes', flush=True)
        return result

    records = []
    with ThreadPoolExecutor(max_workers=4) as executor:
        futures = [executor.submit(download, item) for item in read_lock(lock)]
        for future in as_completed(futures):
            records.append(future.result())
    report = {'schema': 'dejaview.official_wheel_manifest.v1',
              'source': 'https://pypi.org', 'python': sys.version,
              'requirements_sha256': sha256(lock),
              'files': sorted(records, key=lambda item: item['name'].lower())}
    Path(output).write_text(json.dumps(report, indent=2) + '\n')
    hashed_lock = Path(output).with_suffix('.requirements.txt')
    hashed_lock.write_text(''.join(f"{item['name']}=={item['version']} --hash=sha256:{item['sha256']}\n" for item in report['files']))
    return report


def check_environment(lock, wheel_manifest, receipt):
    if os.environ.get('CUDA_VISIBLE_DEVICES') != '':
        raise RuntimeError('Migration verification requires CUDA_VISIBLE_DEVICES to be explicitly empty')
    base = Path(os.environ['DEJAVIEW_DATA_ROOT']).resolve()
    if not str(base).startswith('/local_data/'):
        raise RuntimeError('Environment must be stored under /local_data')
    if sys.version_info[:2] != (3, 11):
        raise RuntimeError('Python 3.11 is required')
    expected = read_lock(lock)
    versions = {}
    for name, version in expected:
        actual = importlib.metadata.version(name)
        if actual != version:
            raise RuntimeError(f'{name}: expected {version}, got {actual}')
        versions[name] = actual
    imports = {}
    for name in ['torch', 'torchvision', 'transformers', 'tokenizers', 'safetensors',
                 'numpy', 'PIL', 'yaml', 'websockets', 'einops', 'accelerate']:
        module = importlib.import_module(name)
        imports[name] = str(Path(module.__file__).resolve())
    # Explicitly CPU: no CUDA methods, CUDA context, model load or GPU probes.
    import torch
    cpu_result = (torch.tensor([1.0, 2.0], device='cpu') * 2).tolist()
    if cpu_result != [2.0, 4.0]:
        raise RuntimeError('CPU tensor calculation failed')
    required_paths = ['XDG_CACHE_HOME', 'XDG_DATA_HOME', 'XDG_STATE_HOME',
                      'XDG_CONFIG_HOME', 'UV_CACHE_DIR', 'UV_PYTHON_INSTALL_DIR',
                      'UV_PYTHON_BIN_DIR', 'UV_TOOL_DIR', 'UV_TOOL_BIN_DIR',
                      'PIP_CACHE_DIR', 'PYTHONUSERBASE', 'HF_HOME', 'HF_HUB_CACHE',
                      'HUGGINGFACE_HUB_CACHE', 'HF_DATASETS_CACHE', 'HF_ASSETS_CACHE', 'HF_XET_CACHE', 'TORCH_HOME',
                      'TORCHINDUCTOR_CACHE_DIR', 'TRITON_CACHE_DIR', 'CUDA_CACHE_PATH',
                      'NUMBA_CACHE_DIR', 'MPLCONFIGDIR', 'CONDA_PKGS_DIRS',
                      'CONDA_ENVS_PATH', 'MAMBA_ROOT_PREFIX', 'TMPDIR', 'TMP', 'TEMP']
    paths = {}
    for key in required_paths:
        path = Path(os.environ[key]).resolve()
        if not path.is_relative_to(base):
            raise RuntimeError(f'{key} escapes /local_data user storage: {path}')
        path.mkdir(parents=True, exist_ok=True)
        paths[key] = str(path)
    for key in ['PIP_CONFIG_FILE', 'CONDARC']:
        path = Path(os.environ[key]).resolve()
        if not path.is_relative_to(base):
            raise RuntimeError(f'{key} escapes user data root: {path}')
        path.parent.mkdir(parents=True, exist_ok=True)
        paths[key] = str(path)
    if os.environ.get('MAXWELL_DATA_ROOT') != str(base):
        raise RuntimeError('MAXWELL_DATA_ROOT must match DEJAVIEW_DATA_ROOT')
    for path in [Path(sys.executable).resolve(), Path(sys.prefix).resolve()]:
        if not path.is_relative_to(base):
            raise RuntimeError(f'Python environment escapes data root: {path}')
    home = Path.home()
    if str(home).startswith('/local_data/'):
        raise RuntimeError('HOME must remain the actual login home')
    home_cache = home / '.cache'
    if not home_cache.is_symlink() or home_cache.resolve() != base / 'cache':
        raise RuntimeError('Home .cache must link to the verified data cache')
    uv = base / 'tools/bin/uv'
    package_check = subprocess.run([str(uv), 'pip', 'check', '--python', sys.executable],
                                   text=True, capture_output=True, check=True)
    manifest = json.loads(Path(wheel_manifest).read_text())
    if manifest['requirements_sha256'] != sha256(lock) or len(manifest['files']) != len(expected):
        raise RuntimeError('Wheel manifest does not match locked requirements')
    wheel_dir = base / 'wheels/lightnav'
    for item in manifest['files']:
        path = wheel_dir / item['filename']
        if path.stat().st_size != item['bytes'] or sha256(path) != item['sha256']:
            raise RuntimeError(f'Wheel manifest hash mismatch: {path.name}')
    usage = shutil.disk_usage(base)
    home_kib = int(subprocess.check_output(['du', '-sk', str(home)], text=True).split()[0])
    if home_kib >= 10 * 1024 * 1024:
        raise RuntimeError('Home usage reached the 10 GiB login quota')
    tree_kib = {}
    for relative in ['envs/lightnav', 'python', 'cache', 'wheels/lightnav', 'tools', 'tmp']:
        path = base / relative
        tree_kib[relative] = int(subprocess.check_output(['du', '-sk', str(path)], text=True).split()[0])
    report = {'schema': 'dejaview.maxwell.environment_receipt.v1', 'ok': True,
              'validated_at_unix_ns': time.time_ns(), 'hostname': os.uname().nodename,
              'data_root': str(base), 'home': str(home), 'home_usage_kib': home_kib,
              'home_cache_symlink': str(home_cache), 'home_cache_target': str(home_cache.resolve()),
              'python': sys.version, 'python_executable': sys.executable,
              'python_real_executable': str(Path(sys.executable).resolve()),
              'environment_prefix': sys.prefix, 'packages': versions, 'imports': imports,
              'cache_and_install_paths': paths,
              'gpu_policy': {'cuda_visible_devices': '', 'cuda_api_called': False,
                             'model_loaded': False, 'inference_performed': False,
                             'gpu_reservation': None, 'validation': 'CPU imports and CPU tensor only'},
              'pip_check': (package_check.stdout + package_check.stderr).strip(),
              'uv_version': subprocess.check_output([str(uv), '--version'], text=True).strip(),
              'requirements_sha256': sha256(lock), 'wheel_manifest_sha256': sha256(wheel_manifest),
              'wheel_count': len(manifest['files']), 'wheel_bytes': sum(item['bytes'] for item in manifest['files']),
              'filesystem': {'total_bytes': usage.total, 'free_bytes': usage.free},
              'directory_usage_kib_apparent_with_hardlink_sharing': tree_kib}
    Path(receipt).parent.mkdir(parents=True, exist_ok=True)
    Path(receipt).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'ok': True, 'receipt': str(receipt), 'packages': len(versions),
                      'home_usage_kib': home_kib, 'gpu_used': False}, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--requirements', type=Path, required=True)
    parser.add_argument('--download-wheels', type=Path)
    parser.add_argument('--wheel-manifest', type=Path, required=True)
    parser.add_argument('--receipt', type=Path)
    args = parser.parse_args()
    if args.download_wheels:
        download_wheels(args.requirements, args.download_wheels, args.wheel_manifest)
    else:
        if args.receipt is None:
            parser.error('--receipt is required for CPU verification')
        check_environment(args.requirements, args.wheel_manifest, args.receipt)

if __name__ == '__main__':
    main()
