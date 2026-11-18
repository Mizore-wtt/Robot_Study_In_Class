#!/usr/bin/env python3
"""Unpack official Humble runtime packages into this workspace (no sudo)."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[4])
    args = parser.parse_args()
    root = args.workspace.resolve() / 'install' / 'cartographer_runtime'
    packages = ['ros-humble-cartographer', 'ros-humble-cartographer-ros',
                'ros-humble-cartographer-ros-msgs', 'liblua5.2-0']
    root.mkdir(parents=True, exist_ok=True)
    versions = {}
    with tempfile.TemporaryDirectory(prefix='scout_cartographer_') as temp:
        subprocess.run(['apt-get', 'download', *packages], cwd=temp, check=True)
        for deb in Path(temp).glob('*.deb'):
            info = subprocess.check_output(['dpkg-deb', '-f', str(deb), 'Package', 'Version'], text=True)
            versions[deb.name] = info.strip()
            subprocess.run(['dpkg-deb', '-x', str(deb), str(root)], check=True)
    (root / 'packages.json').write_text(json.dumps(versions, indent=2) + '\n')
    print(f'Runtime installed in {root}; rebuild scout_gazebo and source install/local_setup.bash.')


if __name__ == '__main__':
    main()
