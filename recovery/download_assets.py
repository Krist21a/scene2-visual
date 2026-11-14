"""Download pinned non-Docker release assets and verify before extraction."""
import argparse
import hashlib
import json
import subprocess
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = 'Krist21a/scene2-visual'
TAG = 'scene2-v48-20261115'


def sha256(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def extract(path):
    with tarfile.open(path, 'r:gz') as archive:
        members = archive.getmembers()
        for member in members:
            destination = (ROOT/member.name).resolve()
            if not destination.is_relative_to(ROOT) or member.name.startswith('/'):
                raise ValueError('Archive path escapes repository: '+member.name)
            if member.ischr() or member.isblk() or member.isfifo():
                raise ValueError('Unexpected special file: '+member.name)
            if member.issym() or member.islnk():
                target = ((destination.parent/member.linkname) if member.issym()
                          else (ROOT/member.linkname)).resolve()
                if not target.is_relative_to(ROOT):
                    raise ValueError('Archive link escapes repository: '+member.name)
        archive.extractall(ROOT, members=members)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime', action='store_true', help='A/C controllers and deployment SDK source')
    parser.add_argument('--wheels', action='store_true', help='CPython 3.12 Linux x86_64 offline test wheels')
    parser.add_argument('--evidence', action='store_true', help='Latest full-run video and raw evidence')
    args = parser.parse_args()
    selected = {key for key in ('runtime', 'wheels', 'evidence') if getattr(args, key)} or {'runtime', 'wheels'}
    manifest = json.loads((ROOT/'recovery/release-assets.json').read_text())
    downloads = ROOT/'downloads'
    downloads.mkdir(exist_ok=True)
    for asset in manifest['assets']:
        if asset['kind'] not in selected:
            continue
        path = downloads/asset['name']
        if not path.exists():
            subprocess.run(['gh', 'release', 'download', TAG, '--repo', REPOSITORY,
                            '--pattern', asset['name'], '--dir', str(downloads)], check=True)
        if path.stat().st_size != asset['size'] or sha256(path) != asset['sha256']:
            raise SystemExit('Asset checksum failed: '+str(path)+'; remove this download and retry.')
        extract(path)
        print('Restored:', asset['name'])
    if 'runtime' in selected:
        subprocess.run(['python3', str(ROOT/'recovery/verify.py'), '--runtime'], check=True)


if __name__ == '__main__':
    main()
