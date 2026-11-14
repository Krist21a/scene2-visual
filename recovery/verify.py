"""Verify the preserved v48 policy, optionally the downloaded runtime files."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scene2.provenance import source_manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime', action='store_true')
    args = parser.parse_args()
    expected = json.loads((ROOT/'docs/validation/source-stability-20261114-48.json').read_text())
    if source_manifest(ROOT) != expected:
        raise SystemExit('Policy files differ from frozen v48; do not claim the old validation result.')
    print('Verified policy:', expected['source_sha256'], 'files:', len(expected['files']))
    if args.runtime:
        files = json.loads((ROOT/'recovery/runtime-files.json').read_text())
        for name, record in files.items():
            path = ROOT/name
            if not path.is_file() or path.stat().st_size != record['size']:
                raise SystemExit('Missing/wrong-sized runtime file: '+name)
            if hashlib.sha256(path.read_bytes()).hexdigest() != record['sha256']:
                raise SystemExit('Runtime hash mismatch: '+name)
        print('Verified runtime files:', len(files))


if __name__ == '__main__':
    main()
