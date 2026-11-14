"""Hash the deployed policy, geometry, inherited runtime Python and entry tools."""
import hashlib
import json
import shutil
from pathlib import Path


def source_manifest(root):
    root = Path(root)
    paths = [root / name for name in ('run_scene2.py', 'run.sh', 'bootstrap.py')]
    for directory, pattern in (('scene2', '*.py'), ('tools', '*.py'), ('tests', '*.py'),
                               ('assets', '*.json'), ('vendor/scene1/scene1', '*.py')):
        paths.extend((root / directory).rglob(pattern))
    paths.append(root / 'vendor/scene1/config.yaml')
    paths.extend(p for p in (root/'tests/fixtures').rglob('*') if p.suffix in ('.json','.npz','.urdf'))
    files = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted(paths)}
    digest = hashlib.sha256(json.dumps(files, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return dict(source_sha256=digest, files=files)


def verify_source_tree(root,expected,prune_stale=False):
    """Match the deployed source set, including removals, before any motion."""
    root=Path(root)
    actual=source_manifest(root)
    if prune_stale:
        # Only files already classified as policy/test sources may be pruned.
        # Run recordings, RGB-D evidence and other container data are excluded.
        for name in actual['files'].keys()-expected['files'].keys():
            (root/name).unlink()
        for directory in ('scene2','tools','tests','vendor/scene1/scene1'):
            for cache in (root/directory).rglob('__pycache__'):
                if cache.is_dir():shutil.rmtree(cache)
        actual=source_manifest(root)
    if actual!=expected:
        extra=sorted(actual['files'].keys()-expected['files'].keys())
        missing=sorted(expected['files'].keys()-actual['files'].keys())
        changed=sorted(name for name in actual['files'].keys()&expected['files'].keys()
                       if actual['files'][name]!=expected['files'][name])
        raise RuntimeError(f'Deployed source mismatch: extra={extra}, missing={missing}, changed={changed}')
    return actual
