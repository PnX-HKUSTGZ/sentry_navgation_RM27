#!/usr/bin/env python3
"""Apply reviewed patches to an isolated HEAD snapshot, preserving the submodule."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile


def prepare(root, revision):
    source = root / 'src/point_lio'
    destination = root / 'tools/global_relocalization/.build/mapping_source'
    manifest_path = destination.parent / 'mapping_source_manifest.json'
    previous = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    previous_hashes = previous.get('source_sha256', {}) if previous.get('revision') == revision else {}
    patches = sorted((root / 'tools/global_relocalization/pointlio_patches').glob('*.patch'))
    archive = subprocess.check_output(['git', '-C', str(source), 'archive', revision])
    with tempfile.TemporaryDirectory(prefix='pointlio-reviewed-') as temporary:
        pristine = Path(temporary)
        with tarfile.open(fileobj=io.BytesIO(archive)) as tree:
            tree.extractall(pristine, filter='data')
        originals = {str(p.relative_to(pristine)): p.read_bytes()
                     for p in pristine.rglob('*') if p.is_file()}
        for patch in patches:
            subprocess.run(['patch', '--batch', '-p1', '-i', str(patch.resolve())],
                           cwd=pristine, check=True)
        prepared = {str(p.relative_to(pristine)): p.read_bytes()
                    for p in pristine.rglob('*') if p.is_file()}
        # Refuse to discard local isolated-source edits. Both an original snapshot
        # and a previously prepared snapshot are accepted for repeatable builds.
        for name, data in prepared.items():
            target = destination / name
            if target.exists() and target.read_bytes() not in (originals.get(name), data):
                if name.endswith(('.cpp', '.h', '.hpp')) or name in ('CMakeLists.txt', 'package.xml'):
                    if hashlib.sha256(target.read_bytes()).hexdigest() != previous_hashes.get(name):
                        raise RuntimeError('Unrecognized local mapping-source edit: ' + name)
        for name, data in prepared.items():
            target = destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.exists() or target.read_bytes() != data:
                target.write_bytes(data)
        hashes = {name: hashlib.sha256(data).hexdigest() for name, data in prepared.items()
                  if name.endswith(('.cpp', '.h', '.hpp')) or name in ('CMakeLists.txt', 'package.xml')}
        manifest = {'revision': revision, 'source_sha256': hashes,
                    'patch_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in patches},
                    'modified_source_files': [name for name in hashes if prepared[name] != originals.get(name)]}
        (destination.parent / 'mapping_source_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--revision', required=True)
    args = parser.parse_args()
    prepare(args.root.resolve(), args.revision)
