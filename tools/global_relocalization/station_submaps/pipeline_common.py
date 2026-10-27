"""Shared IO/provenance for the independent, sensor-only station experiment."""
from pathlib import Path
import hashlib
import json
import re

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def new_directory(path):
    path = Path(path).resolve()
    path.mkdir(parents=True, exist_ok=False)
    return path


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


class Inputs:
    def __init__(self):
        self.hashes = {}

    def add(self, path, expected=None):
        path = Path(path).resolve()
        if any(x in path.name for x in ('post_evaluation', 'relative_scores')):
            raise ValueError('post-evaluation input is forbidden: ' + str(path))
        digest = sha256(path)
        if expected is not None and digest != expected:
            raise ValueError('input SHA mismatch: ' + str(path))
        if str(path) in self.hashes and self.hashes[str(path)] != digest:
            raise ValueError('input changed: ' + str(path))
        self.hashes[str(path)] = digest
        return path

    def inherit(self, hashes):
        for path, digest in hashes.items():
            self.add(path, digest)

    def artifact(self, path, stage_directory=None):
        """Verify the consumed artifact against its completed upstream stage.

        The upstream manifest retains complete transitive source hashes; stages
        need not reread multi-GB bags they do not consume.
        """
        path = Path(path).resolve()
        stage = Path(stage_directory).resolve() if stage_directory else path.parent
        provenance_path = self.add(stage / 'provenance.json')
        provenance = read_json(provenance_path)
        if str(path) not in provenance.get('artifact_sha256', {}):
            raise ValueError('artifact not listed in completed upstream stage: ' + str(path))
        return self.add(path, provenance['artifact_sha256'][str(path)])

    def verify(self):
        for path, digest in self.hashes.items():
            if sha256(path) != digest:
                raise ValueError('input changed during stage: ' + path)

    def save(self, output, **extra):
        self.verify()
        artifacts = {str(path.resolve()): sha256(path) for path in sorted(Path(output).rglob('*'))
                     if path.is_file() and path.name != 'provenance.json'}
        write_json(Path(output) / 'provenance.json', {
            'schema_version': 1, 'sensor_only': True, 'GT_or_query_read': False,
            'source_sha256': self.hashes, 'artifact_sha256': artifacts, **extra})


def safe_id(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', value):
        raise ValueError('station ID is not a safe path component')
    return value


def rigid(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], rtol=0, atol=1e-10)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), rtol=0, atol=1e-6)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1., rtol=0, atol=1e-6)):
        raise ValueError('invalid SE3')
    return t


def ns_stamp(row):
    return int(row['stamp_ns']) if 'stamp_ns' in row else round(row['stamp'] * 1e9)


def record_code(inputs):
    for path in sorted(HERE.glob('pipeline*.py')):
        inputs.add(path)
    for path in [HERE/'gyro_turns.py', HERE/'supported_tree.py', HERE/'export_station_prior.py',
                 HERE.parent/'refine_sensor_prior.py', HERE.parent/'export_dense_sensor_prior.py']:
        inputs.add(path)
