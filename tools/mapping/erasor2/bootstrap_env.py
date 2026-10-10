"""Bootstrap pinned pip in a venv without Debian ensurepip; verify PyPI SHA256."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import urllib.request

if sys.prefix == sys.base_prefix:
    raise SystemExit('Refusing pip bootstrap outside a virtual environment')
try:
    import pip
except ImportError:
    version = '25.3'
    with urllib.request.urlopen(f'https://pypi.org/pypi/pip/{version}/json', timeout=60) as response:
        metadata = json.load(response)
    wheel = next(file for file in metadata['urls'] if file['filename'].endswith('py3-none-any.whl'))
    with urllib.request.urlopen(wheel['url'], timeout=60) as response:
        payload = response.read()
    if hashlib.sha256(payload).hexdigest() != wheel['digests']['sha256']:
        raise SystemExit('pip wheel SHA256 mismatch')
    with tempfile.TemporaryDirectory(prefix='mapping-pip-') as temp:
        path = Path(temp) / wheel['filename']
        path.write_bytes(payload)
        subprocess.run([sys.executable, '-c',
                        'import runpy,sys; sys.path.insert(0,sys.argv.pop(1)); runpy.run_module("pip",run_name="__main__")',
                        str(path), 'install', '--no-deps', str(path)], check=True)
