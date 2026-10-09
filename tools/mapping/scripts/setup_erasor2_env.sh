#!/usr/bin/env bash
# Explicit local environment setup. No apt, sudo, system site-packages, or global pip.
set -euo pipefail
if [[ ${1:-} == -h || ${1:-} == --help ]]; then
  echo 'Usage: setup_erasor2_env.sh (ERASOR2_ENV_DIR overrides tools/mapping/.venv)'
  echo 'Creates only a local virtual environment; downloads pinned pip and dependencies from PyPI.'
  exit 0
fi
if [[ $# -ne 0 ]]; then echo 'Usage: setup_erasor2_env.sh' >&2; exit 2; fi
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_DIR=${ERASOR2_ENV_DIR:-$ROOT/.venv}
if [[ ! -x $ENV_DIR/bin/python ]]; then
  /usr/bin/python3 -m venv --without-pip "$ENV_DIR"
fi
"$ENV_DIR/bin/python" "$ROOT/erasor2/bootstrap_env.py"
"$ENV_DIR/bin/python" -m pip install -r "$ROOT/erasor2/requirements.txt"
"$ENV_DIR/bin/python" -m pip freeze > "$ENV_DIR/mapping-requirements.lock.txt"
"$ENV_DIR/bin/python" -c 'import numpy, yaml, hdbscan, pypatchworkpp; print("ERASOR2 preprocessing imports OK")'
echo "Use ERASOR2_PYTHON=$ENV_DIR/bin/python"
