#!/usr/bin/env bash
# Same positional interface as upstream; outputs must be a fresh separate run directory.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} == -h || ${1:-} == --help ]]; then
  echo 'Usage: run_sequence.sh <sequence_dir> <parameter.yaml>'
  echo 'ERASOR2_OUTPUT_DIR: new run directory outside the input (default: sibling <sequence>_erasor2_output)'
  echo 'ERASOR2_PYTHON: local venv interpreter; ERASOR2_CONDA_ENV: alternative environment prefix'
  echo 'ERASOR2_BUILD_DIR: CMake build path; ERASOR2_JOBS: build jobs (default 2)'
  exit 0
fi
if [[ $# -ne 2 ]]; then echo 'Usage: run_sequence.sh <sequence_dir> <parameter.yaml>' >&2; exit 2; fi
PYTHON=${ERASOR2_PYTHON:-${ERASOR2_CONDA_ENV:+$ERASOR2_CONDA_ENV/bin/python}}
PYTHON=${PYTHON:-$ROOT/.venv/bin/python}
if [[ ! -x $PYTHON ]]; then echo 'Run scripts/setup_erasor2_env.sh or set ERASOR2_PYTHON/ERASOR2_CONDA_ENV.' >&2; exit 2; fi
exec "$PYTHON" "$ROOT/erasor2/run_sequence.py" "$@"
