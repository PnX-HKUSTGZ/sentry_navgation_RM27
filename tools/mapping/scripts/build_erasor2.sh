#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} == -h || ${1:-} == --help ]]; then
  echo 'Usage: build_erasor2.sh [-j jobs] (default 2; ERASOR2_BUILD_DIR overrides local build path)'
  exit 0
fi
jobs=${ERASOR2_JOBS:-2}
if [[ ! $jobs =~ ^[1-9][0-9]*$ ]]; then echo 'ERASOR2_JOBS must be a positive integer' >&2; exit 2; fi
if [[ $# -gt 0 ]]; then
  if [[ $# -ne 2 || $1 != -j || ! $2 =~ ^[1-9][0-9]*$ ]]; then echo 'Usage: build_erasor2.sh [-j jobs]' >&2; exit 2; fi
  jobs=$2
fi
BUILD=${ERASOR2_BUILD_DIR:-$ROOT/.build/erasor2}
cmake -S "$ROOT/erasor2/vendor" -B "$BUILD" -DERASOR2_ENABLE_RERUN=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD" --target mapgen run_erasor2 --parallel "$jobs"
