#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Caller sources ROS Jazzy; ERASOR2 subprocess uses the separate local venv.
exec "${POINTLIO_EXPORT_PYTHON:-/usr/bin/python3}" "$ROOT/pointlio_export/smoke_pipeline.py" "$@"
