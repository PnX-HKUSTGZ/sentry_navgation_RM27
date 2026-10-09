#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Use ROS's system interpreter, separately from the ERASOR2 Python environment.
exec "${POINTLIO_EXPORT_PYTHON:-/usr/bin/python3}" "$ROOT/pointlio_export/export_bag.py" "$@"
