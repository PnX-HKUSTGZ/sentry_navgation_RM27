#!/usr/bin/env bash
set -eo pipefail
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ "${1:-}" != --stop && "${1:-}" != --help ]]; then
  if ! /usr/bin/python3 "$root/tools/global_relocalization/runtime.py" check-build; then
    echo 'Building the current global relocalization sources...'
    bash "$root/tools/global_relocalization/build.bash"
  fi
fi
source "$root/tools/global_relocalization/environment.bash"
exec /usr/bin/python3 "$root/tools/global_relocalization/visualize.py" "$@"
