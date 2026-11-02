#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/tools/global_relocalization/environment.bash"
exec /usr/bin/python3 "$root/tools/global_relocalization/bag_replay_session.py" "$@"
