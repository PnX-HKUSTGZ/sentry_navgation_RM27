"""Build provenance and process ownership shared by the visualization launcher."""

import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import psutil

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "tools/global_relocalization/.build/ros"
INSTALL = BUILD / "install"
MANIFEST = BUILD / "source_manifest.json"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_hashes():
    files = {}
    for package in ("global_relocalization", "global_relocalization_interfaces"):
        for path in sorted((ROOT / "src" / package).rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts:
                files[str(path.relative_to(ROOT))] = digest(path)
    return files


def binaries():
    library = INSTALL / "global_relocalization/lib"
    return [
        library / "libglobal_relocalization_core.so",
        library / "global_relocalization/global_relocalization_node",
        library / "global_relocalization/display_helper.py",
    ]


def build_current():
    try:
        frozen = json.loads(MANIFEST.read_text())
        return frozen["sources"] == source_hashes() and frozen["binaries"] == {
            str(p.relative_to(ROOT)): digest(p) for p in binaries()
        }
    except (OSError, KeyError, ValueError):
        return False


def freeze_build():
    # colcon install does not remove deleted profiles or launch files.
    for name in ("config", "launch", "rviz"):
        source = ROOT / "src/global_relocalization" / name
        installed = INSTALL / "global_relocalization/share/global_relocalization" / name
        if installed.exists():
            for path in installed.rglob("*"):
                if (
                    path.is_file()
                    and not (source / path.relative_to(installed)).is_file()
                ):
                    path.unlink()
    executable_dir = INSTALL / "global_relocalization/lib/global_relocalization"
    for path in executable_dir.glob("*.py"):
        if path.name != "display_helper.py":
            path.unlink()
    content = {
        "sources": source_hashes(),
        "binaries": {str(p.relative_to(ROOT)): digest(p) for p in binaries()},
    }
    temporary = MANIFEST.with_suffix(".tmp")
    temporary.write_text(json.dumps(content, indent=2) + "\n")
    temporary.replace(MANIFEST)


def boot_id():
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def start_ticks(pid):
    # Kernel start ticks survive WSL wall-clock corrections; create_time does not.
    return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]


def owned_process(info):
    if not info:
        return None
    try:
        process = psutil.Process(info["pid"])
        if (
            info["boot_id"] == boot_id()
            and info["start_ticks"] == start_ticks(process.pid)
            and process.status() != psutil.STATUS_ZOMBIE
        ):
            return process
    except (OSError, KeyError, psutil.NoSuchProcess):
        pass
    return None


def start_process(command, log_path, environment):
    with Path(log_path).open("a") as log:
        process = subprocess.Popen(
            command,
            env=environment,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    return {
        "pid": process.pid,
        "boot_id": boot_id(),
        "start_ticks": start_ticks(process.pid),
        "command": command,
        "started_wall": time.time(),
    }


def stop_process(info):
    process = owned_process(info)
    if process is None:
        return
    if os.getpgid(process.pid) != process.pid:
        raise RuntimeError(f"Unexpected process group for PID {process.pid}")
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except psutil.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


if __name__ == "__main__":
    if sys.argv[1:] == ["check-build"]:
        raise SystemExit(0 if build_current() else 1)
    if sys.argv[1:] != ["freeze-build"]:
        raise SystemExit("usage: runtime.py freeze-build")
    freeze_build()
