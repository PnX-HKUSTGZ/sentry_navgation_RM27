import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

import rm27_rendering as rendering


def test_render_mode_explicit_selection_overrides_legacy_flag():
    assert rendering.resolve_render_mode("", "false") == "cpu"
    assert rendering.resolve_render_mode("", "true") == "auto"
    assert rendering.resolve_render_mode("GPU", "false") == "gpu"


@pytest.mark.parametrize("legacy", ["0", "false", "no", "off"])
def test_render_mode_maps_false_legacy_values_to_cpu(legacy):
    assert rendering.resolve_render_mode("", legacy) == "cpu"


def test_wsl_rendering_environment_keeps_d3d12_gpu_and_llvmpipe_cpu_separate():
    inherited = {
        "DISPLAY": ":0",
        "WSL_INTEROP": "/run/WSL/123_interop",
        "GALLIUM_DRIVER": "llvmpipe",
        "MESA_LOADER_DRIVER_OVERRIDE": "llvmpipe",
        "MESA_D3D12_DEFAULT_ADAPTER_NAME": "Intel",
        "LIBGL_ALWAYS_SOFTWARE": "1",
        "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
        "VGL_DISPLAY": ":9",
    }
    original = dict(inherited)

    gpu = rendering.rendering_environment("gpu", inherited, wsl=True)
    cpu = rendering.rendering_environment("cpu", inherited, wsl=True)

    assert inherited == original
    assert gpu["GALLIUM_DRIVER"] == "d3d12"
    assert gpu["MESA_LOADER_DRIVER_OVERRIDE"] == "d3d12"
    assert gpu["MESA_D3D12_DEFAULT_ADAPTER_NAME"] == "NVIDIA"
    assert gpu["QT_QPA_PLATFORM"] == "xcb"
    assert "LIBGL_ALWAYS_SOFTWARE" not in gpu
    assert "__GLX_VENDOR_LIBRARY_NAME" not in gpu
    assert cpu["GALLIUM_DRIVER"] == "llvmpipe"
    assert cpu["LIBGL_ALWAYS_SOFTWARE"] == "1"
    assert "MESA_LOADER_DRIVER_OVERRIDE" not in cpu
    assert "MESA_D3D12_DEFAULT_ADAPTER_NAME" not in cpu
    assert "__GLX_VENDOR_LIBRARY_NAME" not in cpu
    assert "VGL_DISPLAY" not in cpu
    assert cpu["QT_QPA_PLATFORM"] == "xcb"


def test_gpu_mode_rejects_software_renderer_with_effective_child_environment(monkeypatch):
    monkeypatch.setattr(rendering, "is_wsl", lambda _environment: True)
    calls = []

    def software_probe(environment, **kwargs):
        calls.append((dict(environment), kwargs))
        return rendering.ProbeResult(renderer="llvmpipe (LLVM 19.1.7, 256 bits)")

    monkeypatch.setattr(rendering, "probe_renderer", software_probe)
    inherited = {"DISPLAY": ":0", "LIBGL_ALWAYS_SOFTWARE": "1"}

    with pytest.raises(RuntimeError, match="requires hardware OpenGL.*software renderer"):
        rendering.select_rendering("gpu", gui=False, purpose="gazebo", environ=inherited)

    assert inherited == {"DISPLAY": ":0", "LIBGL_ALWAYS_SOFTWARE": "1"}
    assert len(calls) == 1
    environment, options = calls[0]
    assert environment["GALLIUM_DRIVER"] == "d3d12"
    assert environment["MESA_D3D12_DEFAULT_ADAPTER_NAME"] == "NVIDIA"
    assert environment["QT_QPA_PLATFORM"] == "xcb"
    # Sensors use WSLg GLX even when the Gazebo GUI is disabled.
    assert options["backend"] == "glx"


def test_wsl_gpu_mode_requires_the_nvidia_adapter(monkeypatch):
    monkeypatch.setattr(rendering, "is_wsl", lambda _environment: True)
    monkeypatch.setattr(
        rendering,
        "probe_renderer",
        lambda _environment, **_kwargs: rendering.ProbeResult(
            renderer="AMD Radeon RX 7900", version="4.6"
        ),
    )

    with pytest.raises(RuntimeError, match="expected NVIDIA adapter"):
        rendering.select_rendering("gpu", purpose="rviz", environ={"DISPLAY": ":0"})


def test_wsl_gpu_mode_accepts_nvidia_d3d12_renderer(monkeypatch):
    monkeypatch.setattr(rendering, "is_wsl", lambda _environment: True)
    monkeypatch.setattr(
        rendering,
        "probe_renderer",
        lambda _environment, **_kwargs: rendering.ProbeResult(
            renderer="D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)",
            vendor="Microsoft Corporation",
            version="4.6 (Core Profile) Mesa 25.0",
        ),
    )

    selected = rendering.select_rendering(
        "gpu", purpose="rviz", environ={"DISPLAY": ":0"}
    )

    assert selected.mode == "gpu"
    assert selected.backend == "glx"
    assert "NVIDIA GeForce RTX 5070 Ti" in selected.renderer


def test_auto_falls_back_to_llvmpipe_after_gpu_probe_returns_software(monkeypatch):
    monkeypatch.setattr(rendering, "is_wsl", lambda _environment: True)
    calls = []

    def probe(environment, **kwargs):
        calls.append((dict(environment), kwargs))
        if len(calls) == 1:
            return rendering.ProbeResult(renderer="llvmpipe (LLVM 19.1.7, 256 bits)")
        return rendering.ProbeResult(
            renderer="llvmpipe (LLVM 19.1.7, 256 bits)", version="4.5"
        )

    monkeypatch.setattr(rendering, "probe_renderer", probe)
    inherited = {"DISPLAY": ":0", "MESA_D3D12_DEFAULT_ADAPTER_NAME": "Intel"}
    original = dict(inherited)

    selected = rendering.select_rendering(
        "auto", gui=False, purpose="gazebo", environ=inherited
    )

    assert selected.mode == "cpu"
    assert selected.backend == "glx"
    assert selected.renderer.startswith("llvmpipe")
    assert "falling back to CPU llvmpipe" in selected.warning
    assert len(calls) == 2
    assert calls[0][0]["GALLIUM_DRIVER"] == "d3d12"
    assert calls[1][0]["GALLIUM_DRIVER"] == "llvmpipe"
    assert calls[1][0]["LIBGL_ALWAYS_SOFTWARE"] == "1"
    assert inherited == original


def test_cpu_mode_rejects_non_llvmpipe_probe(monkeypatch):
    monkeypatch.setattr(rendering, "is_wsl", lambda _environment: True)
    calls = []

    def unexpected_probe(environment, **kwargs):
        calls.append((dict(environment), kwargs))
        return rendering.ProbeResult(renderer="D3D12 (NVIDIA GeForce RTX)")

    monkeypatch.setattr(rendering, "probe_renderer", unexpected_probe)

    with pytest.raises(RuntimeError, match="CPU llvmpipe preflight failed.*unexpected renderer"):
        rendering.select_rendering(
            "cpu", gui=False, purpose="gazebo", environ={"DISPLAY": ":0"}
        )

    assert len(calls) == 1
    assert calls[0][0]["GALLIUM_DRIVER"] == "llvmpipe"
    assert calls[0][0]["LIBGL_ALWAYS_SOFTWARE"] == "1"
    assert calls[0][1]["backend"] == "glx"


def test_probe_timeout_is_bounded_and_reported_to_required_gpu_mode(monkeypatch):
    calls = []
    killed = []
    timeouts = []

    class TimedOutProcess:
        pid = 31415
        returncode = -9

        def __init__(self, command):
            self.command = command
            self.communicate_calls = 0

        def communicate(self, timeout=None):
            self.communicate_calls += 1
            timeouts.append(timeout)
            if self.communicate_calls == 1:
                raise subprocess.TimeoutExpired(self.command, timeout)
            return "", "terminated"

    def start_probe(command, **kwargs):
        calls.append((command, kwargs))
        return TimedOutProcess(command)

    monkeypatch.setattr(rendering.subprocess, "Popen", start_probe)
    monkeypatch.setattr(
        rendering.os,
        "killpg",
        lambda process_group, signal_number: killed.append(
            (process_group, signal_number)
        ),
    )
    monkeypatch.setattr(rendering, "is_wsl", lambda _environment: False)

    with pytest.raises(RuntimeError, match="render_mode=gpu requires hardware OpenGL.*timed out"):
        rendering.select_rendering(
            "gpu", purpose="rviz", environ={"DISPLAY": ":0"}, timeout=0.125
        )

    assert len(calls) == 1
    assert calls[0][1]["stdout"] == subprocess.PIPE
    assert calls[0][1]["stderr"] == subprocess.PIPE
    assert calls[0][1]["start_new_session"] is True
    assert timeouts == [0.125, None]
    assert calls[0][1]["env"]["DISPLAY"] == ":0"
    assert calls[0][0][-1] == "glx"
    assert killed == [(31415, rendering.signal.SIGKILL)]


def test_selected_child_environment_is_applied_only_to_that_child(monkeypatch):
    monkeypatch.setattr(rendering, "is_wsl", lambda _environment: True)
    monkeypatch.setattr(
        rendering,
        "probe_renderer",
        lambda _environment, **_kwargs: rendering.ProbeResult(
            renderer="llvmpipe (LLVM 19.1.7, 256 bits)", version="4.5"
        ),
    )
    inherited = {
        "PATH": os.environ["PATH"],
        "DISPLAY": ":0",
        "GALLIUM_DRIVER": "d3d12",
        "MESA_D3D12_DEFAULT_ADAPTER_NAME": "NVIDIA",
        "LIBGL_ALWAYS_SOFTWARE": "0",
    }
    original = dict(inherited)
    selection = rendering.select_rendering(
        "cpu", purpose="rviz", environ=inherited
    )
    child = subprocess.run(
        [
            sys.executable,
            "-c",
            "import os; print(os.environ['GALLIUM_DRIVER'] + ':' + os.environ['LIBGL_ALWAYS_SOFTWARE'])",
        ],
        env=selection.environment,
        capture_output=True,
        text=True,
        check=True,
    )

    assert child.stdout.strip() == "llvmpipe:1"
    assert inherited == original
    assert selection.set_env["GALLIUM_DRIVER"] == "llvmpipe"
    assert "MESA_D3D12_DEFAULT_ADAPTER_NAME" in selection.unset_env


def test_wrapper_appends_user_launch_overrides_after_defaults(tmp_path):
    workspace = tmp_path / "workspace"
    stimulation_dir = workspace / "tools" / "stimulation"
    install_dir = workspace / "install"
    fake_bin = tmp_path / "fake-bin"
    stimulation_dir.mkdir(parents=True)
    install_dir.mkdir()
    fake_bin.mkdir()

    script = Path(__file__).resolve().parents[3] / "tools" / "stimulation" / "start_nav_stimulation.bash"
    (stimulation_dir / "start_nav_stimulation.bash").write_text(script.read_text())
    record_path = tmp_path / "ros2-argv.json"
    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text(
        "#!/usr/bin/env python3\n"
        "import json, os, sys\n"
        "with open(os.environ['RM27_TEST_ARGV'], 'w') as stream:\n"
        "    json.dump({'argv': sys.argv[1:], "
        "'local_setup': os.environ.get('RM27_LOCAL_SETUP')}, stream)\n"
    )
    fake_ros2.chmod(0o755)
    copied_script = stimulation_dir / "start_nav_stimulation.bash"
    copied_script.chmod(0o755)
    (install_dir / "local_setup.bash").write_text(
        "export RM27_LOCAL_SETUP=loaded\n"
        f"export PATH={str(fake_bin)!r}:\"$PATH\"\n"
    )
    arguments = [
        "render_mode:=gpu",
        "hardware_acceleration:=false",
        "gui:=false",
        "use_rviz:=false",
    ]
    completed = subprocess.run(
        [str(stimulation_dir / "start_nav_stimulation.bash"), *arguments],
        cwd=tmp_path,
        env={
            **os.environ,
            "AMENT_PREFIX_PATH": "",
            "COLCON_PREFIX_PATH": "",
            "CMAKE_PREFIX_PATH": "",
            "ROS_PACKAGE_PATH": "",
            "PYTHONPATH": "",
            "LD_LIBRARY_PATH": "",
            "GZ_SIM_RESOURCE_PATH": "",
            "GZ_CONFIG_PATH": "",
            "RM27_TEST_ARGV": str(record_path),
        },
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    recorded = json.loads(record_path.read_text())
    forwarded = recorded["argv"]
    assert forwarded[:3] == ["launch", "rm_27_stimulation", "sim_with_nav.launch.py"]
    assert forwarded[-len(arguments):] == arguments
    assert forwarded.index("gui:=true") < forwarded.index("gui:=false")
    assert forwarded.index("use_rviz:=true") < forwarded.index("use_rviz:=false")
    assert "render_mode:=auto" not in forwarded
    assert recorded["local_setup"] == "loaded"
