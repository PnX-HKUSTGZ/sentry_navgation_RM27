"""Shared rendering policy. Nothing here changes the parent process environment."""
from dataclasses import dataclass
import json
import os
import platform
import re
import shutil
import signal
import subprocess
import sys


@dataclass(frozen=True)
class ProbeResult:
    renderer: str = ""
    vendor: str = ""
    version: str = ""
    error: str = ""

    @property
    def opengl_supported(self):
        match = re.match(r"(\d+)\.(\d+)", self.version)
        return bool(match) and tuple(map(int, match.groups())) >= (3, 3)

    @property
    def hardware(self):
        text = (self.renderer + " " + self.vendor).lower()
        return bool(self.renderer) and not self.error and not any(
            name in text for name in (
                "llvmpipe", "softpipe", "swrast", "software rasterizer",
                "swiftshader", "lavapipe", "microsoft basic render", "gdi generic",
            )
        )


@dataclass(frozen=True)
class RenderingSelection:
    requested_mode: str
    mode: str
    backend: str
    renderer: str
    environment: dict
    set_env: dict
    unset_env: tuple
    prefix: tuple = ()
    warning: str = ""
    vendor: str = ""
    version: str = ""


def resolve_render_mode(mode="", legacy="true"):
    """Explicit mode wins; empty mode retains the legacy boolean migration."""
    mode = str(mode).strip().lower()
    if mode:
        if mode not in ("auto", "gpu", "cpu"):
            raise ValueError("render_mode must be one of: auto, gpu, cpu")
        return mode
    legacy = str(legacy).strip().lower()
    if legacy in ("1", "true", "yes", "on"):
        return "auto"
    if legacy in ("0", "false", "no", "off"):
        return "cpu"
    raise ValueError("legacy hardware acceleration must be true or false")


def is_wsl(environ=None):
    environment = os.environ if environ is None else environ
    return "microsoft" in platform.release().lower() or bool(environment.get("WSL_INTEROP"))


def rendering_environment(mode, environ=None, wsl=None):
    """Return a child environment with conflicting driver selectors removed."""
    environment = dict(os.environ if environ is None else environ)
    wsl = is_wsl(environment) if wsl is None else wsl
    if mode == "cpu":
        for key in (
            "MESA_LOADER_DRIVER_OVERRIDE", "MESA_D3D12_DEFAULT_ADAPTER_NAME",
            "__GLX_VENDOR_LIBRARY_NAME", "__NV_PRIME_RENDER_OFFLOAD",
            "__NV_PRIME_RENDER_OFFLOAD_PROVIDER", "LIBGL_ALWAYS_INDIRECT",
            "DRI_PRIME", "GALLIUM_HUD", "EGL_PLATFORM", "VGL_DISPLAY",
            "__EGL_VENDOR_LIBRARY_FILENAMES", "__EGL_VENDOR_LIBRARY_DIRS",
        ):
            environment.pop(key, None)
        environment.update(LIBGL_ALWAYS_SOFTWARE="1", GALLIUM_DRIVER="llvmpipe")
        if not wsl:
            environment["__GLX_VENDOR_LIBRARY_NAME"] = "mesa"
        mesa_egl_vendor = "/usr/share/glvnd/egl_vendor.d/50_mesa.json"
        if not wsl and os.path.isfile(mesa_egl_vendor):
            # Native NVIDIA EGL devices ignore LIBGL_ALWAYS_SOFTWARE; select
            # Mesa's EGL provider before enumerating the llvmpipe device.
            environment["__EGL_VENDOR_LIBRARY_FILENAMES"] = mesa_egl_vendor
    elif mode == "gpu":
        for key in ("LIBGL_ALWAYS_SOFTWARE", "LIBGL_ALWAYS_INDIRECT"):
            environment.pop(key, None)
        for key in ("GALLIUM_DRIVER", "MESA_LOADER_DRIVER_OVERRIDE"):
            if environment.get(key, "").lower() in ("llvmpipe", "softpipe", "swrast"):
                environment.pop(key, None)
        if wsl:
            environment.pop("__GLX_VENDOR_LIBRARY_NAME", None)
            environment.pop("__NV_PRIME_RENDER_OFFLOAD", None)
            environment.pop("EGL_PLATFORM", None)
            environment.update(
                GALLIUM_DRIVER="d3d12", MESA_LOADER_DRIVER_OVERRIDE="d3d12",
                MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA", QT_QPA_PLATFORM="xcb",
            )
    else:
        raise ValueError("rendering_environment expects gpu or cpu")
    if wsl:
        environment["QT_QPA_PLATFORM"] = "xcb"
    return environment


def probe_renderer(environment, *, backend="glx", prefix=(), timeout=8.0):
    """Probe actual GL strings in a bounded child; driver hangs cannot hang launch."""
    command = [*prefix, sys.executable, os.path.join(os.path.dirname(__file__), "probe.py"), backend]
    try:
        process = subprocess.Popen(command, env=environment, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True, start_new_session=True)
    except OSError as error:
        return ProbeResult(error=f"OpenGL {backend} probe failed: {error}")
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        # A VirtualGL wrapper may have children holding the output pipes open.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.communicate()
        return ProbeResult(error=f"OpenGL {backend} probe timed out after {timeout:g}s")
    try:
        data = json.loads(stdout.strip().splitlines()[-1])
    except (ValueError, IndexError):
        return ProbeResult(error=f"OpenGL {backend} probe exited {process.returncode}: "
                           + (stderr.strip() or "no renderer result")[-500:])
    if process.returncode:
        return ProbeResult(error=data.get("error", f"probe exited {process.returncode}"))
    return ProbeResult(**{name: data.get(name, "") for name in ("renderer", "vendor", "version", "error")})


def _virtualgl(environment):
    candidates = [shutil.which("vglrun", path=environment.get("PATH")),
                  "/usr/NX/scripts/vgl/vglrun", "/opt/VirtualGL/bin/vglrun"]
    for candidate in candidates:
        if candidate and os.access(candidate, os.X_OK):
            device = "egl" if os.path.realpath(candidate).startswith("/usr/NX/") else "egl0"
            return (candidate, "-d", device)
    return ()


def select_rendering(mode, *, gui=True, purpose="gazebo", environ=None, timeout=8.0):
    mode = resolve_render_mode(mode)
    if purpose not in ("gazebo", "rviz"):
        raise ValueError("purpose must be gazebo or rviz")
    original = dict(os.environ if environ is None else environ)
    wsl = is_wsl(original)
    backend = "egl" if purpose == "gazebo" and not gui and not wsl else "glx"
    warning = ""
    prefix = ()
    if mode != "cpu":
        environment = rendering_environment("gpu", original, wsl)
        if not wsl and backend == "glx":
            prefix = _virtualgl(environment)
        probe = probe_renderer(environment, backend=backend, prefix=prefix, timeout=timeout)
        if probe.hardware and probe.opengl_supported and (not wsl or "nvidia" in probe.renderer.lower()):
            selected = "gpu"
        else:
            reason = probe.error or (
                f"software renderer '{probe.renderer}'" if not probe.hardware else
                f"OpenGL 3.3 or newer required, got '{probe.version}'" if not probe.opengl_supported else
                f"expected NVIDIA adapter, got '{probe.renderer}'"
            )
            if mode == "gpu":
                raise RuntimeError(f"render_mode=gpu requires hardware OpenGL for {purpose}: {reason}")
            warning = f"GPU preflight failed for {purpose}: {reason}; falling back to CPU llvmpipe"
            selected = "cpu"
    else:
        selected = "cpu"
    if selected == "cpu":
        prefix = ()
        environment = rendering_environment("cpu", original, wsl)
        probe = probe_renderer(environment, backend=backend, timeout=timeout)
        if probe.error or "llvmpipe" not in probe.renderer.lower() or not probe.opengl_supported:
            reason = probe.error or (
                f"unexpected renderer '{probe.renderer}'" if "llvmpipe" not in probe.renderer.lower() else
                f"OpenGL 3.3 or newer required, got '{probe.version}'"
            )
            raise RuntimeError(f"CPU llvmpipe preflight failed for {purpose}: {reason}")
    set_env = {key: value for key, value in environment.items() if original.get(key) != value}
    unset_env = tuple(sorted(set(original) - set(environment)))
    return RenderingSelection(mode, selected, backend, probe.renderer, environment,
                              set_env, unset_env, prefix, warning, probe.vendor, probe.version)
