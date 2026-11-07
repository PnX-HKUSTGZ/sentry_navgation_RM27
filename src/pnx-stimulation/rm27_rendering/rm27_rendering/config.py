"""Launch-only rendering defaults from the selected simulation YAML profile."""

import yaml

from . import resolve_render_mode


def configured_render_mode(params_file, mode="", legacy=""):
    """CLI mode > explicit legacy flag > YAML > auto for older custom profiles."""
    if str(mode).strip() or str(legacy).strip():
        return resolve_render_mode(mode, legacy)
    with open(params_file, encoding="utf-8") as stream:
        configuration = yaml.safe_load(stream) or {}
    setting = (
        configuration.get("simulation_rendering", {})
        .get("ros__parameters", {})
        .get("render_mode", "auto")
    )
    # YAML booleans and empty values must not silently enable acceleration.
    if not isinstance(setting, str) or not setting.strip():
        raise ValueError("simulation_rendering.render_mode must be auto, gpu, or cpu")
    return resolve_render_mode(setting)
