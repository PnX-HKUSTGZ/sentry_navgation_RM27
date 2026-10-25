import sys
from unittest.mock import patch
import psutil
import pytest
import runtime


@pytest.fixture
def child(tmp_path):
    info = runtime.start_process(
        [
            sys.executable,
            "-c",
            "import signal,time,sys;signal.signal(signal.SIGINT,lambda *_:sys.exit(0));time.sleep(60)",
        ],
        tmp_path / "child.log",
        None,
    )
    try:
        yield info
    finally:
        runtime.stop_process(info)


def test_owned_identity_ignores_wall_clock_adjustments(child):
    with patch.object(psutil.Process, "create_time", return_value=-1000):
        assert runtime.owned_process(child) is not None


def test_reused_pid_or_previous_boot_cannot_be_stopped(child):
    for changed in (
        dict(child, start_ticks="invalid"),
        dict(child, boot_id="previous-boot"),
    ):
        with patch.object(runtime.os, "killpg") as kill:
            runtime.stop_process(changed)
            kill.assert_not_called()
    assert runtime.owned_process(child) is not None


def test_stopping_an_owned_child_leaves_no_running_worker(child):
    runtime.stop_process(child)
    assert runtime.owned_process(child) is None


def test_session_reuse_detects_changed_contents_at_the_same_paths(
    tmp_path, monkeypatch
):
    from types import SimpleNamespace
    import visualize

    files = {key: tmp_path / key for key in ("map", "manifest", "params")}
    for path in files.values():
        path.write_text("original")
    build = tmp_path / "build_manifest.json"
    build.write_text("first-build")
    monkeypatch.setattr(visualize, "BUILD_MANIFEST", build)
    args = SimpleNamespace(**files, render_mode="auto")
    first = visualize.session_settings(args)
    files["params"].write_text("updated-parameters")
    second = visualize.session_settings(args)
    assert first["params"] == second["params"]
    assert first != second
    build.write_text("updated-build")
    assert second != visualize.session_settings(args)
