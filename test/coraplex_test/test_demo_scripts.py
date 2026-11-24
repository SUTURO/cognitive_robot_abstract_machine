"""
Coverage for the demo scripts CI runs as regression tests.
"""

import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import pytest


DEMOS_ROOT = Path(__file__).resolve().parents[2] / "coraplex" / "demos"

CHALLENGE_ROOT = DEMOS_ROOT / "suturo_human_robot_interaction_challenge"


@dataclass(kw_only=True)
class DemoWrapper:
    """
    The script CI runs to check a demo.
    """

    path: Path
    """
    Where the script is.
    """

    demo_module: str = "demo"
    """
    The module of the demo the script imports first.
    """


WRAPPERS = [
    DemoWrapper(path=DEMOS_ROOT / "coraplex_bullet_world_demo" / "test_demo.py"),
    DemoWrapper(path=DEMOS_ROOT / "coraplex_failure_handling_demo" / "test_demo.py"),
    DemoWrapper(
        path=CHALLENGE_ROOT
        / "milestone_0"
        / "hsr_door_to_sofa"
        / "test_find_free_seats.py",
        demo_module="free_seats_demonstration",
    ),
    DemoWrapper(
        path=CHALLENGE_ROOT
        / "milestone_1"
        / "hsr_look_at_sofa_while_driving"
        / "test_look_at_sofa_while_driving.py",
        demo_module="look_at_sofa_while_driving_in_mujoco",
    ),
    DemoWrapper(path=DEMOS_ROOT / "coraplex_hsr_mujoco_demo" / "test_demo.py"),
    DemoWrapper(path=DEMOS_ROOT / "coraplex_real_tracy" / "test_demo.py"),
    DemoWrapper(
        path=DEMOS_ROOT / "coraplex_unitree_g1_warehouse_demo" / "test_demo.py"
    ),
]

# %% wrapper failure reporting


@pytest.mark.parametrize(
    "wrapper", WRAPPERS, ids=[wrapper.path.parent.name for wrapper in WRAPPERS]
)
def test_wrapper_reports_the_real_import_failure(tmp_path, wrapper):
    """
    A demo's ``test_demo.py`` wrapper must surface the real exception from a failing
    import of the demo (as a traceback on stderr) instead of silently exiting, so CI
    failures are diagnosable.
    """
    (tmp_path / f"{wrapper.demo_module}.py").write_text(
        "raise RuntimeError('boom from demo fixture')\n"
    )
    (tmp_path / "test_demo.py").write_text(wrapper.path.read_text())

    result = subprocess.run(
        [sys.executable, "test_demo.py"],
        cwd=tmp_path,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    assert "boom from demo fixture" in result.stderr
    assert "Traceback" in result.stderr


BULLET_WORLD_WRAPPER = DEMOS_ROOT / "coraplex_bullet_world_demo" / "test_demo.py"
"""
The wrapper of the one demo that is imported first and run through its ``main``.
"""


@pytest.mark.parametrize(
    "main_body, exit_code",
    [("raise RuntimeError('boom from main')", 1), ("pass", 0)],
    ids=["main-fails", "main-succeeds"],
)
def test_wrapper_reports_whether_the_demos_main_succeeded(
    tmp_path, main_body, exit_code
):
    """
    A demo that runs from ``main`` rather than on import fails inside ``main``, and the
    wrapper has to surface that failure as it does a failing import.
    """
    (tmp_path / "demo.py").write_text(f"def main():\n    {main_body}\n")
    (tmp_path / "test_demo.py").write_text(BULLET_WORLD_WRAPPER.read_text())

    result = subprocess.run(
        [sys.executable, "test_demo.py"],
        cwd=tmp_path,
        capture_output=True,
        text=True,
    )

    assert result.returncode == exit_code
    assert ("boom from main" in result.stderr) is (exit_code == 1)
