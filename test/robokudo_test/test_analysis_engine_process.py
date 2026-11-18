"""
Tests for :class:`robokudo.analysis_engine_process.AnalysisEngineProcess`, and for what
the analysis engines it starts load.
"""

from __future__ import annotations

import importlib.util
import subprocess
import sys
from datetime import timedelta
from enum import StrEnum
from unittest.mock import Mock

import pytest
from rclpy.action import ActionClient
from rclpy.node import Node
from robokudo_msgs.action import Query

from robokudo.analysis_engine_process import (
    AnalysisEngineModule,
    AnalysisEngineProcess,
    InterpreterOption,
)
from robokudo.descriptors.analysis_engines import human_detection
from robokudo.exceptions import AnalysisEngineDidNotStart
from robokudo.utils.module_loader import RobokudoModuleType
from robokudo.vis import ros_visualizer

# %% what an analysis engine loads


class ImportReport(StrEnum):
    """
    The option that makes a Python interpreter report every module it imports, and the
    parts of the lines it reports them in.
    """

    IMPLEMENTATION_OPTION = "-X"
    """
    Sets an option specific to the interpreter.
    """

    IMPORT_TIME = "importtime"
    """
    Reports every imported module.
    """

    LINE_START = "import time:"
    """
    Begins every line of the report.
    """

    COLUMN_SEPARATOR = "|"
    """
    Separates the timings of a line from the name of the imported module.
    """


class GpsrModelLibrary(StrEnum):
    """
    Libraries that only the models of GPSR run on.
    """

    LLAMA_CPP = "llama_cpp"
    """
    Runs the vision language model.
    """

    TRANSFORMERS = "transformers"
    """
    Runs the activity and open vocabulary models.
    """


class NumPy1Library(StrEnum):
    """
    Libraries of ROS built for NumPy 1, which report an import error under NumPy 2.
    """

    CV_BRIDGE = "cv_bridge"
    """
    Converts between images and their ROS messages.
    """


def modules_loaded_to_run(module_name: str) -> set[str]:
    """
    :param module_name: The module a new Python interpreter runs.
    :return: The names of the modules the interpreter loads to run it.
    """
    report = subprocess.run(
        [
            sys.executable,
            ImportReport.IMPLEMENTATION_OPTION,
            ImportReport.IMPORT_TIME,
            InterpreterOption.MODULE,
            module_name,
        ],
        capture_output=True,
        text=True,
        check=True,
    ).stderr
    return {
        line.rsplit(ImportReport.COLUMN_SEPARATOR, 1)[-1].strip()
        for line in report.splitlines()
        if line.startswith(ImportReport.LINE_START)
    }


def test_detecting_humans_loads_no_library_of_the_gpsr_models() -> None:
    """
    The human detection runs where the libraries of the GPSR models are missing.
    """
    assert modules_loaded_to_run(human_detection.__name__).isdisjoint(GpsrModelLibrary)


def test_the_ros_visualizers_load_no_library_built_for_numpy_1() -> None:
    """
    Starting RoboKudo reports no import error of a library built for NumPy 1.
    """
    assert modules_loaded_to_run(ros_visualizer.__name__).isdisjoint(NumPy1Library)


# %% analysis engines to choose from


@pytest.mark.parametrize("analysis_engine", AnalysisEngineModule)
def test_each_analysis_engine_to_choose_from_can_be_found(
    analysis_engine: AnalysisEngineModule,
) -> None:
    """
    Each choice names a module RoboKudo loads its analysis engine from, found here
    without loading the models the analysis engine needs.
    """
    process = AnalysisEngineProcess(analysis_engine=analysis_engine)
    module = ".".join(
        [
            process.ros_package,
            *RobokudoModuleType.AnalysisEngine.value,
            process.analysis_engine,
        ]
    )

    assert importlib.util.find_spec(module) is not None


# %% starting and stopping


def test_stopping_an_engine_that_failed_to_launch(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    Cleanup after a launch failure does not hide the original error.
    """
    engine = AnalysisEngineProcess(analysis_engine=AnalysisEngineModule.HUMAN_DETECTION)
    monkeypatch.setattr(subprocess, "Popen", Mock(side_effect=OSError))

    with pytest.raises(OSError):
        engine.start()

    engine.stop()


def test_start_timeout_reports_that_the_engine_did_not_start() -> None:
    """
    A startup timeout describes the missing start announcement.
    """
    failure = AnalysisEngineDidNotStart(analysis_engine="query", timeout_seconds=5.0)

    assert failure.error_message() == (
        f"The analysis engine {failure.analysis_engine} did not announce that it is "
        f"running within {failure.timeout_seconds} seconds."
    )


@pytest.fixture
def query_analysis_engine() -> AnalysisEngineProcess:
    """
    A small analysis engine that answers queries, without windows.
    """
    return AnalysisEngineProcess(analysis_engine="query", shows_views=False)


def test_started_analysis_engine_exposes_the_action_server(
    node: Node, query_analysis_engine: AnalysisEngineProcess
) -> None:
    """
    The started engine exposes its query action.
    """
    query_analysis_engine.start()
    client = ActionClient(node, Query, "robokudo/query")

    assert client.wait_for_server(timeout_sec=0)

    client.destroy()
    query_analysis_engine.stop()


def test_stopped_analysis_engine_process_has_ended(
    query_analysis_engine: AnalysisEngineProcess,
) -> None:
    """
    Stopping the engine waits for its process to exit.
    """
    query_analysis_engine.start()

    query_analysis_engine.stop()

    assert query_analysis_engine._process.poll() is not None


def test_analysis_engine_that_cannot_load_does_not_start() -> None:
    """
    An unavailable engine raises an error and leaves no process running.
    """
    analysis_engine = AnalysisEngineProcess(
        analysis_engine="no_such_analysis_engine",
        shows_views=False,
        startup_timeout=timedelta(seconds=5),
    )

    with pytest.raises(AnalysisEngineDidNotStart):
        analysis_engine.start()

    assert analysis_engine._process.poll() is not None
