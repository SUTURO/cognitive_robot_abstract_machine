"""
Tests for :class:`robokudo.analysis_engine_process.AnalysisEngineProcess`.
"""

from __future__ import annotations

from datetime import timedelta

import pytest
from rclpy.action import ActionClient
from rclpy.node import Node
from robokudo_msgs.action import Query

from robokudo.analysis_engine_process import AnalysisEngineProcess
from robokudo.exceptions import AnalysisEngineDidNotStart

# %% starting and stopping


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
        analysis_engine="no_such_analysis_engine", shows_views=False
    )

    with pytest.raises(AnalysisEngineDidNotStart):
        analysis_engine.start(timeout=timedelta(seconds=5))

    assert analysis_engine._process.poll() is not None
