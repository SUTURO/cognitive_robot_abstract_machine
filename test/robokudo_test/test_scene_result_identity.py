"""
Scene result messages retain the identity of each object hypothesis.
"""

from __future__ import annotations

from unittest.mock import Mock
import pytest
from robokudo.annotators.query import GenerateQueryResult
from robokudo.types.scene import ObjectHypothesis
from robokudo.utils.query import QueryHandler

# %% batch identities


def test_scene_results_preserve_hypothesis_identity(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    Two hypotheses remain distinct even when their classifications are equal.
    """
    hypotheses = [ObjectHypothesis(id="first"), ObjectHypothesis(id="second")]
    cas = Mock(annotations=hypotheses)
    annotator = GenerateQueryResult()
    monkeypatch.setattr(annotator, "get_cas", Mock(return_value=cas))
    monkeypatch.setattr(QueryHandler, "preempt_requested", Mock(return_value=False))
    reply = Mock()
    monkeypatch.setattr(QueryHandler, "send_answer", reply)
    annotator.update()
    assert [item.uid for item in reply.call_args.args[0].res] == [
        str(item.id) for item in hypotheses
    ]
