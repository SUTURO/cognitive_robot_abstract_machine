from __future__ import annotations

import os

from krrood import lubm_with_predicates
from krrood.helpers import evaluate_sparql, make_rdf_graph
from krrood.lubm_sparql_queries import sparql_queries
from krrood.lubm_eql_queries import evaluate_eql, get_eql_queries
from krrood.owl_instances_loader import load_instances


def test_eql_counts_match_sparql():
    instances_path = os.path.join(
        os.path.dirname(__file__), "..", "resources", "lubm_instances.owl"
    )
    instances_path = os.path.abspath(instances_path)

    rdf_graph = make_rdf_graph(instances_path)
    expected = evaluate_sparql(rdf_graph, sparql_queries)

    _ = load_instances(instances_path, lubm_with_predicates)
    actual = evaluate_eql(get_eql_queries())

    # test only the first query for now as the queries of sparql are not correct yet.
    assert actual[0] == expected[0]
