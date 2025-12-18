import os
from typing import List

from entity_query_language.symbolic import SymbolicExpression, ResultQuantifier
from owlrl import DeductiveClosure, OWLRL_Semantics
from rdflib import Graph

from .owl_to_python import OwlToPythonConverter


def generate_lubm_with_predicates():
    # Provide default overrides for common LUBM datatype properties
    _default_overrides = {
        "Person": {
            "age": "int",
            "telephone": "str",
            "title": "str",
            "email_address": "str",
        },
        "Professor": {
            "tenured": "bool",
        },
        "Publication": {
            "publication_date": "str",
        },
        "Software": {
            "software_version": "str",
        },
        "Thing": {
            "name": "str",
            "office_number": "int",
            "research_interest": "str",
        },
    }
    converter = OwlToPythonConverter(predefined_data_types=_default_overrides)
    resources_path = os.path.join(os.path.dirname(__file__), "..", "..", "resources")
    converter.load_ontology(os.path.join(resources_path, "lubm.owl"))
    # Save into the package module so tests import the updated code
    output_path = os.path.join(os.path.dirname(__file__), "lubm_with_predicates.py")
    converter.save_to_file(output_path)


def make_rdf_graph(instances_path: str):
    g = Graph()
    g.parse(instances_path)
    return g


def evaluate_sparql(rdf_graph: Graph, sparql_queries: List[str]):
    DeductiveClosure(OWLRL_Semantics, rdfs_closure=True, axiomatic_triples=True).expand(
        rdf_graph
    )
    counts: List[int] = []
    for q in sparql_queries:
        res = rdf_graph.query(q)
        counts.append(len(res))
    return counts


def evaluate_eql(eql_queries: List[ResultQuantifier]) -> List[int]:
    """Load instances and evaluate 14 EQL queries, returning counts per query."""
    counts: List[int] = []
    for q in eql_queries:
        result = list(q.evaluate())
        counts.append(len(result))
    return counts
