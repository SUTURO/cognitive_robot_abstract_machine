import os

from krrood import lubm_with_predicates
from krrood.helpers import evaluate_eql
from krrood.lubm_eql_queries import get_eql_queries
from krrood.owl_instances_loader import load_instances

if __name__ == "__main__":
    # load instances
    _ = load_instances(
        os.path.join("..", "resources", "lubm_instances.owl"), lubm_with_predicates
    )
    evaluate_eql(get_eql_queries())
