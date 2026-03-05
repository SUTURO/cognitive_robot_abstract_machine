from sqlalchemy.orm import sessionmaker

from krrood.entity_query_language.backends import (
    SQLAlchemyBackend,
    PythonBackend,
    ProbabilisticBackend,
)
from krrood.entity_query_language.factories import (
    variable,
    entity,
    an,
    underspecified,
)
from krrood.ormatic.dao import to_dao
from krrood.parametrization.model_registries import DictRegistry
from krrood.parametrization.parameterizer import UnderspecifiedParameters
from probabilistic_model.probabilistic_circuit.rx.helper import fully_factorized
from ..dataset.example_classes import Pose, Position, Orientation


def test_same_query_multiple_backends(session, database):

    p1 = Pose(position=Position(1, 0, 0), orientation=Orientation(0, 0, 0, 1))
    p2 = Pose(position=Position(0, 1, 0), orientation=Orientation(0, 0, 0, 1))

    python_domain = [p1, p2]

    daos = [to_dao(p1), to_dao(p2)]
    session.add_all(daos)
    session.commit()
    session_maker = sessionmaker(session.bind)

    pose_variable = variable(Pose, python_domain)

    q = an(
        entity(pose_variable).where(
            pose_variable.position.x > 0.5,
        )
    )

    python_backend = PythonBackend()
    result = list(python_backend.evaluate(q))
    assert len(result) == 1

    database_backend = SQLAlchemyBackend(session_maker)
    result = list(database_backend.evaluate(q))
    assert len(result) == 1

    prob_q = underspecified(Pose)(
        position=underspecified(Position)(x=..., y=..., z=...),
        orientation=Orientation(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    prob_q.where(prob_q.variable.position.x > 0.5)

    parameters = UnderspecifiedParameters(prob_q)
    model = fully_factorized(parameters.random_event_variables)

    registry = DictRegistry({Pose: model})

    pm_backend = ProbabilisticBackend(registry, 10)
    values = list(pm_backend.evaluate(prob_q))
    for value in values:
        assert value.position.x > 0.5

    assert pm_backend.number_of_samples == len({v.position for v in values})
