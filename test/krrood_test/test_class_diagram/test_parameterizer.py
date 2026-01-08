
from random_events.set import Set
from krrood.class_diagrams.parameterizer import Parameterizer
from random_events.variable import Continuous, Integer, Symbolic
from ..dataset.example_classes import Position, Orientation, Pose, Atom, Element


def test_parameterizer_with_example_classes():
    """
    Test the Parameterizer on the example dataclasses:
    Position, Orientation, Pose, Atom.

    - Explicitly constructs expected variables for comparison.
    - Verifies variable types, names, and PC construction.
    """
    param = Parameterizer()

    position_variables = param(Position)
    orientation_variables = param(Orientation)
    pose_variables = param(Pose)
    atom_variables = param(Atom)

    expected_position_variables = [
        Continuous("Position.x"),
        Continuous("Position.y"),
        Continuous("Position.z"),
    ]

    expected_orientation_variables = [
        Continuous("Orientation.x"),
        Continuous("Orientation.y"),
        Continuous("Orientation.z"),
        Continuous("Orientation.w"),
    ]

    expected_pose_variables = [
        Continuous("Pose.position.x"),
        Continuous("Pose.position.y"),
        Continuous("Pose.position.z"),
        Continuous("Pose.orientation.x"),
        Continuous("Pose.orientation.y"),
        Continuous("Pose.orientation.z"),
        Continuous("Pose.orientation.w"),
    ]

    expected_atom_variables = [
        Symbolic(
            "Atom.element",
            Set.from_iterable([Element.C, Element.H])
        ),
        Integer("Atom.type"),
        Continuous("Atom.charge"),
    ]

    variables_for_pc = [Continuous(v.name) if isinstance(v, Integer) else v for v in atom_variables]
    pc = param.create_fully_factorized_distribution(variables_for_pc)

    assert position_variables == expected_position_variables
    assert orientation_variables == expected_orientation_variables
    assert pose_variables == expected_pose_variables
    assert [
               (type(v), v.name)
               for v in atom_variables
           ] == [
               (type(v), v.name)
               for v in expected_atom_variables
           ]
    assert set(pc.variables) == set(variables_for_pc)
