import pytest
from sqlalchemy import select, text

# The alternative mapping needs to be imported for the stretch to work properly
import pycram.alternative_motion_mappings.stretch_motion_mapping  # type: ignore
import pycram.alternative_motion_mappings.tiago_motion_mapping  # type: ignore
from pycram.datastructures.grasp import GraspDescription
from pycram.motion_executor import simulated_robot
from pycram.orm.ormatic_interface import *  # type: ignore
from krrood.ormatic.dao import to_dao
from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from pycram.datastructures.pose import PoseStamped

from pycram.plans.factories import sequential, execute_single
from pycram.robot_plans.actions.composite.transporting import TransportAction
from pycram.robot_plans.actions.core.navigation import NavigateAction
from pycram.robot_plans.actions.core.pick_up import PickUpAction
from pycram.robot_plans.actions.core.placing import PlaceAction
from pycram.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from semantic_digital_twin.datastructures.definitions import TorsoState


@pytest.fixture()
def simple_plan_fixture(immutable_model_world):
    world, robot_view, context = immutable_model_world

    plan = sequential(
        [
            NavigateAction(
                PoseStamped.from_list([1.6, 1.9, 0], [0, 0, 0, 1], world.root),
                True,
            ),
            MoveTorsoAction(TorsoState.HIGH),
            ParkArmsAction(Arms.BOTH),
        ],
        context=context,
    ).plan
    return plan


def test_pose(pycram_testing_session, simple_plan_fixture):
    session = pycram_testing_session
    plan = simple_plan_fixture
    dao = to_dao(plan)
    session.add(dao)
    session.commit()
    result = session.scalars(select(PyCramPoseDAO)).all()
    assert len(result) == 1


def test_plan_serialization(pycram_testing_session, simple_plan_fixture):
    session = pycram_testing_session
    plan = simple_plan_fixture

    with simulated_robot:
        plan.perform()

    dao = to_dao(plan)
    session.add(dao)
    session.commit()

    result = session.scalars(
        select(ActionNodeDAO).join(NavigateActionDAO, ActionNodeDAO.designator)
    ).all()
    assert all(
        [
            r.execution_data.execution_start_pose is not None
            and r.execution_data.execution_end_pose is not None
            for r in result
        ]
    )

    motions = session.scalars(select(BaseMotionDAO)).all()
    assert len(motions) == 3


@pytest.fixture
def complex_plan(mutable_model_world):
    world, robot_view, context = mutable_model_world

    plan = execute_single(
        TransportAction(
            object_designator=world.get_body_by_name("milk.stl"),
            target_location=PoseStamped.from_list(
                [2.3, 2.5, 1], [0, 0, 0, 1], world.root
            ),
            arm=Arms.LEFT,
        ),
        context=context,
    ).plan

    return plan


def test_manipulated_body_pose(pycram_testing_session, complex_plan):

    with simulated_robot:
        complex_plan.perform()

    session = pycram_testing_session
    plan = complex_plan
    dao = to_dao(plan)
    session.add(dao)
    session.commit()

    pick_up_node = session.scalars(
        select(ActionNodeDAO).join(PickUpActionDAO, ActionNodeDAO.designator)
    ).one()
    place_node = session.scalars(
        select(ActionNodeDAO).join(PlaceActionDAO, ActionNodeDAO.designator)
    ).one()

    assert plan.initial_world is not None
    assert pick_up_node.execution_data is not None
    assert place_node.execution_data is not None


def test_replay_from_db(pycram_testing_session, complex_plan):

    with simulated_robot:
        complex_plan.perform()

    session = pycram_testing_session
    plan = complex_plan
    dao = to_dao(plan)
    session.add(dao)
    session.commit()

    fetched_plan = session.scalars(select(PlanMappingDAO)).one()

    recreated_plan = fetched_plan.from_dao()
