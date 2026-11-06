"""
The HSR drives in front of the kitchen lab's sofa, looks at each of its seat places and
reports which of them are free.

Run it from this folder::

    python demo.py

Running it as a script drives the real robot and fetches the world from the running world
server. :func:`main` alone runs the whole plan in kinematic simulation, where the people
on the sofa are read from the world model, so nothing on the network is needed.

.. warning:: A real run needs a perception pipeline answering
    :data:`~coraplex.perception.ROBOKUDO_QUERY_ACTION_NAME` as well as the world server:
    every seat place is looked at and the pipeline is asked for the people sitting in it.
    Start the controller and the pipeline separately, as on the robot. RoboKudo's entry
    point takes its arguments with an underscore prefix::

        python3 -m robokudo.scripts.main _ae hsr_free_seats
"""

from __future__ import annotations

from dataclasses import dataclass

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import (
    KinematicallySimulatedPlanExecutor,
    PlanExecutor,
    RealRobotPlanExecutor,
)
from coraplex.robot_plans.actions.core.misc import DetectOccupantsAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.statechart import Statechart
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import Sofa
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World

# %% the demonstration


@dataclass
class FreeSeatsDemonstration(RobotDemonstration):
    """
    The HSR scans the seat places of the kitchen lab's sofa for people sitting on them.

    Afterwards the world holds a
    :class:`~semantic_digital_twin.semantic_annotations.semantic_annotations.Human` on
    every seat place a person was seen on, and none on the others.
    """

    used_robot: type[AbstractRobot] = HSRB

    ros_node_name: str = "hsr_free_seats_demo"

    scanning_distance: float = 1.5
    """
    How far in front of the front edge of the sofa the robot's base stands to scan it,
    in meters.
    """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(world_source=KitchenEnvironment())

    def robot_specification(self) -> RobotSpecification:
        """
        :return: The HSR on free floor of the kitchen lab.
        """
        return RobotSpecification(
            semantic_annotation_type=self.used_robot,
            odom_T_robot_start=HomogeneousTransformationMatrix.from_xyz_rpy(1, 1),
        )

    def is_scene_populated(self, world: World) -> bool:
        return all(
            world.is_kinematic_structure_entity_in_world_by_name(seat.value)
            for seat in SofaSeat
        )

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Raise the torso, park the arm, drive in front of the sofa and look for people on
        each of its seat places.
        """
        world = context.world
        hsrb = context.require_extension(RobotAccess).robot
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                    ParkArmsAction(arms=hsrb.all_arms),
                    NavigateAction(target_location=self.scanning_pose(world)),
                    DetectOccupantsAction(
                        regions=[seat.region_in(world) for seat in SofaSeat]
                    ),
                ]
            )
        )
        return statechart

    def scanning_pose(self, world: World) -> Pose:
        """
        :param world: A world holding the scene.
        :return: Where the robot's base stands on the floor to scan the sofa,
            :attr:`scanning_distance` in front of its front edge and facing it.
        """
        [sofa] = world.get_semantic_annotations_by_type(Sofa)
        sofa_front = (
            sofa.root.collision.as_bounding_box_collection_in_frame(sofa.root)
            .bounding_box()
            .min_x
        )
        world_T_scanning_pose = (
            sofa.root.global_transform
            @ HomogeneousTransformationMatrix.from_xyz_rpy(
                x=sofa_front - self.scanning_distance
            )
        ).pose
        return Pose.from_xyz_rpy(
            x=world_T_scanning_pose.x,
            y=world_T_scanning_pose.y,
            yaw=world_T_scanning_pose.yaw,
            reference_frame=world.root,
        )


def main(
    executor_type: type[PlanExecutor] = KinematicallySimulatedPlanExecutor,
) -> list[SofaSeat]:
    """
    Run the demonstration.

    :param executor_type: The executor running the plan, which decides whether it drives
        the real robot or simulates it.
    :return: The seat places no one was seen sitting on.
    """
    world = FreeSeatsDemonstration(executor_type=executor_type).run()
    return [seat for seat in SofaSeat if seat.is_free_in(world)]


if __name__ == "__main__":
    free_seats = main(executor_type=RealRobotPlanExecutor)
    print("Free seats:", ", ".join(seat.name for seat in free_seats) or "none")
