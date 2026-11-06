"""
The HSR drives in front of the kitchen lab's sofa, looks at each of its seat places and
reports which of them are free.

Run it from this folder::

    python demo.py

Running it as a script drives the real robot and fetches the world from the running world
server. :func:`main` alone runs the whole plan in kinematic simulation, where the people
on the sofa are read from the world model, so nothing on the network is needed.

..warning:: A real run needs a perception pipeline answering
    :data:`~coraplex.perception.ROBOKUDO_QUERY_ACTION_NAME` as well as the world server:
    every seat place is looked at and the pipeline is asked for the people sitting in it.
    Start the controller and the pipeline separately, as on the robot. RoboKudo's entry
    point takes its arguments with an underscore prefix::

        python3 -m robokudo.scripts.main _ae hsr_free_seats
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import ExecutionType
from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.factories import sequential
from coraplex.plans.plan_node import PlanNode
from coraplex.robot_plans.actions.core.misc import DetectOccupantsAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import Sofa
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
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
        """
        :return: A world holding nothing but a map body, which :meth:`populate_scene`
            furnishes.
        """
        return WorldSpecification(world_parser=None)

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

    def populate_scene(self, world: World) -> None:
        """
        Spawn the kitchen lab, with the seat places of its sofa.
        """
        KitchenEnvironment().populate(world)

    def build_context(self, world: World) -> Context:
        """
        Build the plan context around the HSR in ``world``.

        ..note:: The ROS node has to be in the context for a real robot.
        """
        return Context(
            world=world,
            robot=world.get_semantic_annotations_by_type(self.used_robot)[0],
            ros_node=self.ros_node,
            alternative_motion_mappings=self.alternative_motion_mappings,
            _debug=self.debug,
        )

    def build_plan(self, context: Context) -> PlanNode:
        """
        Raise the torso, park the arm, drive in front of the sofa and look for people on
        each of its seat places.
        """
        world = context.world
        return sequential(
            [
                MoveTorsoAction(TorsoState.HIGH),
                ParkArmsAction(context.robot.all_arms),
                NavigateAction(self.scanning_pose(world)),
                DetectOccupantsAction([seat.region_in(world) for seat in SofaSeat]),
            ],
            context=context,
        )

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
            .max_x
        )
        world_T_scanning_pose = (
            sofa.root.global_transform
            @ HomogeneousTransformationMatrix.from_xyz_rpy(
                x=sofa_front + self.scanning_distance, yaw=np.pi
            )
        ).pose
        return Pose.from_xyz_rpy(
            x=world_T_scanning_pose.x,
            y=world_T_scanning_pose.y,
            yaw=world_T_scanning_pose.yaw,
            reference_frame=world.root,
        )


def main(
    execution_type: ExecutionType = ExecutionType.KINEMATICALLY_SIMULATED,
) -> list[SofaSeat]:
    """
    Run the demonstration.

    :param execution_type: Whether to drive the real robot or simulate it.
    :return: The seat places no one was seen sitting on.
    """
    world = FreeSeatsDemonstration(execution_type=execution_type).run()
    return [seat for seat in SofaSeat if seat.is_free_in(world)]


if __name__ == "__main__":
    free_seats = main(execution_type=ExecutionType.REAL)
    print("Free seats:", ", ".join(seat.name for seat in free_seats) or "none")
