"""
The HSR carries a milk carton from the counter to the kitchen island in the lab, in MuJoCo.

Run it from this folder::

    python demo-lab.py
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

import mujoco
import numpy as np
from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import (
    PhysicallySimulatedPlanExecutor,
    RealRobotPlanExecutor,
)
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from cramph.composites import Sequence
from cramph.statechart import Statechart
from krrood.patterns.path_enumeration import PathEnumeration
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from segmind.detectors.agent_event_detector_nodes import GraspDetector
from segmind.detectors.coarse_event_detector_nodes import (
    PickUpDetector,
    PlacingDetector,
)
from segmind.event_segmentation import Segmind
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
    AllowAllCollisions,
)
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Floor, Milk
from semantic_digital_twin.specifications.connections import (
    Connection6DoFSpecification,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.semantic_annotations import (
    SemanticAnnotationWithRootSpecification,
)
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world_description.contact import ContactParameters
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from contextlib import AbstractContextManager

    from coraplex.plans.executors import PlanExecutor
    from cramph.context import ContextExtension, StatechartContext
    from semantic_digital_twin.robots.robot_parts import AbstractRobot
    from semantic_digital_twin.world import World
    from semantic_digital_twin.world_description.world_entity import Body

# %% the demonstration

@dataclass
class LabMilkTransportDemonstration(RobotDemonstration):
    """
    The HSR takes the milk from the counter in the lab, carries it to the kitchen island
    with its arm parked and its torso lowered, and sets it down there.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The robot that carries the milk.
    """

    ros_node_name: str = "hsr_lab_minimal_demo"
    """
    Name of the node this demonstration registers.
    """

    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor
    """
    The executor running the plan, which performs it in the physics of
    :meth:`build_simulation`.
    """

    collision_avoidance: bool = True
    """
    Whether collision avoidance is added to every motion state chart of this run.
    """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(world_source=KitchenEnvironment())

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(
            semantic_annotation_type=self.used_robot, world_T_odom=self.robot_start
        )

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(SceneBody.MILK)

    def acquire_world(self) -> World:
        """
        Obtain the world as :meth:`RobotDemonstration.acquire_world` does, in which the
        HSR's hand and wrist may touch the milk throughout the run, as physics never
        counts it as held, and the whole HSR may touch the lab's floors.
        """
        world = super().acquire_world()
        [robot] = world.get_semantic_annotations_by_type(self.used_robot)
        [arm] = robot.all_arms
        with world.modify_world():
            world.collision_manager.add_ignore_collision_rule(
                AllowAllCollisions(
                    body_group_a=[
                        world.get_body_by_name(HSRBBody.WRIST),
                        *arm.end_effector.bodies_with_collision,
                    ],
                )
            )
            world.collision_manager.add_ignore_collision_rule(
                AllowCollisionBetweenGroups(
                    body_group_a=robot.bodies_with_collision,
                    body_group_b=[
                        floor.root
                        for floor in world.get_semantic_annotations_by_type(Floor)
                    ],
                )
            )
        return world

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        robot = context.require_extension(RobotAccess).robot
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=robot.all_arms),
                ]
            )
        )
        return statechart

def main(
    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor,
    collision_avoidance: bool = True,
    event_segmentation: bool = False,
    debug: bool = False,
    headless: bool = False,
) -> World:
    """
    Run the demonstration.

    :param executor_type: The executor running the plan, which decides whether it drives
        the real robot or simulates it.
    :param collision_avoidance: Whether the statechart avoids collisions.
    :param event_segmentation: Whether SegMind segments the run into events.
    :param debug: Whether to run in debug mode, publishing every copy of the world a
        candidate is tried in.
    :param headless: Whether the physics runs without a viewer.
    :return: The world the milk was carried in.
    """
    return LabMilkTransportDemonstration(
        executor_type=executor_type,
        collision_avoidance=collision_avoidance,
        event_segmentation=event_segmentation,
        debug=debug,
        headless=headless,
    ).run()


if __name__ == "__main__":
    main(executor_type=RealRobotPlanExecutor, collision_avoidance=False)
