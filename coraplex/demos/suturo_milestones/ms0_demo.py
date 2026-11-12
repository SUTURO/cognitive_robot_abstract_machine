"""
The real HSR picks up a milk carton in the lab kitchen from where it stands.

Run it from this folder, with the robot's controller running::

    python temp.py
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import RealRobotPlanExecutor
from coraplex.querying.grasp_closeness import GraspsByCloseness
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Sequence
from cramph.statechart import Statechart
from krrood.patterns.path_enumeration import PathEnumeration
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.specifications.connections import (
    Connection6DoFSpecification,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from coraplex.plans.executors import PlanExecutor
    from cramph.context import ContextExtension, StatechartContext
    from semantic_digital_twin.robots.robot_parts import AbstractRobot
    from semantic_digital_twin.world import World

# %% the scene


class SceneFile(PathEnumeration):
    """
    The files the scene is built from, under the coraplex resources.
    """

    MILK = Path(__file__).resolve().parents[2] / "resources" / "objects" / "milk.stl"
    """
    The mesh of the milk carton.
    """


class SceneBody(StrEnum):
    """
    The names of the scene's bodies the demonstration acts on.
    """

    MILK = "milk"
    """
    The milk carton the plan picks up, which marks whether the scene was already
    populated.
    """


# %% the demonstration


@dataclass
class MilkPickUpDemonstration(RobotDemonstration):
    """
    The real HSR parks its arm and picks up the milk from where it stands.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The robot that picks up the milk.
    """

    ros_node_name: str = "hsr_milk_pick_up_demo"
    """
    Name of the node this demonstration registers.
    """

    executor_type: type[PlanExecutor] = RealRobotPlanExecutor
    """
    The executor running the plan, which drives the real robot.
    """

    world_T_milk: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=1.29, y=0.9, z=0.6
        )
    )
    """
    Where the milk stands before the plan begins, in the map frame.
    """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(world_source=KitchenEnvironment())

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(semantic_annotation_type=self.used_robot)

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(SceneBody.MILK)

    def populate_scene(self, world: World) -> None:
        """
        Put the milk at :attr:`world_T_milk`, free to be picked up, see
        :meth:`RobotDemonstration.populate_scene`.
        """
        Milk.get_annotation_specification(
            name=SceneBody.MILK,
            root_specification=BodySpecification.mesh(
                SceneBody.MILK,
                str(SceneFile.MILK),
                parent_T_self=self.world_T_milk,
                connection_specification=Connection6DoFSpecification(),
            ),
        ).spawn(world)

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        robot = context.require_extension(RobotAccess).robot
        [milk] = context.world.get_semantic_annotations_by_type(Milk)
        [arm] = robot.all_arms
        [closest_grasp] = GraspsByCloseness(
            milk, robot.root.global_pose, number_of_grasps=1
        )()
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=robot.all_arms),
                    # PickUpAction(grasp=closest_grasp, arm=arm),
                ]
            )
        )
        return statechart


def main() -> World:
    """
    Run the demonstration.

    :return: The world the milk was picked up in.
    """
    return MilkPickUpDemonstration().run()


if __name__ == "__main__":
    main()
