"""
The real HSR parks its arm, is driven to the cooking table of the lab by Pumas, looks at
it and reports whether RoboKudo sees the milk there.

Start AMCL and Pumas on the map ``apartment_master1``, Giskard and RoboKudo, each in its
own terminal, then run the demo from this folder::

    source ~/ros_clean_env.sh
    python -m robokudo.scripts.main _ae yolo_object_detection  # RoboKudo
    python demo.py
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

import numpy as np

from coraplex.demonstrations import RobotDemonstration
from coraplex.perception import PerceptionQuery, RoboKudoPerception
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import RealRobotPlanExecutor
from coraplex.robot_plans.actions.core.navigation import LookAtAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Sequence
from cramph.statechart import Statechart
from krrood.patterns.path_enumeration import PathEnumeration
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from pumas_navigation import PumasNavigation
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
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from coraplex.plans.executors import PlanExecutor
    from cramph.context import ContextExtension, StatechartContext
    from cramph.node import StatechartNode
    from semantic_digital_twin.robots.robot_parts import AbstractRobot
    from semantic_digital_twin.world import World
    from semantic_digital_twin.world_description.world_entity import Body

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
    The milk carton the robot looks for, which marks whether the scene was already
    populated.
    """

    COOKING_TABLE = "cooking_table"
    """
    The table of the kitchen map, in the room beside the office, the robot drives to and
    looks at.
    """


# %% the report


class MilkSentence(StrEnum):
    """
    The English sentences saying whether the robot saw the milk.
    """

    SEEN = "I see the milk."
    """
    The milk was seen on the cooking table.
    """

    NOT_SEEN = "I found nothing."
    """
    No milk was seen on the cooking table.
    """


@dataclass
class MilkReport:
    """
    What the robot saw of the milk on the cooking table.
    """

    milk_poses: list[Pose]
    """
    Where milk was seen, in the map frame; empty if none was seen.
    """

    @property
    def sentence(self) -> MilkSentence:
        """
        :return: The sentence saying whether milk was seen.
        """
        return MilkSentence.SEEN if self.milk_poses else MilkSentence.NOT_SEEN


# %% the demonstration


@dataclass
class MilkPickUpDemonstration(RobotDemonstration):
    """
    The real HSR parks its arm, is driven to the cooking table by Pumas, looks at its
    middle and reports whether RoboKudo sees the milk there.

    Giskard commands the base on every control cycle of a run, so Pumas only drives
    between the two runs of Giskard.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The robot that looks for the milk.
    """

    ros_node_name: str = "hsr_milk_pick_up_demo"
    """
    Name of the node this demonstration registers.
    """

    executor_type: type[PlanExecutor] = RealRobotPlanExecutor
    """
    The executor sending the arm and head motions to Giskard.
    """

    map_T_scene: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.44, y=0.14, yaw=-0.058
        )
    )
    """
    Where the kitchen lies in the map frame of the laser map ``apartment_master1``, as
    the kitchen to sofa demo fitted it to three room corners clicked on the laser map.
    """

    world_T_milk: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=1.28, y=5.99, z=0.73
        )
    )
    """
    Where the milk stands in the world model, in the frame of the kitchen: on the middle
    of the cooking table.
    """

    viewing_distance: float = 1.0
    """
    How far in front of the front edge of the cooking table the robot's base stands to
    look at it, in meters.
    """

    search_height: float = 0.5
    """
    How far above the cooking table milk is looked for, in meters.
    """

    milk_report: MilkReport | None = field(init=False, default=None)
    """
    What the robot saw of the milk in the last run, None before the first run.
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

    def perform_plan(self, world: World) -> None:
        """
        Park the arm with :meth:`build_statechart`, let Pumas drive the base to
        :meth:`viewing_pose`, look at the cooking table with
        :meth:`build_viewing_statechart` and keep what RoboKudo sees of the milk there
        in :attr:`milk_report`.
        """
        self.execute_statechart(world, self.build_statechart)
        PumasNavigation(
            node=self.ros_node,
            robot=world.get_semantic_annotations_by_type(self.used_robot)[0],
        ).drive_to(self.viewing_pose(self.cooking_table(world)))
        self.execute_statechart(world, self.build_viewing_statechart)
        self.milk_report = self.look_for_milk(world)

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        :return: A statechart parking the arm before the robot drives.
        """
        return self.statechart_of(
            context,
            [
                ParkArmsAction(
                    arms=context.require_extension(RobotAccess).robot.all_arms
                )
            ],
        )

    def build_viewing_statechart(self, context: StatechartContext) -> Statechart:
        """
        :param context: The context the statechart is built in.
        :return: A statechart looking at the middle of the cooking table.
        """
        return self.statechart_of(
            context,
            [LookAtAction(target=self.cooking_table(context.world).global_pose)],
        )

    @staticmethod
    def statechart_of(
        context: StatechartContext, nodes: list[StatechartNode]
    ) -> Statechart:
        """
        :param context: The context the statechart is built in.
        :param nodes: The nodes to perform one after the other.
        :return: A statechart performing ``nodes`` in sequence.
        """
        statechart = Statechart(context=context)
        statechart.add_node(Sequence(nodes=nodes))
        return statechart

    @staticmethod
    def cooking_table(world: World) -> Body:
        """
        :param world: The world built from the kitchen map.
        :return: The body of the cooking table.
        """
        return world.get_kinematic_structure_entity_by_name(SceneBody.COOKING_TABLE)

    def viewing_pose(self, table: Body) -> Pose:
        """
        :param table: The table to look at, whose front edge is its side towards its -y
            axis.
        :return: Where the robot's base stands to look at ``table``: centred
            :attr:`viewing_distance` in front of its front edge, facing it.
        """
        table_front = (
            table.collision.as_bounding_box_collection_in_frame(table)
            .bounding_box()
            .min_y
        )
        world_T_viewing_pose = (
            table.global_transform
            @ HomogeneousTransformationMatrix.from_xyz_rpy(
                y=table_front - self.viewing_distance, yaw=np.pi / 2
            )
        ).pose
        return Pose.from_xyz_rpy(
            x=world_T_viewing_pose.x,
            y=world_T_viewing_pose.y,
            yaw=world_T_viewing_pose.yaw,
            reference_frame=world_T_viewing_pose.reference_frame,
        )

    def look_for_milk(self, world: World) -> MilkReport:
        """
        :param world: The world the robot looks at the cooking table in.
        :return: Where RoboKudo sees milk on the cooking table.
        """
        query = PerceptionQuery(
            Milk,
            self.search_region(world),
            world.get_semantic_annotations_by_type(self.used_robot)[0],
            world,
        )
        return MilkReport(
            milk_poses=[
                world.transform(detection.pose, world.root)
                for detection in RoboKudoPerception(ros_node=self.ros_node).look_for(
                    query
                )
            ]
        )

    def search_region(self, world: World) -> VolumetricBoundingBox:
        """
        :param world: The world built from the kitchen map.
        :return: The space from the plate of the cooking table up to
            :attr:`search_height` above it.
        """
        table = self.cooking_table(world)
        table_box = table.collision.as_bounding_box_collection_in_frame(
            table
        ).bounding_box()
        return VolumetricBoundingBox(
            origin=HomogeneousTransformationMatrix(reference_frame=table),
            min_x=table_box.min_x,
            min_y=table_box.min_y,
            min_z=table_box.min_z,
            max_x=table_box.max_x,
            max_y=table_box.max_y,
            max_z=table_box.max_z + self.search_height,
        )


# %% running it


def main() -> MilkReport:
    """
    Run the demonstration and print what the robot saw of the milk.

    :return: What the robot saw of the milk.
    """
    demonstration = MilkPickUpDemonstration()
    demonstration.run()
    report = demonstration.milk_report
    print(report.sentence, flush=True)
    for milk_pose in report.milk_poses:
        print(
            f"Milk seen at x={milk_pose.x} y={milk_pose.y} z={milk_pose.z}", flush=True
        )
    return report


if __name__ == "__main__":
    main()
