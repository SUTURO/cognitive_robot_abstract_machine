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
from coraplex.ros_scene_perception import RoboKudoScenePerception
from coraplex.scene_perception import SceneObservationNode, SceneObserver
from cramph.composites import ChildChooserAccess, Sequence
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


class MilkClass(StrEnum):
    """
    The classes of RoboKudo's RoboCup model that the Suturo type mapping counts as
    :class:`Milk`.
    """

    BUTTERMILK_MUELLER_ORIGINAL = "buttermilk_mueller_bottle_original"
    BUTTERMILK_MUELLER_RASPBERRY = "buttermilk_mueller_bottle_raspberry"
    BAERENMARKE = "milk_baerenmarke_pack"
    HANSANO = "milk_hansano_pack"
    JA = "milk_ja_pack"
    JUMBO_HALFVOLL = "milk_jumbo_pack_halfvoll"
    JUMBO_VOLL = "milk_jumbo_pack_voll"
    MILBONA = "milk_milbona_pack"
    WEIHENSTEPHAN = "milk_weihenstephan_pack"
    MILSANI_LACTOSEFREE = "milk_milsani_pack_lactosefree"


# %% the demonstration


@dataclass
class MilkPickUpDemonstration(RobotDemonstration):
    """
    The real HSR parks its arm, then asks RoboKudo what its camera sees, keeps
    everything RoboKudo answers and moves the milk to where RoboKudo saw one, as steps
    of one plan.
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

    scene_observation: SceneObservationNode | None = field(init=False, default=None)
    """
    The step of the plan asking RoboKudo what the camera sees, whose
    :attr:`~SceneObservationNode.observation` holds the answer once it ran.
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
        scene_observation = SceneObservationNode()
        self.scene_observation = scene_observation
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=robot.all_arms),
                    scene_observation,
                    # PickUpAction(grasp=closest_grasp, arm=arm),
                ]
            )
        )
        return statechart

    def create_executor(self, world: World) -> PlanExecutor:
        """
        Differs from :meth:`RobotDemonstration.create_executor` in letting RoboKudo
        answer every :class:`SceneObservationNode` of the plan, which moves the milk of
        the world to where RoboKudo saw a milk.
        """
        executor = super().create_executor(world)
        robokudo = RoboKudoScenePerception(ros_node=self.ros_node)
        executor.context.require_extension(ChildChooserAccess).choosers.append(
            SceneObserver(
                source=robokudo,
                annotation_types_by_category={
                    milk_class: Milk for milk_class in MilkClass
                },
            )
        )
        return executor

    def print_observed_objects(self) -> None:
        """
        Print every object RoboKudo answered :attr:`scene_observation` with, with its
        class, position, orientation and size, as far as RoboKudo reported them.
        """
        observation = self.scene_observation.observation
        if observation is None:
            print("RoboKudo was not asked, the plan ended before.")
            return
        print(f"RoboKudo reported {len(observation.objects)} objects:")
        for observed_object in observation.objects:
            print(f"- {observed_object.category or 'unclassified'}")
            if observed_object.pose is not None:
                x, y, z, _ = observed_object.pose.position.to_np()
                qx, qy, qz, qw = observed_object.pose.quaternion.to_np()
                frame = observed_object.pose.reference_frame.name
                print(f"    position in {frame}: x={x:.3f} y={y:.3f} z={z:.3f}")
                print(f"    orientation: x={qx:.3f} y={qy:.3f} z={qz:.3f} w={qw:.3f}")
            if observed_object.dimensions is not None:
                size = observed_object.dimensions
                print(f"    size: {size.x:.3f} x {size.y:.3f} x {size.z:.3f} m")


def main() -> World:
    """
    Run the demonstration.

    :return: The world the milk was picked up in.
    """
    demonstration = MilkPickUpDemonstration()
    world = demonstration.run()
    demonstration.print_observed_objects()
    return world


if __name__ == "__main__":
    main()
