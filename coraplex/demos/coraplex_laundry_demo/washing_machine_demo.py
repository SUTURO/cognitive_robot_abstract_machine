"""
A robot opens the door of the washing machine in the kitchen environment, simulated in
MuJoCo.

Run it from this folder::

    python washing_machine_demo.py
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import PhysicallySimulatedPlanExecutor, PlanExecutor
from coraplex.robot_plans.actions.core.container import OpenAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.statechart import Statechart
from krrood_spatial_math.spatial_types import (
    Pose,
    RotationMatrix,
    Vector3,
)
from semantic_digital_twin.datastructures.definitions import StaticJointState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    WashingMachine,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World

# %% demonstration


@dataclass(kw_only=True)
class LaundryDemonstration(RobotDemonstration):
    """
    The robot opens the cracked open door of the washing machine by reaching behind it.
    """

    door_crack_angle: float = 0.2
    """
    How far the door of the washing machine stands open when the run starts, in radians:

    wide enough for a finger to fit behind the door at its grip recess.
    """

    standing_distance: float = 0.9
    """
    How far in front of the centre of the washing machine's footprint the robot's base
    starts, facing the machine, in meters.
    """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(world_source=KitchenEnvironment())

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(self.used_robot)

    def build_simulated_world(self) -> World:
        """
        :meth:`RobotDemonstration.build_simulated_world`, with the robot starting
        :attr:`standing_distance` in front of the washing machine, facing it, and its
        arms parked, so the robot does not start with an outstretched arm inside the
        furniture.
        """
        world = self.build_scene()
        standing_pose = Pose.from_xyz_rpy(
            x=-self.standing_distance,
            reference_frame=self._washing_machine(world=world).root,
        )
        robot = replace(
            self.robot_specification(),
            localization_pose=world.transform(
                standing_pose, world.root
            ).homogeneous_matrix,
        ).spawn(world)
        # Tiago starts in a T pose otherwise
        for arm in robot.all_arms:
            arm.get_joint_state_by_type(StaticJointState.PARK).apply_to(world)
        return world

    def is_scene_populated(self, world: World) -> bool:
        return bool(world.get_semantic_annotations_by_type(WashingMachine))

    def populate_scene(self, world: World) -> None:
        """
        Crack the door of the washing machine open by :attr:`door_crack_angle`.
        """
        [door] = self._washing_machine(world=world).doors
        door.root.parent_connection.position = self.door_crack_angle

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [
            RobotAccess(
                robot=world.get_semantic_annotations_by_type(self.used_robot)[0]
            )
        ]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Park the arms and open the door with the arm whose wrist is closest to the door
        handle, by reaching over the rim of the door ring at its grip recess, the
        fingers closing across the ring so one of them ends up behind the door.
        """
        world = context.world
        robot = context.require_extension(extension_type=RobotAccess).robot
        washing_machine = self._washing_machine(world=world)
        [door] = washing_machine.doors
        handle = door.handle.root
        world_P_handle = handle.global_pose.position
        arm = min(
            robot.all_arms,
            key=lambda arm: float(
                arm.tip.global_pose.position.euclidean_distance(world_P_handle)
            ),
        )
        grasp = GraspCandidate(
            door.handle,
            Pose(
                orientation=RotationMatrix.from_vectors(
                    x=Vector3(0, 0, -1, reference_frame=handle),
                    y=Vector3(1, 0, 0, reference_frame=handle),
                ).quaternion,
                reference_frame=handle,
            ),
        )
        statechart = Statechart(context=context)
        statechart.add_node(
            node=Sequence(
                nodes=[
                    ParkArmsAction(arms=robot.all_arms),
                    OpenAction(grasp=grasp, arm=arm),
                    ParkArmsAction(arms=robot.all_arms),
                ]
            )
        )
        return statechart

    @staticmethod
    def _washing_machine(*, world: World) -> WashingMachine:
        """
        :param world: The world holding the washing machine.
        :return: The washing machine.
        """
        [washing_machine] = world.get_semantic_annotations_by_type(WashingMachine)
        return washing_machine


# %% running it


def main() -> None:
    """
    Run the demonstration.
    """
    LaundryDemonstration(
        used_robot=HSRB, executor_type=PhysicallySimulatedPlanExecutor
    ).run()


if __name__ == "__main__":
    main()
