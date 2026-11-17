"""
An HSRB picks a crumpled T-shirt up off a table by friction alone, holds it clear of the
table, and drops it again, simulated in MuJoCo.

Run it as a script::
    python3 -m experiments.t_shirt_demo.pick_and_drop
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import timedelta

import mujoco
import numpy

from coraplex.plans.context_extensions import RobotAccess
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import (
    MoveFingersToWidthAction,
    ParkArmsAction,
    SetGripperAction,
)
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.monitors import CountSimulationTimeSeconds
from cramph.statechart import Statechart
from experiments.t_shirt_demo.demo import SceneEntity, TShirtOnTableDemonstration
from experiments.t_shirt_demo.exceptions import NoTShirtWithinReach
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.contact import (
    ContactFriction,
    ContactParameters,
)
from semantic_digital_twin.world_description.geometry import Scale

# %% picking it up and dropping it


@dataclass
class TShirtPickAndDropDemonstration(TShirtOnTableDemonstration):
    """
    The T-shirt falls onto the table standing upright and crumples into a pile.

    The HSRB takes the pile by its highest fold within reach, backs away from the table
    with the T-shirt hanging from its gripper, holds it, and opens its gripper to drop
    it.

    The gripper holds the T-shirt by friction alone.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The HSRB, whose hand closes by a single motor.
    """

    ros_node_name: str = "t_shirt_pick_and_drop_demo"
    """
    Name of the node a run registers.
    """

    settling_duration: timedelta = timedelta(seconds=3)
    """
    How long the T-shirt is given to fall onto the table and crumple before the robot
    acts.
    """

    table_distance: float = 0.3
    """
    How far in front of the robot's base the near edge of the table stands, in metres.
    """

    table_scale: Scale = field(default_factory=lambda: Scale(0.9, 1.4, 0.5))
    """
    The depth, width and height of the table.
    """

    t_shirt_drop_distance: float = 0.2
    """
    How far behind the near edge of the table the T-shirt is let fall, in metres.
    """

    t_shirt_drop_height: float = 0.02
    """
    How far above the table top the hem of the upright T-shirt starts, in metres.
    """

    reach_into_table: float = 0.2
    """
    How far behind the near edge of the table the gripper reaches down to the T-shirt,
    in metres.
    """

    lateral_reach: float = 0.15
    """
    How far to either side of the robot the gripper reaches down to the T-shirt, in
    metres.
    """

    grasp_depth: float = 0.03
    """
    How far below the top of the fold it takes the fingertips close, in metres.
    """

    fingertip_friction: float = 2.0
    """
    The sliding friction of the HSRB's fingertips.
    """

    retreat_distance: float = 0.5
    """
    How far the robot backs away from the table with the T-shirt held, in metres.
    """

    holding_duration: float = 5.0
    """
    How long the robot holds the T-shirt clear of the table before dropping it, in
    seconds of simulated time.
    """

    watching_duration: float = 2.0
    """
    How long the T-shirt is given to fall after the gripper opened, in seconds of
    simulated time.
    """

    noslip_iterations: int = 20
    friction_cone: mujoco.mjtCone = mujoco.mjtCone.mjCONE_ELLIPTIC
    frictional_to_normal_impedance_ratio: float = 10.0

    @property
    def world_T_t_shirt(self) -> HomogeneousTransformationMatrix:
        """
        The T-shirt stands upright on its hem above the table, neckline on top and its
        sleeves to the sides, to crumple as it falls.
        """
        hem_x = float(
            self.t_shirt_garment.create_surface(
                HomogeneousTransformationMatrix()
            ).mesh.bounds[1][0]
        )
        return HomogeneousTransformationMatrix.from_xyz_rpy(
            x=self.table_distance + self.t_shirt_drop_distance,
            z=self.table_scale.z + self.t_shirt_drop_height + hem_x,
            pitch=numpy.pi / 2,
        )

    def build_simulated_world(self) -> World:
        """
        :meth:`TShirtOnTableDemonstration.build_simulated_world`, with the HSRB's
        fingertips as grippy as :attr:`fingertip_friction`.
        """
        world = super().build_simulated_world()
        [robot] = world.get_semantic_annotations_by_type(self.used_robot)
        with world.modify_world():
            ContactParameters(
                friction=ContactFriction(sliding=self.fingertip_friction)
            ).apply_to(robot.end_effector.fingertips.both)
        return world

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Park the arm, take the pile by its highest fold within reach, back away from the
        table, hold the T-shirt and drop it.
        """
        world = context.world
        robot = context.require_extension(RobotAccess).robot
        gripper = robot.end_effector
        [arm] = robot.all_arms
        fold = self.highest_fold_within_reach(
            self.simulation.simulator.get_flex_vertex_positions(
                flex_name=SceneEntity.T_SHIRT
            ).result,
            world,
        )
        fingertip_reach = self._fingertip_reach(gripper)
        world_T_table_side = HomogeneousTransformationMatrix(reference_frame=world.root)
        fold_x, fold_y, fold_z = float(fold.x), float(fold.y), float(fold.z)
        above_fold = self._gripper_pose_in(
            world_T_table_side,
            fold_x,
            fold_y,
            fold_z + self.approach_height + fingertip_reach,
            gripper,
        )
        closing_height = max(
            fold_z - self.grasp_depth, self.table_scale.z + self.fingertip_clearance
        )
        retreated_base = (
            robot.root.global_pose.homogeneous_matrix
            @ HomogeneousTransformationMatrix.from_xyz_rpy(x=-self.retreat_distance)
        )
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    ParkArmsAction(arms=robot.all_arms),
                    SetGripperAction(
                        configuration=gripper.default_configuration(GripperState.OPEN)
                    ),
                    self._move_gripper_to(above_fold, arm),
                    self._move_gripper_to(
                        self._gripper_pose_in(
                            world_T_table_side,
                            fold_x,
                            fold_y,
                            closing_height + fingertip_reach,
                            gripper,
                        ),
                        arm,
                    ),
                    MoveFingersToWidthAction(
                        gripper=gripper, width=0.0, squeeze=gripper.squeeze
                    ),
                    CountSimulationTimeSeconds(seconds=self.closing_duration),
                    self._move_gripper_to(above_fold, arm),
                    NavigateAction(target_location=retreated_base.pose),
                    CountSimulationTimeSeconds(seconds=self.holding_duration),
                    SetGripperAction(
                        configuration=gripper.default_configuration(GripperState.OPEN)
                    ),
                    CountSimulationTimeSeconds(seconds=self.watching_duration),
                ]
            )
        )
        return statechart

    def highest_fold_within_reach(
        self, t_shirt_vertices: numpy.ndarray, world: World
    ) -> Point3:
        """
        :param t_shirt_vertices: Where each vertex of the T-shirt lies, one row of world
            coordinates per vertex.
        :param world: The world holding the T-shirt.
        :return: The highest of the vertices within :attr:`reach_into_table` behind the
            near edge of the table and :attr:`lateral_reach` to either side of the robot.
        :raises NoTShirtWithinReach: If no vertex lies there.
        """
        within_reach = t_shirt_vertices[
            (t_shirt_vertices[:, 0] <= self.table_distance + self.reach_into_table)
            & (numpy.abs(t_shirt_vertices[:, 1]) <= self.lateral_reach)
        ]
        if len(within_reach) == 0:
            raise NoTShirtWithinReach(
                t_shirt_name=SceneEntity.T_SHIRT,
                reach_into_table=self.reach_into_table,
                lateral_reach=self.lateral_reach,
            )
        return Point3(
            *within_reach[within_reach[:, 2].argmax()], reference_frame=world.root
        )


# %% running it


def main(headless: bool = False) -> World:
    """
    Run the demonstration.

    :param headless: Whether the physics runs without a viewer.
    :return: The world the T-shirt was dropped in.
    """
    return TShirtPickAndDropDemonstration(headless=headless).run()


if __name__ == "__main__":
    main()
