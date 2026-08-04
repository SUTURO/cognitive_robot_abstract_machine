#!/usr/bin/env python3
from __future__ import annotations

import argparse
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor

from giskardpy.ros2_tools.wrench_compensation_node import COMPENSATED_WRENCH_TOPIC

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, WipeMode
from pycram.motion_executor import real_robot
from pycram.plans.factories import execute_single
from pycram.robot_plans.actions.core.wiping import WipeAction
from pycram.robot_plans.motions.wipe_coverage import Reach
from pycram.robot_plans.motions.wiping import WipeTableMotion

from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    ModelSynchronizer,
    StateSynchronizer,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Vector3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

from suturo_resources.suturo_map import load_environment

GRIPPER_TOOL_FRAME = "hand_gripper_tool_frame"

DEFAULT_SPONGE = (0.08, 0.055, 0.03)
SPONGE_NAME = "wipe_sponge"

# Wipe configuration -- spill/crumb comes from the CLI (--mode); Toya must
# already be positioned in front of the table.
TABLE_NAME = "dining_table"
PRESS_FORCE = 8.0          # N, table frame
AVOID_COLLISIONS = True    # collision avoidance is added inside the motion


def _to_wipe_mode(name: str) -> WipeMode:
    return WipeMode[name.upper()]


def wipe_mode_argument_parser(description: str) -> argparse.ArgumentParser:
    """An argument parser with the shared ``--mode spill|crumb`` option
    (default spill), used by the real-robot script and the RViz demo."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        "--mode",
        type=_to_wipe_mode,
        choices=list(WipeMode),
        default=WipeMode.SPILL,
        metavar="{" + ",".join(mode.name.lower() for mode in WipeMode) + "}",
        help="spill = serpentine absorb, crumb = gather strokes to one pile",
    )
    return parser


def attach_sponge(world: World, sponge_dims: tuple[float, float, float]) -> Body:
    """Bolt a mock sponge to the gripper and return it. The sponge hangs along
    the tool +Z; its frame follows the wipe waypoints and its box sets the lane
    width."""
    tool = world.get_kinematic_structure_entity_by_name(GRIPPER_TOOL_FRAME)
    scale = Scale(*sponge_dims)
    sponge = Body(
        name=PrefixedName(SPONGE_NAME),
        collision=ShapeCollection([Box(scale=scale)]),
        visual=ShapeCollection([Box(scale=scale)]),
    )
    with world.modify_world():
        world.add_connection(
            FixedConnection(
                parent=tool,
                child=sponge,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    z=sponge_dims[2] / 2.0
                ),
            )
        )
    return sponge


def build_wipe_motion(
    world: World,
    surface: Body,
    mode: WipeMode,
    sponge: Body,
    region: tuple[float, float, float, float] | None = None,
    reach: Reach | None = None,
) -> WipeTableMotion:
    """The wipe both the real-robot script and the RViz demo run: collision
    avoidance lives inside the motion and the wrench is read in the robot's FT
    sensor frame. ``region`` confines the wipe to one table strip and ``reach``
    (both from the coverage planner) tapers its lanes to the arm's reach from
    where the base can stand; ``None``/``None`` wipes the whole top with no reach
    limit."""
    return WipeTableMotion(
        table=Table(root=surface),
        arm=Arms.LEFT,
        mode=mode,
        tool=sponge,
        region=region,
        reach=reach,
        desired_force=Vector3(z=PRESS_FORCE),
        avoid_collisions=AVOID_COLLISIONS,
    )


def main() -> None:
    mode = wipe_mode_argument_parser("Wipe a table with the real HSR.").parse_args().mode
    rclpy.init()
    node = rclpy.create_node("wipe_table_real_robot")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True, name="rclpy-spin").start()

    print(f"FT reminder: confirm '{COMPENSATED_WRENCH_TOPIC}' is published and "
          "re-tared (< ~1 N at rest) before continuing.")

    world = fetch_world_from_service(node)
    ModelSynchronizer(_world=world, node=node, synchronous=True)
    StateSynchronizer(_world=world, node=node, synchronous=True)
    with world.modify_world():
        world.merge_world(load_environment())
    robot_view = world.get_semantic_annotations_by_type(HSRB)[0]
    context = Context(world, robot_view, ros_node=node)

    surface = world.get_body_by_name(TABLE_NAME)
    print(f'robot position: {context.robot.root.global_pose.to_position().to_np()}')
    print(f'Table position: {surface.global_pose.to_position().to_np()}')

    sponge = attach_sponge(world, DEFAULT_SPONGE)

    motion = build_wipe_motion(world, surface, mode, sponge)
    print(f"\nexecuting: {mode.name} wipe of '{surface.name}', {PRESS_FORCE} N press.")
    with real_robot:
        execute_single(WipeAction(motion=motion), context=context).perform()
    print("wipe finished.")


if __name__ == "__main__":
    main()
