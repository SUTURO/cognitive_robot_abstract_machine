#!/usr/bin/env python3
from __future__ import annotations

import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor

from giskardpy.motion_statechart.goals.collision_avoidance import (
    ExternalCollisionAvoidance,
    UpdateTemporaryCollisionRules,
)
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.ros2_tools.wrench_compensation_node import COMPENSATED_WRENCH_TOPIC

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, WipeMode
from pycram.plans.factories import execute_single
from pycram.robot_plans.motions.wiping import WipeTableMotion

from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    ModelSynchronizer,
    StateSynchronizer,
)
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
    AvoidExternalCollisions,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Vector3
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

from suturo_resources.suturo_map import load_environment

# The compensated wrench stays in the sensor frame, so the admittance must read
# it there (not the tool tip). Sensor link name on the HSR.
SENSOR_FRAME = "wrist_ft_sensor_frame"
GRIPPER_TOOL_FRAME = "hand_gripper_tool_frame"

DEFAULT_SPONGE = (0.08, 0.055, 0.03)
SPONGE_NAME = "wipe_sponge"
SPONGE_BOTTOM_NAME = "wipe_sponge_bottom"

# Wipe configuration -- edit these instead of passing CLI args. Toya must
# already be positioned in front of the table.
TABLE_NAME = "dining_table"
PRESS_FORCE = 8.0          # N, table frame
CONTACT_THRESHOLD = 3.0    # N that ends the contact-seeking descent
WIPE_PATCH = (0.4, 0.3)    # m, centred sub-rectangle of the table top
APPROACH_HEIGHT = 0.15     # m
WIPE_MODE = WipeMode.SPILL
USE_SPONGE = True          # False -> wipe with the bare gripper tool frame


def _surface_extent(world, body):
    """``(min_x, max_x, min_y, max_y, top_z)`` of the table top, in its frame."""
    boxes = list(body.collision.as_bounding_box_collection_in_frame(body))
    if not boxes:
        raise RuntimeError(f"{body.name} has no collision geometry.")
    return (
        min(b.min_x for b in boxes),
        max(b.max_x for b in boxes),
        min(b.min_y for b in boxes),
        max(b.max_y for b in boxes),
        max(b.max_z for b in boxes),
    )


def attach_sponge(world, sponge_dims):
    """Bolt a mock sponge to the gripper; return ``(sponge, sponge_bottom)``. The
    sponge hangs along the tool +Z so its flat underside is the contact point."""
    tool = world.get_kinematic_structure_entity_by_name(GRIPPER_TOOL_FRAME)
    scale = Scale(*sponge_dims)
    sponge = Body(
        name=PrefixedName(SPONGE_NAME),
        collision=ShapeCollection([Box(scale=scale)]),
        visual=ShapeCollection([Box(scale=scale)]),
    )
    sponge_bottom = Body(name=PrefixedName(SPONGE_BOTTOM_NAME))
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
        world.add_connection(
            FixedConnection(
                parent=tool,
                child=sponge_bottom,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    z=sponge_dims[2]
                ),
            )
        )
    return sponge, sponge_bottom


def centered_region(world, surface, patch):
    """A ``patch`` (px, py) sub-rectangle centred on the table top, in the table
    frame. Bounded so the full-serpentine stays reachable from one base pose."""
    min_x, max_x, min_y, max_y, top_z = _surface_extent(world, surface)
    cx, cy = (min_x + max_x) / 2.0, (min_y + max_y) / 2.0
    px, py = patch
    region = (
        max(min_x, cx - px / 2),
        min(max_x, cx + px / 2),
        max(min_y, cy - py / 2),
        min(max_y, cy + py / 2),
    )
    print(
        f"table top extent (table frame): x[{min_x:.2f},{max_x:.2f}] "
        f"y[{min_y:.2f},{max_y:.2f}] top_z={top_z:.3f}\nwipe region: {region}"
    )
    return region


def build_msc(goal, robot_view, world, sponge, surface):
    """WipeGoal + collision rules + EndMotion. WipeGoal adds its own FT node.
    The table is the one surface the sponge and hand are allowed to touch."""
    gripper = [b for b in world.bodies_with_collision if "hand_" in str(b.name)]
    tool_group = gripper + ([sponge] if sponge is not None else [])
    rules = [
        AvoidExternalCollisions(robot=robot_view),
        AllowCollisionBetweenGroups(body_group_a=tool_group, body_group_b=[surface]),
    ]
    if sponge is not None:
        # The sponge is bolted to the arm; never avoid it touching the arm.
        rules.append(
            AllowCollisionBetweenGroups(
                body_group_a=[sponge], body_group_b=list(robot_view.bodies_with_collision)
            )
        )
    msc = MotionStatechart()
    msc.add_node(UpdateTemporaryCollisionRules(temporary_rules=rules))
    msc.add_node(ExternalCollisionAvoidance(robot=robot_view))
    msc.add_node(goal)
    msc.add_node(EndMotion.when_true(goal))
    return msc


def main():
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

    sponge = sponge_bottom = None
    if USE_SPONGE:
        sponge, sponge_bottom = attach_sponge(world, DEFAULT_SPONGE)

    region = centered_region(world, surface, WIPE_PATCH)
    wrist = world.get_kinematic_structure_entity_by_name(SENSOR_FRAME)

    motion = WipeTableMotion(
        table=Table(root=surface),
        arm=Arms.LEFT,
        mode=WIPE_MODE,
        tool=sponge,
        tool_contact_frame=sponge_bottom,
        region=region,
        stroke_sample_count=6,
        approach_height=APPROACH_HEIGHT,
        wipe_threshold=0.04,
        desired_force=Vector3(z=PRESS_FORCE),
        contact_force_threshold=CONTACT_THRESHOLD,
        force_torque_reference_frame=wrist,
    )
    execute_single(motion, context=context)
    goal = motion.motion_chart
    msc = build_msc(goal, robot_view, world, sponge, surface)

    print(f"\nexecuting: wipe '{surface.name}', {PRESS_FORCE} N press, region {region}.")

    from giskardpy_ros.python_interface.python_interface import GiskardWrapper

    GiskardWrapper(node).execute(msc)
    print("wipe finished.")


if __name__ == "__main__":
    main()
