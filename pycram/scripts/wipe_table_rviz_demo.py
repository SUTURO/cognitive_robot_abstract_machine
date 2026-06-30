    #!/usr/bin/env python3
"""
Watch a :class:`WipeTableMotion` run in RViz, in the SUTURO lab environment.

Loads the lab (``suturo_resources.suturo_map.load_environment``), drops an HSR
in front of the lab counter, puts a box on it, and wipes a reachable patch of
the counter *around* that box. The counter (top ~0.545 m) is much lower than the
lab table, so the short HSR can lean over and reach more of it. The gripper holds
a mock sponge and points straight down, so the sponge's flat bottom rides on the
counter top. Where the serpentine would cross the box it is split into separate
segments, so the tool lifts and hops over the object rather than dragging
through it.

The counter is over 2 m wide, too large to cover from one base pose, so the wipe
is bounded to a reachable ``region`` near the front edge.

A Hunt-Crossley contact model injects the wrench the admittance reacts to (the
same trick the unit tests use), so the sponge descends until it touches the
surface, yields under admittance while wiping, then retracts.

Run it::

    python pycram/scripts/wipe_table_rviz_demo.py --mode spill
    python pycram/scripts/wipe_table_rviz_demo.py --mode crumb --loop

Then in RViz:
    1. Fixed Frame -> ``root``.
    2. Add a TF display.
    3. Add a MarkerArray display on topic ``/semworld/viz_marker`` with QoS
       Durability ``Transient Local``.
"""

from __future__ import annotations

import argparse
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import SingleThreadedExecutor

from giskardpy.executor import SimulationPacer
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.goals.collision_avoidance import (
    ExternalCollisionAvoidance,
    UpdateTemporaryCollisionRules,
)
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceTorqueSymbolNode,
)
from giskardpy.ros_executor import Ros2Executor

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, WipeMode
from pycram.plans.factories import execute_single
from pycram.robot_plans.motions.wiping import WipeTableMotion

from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
    AvoidExternalCollisions,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Vector3
from semantic_digital_twin.world_description.connections import (
    FixedConnection,
    OmniDrive,
)
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# Contact model: penetration depth -> upward wrench, so the descent stops on
# contact and the admittance has something to yield against.
CONTACT_STIFFNESS_BASE = 2000.0  # N / m
CONTACT_VELOCITY_GAIN = 4000.0  # extra N/m per m/s of tip speed

# Surface to wipe. The countertop (top at ~0.545 m) is much lower than the lab
# table (~0.845 m), so the short HSR can lean over and reach more of it.
SURFACE_BODY_NAME = "counterTop"

# Toya stands on the open floor on the +y (room) side of the counter and faces
# it (-y). The counter is yaw ~6 deg off axis; the omni base yaw is a controlled
# DOF, so it squares up on its own from this roughly-facing start.
HSR_BASE_POSE = HomogeneousTransformationMatrix.from_xyz_rpy(
    x=2.40, y=-0.75, z=0.0, yaw=-math.pi / 2
)

# Reachable patch of the counter to wipe, in the counter frame (the near strip
# along the +y front edge, which sits at counter-frame y ~ +0.33). Lower surface
# -> a deeper, wider patch than on the tall table.
WIPE_REGION = (-0.25, 0.35, -0.05, 0.30)

# Box obstacle sitting on the counter, in the counter frame, inside the patch.
# Kept shorter than APPROACH_HEIGHT so the tool clears it when it lifts around.
OBSTACLE_LOCAL_XY = (0.05, 0.12)
OBSTACLE_SCALE = Scale(0.08, 0.08, 0.08)

# Mock sponge held in the gripper. It hangs along the gripper's approach axis
# (local +Z), so with the gripper pointing down its flat bottom is the lowest
# point -- that bottom is what tracks and presses onto the table surface.
SPONGE_SCALE = Scale(0.10, 0.07, 0.05)  # x, y across the tool; z = protrusion
SPONGE_NAME = "wipe_sponge"
SPONGE_BOTTOM_NAME = "wipe_sponge_bottom"

# Lift height for approach/retract and for hopping over the obstacle. Above the
# obstacle height so the lift-around actually clears the box.
APPROACH_HEIGHT = 0.15

# Slightly loose wipe threshold so the sequence advances past corners that the
# keep-pointing-down pose can't quite reach.
WIPE_THRESHOLD = 0.04  # m


def _build_hsr_for_merge() -> Body:
    """Bare HSR under an ``odom_combined`` omni-drive, ready to merge into the
    lab world (which provides the fixed ``root`` frame)."""
    urdf_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "resources",
        "robots",
        "hsrb.urdf",
    )
    world = URDFParser.from_file(file_path=urdf_path).parse()
    with world.modify_world():
        odom = Body(name=PrefixedName("odom_combined"))
        world.add_kinematic_structure_entity(odom)
        world.add_connection(
            OmniDrive.create_with_dofs(parent=odom, child=world.root, world=world)
        )
    return world


def _add_box_on_table(world, table_body: Body, name: str, local_x: float, local_y: float) -> Body:
    """Drop a box onto the table top (table frame) and return it."""
    boxes = list(table_body.collision.as_bounding_box_collection_in_frame(table_body))
    top_z = max(box.max_z for box in boxes)
    half_height = OBSTACLE_SCALE.z / 2
    box = Body(
        name=PrefixedName(name),
        collision=ShapeCollection([Box(scale=OBSTACLE_SCALE)]),
        visual=ShapeCollection([Box(scale=OBSTACLE_SCALE)]),
    )
    with world.modify_world():
        world.add_connection(
            FixedConnection(
                parent=table_body,
                child=box,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=local_x, y=local_y, z=top_z + half_height
                ),
            )
        )
    return box


def _attach_sponge(world):
    """Bolt a mock sponge under the gripper tool frame and return
    ``(sponge_body, sponge_bottom_frame)``.

    The sponge hangs along the tool's approach axis (local +Z): its top face is
    at the tool frame and it protrudes by ``SPONGE_SCALE.z``. ``sponge_bottom``
    is a collision-less frame at that bottom face -- the wipe drives it onto the
    table, so the sponge's underside is what rides the surface."""
    tool_frame = world.get_kinematic_structure_entity_by_name(
        "hand_gripper_tool_frame"
    )
    protrusion = SPONGE_SCALE.z
    sponge = Body(
        name=PrefixedName(SPONGE_NAME),
        collision=ShapeCollection([Box(scale=SPONGE_SCALE)]),
        visual=ShapeCollection([Box(scale=SPONGE_SCALE)]),
    )
    sponge_bottom = Body(name=PrefixedName(SPONGE_BOTTOM_NAME))
    with world.modify_world():
        world.add_connection(
            FixedConnection(
                parent=tool_frame,
                child=sponge,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    z=protrusion / 2.0
                ),
            )
        )
        world.add_connection(
            FixedConnection(
                parent=tool_frame,
                child=sponge_bottom,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    z=protrusion
                ),
            )
        )
    return sponge, sponge_bottom


def _table_top_world_z(world, table_body: Body) -> float:
    boxes = list(table_body.collision.as_bounding_box_collection_in_frame(table_body))
    top_in_table_frame = max(box.max_z for box in boxes)
    table_z = float(world.compute_forward_kinematics_np(world.root, table_body)[2, 3])
    return table_z + top_in_table_frame


def _make_wrench(force_z: float) -> WrenchStamped:
    msg = WrenchStamped()
    msg.wrench.force.z = force_z
    return msg


def _inject_wrench(node: ForceTorqueSymbolNode, force_z: float) -> None:
    # Bypass the ROS subscription by setting the name-mangled latest msg.
    setattr(node, "_TopicSubscriberNode__last_msg", _make_wrench(force_z))


def _spin_in_background(ros_node):
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-spin")
    thread.start()
    return executor, thread


def _build_lab_scene():
    """Lab world + HSR (sponge in gripper) + a box on the lab table. Returns
    ``(world, robot_view, table_body, obstacle_body, sponge_body,
    sponge_bottom)``."""
    from suturo_resources import suturo_map

    world = suturo_map.load_environment()
    world.merge_world_at_pose(_build_hsr_for_merge(), HSR_BASE_POSE)
    robot_view = HSRB.from_world(world)

    table_body = world.get_body_by_name(PrefixedName(SURFACE_BODY_NAME))
    if table_body is None or not table_body.has_collision():
        raise RuntimeError(
            f"Lab surface '{SURFACE_BODY_NAME}' not found / has no collision geometry."
        )

    sponge_body, sponge_bottom = _attach_sponge(world)
    obstacle_body = _add_box_on_table(
        world, table_body, "wipe_obstacle", *OBSTACLE_LOCAL_XY
    )
    return world, robot_view, table_body, obstacle_body, sponge_body, sponge_bottom


def _build_goal(
    world, robot_view, table_body, obstacle_body, sponge_body, sponge_bottom, mode,
    lane_spacing,
):
    motion = WipeTableMotion(
        table=Table(root=table_body),
        arm=Arms.LEFT,
        mode=mode,
        # The sponge sizes the lane spacing and margins; its underside is the
        # frame that rides the surface. lane_spacing=None lets it derive.
        tool=sponge_body,
        tool_contact_frame=sponge_bottom,
        lane_spacing=lane_spacing,
        stroke_sample_count=6,
        region=WIPE_REGION,
        obstacles=[obstacle_body],
        approach_height=APPROACH_HEIGHT,
        wipe_threshold=WIPE_THRESHOLD,
        # Press into the surface so the sponge holds contact height during the
        # wipe; express the wrench in the world root for a clean +z contact.
        desired_force=Vector3(z=8.0),
        force_torque_reference_frame=world.root,
    )
    execute_single(motion, context=Context(world, robot_view))
    return motion.motion_chart


def _box_world_aabb(world, box_body: Body):
    """World-frame axis-aligned bounding box of ``box_body`` as
    ``(min_x, max_x, min_y, max_y, min_z, max_z)``."""
    boxes = list(box_body.collision.as_bounding_box_collection_in_frame(world.root))
    return (
        min(b.min_x for b in boxes),
        max(b.max_x for b in boxes),
        min(b.min_y for b in boxes),
        max(b.max_y for b in boxes),
        min(b.min_z for b in boxes),
        max(b.max_z for b in boxes),
    )


def _point_penetration_into_aabb(point, aabb) -> float:
    """How far ``point`` (x, y, z) is *inside* the AABB, or 0 if outside."""
    min_x, max_x, min_y, max_y, min_z, max_z = aabb
    x, y, z = point
    if not (min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z):
        return 0.0
    return min(x - min_x, max_x - x, y - min_y, max_y - y, z - min_z, max_z - z)


def _gripper_down_error(world, tool_frame) -> float:
    """Angle in rad between the gripper's approach axis (tool +Z) and straight
    down (root -Z). 0 means the gripper points exactly at the table."""
    rotation = world.compute_forward_kinematics_np(world.root, tool_frame)[:3, :3]
    approach_world = rotation @ np.array([0.0, 0.0, 1.0])
    cos_angle = float(np.clip(approach_world @ np.array([0.0, 0.0, -1.0]), -1.0, 1.0))
    return float(np.arccos(cos_angle))


def _run_wipe(
    world,
    robot_view,
    table_body,
    obstacle_body,
    sponge_body,
    sponge_bottom,
    goal,
    ros_node,
    tick_budget,
):
    """Tick the wipe motion, injecting the contact wrench each step, until the
    motion ends or the budget runs out."""
    root = world.root
    # The sponge bottom is what the wipe drives onto the surface, so contact is
    # measured there (not at the bare tool frame, which is a sponge-length up).
    tip = sponge_bottom
    tool_frame = world.get_kinematic_structure_entity_by_name(
        "hand_gripper_tool_frame"
    )
    table_top_z = _table_top_world_z(world, table_body)
    gripper_bodies = [
        body for body in world.bodies_with_collision if "hand_" in str(body.name)
    ]
    robot_bodies = list(robot_view.bodies_with_collision)

    msc = MotionStatechart()
    msc.add_node(goal.ft_node)
    msc.add_node(
        UpdateTemporaryCollisionRules(
            temporary_rules=[
                # Avoid everything external (incl. the box on the table)...
                AvoidExternalCollisions(robot=robot_view),
                # ...except the table top, which the hand and sponge ride on.
                AllowCollisionBetweenGroups(
                    body_group_a=gripper_bodies + [sponge_body],
                    body_group_b=[table_body],
                ),
                # The sponge is bolted to the gripper, so never treat the sponge
                # touching the arm it hangs from as a collision to avoid.
                AllowCollisionBetweenGroups(
                    body_group_a=[sponge_body],
                    body_group_b=robot_bodies,
                ),
            ]
        )
    )
    msc.add_node(ExternalCollisionAvoidance(robot=robot_view))
    msc.add_node(goal)
    msc.add_node(EndMotion.when_true(goal))

    executor = Ros2Executor(
        context=MotionStatechartContext(world=world),
        ros_node=ros_node,
        pacer=SimulationPacer(0.01),
    )
    executor.compile(motion_statechart=msc)
    control_dt = executor.context.qp_controller_config.control_dt

    box_aabb = _box_world_aabb(world, obstacle_body)
    worst_box_penetration = 0.0

    previous_tip_z = None
    for tick in range(tick_budget):
        time.sleep(0.01)
        tip_world = world.compute_forward_kinematics_np(root, tip)[:3, 3]
        tip_z = float(tip_world[2])
        if previous_tip_z is None:
            tip_speed = 0.0
        else:
            tip_speed = abs(tip_z - previous_tip_z) / control_dt
        previous_tip_z = tip_z

        penetration = max(0.0, table_top_z - tip_z)
        effective_stiffness = CONTACT_STIFFNESS_BASE + CONTACT_VELOCITY_GAIN * tip_speed
        _inject_wrench(goal.ft_node, effective_stiffness * penetration)

        # Once in the wiping phase, the sponge should never enter the box.
        worst_box_penetration = max(
            worst_box_penetration,
            _point_penetration_into_aabb(tip_world, box_aabb),
        )

        executor.tick()
        if msc.is_end_motion():
            print(f"Wipe finished after {tick + 1} ticks.")
            break
    else:
        print(f"Wipe did not finish within {tick_budget} ticks.")

    print(
        f"  sponge bottom into box: {100 * worst_box_penetration:.1f} cm (worst); "
        f"gripper-down error at end: "
        f"{math.degrees(_gripper_down_error(world, tool_frame)):.1f} deg."
    )
    return msc.is_end_motion()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=[m.name.lower() for m in WipeMode],
        default="spill",
        help="Wipe strategy: spill (serpentine absorb) or crumb (gather to a pile).",
    )
    parser.add_argument(
        "--lane-spacing",
        type=float,
        default=None,
        help="Distance between strokes, m. Default: derived from the sponge width.",
    )
    parser.add_argument(
        "--tick-budget", type=int, default=6000, help="Max ticks per wipe."
    )
    parser.add_argument(
        "--loop", action="store_true", help="Repeat the wipe until Ctrl-C."
    )
    args = parser.parse_args()
    mode = WipeMode[args.mode.upper()]

    rclpy.init()
    ros_node = rclpy.create_node("wipe_table_rviz_demo")
    spin_executor, spin_thread = _spin_in_background(ros_node)

    try:
        (
            world,
            robot_view,
            table_body,
            obstacle_body,
            sponge_body,
            sponge_bottom,
        ) = _build_lab_scene()

        VizMarkerPublisher(node=ros_node, _world=world).with_tf_publisher()
        print(
            "Publishing TF + markers. In RViz: Fixed Frame = root, add a TF "
            "display and a MarkerArray on /semworld/viz_marker (QoS Durability "
            "= Transient Local). Toya holds the 'wipe_sponge' and cleans the "
            "counter around the box 'wipe_obstacle'."
        )
        time.sleep(1.0)  # give RViz a moment to subscribe before motion starts

        while True:
            goal = _build_goal(
                world,
                robot_view,
                table_body,
                obstacle_body,
                sponge_body,
                sponge_bottom,
                mode,
                args.lane_spacing,
            )
            _run_wipe(
                world,
                robot_view,
                table_body,
                obstacle_body,
                sponge_body,
                sponge_bottom,
                goal,
                ros_node,
                args.tick_budget,
            )
            if not args.loop:
                break
            time.sleep(1.0)

        print("Done. Ctrl-C to exit (keeps publishing so RViz holds the pose).")
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        spin_executor.shutdown()
        spin_thread.join(timeout=2.0)
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
