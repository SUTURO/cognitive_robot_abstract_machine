#!/usr/bin/env python3
"""RViz preview of the real-robot table wipe.

Runs the same wipe as ``wipe_table_real_robot.py`` -- the motion comes from the
shared :func:`wipe_table_real_robot.build_wipe_motion`, so sponge, collision
rules, measured-reach cut and all wipe parameters are identical -- with the
real-robot pieces swapped for simulation:

- the world is built locally (lab map + HSR merged at ``HSR_BASE_POSE``, drive
  marked as controlled like on the real robot) instead of fetched from Toya,
- the motion statechart is ticked locally with a Hunt-Crossley contact model
  injecting the wrench in the FT sensor frame (the stand-in for the wrench
  compensation node) instead of being shipped to the Giskard server,
- TF + markers are published for RViz.

Run it::

    python pycram/scripts/wipe_table_rviz_demo.py [--mode spill|crumb] [--loop]

Then in RViz: Fixed Frame -> ``root``, add a TF display and a MarkerArray on
``/semworld/viz_marker`` (QoS Durability ``Transient Local``).
"""

from __future__ import annotations

import math
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import MarkerArray

from giskardpy.executor import SimulationPacer
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.goals.templates import Sequence
from giskardpy.motion_statechart.goals.wipe_goals import WipeGoal
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy.ros_executor import Ros2Executor
from semantic_digital_twin.robots.abstract_robot import ForceTorqueSensor

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import WipeMode
from pycram.plans.factories import execute_single
from pycram.robot_plans.motions.wipe_coverage import WipeCoveragePlanner, WipePass
from pycram.robot_plans.motions.wipe_markers import WipeMarkers

from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import OmniDrive
from semantic_digital_twin.world_description.world_entity import Body

from suturo_resources.suturo_map import load_environment

from wipe_table_real_robot import (
    DEFAULT_SPONGE,
    PRESS_FORCE,
    attach_sponge,
    build_wipe_motion,
    wipe_mode_argument_parser,
)

TABLE_NAME = "dining_table"

HSR_BASE_POSE = HomogeneousTransformationMatrix.from_xyz_rpy(
    x=3.40, y=6.05, z=0.0, yaw=-np.pi
)

TICK_BUDGET = 20000
# Contact model: penetration depth -> upward wrench, so the descent stops on
# contact and the admittance has something to yield against.
CONTACT_STIFFNESS_BASE = 2000.0  # N / m
CONTACT_VELOCITY_GAIN = 4000.0  # extra N/m per m/s of tip speed

# Topic for the wipe preview markers (frontier + trajectory), kept off the
# world's /semworld/viz_marker so the leading DELETEALL only clears the wipe.
WIPE_MARKER_TOPIC = "/wipe/markers"


def _build_hsr_for_merge() -> World:
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


def _build_lab_scene() -> tuple[World, HSRB, OmniDrive]:
    """The stand-in for ``fetch_world_from_service`` + synchronizers: the lab
    map with an HSR merged in at ``HSR_BASE_POSE``."""
    world = load_environment()
    world.merge_world_at_pose(_build_hsr_for_merge(), HSR_BASE_POSE)
    robot_view = HSRB.from_world(world)
    # The real drive is software-controlled. Without this flag the collision
    # rules discard every base contact as an ignorable adjacency, so the base
    # would neither avoid the counter nor count for the reach measurement.
    drive = [c for c in world.connections if isinstance(c, OmniDrive)][0]
    drive.has_hardware_interface = True
    return world, robot_view, drive


def _place_base(world: World, drive: OmniDrive, base_pose) -> None:
    """Drive the base to a world-frame pose -- the sim stand-in for navigating
    Toya to the pose a coverage pass wipes from."""
    world_T_odom = world.compute_forward_kinematics_np(world.root, drive.parent)
    drive.origin = np.linalg.inv(world_T_odom) @ base_pose.to_np()


def _table_top_world_z(world: World, table_body: Body) -> float:
    boxes = list(table_body.collision.as_bounding_box_collection_in_frame(table_body))
    top_in_table_frame = max(box.max_z for box in boxes)
    table_z = float(world.compute_forward_kinematics_np(world.root, table_body)[2, 3])
    return table_z + top_in_table_frame


def _inject_contact_wrench(
    sensor: ForceTorqueSensor, world: World, upward_force: float
) -> None:
    """Feed the contact model's world +z force to the wipe, expressed in the
    sensor's frame -- the same frame the wrench compensation node publishes in
    on the real robot -- by writing it straight into the sensor's live wrench."""
    world_R_sensor = world.compute_forward_kinematics_np(world.root, sensor.root)[
        :3, :3
    ]
    force = world_R_sensor.T @ np.array([0.0, 0.0, upward_force])
    sensor.write_wrench(force, np.zeros(3))


def _run_wipe(world: World, goal: WipeGoal, ros_node: Node, tick_budget: int) -> None:
    """Tick the wipe locally, injecting the contact wrench each step -- the
    stand-in for the Giskard-server execution of the real script. The
    statechart mirrors ``MotionExecutor.construct_msc``."""
    waypoints = [point for points in goal.segments for point in points]
    if not waypoints:
        print("no reachable waypoints -- skipping execution.")
        return
    tip = goal.tip_link
    table_top_z = _table_top_world_z(world, waypoints[0].reference_frame)
    force_torque_sensor = ForceTorqueSensor.for_tip(world, tip)

    setup_start = time.perf_counter()
    msc = MotionStatechart()
    sequence = Sequence(nodes=[goal])
    msc.add_node(sequence)
    msc.add_node(EndMotion.when_true(sequence))
    after_msc = time.perf_counter()

    executor = Ros2Executor(
        context=MotionStatechartContext(
            world=world,
            qp_controller_config=QPControllerConfig(
                target_frequency=50, prediction_horizon=4, verbose=False
            ),
        ),
        ros_node=ros_node,
        pacer=SimulationPacer(1),
    )
    after_executor = time.perf_counter()
    executor.compile(motion_statechart=msc)
    after_compile = time.perf_counter()
    control_dt = executor.context.qp_controller_config.control_dt
    print(
        f"  [profile] {len(waypoints)} waypoints | msc {1000 * (after_msc - setup_start):.0f} ms | "
        f"executor {1000 * (after_executor - after_msc):.0f} ms | "
        f"compile {1000 * (after_compile - after_executor):.0f} ms"
    )

    first_tick_start = time.perf_counter()
    previous_tip_z = None
    for tick in range(tick_budget):
        time.sleep(0.01)
        tip_pose = world.compute_forward_kinematics_np(world.root, tip)
        tip_z = float(tip_pose[2, 3])
        if tick % 1000 == 999:
            tilt = math.degrees(
                math.acos(float(np.clip(-tip_pose[2, 2], -1.0, 1.0)))
            )
            print(
                f"tick {tick + 1}: tip at {np.round(tip_pose[:3, 3], 3)}, "
                f"tool tilt {tilt:.1f} deg"
            )
        tip_speed = (
            0.0 if previous_tip_z is None else abs(tip_z - previous_tip_z) / control_dt
        )
        previous_tip_z = tip_z

        penetration = max(0.0, table_top_z - tip_z)
        stiffness = CONTACT_STIFFNESS_BASE + CONTACT_VELOCITY_GAIN * tip_speed
        _inject_contact_wrench(force_torque_sensor, world, stiffness * penetration)

        tick_call_start = time.perf_counter()
        executor.tick()
        if tick == 0:
            print(
                f"  [profile] teleport->first-tick-return "
                f"{1000 * (time.perf_counter() - first_tick_start):.0f} ms "
                f"(first executor.tick() {1000 * (time.perf_counter() - tick_call_start):.0f} ms)"
            )
        if msc.is_end_motion():
            print(f"wipe finished after {tick + 1} ticks.")
            return
    print(f"wipe did not finish within {tick_budget} ticks.")


def _plan_and_preview(
    world: World,
    drive: OmniDrive,
    surface: Body,
    passes: list[WipePass],
    context: Context,
    mode: WipeMode,
    sponge: Body,
    wipe_markers: WipeMarkers,
) -> tuple[list[tuple[WipePass, WipeGoal]], MarkerArray]:
    """Build each pass's wipe from its own base pose and return the goals plus
    one accumulated marker array previewing every pass's trajectory at once."""
    built: list[tuple[WipePass, WipeGoal]] = []
    combined = MarkerArray()
    for index, wipe_pass in enumerate(passes):
        _place_base(world, drive, wipe_pass.base_pose)
        motion = build_wipe_motion(
            world, surface, mode, sponge,
            region=wipe_pass.region, reach=wipe_pass.reach,
        )
        execute_single(motion, context=context)
        goal = motion.motion_chart
        built.append((wipe_pass, goal))
        pass_markers = wipe_markers.build(
            goal.segments, namespace_suffix=f"_{index}", clear=index == 0,
        )
        combined.markers.extend(pass_markers.markers)
    return built, combined


def main() -> None:
    parser = wipe_mode_argument_parser("Preview the real-robot table wipe in RViz.")
    parser.add_argument("--loop", action="store_true", help="repeat until Ctrl-C")
    arguments = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("wipe_table_rviz_demo")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True, name="rclpy-spin").start()

    world, robot_view, drive = _build_lab_scene()
    context = Context(world, robot_view, ros_node=node)

    surface = world.get_body_by_name(TABLE_NAME)
    print(f'robot position: {context.robot.root.global_pose.to_position().to_np()}')
    print(f'Table position: {surface.global_pose.to_position().to_np()}')

    sponge = attach_sponge(world, DEFAULT_SPONGE)

    VizMarkerPublisher(node=node, _world=world).with_tf_publisher()
    wipe_marker_publisher = node.create_publisher(
        MarkerArray,
        WIPE_MARKER_TOPIC,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    wipe_markers = WipeMarkers()
    print(
        "Publishing TF + markers. In RViz: Fixed Frame = root, add a TF display, "
        "a MarkerArray on /semworld/viz_marker for the scene and another on "
        f"{WIPE_MARKER_TOPIC} for the reach frontier + wipe trajectory (QoS "
        "Durability = Transient Local)."
    )
    time.sleep(1.0)  # give RViz a moment to subscribe before motion starts

    table = Table(root=surface)
    try:
        while True:
            passes = WipeCoveragePlanner(
                world, table, robot_view, reach_margin=-0.03
            ).plan()
            print(f"reachable from {len(passes)} side(s); wiping each.")
            built, combined_markers = _plan_and_preview(
                world, drive, surface, passes, context, arguments.mode,
                sponge, wipe_markers,
            )
            wipe_marker_publisher.publish(combined_markers)
            for index, (wipe_pass, goal) in enumerate(built):
                _place_base(world, drive, wipe_pass.base_pose)
                print(f"\npass {index + 1}/{len(built)}: {arguments.mode.name} wipe of "
                      f"'{surface.name}' strip {wipe_pass.region}, "
                      f"{PRESS_FORCE} N press.")
                _run_wipe(world, goal, node, TICK_BUDGET)
            if not arguments.loop:
                break
            time.sleep(1.0)
        print("Done. Ctrl-C to exit (keeps publishing so RViz holds the pose).")
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
