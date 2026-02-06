import logging
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from suturo_resources.suturo_map import load_environment

from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import NavigateActionDescription
from pycram.ros_utils.text_to_image import TextToImagePublisher
from geometry_msgs.msg import PoseStamped as ROSPoseStamped

from simulation_setup_drive import setup_hsrb_in_environment
from pycram.external_interfaces import nav2_move

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class NavTarget:
    """Human pose in world.root frame (map)."""

    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


def main() -> None:
    """HSRB simulation: navigate to one or more targets sequentially.

    This version mirrors the other simulation demos in `hsrb_simulation/`:
    - build a SemDT world (HSRB + environment)
    - create a PyCRAM Context
    - execute navigation sequentially via `SequentialPlan` (sequential nodes)

    No Nav2 action server is required.
    """

    logging.basicConfig(level=logging.INFO)

    if not rclpy.ok():
        rclpy.init()

    result = setup_hsrb_in_environment(load_environment=load_environment, with_viz=True)
    world = result.world
    context = result.context

    min_distance_to_human_m = 0.5
    rotate_180_after_distance = True

    humans: List[NavTarget] = [
        NavTarget(
            position=(3.8683114051818848, 5.459158897399902, 0.0),
            orientation=(0.0, 0.0, 0.04904329912700753, 0.9987966533838301),
        )
    ]

    # NOTE: SetupResult.world is dynamically typed; use getattr to keep IDE/type-checker happy.
    world_root = getattr(world, "root")

    # NOTE: We need the robot's *current* pose; in this setup the robot root is base_footprint.
    robot_root = getattr(world, "get_body_by_name")("base_footprint")
    robot_pose_pycram = PoseStamped.from_matrix(
        robot_root.global_pose.to_np(), frame=world_root
    )

    text_pub = TextToImagePublisher()
    actions = []

    # Helper: current robot orientation (used to prevent early turning during the approach)
    current_robot_quat = (
        float(robot_pose_pycram.orientation.x),
        float(robot_pose_pycram.orientation.y),
        float(robot_pose_pycram.orientation.z),
        float(robot_pose_pycram.orientation.w),
    )

    for i, human in enumerate(humans, start=1):
        text_pub.publish_text(
            f"[sim] Approaching human {i} (keeping {min_distance_to_human_m}m)"
        )

        human_pose = PoseStamped.from_list(
            position=list(human.position),
            orientation=list(human.orientation),
            frame=world_root,
        )

        # Use helpers from nav2_move (they operate on ROS PoseStamped):
        # 1) compute a target pose that keeps the requested distance to the human
        # 2) optionally rotate the *target* orientation
        human_ros = human_pose.ros_message()
        robot_ros = robot_pose_pycram.ros_message()

        target_ros = nav2_move.min_distance_2_human(
            human_pose=human_ros,
            robot_pose=robot_ros,
            min_distance=min_distance_to_human_m,
        )

        # Step 1: approach position ONLY (keep current orientation so we don't turn early)
        target_pos_pycram = PoseStamped.from_ros_message(target_ros)
        target_pos_pycram.frame_id = world_root
        target_pos_pycram.pose.orientation.x = current_robot_quat[0]
        target_pos_pycram.pose.orientation.y = current_robot_quat[1]
        target_pos_pycram.pose.orientation.z = current_robot_quat[2]
        target_pos_pycram.pose.orientation.w = current_robot_quat[3]
        actions.append(NavigateActionDescription(target_location=target_pos_pycram))

        # Step 2: after arriving, turn in place by 180° using the interface helper.
        if rotate_180_after_distance:
            # Read the *actually reached* base orientation from the world and rotate that.
            robot_root_after = getattr(world, "get_body_by_name")("base_footprint")
            reached_pose_pycram = PoseStamped.from_matrix(
                robot_root_after.global_pose.to_np(), frame=world_root
            )
            reached_pose_pycram.pose.position.x = target_pos_pycram.position.x
            reached_pose_pycram.pose.position.y = target_pos_pycram.position.y
            reached_pose_pycram.pose.position.z = target_pos_pycram.position.z

            target_turn_pycram = _rotate_pose_180_with_interface(
                pose=reached_pose_pycram, frame_id=str(world_root.name)
            )
            target_turn_pycram.frame_id = world_root
            actions.append(
                NavigateActionDescription(
                    target_location=target_turn_pycram, keep_joint_states=True
                )
            )

        # No LookAtActionDescription here on purpose.

        # Update robot pose/orientation for the next iteration (if any)
        robot_root = getattr(world, "get_body_by_name")("base_footprint")
        robot_pose_pycram = PoseStamped.from_matrix(
            robot_root.global_pose.to_np(), frame=world_root
        )
        current_robot_quat = (
            float(robot_pose_pycram.orientation.x),
            float(robot_pose_pycram.orientation.y),
            float(robot_pose_pycram.orientation.z),
            float(robot_pose_pycram.orientation.w),
        )

    plan = SequentialPlan(context, *actions)

    with simulated_robot:
        plan.perform()


def _rotate_pose_180_with_interface(*, pose: PoseStamped, frame_id: str) -> PoseStamped:
    """Rotate a PyCRAM pose by 180° around z using nav2_move.change_orientation."""
    ros_pose: ROSPoseStamped = pose.ros_message()
    ros_pose.header.frame_id = frame_id
    ros_rot = nav2_move.change_orientation(ros_pose)
    out = PoseStamped.from_ros_message(ros_rot)
    return out


if __name__ == "__main__":
    main()
