import rclpy

from pycram.datastructures.enums import Arms, VerticalAlignment, ApproachDirection
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    PickUpActionDescription,
    NavigateActionDescription,
    PlaceActionDescription,
)
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.real_setup import (
    world_setup_with_test_objects,
)

result = world_setup_with_test_objects()
world, robot_view, context, node = (
    result.world,
    result.robot_view,
    result.context,
    result.node,
)


# IMPORTANT: giskardpy's ROS2 ActionClient needs a valid rclpy.node.Node.
# PyCRAM passes context.ros_node down into the MotionExecutor.
if getattr(context, "ros_node", None) is None:
    context.ros_node = node
# Some setups name it differently; set it too if present.
if hasattr(context, "node") and getattr(context, "node", None) is None:
    context.node = node

# milk = world.get_body_by_name("milk")
grasp = GraspDescription(
    manipulator=next(iter(robot_view.manipulators)),
    approach_direction=ApproachDirection.FRONT,
    vertical_alignment=VerticalAlignment.NoAlignment,
)
#
# # print(world.get_body_by_name("sofa_body").inertial)
# print(world.bodies)
#
# world_root = getattr(world, "root")
# new_robot_pose = PoseStamped.from_list(
#     [1.6, 2.0, 0], [0, 0, 0.7071068, 0.7071068], frame=world_root
# )
# sofa_body = getattr(world, "get_body_by_name")("sofa_body")
# sofa_pose_pycram = PoseStamped.from_matrix(
#     sofa_body.global_pose.to_np(), frame=world_root
# )
# print(sofa_pose_pycram)
#
# place_pose = PoseStamped.from_list([1.9, 3.3, 1], [0, 0, 1, 0.1], frame=world_root)

viz = VizMarkerPublisher(world=world, node=node)
viz.with_tf_publisher()

# nav.start_nav_to_pose(new_robot_pose),
plan = SequentialPlan(context, ParkArmsActionDescription(Arms.LEFT))
with real_robot:
    plan.perform()
