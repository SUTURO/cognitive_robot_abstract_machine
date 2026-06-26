from giskardpy.middleware.ros2 import rospy
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.monitors.monitors import LocalMinimumReached
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceTorqueSymbolNode,
)
from giskardpy.motion_statechart.tasks.admittance_tasks import (
    AdmittanceCartesianPosition,
)
from giskardpy_ros.python_interface.python_interface import GiskardWrapper

from semantic_digital_twin.spatial_types import Point3, Vector3

rospy.init_node("admittance_demo")

giskard = GiskardWrapper(node_handle=rospy.node)

tip = giskard.world.get_kinematic_structure_entity_by_name("hand_gripper_tool_frame")
root = giskard.world.root
ft_sensor_frame = giskard.world.get_kinematic_structure_entity_by_name(
    "wrist_ft_sensor_frame"
)

goal = Point3(x=0.0, y=0.0, z=0.5, reference_frame=root)

ft_node = ForceTorqueSymbolNode(
    reference_frame=ft_sensor_frame,
    topic_name="/wrist_wrench/compensated",
)

cart_goal = AdmittanceCartesianPosition(
    root_link=root,
    tip_link=tip,
    goal_point=goal,
    ft_node=ft_node,
    stiffness=Vector3(50, 50, 50),
)

msc = MotionStatechart()
msc.add_node(ft_node)
msc.add_node(cart_goal)
msc.add_node(EndMotion.when_true(cart_goal))

giskard.execute(msc)
