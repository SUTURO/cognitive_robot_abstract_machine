from giskardpy.motion_statechart.goals.pick_up import PickUp
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy_ros.python_interface.python_interface import GiskardWrapper
from giskardpy_ros.ros2 import rospy
from pycram_hsrb_demo.setup_real_robot import world_setup_with_test_objects
from semantic_digital_twin.robots.abstract_robot import ParallelGripper

rospy.init_node("pickup_test")
giskard = GiskardWrapper(node_handle=rospy.node)

result = world_setup_with_test_objects()
hsrb_world, robot_view, node, context = (
    result.world,
    result.robot_view,
    result.node,
    result.context,
)
milk = hsrb_world.get_body_by_name("milk")
print(milk)
hand = hsrb_world.get_semantic_annotations_by_type(ParallelGripper)[0]

msc = MotionStatechart()

print(msc)
pickup = PickUp(
    manipulator=hand,
    object_geometry=milk,
)
print(msc)
print(pickup)
msc.add_node(pickup)
print(msc)
msc = msc.add_node(EndMotion.when_true(pickup))
print(msc)
giskard.execute(msc)

rospy.shutdown()
