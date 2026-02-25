import os

from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from sqlalchemy import false

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TorsoState, Arms, ApproachDirection, VerticalAlignment
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import MoveTorsoActionDescription, TransportActionDescription, PickUpActionDescription, \
    PlaceActionDescription
from pycram.robot_plans import ParkArmsActionDescription
from pycram.testing import setup_world
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.adapters.ros.tfwrapper import TFWrapper
from semantic_digital_twin.reasoning.world_reasoner import WorldReasoner
from semantic_digital_twin.robots.abstract_robot import ParallelGripper
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.semantic_annotations.semantic_annotations import Bowl, Spoon
from semantic_digital_twin.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
import rclpy

world = setup_world()

rclpy.init()
rclpy_node = rclpy.create_node("viz_maker")

TFPublisher(world=world, node=rclpy_node)
v = VizMarkerPublisher(world, rclpy.create_node("viz_marker"))

spoon = STLParser(
    os.path.join(
        os.path.dirname(__file__), "..", "..", "resources", "objects", "spoon.stl"
    )
).parse()
bowl = STLParser(
    os.path.join(
        os.path.dirname(__file__), "..", "..", "resources", "objects", "bowl.stl"
    )
).parse()

with world.modify_world():
    world.merge_world_at_pose(
        bowl,
        HomogeneousTransformationMatrix.from_xyz_quaternion(
            2.4, 2.2, 1, reference_frame=world.root
        ),
    )
    connection = FixedConnection(
        parent=world.get_body_by_name("cabinet10_drawer_top"), child=spoon.root
    )
    world.merge_world(spoon, connection)


pr2 = PR2.from_world(world)
context = Context.from_world(world)

with world.modify_world():
    world_reasoner = WorldReasoner(world)
    world_reasoner.reason()
    world.add_semantic_annotations(
        [
            Bowl(root=world.get_body_by_name("bowl.stl")),
            Spoon(root=world.get_body_by_name("spoon.stl")),
        ]
    )
gripper = world.get_semantic_annotations_by_type(ParallelGripper)[0]
milk_object = world.get_body_by_name("milk.stl")
milk_pose = PoseStamped.from_spatial_type(milk_object.global_pose.from_xyz_rpy())
print(milk_pose)
grasp = GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, manipulator=gripper, manipulation_offset=False)

plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.HIGH),
    PickUpActionDescription(object_designator=milk_object, arm=Arms.LEFT, grasp_description=grasp),
    # ParkArmsActionDescription(Arms.BOTH),


    #
    PlaceActionDescription(object_designator=milk_object,target_location=milk_pose, arm=Arms.LEFT),

    #TransportActionDescription(
    #    world.get_body_by_name("milk.stl"),
    #    PoseStamped.from_list([4.9, 3.3, 0.8], frame=world.root),
    #    Arms.LEFT,
    #),
)

with simulated_robot:
    plan.perform()
