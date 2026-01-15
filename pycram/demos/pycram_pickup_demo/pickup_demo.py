import threading
import time
import os
import logging
from typing import List

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor

from demos.pycram_multiverse_demo.demo import pick_up_loc
from pycram.failures import PlanFailure
from pycram.ros import sleep
from suturo_resources.suturo_map import load_environment

from pycram.datastructures.enums import (
    TorsoState,
    Arms,
    ApproachDirection,
    VerticalAlignment, GripperState,
)
from pycram.datastructures.grasp import GraspDescription
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot, real_robot
from semantic_digital_twin.adapters.ros.world_fetcher import (
    FetchWorldServer,
    fetch_world_from_service,
)
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    Synchronizer,
    ModelSynchronizer,
    StateSynchronizer,
)
from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from pycram.robot_plans import (
    SimplePouringActionDescription,
    MoveTorsoActionDescription,
    PickUpAction,
    PickUpActionDescription, MoveGripperMotion,
)
from pycram.robot_plans import ParkArmsActionDescription
from pycram.datastructures.dataclasses import Context
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.robots.abstract_robot import ParallelGripper
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
import pycram.alternative_motion_mappings.hsrb_motion_mapping

from semantic_digital_twin.world import World
from pycram.external_interfaces.robokudo import query_object

from pycram.robot_plans.actions.core.pick_up import PickUpAction

from pycram.ros_utils import text_to_image

logger = logging.getLogger(__name__)
rclpy.init()
# TODO:
"""
    FIRST: Demo start through NLP
    SECOND: Let ansgar spawn objects + show how
    THIRD: check if object believed is also percieved, if not - Fallback
    FOURTH: Pick up object and handle the errors - If cool also try to reach for the closest and so on...
    FIFTH: Be happy, yippie we did it

Error-handling:
[] check if object is in reach
[] check if object is graspable
[x] check if object is in proper orientation 
"""

def _here(*parts: str) -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), *parts))

def create_worlds_stl(name: str) -> World:
    STLParser(
        os.path.join(
            os.path.dirname(__file__), "..", "..", "resources", "objects", name
        )
    ).parse()

def setup():
    node = rclpy.create_node("viz_marker")
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-executor")
    thread.start()
    time.sleep(0.1)

    # world from giskard
    hsrb_world = fetch_world_from_service(node)
    model_sync = ModelSynchronizer(world=hsrb_world, node=node)
    state_sync = StateSynchronizer(world=hsrb_world, node=node)
    return hsrb_world, node, executor, model_sync, state_sync

hsrb_world, node, executor, model_sync, state_sync = setup()

env_world = load_environment()

# spawn STL objects
bowl_world = create_worlds_stl("bowl.stl")
milk_world = create_worlds_stl("milk.stl")

# add them to giskard world
# FYI, x coordinates sind immer die breite Seite
with hsrb_world.modify_world():
    hsrb_world.merge_world(env_world)
    hsrb_world.merge_world_at_pose(
        milk_world,
        pose=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.9, y=5.7, z=0.78, yaw=np.pi / 2 # Rotation for the milk, since the coordinate system is right-handed
        ),
    )
    # hsrb_world.merge_world_at_pose(
    #     milk_world,
    #     pose=HomogeneousTransformationMatrix.from_xyz_rpy(x=1.38, y=3.5, z=0.74),
    # )

VizMarkerPublisher(hsrb_world, node) # publish to RVIZ

context = Context(
    hsrb_world, hsrb_world.get_semantic_annotations_by_type(HSRB)[0], ros_node=node
)

gripper = hsrb_world.get_semantic_annotations_by_type(ParallelGripper)[0]
# grasp=gripper.front_facing_orientation


#          .calculate_grasp_orientation(gripper.front_facing_orientation.to_np()))
#
# print(grasp)

# to grasp object possible: milk, cup, bowl
# TODO: if possition is out of reach, ask for human support
obj_name = "milk.stl"

# TODO: CHECK IF OBJECT PERCIEVED
"""
example output of perception:
robokudo_msgs.action.Query_Result(text_result='', 
    res=[robokudo_msgs.msg.ObjectDesignator(uid='', type='', shape=[], shape_size=[], color=[], location='', size='', 
    pose=[geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header
        (stamp=builtin_interfaces.msg.Time(sec=1738235652, nanosec=442297928), frame_id='map'),
    pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=-0.2789910781040086, y=-0.025133508365441593, z=0.7800704898020974), 
    orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))], 
    pose_source=['ClusterPoseBBAnnotator'], 
    attribute=[], 
    description=['mug_red_metal']), 
"""
obj_perception = query_object(obj_name) # why tf does this return a dictionary, its like a little confused thingy lol

"""
example ouput Knowledge:
[(Body(name=PrefixedName(name='banana_body', prefix=None), 
        id=UUID('38035ca9-40fb-4336-9997-c44388a7a006'), index=35, 
        collision_config=CollisionCheckingConfig(buffer_zone_distance=None, 
        violated_distance=0.0, disabled=None, max_avoided_bodies=1), temp_collision_config=None, 
        inertial=Inertial(mass=1.0, center_of_mass=Point3(casadi_sx=SX(@1=0, [@1, @1, @1, 1]), reference_frame=None), 
        inertia=InertiaTensor(data=array([[1., 0., 0.],
       [0., 1., 0.],
       [0., 0., 1.]])))), Scalar()),
"""
obj_knowledge = hsrb_world.get_body_by_name(obj_name) # Those guys have their transformation matrix, no pose to be read without butt pain

# Im not sure if this scuffed stuff works, buut we´ll find out ig
obj_yaw = obj_perception.pose.pose.orientation.z

obj_graspable = False
while not obj_graspable:
    # Check if the object is sideways (approx 90 degrees/pi/2)
    if np.isclose(abs(obj_yaw), np.pi / 2, atol=20 * (np.pi / 180)):
        # The object is sideways -> Ask for help
        text_pub = text_to_image.TextToImagePublisher()
        text_pub.publish_text("I am unable to grasp the object, please position it properly")
        # Wait for the human to fix it before checking again
        time.sleep(5)
        obj_perception = query_object(obj_name)
        obj_yaw = obj_perception.pose.pose.orientation.z
    else:
        # For ALL other orientations (including 0, pi, etc.), accept the grasp
        grasp = GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)
        obj_graspable = True

# for arm_chain in self.robot_view.manipulator_chains:

# TODO execute NLP to start PICKUP_demo
if obj_perception == obj_knowledge:
    plan = SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.BOTH),
        # \MoveTorsoActionDescription(TorsoState.HIGH),
        # PouringActionDescription(world.get_body_by_name("milk.stl")),
        MoveGripperMotion(motion=GripperState.OPEN, gripper=Arms.LEFT),
        PickUpActionDescription(object_designator=hsrb_world.get_body_by_name(obj_name), arm=Arms.LEFT, grasp_description=grasp),
        MoveGripperMotion(motion=GripperState.CLOSE, gripper=Arms.LEFT)
    )
else:
    plan = SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.BOTH),
        # \MoveTorsoActionDescription(TorsoState.HIGH),
        # PouringActionDescription(world.get_body_by_name("milk.stl")),
        MoveGripperMotion(motion=GripperState.OPEN, gripper=Arms.LEFT),
        print("obj. not found")
    )
# )
# SimplePouringActionDescription(hsrb_world.get_body_by_name("bowl.stl"), Arms.LEFT),

with real_robot:
    plan.perform()
