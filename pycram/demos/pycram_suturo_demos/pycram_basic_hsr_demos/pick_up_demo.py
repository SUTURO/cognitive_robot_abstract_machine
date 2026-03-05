from suturo_resources.suturo_map import load_environment
from sympy.stats.sampling.sample_numpy import numpy

import semantic_digital_twin
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PoseStamped
import logging

from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    PickUpActionDescription,
    MoveTorsoActionDescription,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.gripper_open_close_demo import (
    GripperActionClient,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.abstract_robot import ParallelGripper, Manipulator
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)

logger = logging.getLogger(__name__)
rclpy_node, world, robot_view, context = setup_hsrb_context()


def add_box(name: str, scale_xyz: tuple[float, float, float]):
    body = Body(
        name=PrefixedName(name),
        collision=ShapeCollection([Box(scale=Scale(*scale_xyz))]),
    )
    new_world = World()
    with new_world.modify_world():
        new_world.add_body(body)
    return new_world


def perceive_and_spawn_all_objects(hsrb_world: World):
    try:
        from pycram.external_interfaces import robokudo
    except ImportError:
        raise ImportError()
    perceived_objects = {}
    perceived_objects_result = robokudo.query_all_objects().res
    for perceived_object in perceived_objects_result:
        object_size = perceived_object.shape_size[0].dimensions
        object_pose = perceived_object.pose[0].pose
        object_time = perceived_object.pose[0].header.stamp
        object_name = f"{perceived_object.type}"
        try:
            object_to_spawn = hsrb_world.get_body_by_name(object_name)
            with hsrb_world.modify_world():
                hsrb_world.move_branch_to_new_world(object_to_spawn)
        except semantic_digital_twin.exceptions.WorldEntityNotFoundError:
            pass
        object_to_spawn = add_box(
            object_name,
            (object_size.x, object_size.y, object_size.z),
        )
        env_world = load_environment()
        perceived_objects[object_name] = object_to_spawn
        with hsrb_world.modify_world():
            hsrb_world.merge_world(env_world)
            hsrb_world.merge_world_at_pose(
                object_to_spawn,
                pose=HomogeneousTransformationMatrix.from_xyz_quaternion(
                    pos_x=object_pose.position.x,
                    pos_y=object_pose.position.y,
                    pos_z=object_pose.position.z,
                    quat_x=object_pose.orientation.x,
                    quat_y=object_pose.orientation.y,
                    quat_z=object_pose.orientation.z,
                    quat_w=object_pose.orientation.w,
                ),
            )
    return perceived_objects


perceive_and_spawn_all_objects()
print(world.bodies)
object_to_pickup = world.get_body_by_name("muesli_vitalis_box_nutmix")
# object_to_pickup = None
manipulator: Manipulator = next(iter(robot_view.manipulators))
manipulator: ParallelGripper


grasp: GraspDescription = GraspDescription(
    approach_direction=ApproachDirection.FRONT,
    vertical_alignment=VerticalAlignment.NoAlignment,
    manipulator=manipulator,
    rotate_gripper=False,
)


action_client = GripperActionClient()


plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.LOW),
    PickUpActionDescription(
        grasp_description=grasp, arm=Arms.LEFT, object_designator=object_to_pickup
    ),
    ParkArmsActionDescription(Arms.BOTH),
)

with real_robot:
    action_client.send_goal(effort=0.8)
    print("now executing")
    print(object_to_pickup.global_pose.to_np())
    print(PoseStamped.from_spatial_type(object_to_pickup.global_pose))
    plan.perform()
    action_client.send_goal(effort=-0.8)
