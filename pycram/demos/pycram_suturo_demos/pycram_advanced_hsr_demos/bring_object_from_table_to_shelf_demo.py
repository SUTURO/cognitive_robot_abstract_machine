import logging
from enum import Enum
from typing import List

import geometry_msgs.msg
from suturo_resources.queries import (
    query_surface_of_most_similar_obj,
    query_get_next_object_euclidean_x_y,
    query_semantic_annotations_on_surfaces,
)

import semantic_digital_twin
from demos.pycram_suturo_demos.helper_methods_and_useful_classes import (
    object_creation,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    attach_object_to_hsrb,
    detach_object_from_hsrb,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.A_robot_setup import (
    robot_setup,
)
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram.datastructures.partial_designator import PartialDesignator
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import nav2_move
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot, real_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    GiskardPickUpActionDescription,
    NavigateActionDescription,
    GiskardPlaceActionDescription,
    LookAtActionDescription,
    MoveTorsoActionDescription,
    MoveTorsoAction,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
    perceive_and_spawn_all_objects,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.semantic_helper_methods import (
    get_object_class_from_string,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.simulation_setup import (
    SetupResult,
)
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootBody,
    HasSupportingSurface,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Cucumber,
    Banana,
    Cola,
    Table,
    Orange,
    Cupboard,
    ShelfLayer,
)
from semantic_digital_twin.spatial_types import Point3, HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale, Color

logger = logging.getLogger(__name__)
logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)

_TABLE_NAME = "dining_table"
_CUPBOARD_NAME = "cupboard_annotation"

_poses = {
    _TABLE_NAME: Pose().from_xyz_quaternion(
        1.3,
        5.3,
        0.0,
        0.0,
        0.0,
        0.75,
        0.67,
    ),
    _CUPBOARD_NAME: Pose().from_xyz_quaternion(
        3.85,
        4.69,
        0.0,
        0.0,
        0.0,
        0.02,
        0.99,
    ),
}

_torso_thresholds = {"high": 2.0, "mid": 1.5}


def pose_to_ros(pose: Pose):
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.pose.position.x = float(pose.x)
    pose_stamped.pose.position.y = float(pose.y)
    pose_stamped.pose.position.z = float(pose.z)
    pose_stamped.pose.orientation.x = float(pose.to_quaternion().x)
    pose_stamped.pose.orientation.y = float(pose.to_quaternion().y)
    pose_stamped.pose.orientation.z = float(pose.to_quaternion().z)
    pose_stamped.pose.orientation.w = float(pose.to_quaternion().w)
    pose_stamped.header.frame_id = pose.reference_frame.name.name
    return pose_stamped


def calc_closest_point_to_robot(
    context: Context, point_a: Point3, points: List[Point3]
) -> Point3:
    min_dist = 100
    min_dist_point = points[0]
    point_a = context.world.transform(point_a, min_dist_point.reference_frame)
    for point in points:
        dist = point_a.euclidean_distance(point)
        min_dist_point = point if dist < min_dist else min_dist_point
        min_dist = min(dist, min_dist)
    return min_dist_point


def get_pose_from_surface(
    context: Context,
    surface: HasSupportingSurface,
    obj: HasRootBody,
    robot_pose: Point3 = None,
) -> Pose:
    points = surface.sample_points_from_surface(obj)
    if robot_pose is None:
        point = points[0] if points else Point3()
    else:
        point = calc_closest_point_to_robot(context, robot_pose, points)
    point.z -= 0.005  # to make sure object is on table
    pose = Pose(position=point, reference_frame=point.reference_frame)
    return pose


def place_object_on_table(
    context: Context,
    obj: HasRootBody,
    surface_to_place_on: HasSupportingSurface,
):
    pose = get_pose_from_surface(
        context,
        surface=surface_to_place_on,
        obj=obj,
        robot_pose=context.robot.root.global_pose.to_position(),
    )
    pose_stamped = PoseStamped.from_spatial_type(pose.to_homogeneous_matrix())
    plan = SequentialPlan(
        context,
        GiskardPlaceActionDescription(
            object_designator=obj.root,
            arm=Arms.LEFT,
            target_location=pose_stamped,
            simulated=False,
        ),
    )
    with real_robot:
        plan.perform()
        detach_object_from_hsrb(
            world=context.world,
            object_designator=obj.root,
        )


def pickup_object_from_table(context: Context, obj: HasRootBody):
    plan = SequentialPlan(
        context,
        GiskardPickUpActionDescription(
            simulated=False,
            object_designator=obj.root,
            arm=Arms.LEFT,
            gripper_vertical=True,
        ),
    )
    with real_robot:
        plan.perform()
        attach_object_to_hsrb(world=context.world, object_designator=obj.root)


def move_to_pose(context: Context, pose: Pose):
    nav2_move.start_nav_to_pose(pose_to_ros(pose))


def look_at_point(context: Context, point: Point3):
    with real_robot:
        SequentialPlan(
            context,
            LookAtActionDescription(
                PoseStamped.from_spatial_type(
                    HomogeneousTransformationMatrix.from_point_rotation_matrix(
                        point=point, reference_frame=point.reference_frame
                    )
                )
            ),
        ).perform()


def look_at_surface(context: Context, surface: HasSupportingSurface):
    look_at_point(context, surface.supporting_surface.global_pose.to_position())


def scan_shelves(context: Context, shelves: List[ShelfLayer]):
    for shelf in shelves:
        z_shelf = float(shelf.global_pose.z)
        if z_shelf <= _torso_thresholds["mid"]:
            move_torso = MoveTorsoActionDescription(TorsoState.LOW)
        elif _torso_thresholds["mid"] <= z_shelf <= _torso_thresholds["high"]:
            move_torso = MoveTorsoActionDescription(TorsoState.MID)
        else:
            move_torso = MoveTorsoActionDescription(TorsoState.HIGH)
        SequentialPlan(context, move_torso).perform()
        perceive_and_spawn_all_objects(context.world)
        objects_on_shelf = query_semantic_annotations_on_surfaces(
            [shelf], context.world
        )
        for obj in objects_on_shelf:
            with context.world.modify_world():
                shelf.add_object(obj)


def main(
    context: Context,
    object_to_pick: str = None,
):
    # Save starting pose to drive to after demo is finished
    STARTING_POSE = context.robot.root.global_pose.to_pose()

    table: Table = context.world.get_semantic_annotation_by_name(_TABLE_NAME)

    move_to_pose(context, _poses[_TABLE_NAME])
    look_at_surface(context, table)
    perceive_and_spawn_all_objects(context.world)

    # Get object to pick
    objs = context.world.get_semantic_annotations_by_type(
        get_object_class_from_string(object_to_pick)
    )
    obj: HasRootBody
    if objs:
        obj = objs[0]
    else:
        raise Exception("Object not found")

    shelf_layers = context.world.get_semantic_annotations_by_type(ShelfLayer)
    pickup_object_from_table(context, obj=obj)

    move_to_pose(context=context, pose=_poses[_CUPBOARD_NAME])
    scan_shelves(context, shelf_layers)

    surface_to_place_on: HasSupportingSurface = query_surface_of_most_similar_obj(
        obj, shelf_layers
    )

    place_object_on_table(
        context,
        obj,
        surface_to_place_on,
    )

    with real_robot:
        SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH)).perform()
    move_to_pose(context=context, pose=STARTING_POSE)
