import logging
from typing import List, Optional

import geometry_msgs.msg
from suturo_resources.queries import (
    query_surface_of_most_similar_obj,
    query_semantic_annotations_on_surfaces,
    query_class_by_label,
)

import semantic_digital_twin
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    attach_object_to_hsrb,
    detach_object_from_hsrb,
)
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.external_interfaces import nav2_move
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    GiskardPickUpActionDescription,
    GiskardPlaceActionDescription,
    LookAtActionDescription,
    MoveTorsoActionDescription,
    GiskardPlaceAndDetachActionDescription,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
    perceive_and_spawn_all_objects,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.hri_handover import (
    handover_human_robot_no_init,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.talking_demo import TtsPublisher
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootBody,
    HasSupportingSurface,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Table,
    ShelfLayer,
)
from semantic_digital_twin.spatial_types import Point3, HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World

logger = logging.getLogger(__name__)
logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)

_TABLE_NAME = "sofa_table"
_CUPBOARD_NAME = "my_new_cabinet"

_poses = {
    _TABLE_NAME: Pose().from_xyz_quaternion(
        2.967,
        -0.034,
        0.0,
        0.0,
        0.0,
        -0.008,
        0.999,
    ),
    _CUPBOARD_NAME: Pose().from_xyz_quaternion(
        1.199,
        3.273,
        0.0,
        0.0,
        0.0,
        -0.999,
        0.027,
    ),
}

_torso_thresholds = {"high": 1.5, "mid": 0.8}

tts = TtsPublisher()


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


def filter_points_full_on_surface(
    points: List[Point3], obj: HasRootBody, surface: HasSupportingSurface
) -> List[Point3]:
    obj_min, obj_max = obj.min_max_points
    surf_min, surf_max = surface.min_max_points

    return [
        point
        for point in points
        if (
            surf_min.x <= point.x + obj_min.x <= surf_max.x
            and surf_min.x <= point.x + obj_max.x <= surf_max.x
            and surf_min.y <= point.y + obj_min.y <= surf_max.y
            and surf_min.y <= point.y + obj_max.y <= surf_max.y
        )
    ]


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


def place_object_on_surface(
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
        GiskardPlaceAndDetachActionDescription(
            object_designator=obj.root,
            arm=Arms.LEFT,
            target_location=pose_stamped,
            simulated=False,
            ignore_orientation=True,
        ),
    )
    with real_robot:
        plan.perform()


def pickup_object_from_table(context: Context, obj: HasRootBody):
    with real_robot:
        return SequentialPlan(
            context,
            GiskardPickUpActionDescription(
                simulated=False,
                object_designator=obj.root,
                arm=Arms.LEFT,
                gripper_vertical=True,
            ),
        ).perform()


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


def look_at_surface(
    context: Context, surface: HasSupportingSurface, offset: Optional[float] = None
):
    if offset is None or offset == 0.0:
        look_at_point(context, surface.global_pose.to_position())
    else:
        look_side_of_surface_middle(context, surface, offset)


def look_side_of_surface_middle(
    context: Context, surface: HasSupportingSurface, offset: float
):
    surface_middle_point = surface.global_pose.to_translation_matrix()
    middle_point_T_robot = context.world.transform(
        surface_middle_point, context.robot.root
    )

    middle_point_T_robot.y += offset
    look_at_point(context, middle_point_T_robot.to_position())


def park_arms(context: Context):
    with real_robot:
        SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH)).perform()


def move_torso(context: Context, state: TorsoState):
    with real_robot:
        SequentialPlan(context, MoveTorsoActionDescription(state)).perform()


def scan_shelves(context: Context, shelves: List[ShelfLayer]):
    for shelf in shelves:
        z_shelf = float(shelf.global_pose.z)

        if z_shelf <= _torso_thresholds["mid"]:
            move_torso(context, TorsoState.LOW)
        elif _torso_thresholds["mid"] <= z_shelf <= _torso_thresholds["high"]:
            move_torso(context, TorsoState.MID)
        else:
            move_torso(context, TorsoState.HIGH)

        look_at_surface(context=context, surface=shelf)
        perceive_and_spawn_all_objects(context.world)

        objects_on_shelf = query_semantic_annotations_on_surfaces(
            [shelf], context.world
        )
        if not objects_on_shelf:
            continue
        for obj in objects_on_shelf:
            with context.world.modify_world():
                shelf.add_object(obj)


def reset_to_start(context: Context, starting_pose: Pose):
    park_arms(context)
    move_to_pose(context=context, pose=starting_pose)


def try_and_scan_for_object_on_table(
    context: Context,
    object_to_pick_type: type,
    from_table: Table,
):
    offset = 0.2
    for try_count in range(3):
        # Look at surface with different offset each try
        look_at_surface(context, from_table, offset=(offset * try_count))

        # Change offset directions each try
        offset = -offset

        perceive_and_spawn_all_objects(context.world)

        # Check if object was spawned
        objs: List[HasRootBody] = query_semantic_annotations_on_surfaces(
            [from_table], context.world
        ).tolist()

        # Object found on table
        if object_to_pick_type in objs:
            return objs[0]

    return None


def try_to_pick_up_else_hri(context: Context, obj: HasRootBody):
    for try_count in range(3):
        did_pick_up = pickup_object_from_table(context, obj)
        if did_pick_up:
            return
        look_at_point(context, obj.global_pose.to_position())
    tts.publish("Could not pick up the object. Manual handover needed.")
    handover_human_robot_no_init(simulated=False, context=context)


def main(
    context: Context,
    object_to_pick: str = None,
):
    # Save starting pose to drive to after demo is finished
    STARTING_POSE = context.robot.root.global_pose.to_pose()

    table: Table = context.world.get_semantic_annotation_by_name(_TABLE_NAME)
    obj_type = query_class_by_label(object_to_pick)

    move_to_pose(context, _poses[_TABLE_NAME])

    tts.publish("I will try to scan for the object on the table.")
    obj = try_and_scan_for_object_on_table(context, obj_type, table)

    if obj is None:
        tts.publish(
            "Object was not found after multiple tries. I will reset to the starting position."
        )
        reset_to_start(context, STARTING_POSE)
        return

    tts.publish("I found the object and will now try to pick it up.")
    try_to_pick_up_else_hri(context=context, obj=obj)

    tts.publish("I will now move to the shelf.")
    move_to_pose(context=context, pose=_poses[_CUPBOARD_NAME])

    shelf_layers = context.world.get_semantic_annotations_by_type(ShelfLayer)
    scan_shelves(context, shelf_layers)

    surface_to_place_on: HasSupportingSurface = query_surface_of_most_similar_obj(
        obj, shelf_layers
    )
    tts.publish("I will try to place the object in the shelf.")
    place_object_on_surface(
        context,
        obj,
        surface_to_place_on,
    )

    move_to_pose(context=context, pose=_poses[_CUPBOARD_NAME])
    park_arms(context)
    move_torso(context, TorsoState.LOW)

    tts.publish("I finished all tasks and will move to the starting position.")
    reset_to_start(context, STARTING_POSE)
    tts.shutdown()
