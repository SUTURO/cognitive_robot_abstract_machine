import logging
import math
from typing import List, Optional, Type

import rclpy
from sqlalchemy.dialects.oracle.dictionary import all_objects
from suturo_resources.queries import (
    query_semantic_annotations_on_surfaces,
    query_surface_of_most_similar_obj,
    query_get_next_object_euclidean_x_y,
    query_annotations_by_color,
)
from sympy import false

import semantic_digital_twin
from demos.pycram_suturo_demos.helper_methods_and_useful_classes import (
    object_creation,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    attach_object_to_hsrb,
    detach_object_from_hsrb,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)
from krrood.symbolic_math.symbolic_math import Scalar, FloatVariable
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.designators.location_designator import CostmapLocation
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    GiskardPickUpActionDescription,
    NavigateActionDescription,
    GiskardPlaceActionDescription,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.abstract_robot import AbstractRobot
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
)
from semantic_digital_twin.spatial_types import Point3
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale, Color
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation

logger = logging.getLogger(__name__)
logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)


rclpy.init()
SIMULATED = True
result = robot_setup(simulation=SIMULATED, with_simulated_objects=false)
rclpy_node, world, robot_view, context = (
    result.node,
    result.world,
    result.robot_view,
    result.context,
)


def calc_closest_point_to_robot(point_a: Point3, points: List[Point3]) -> Point3:
    min_dist = 100
    min_dist_point = points[0]
    point_a = world.transform(point_a, min_dist_point.reference_frame)
    for point in points:
        dist = point_a.euclidean_distance(point)
        min_dist_point = point if dist < min_dist else min_dist_point
        min_dist = min(dist, min_dist)
    return min_dist_point


def get_pose_from_surface(
    surface: HasSupportingSurface,
    obj: HasRootBody,
    category_of_interest: Optional[Type[SemanticAnnotation]] = None,
    robot_pose: Point3 = None,
) -> Pose:
    points = surface.sample_points_from_surface(obj, category_of_interest)
    if robot_pose is None:
        point = points[0] if points else Point3()
    else:
        point = calc_closest_point_to_robot(robot_pose, points)
    point.z -= 0.005  # to make sure object is on table
    pose = Pose(position=point, reference_frame=point.reference_frame)
    return pose


def spawn_object_on_table(
    obj_name: str,
    semantic_annotation: type[HasRootBody],
    table: Table,
    scale: Scale = Scale(0.1, 0.1, 0.2),
    robot_pose: Point3 = None,
) -> HasRootBody:
    world = table._world
    with world.modify_world():
        obj = semantic_annotation.create_with_new_body_in_world(
            name=PrefixedName(obj_name), world=world, scale=scale
        )
    pose = get_pose_from_surface(surface=table, obj=obj, robot_pose=robot_pose)
    with world.modify_world():
        object_creation.move_object_to_new_pose(obj, pose.to_homogeneous_matrix())
        table.add_object(obj)
    return obj


def spawn_ref_objects(world: World):
    dining_table: Table = world.get_semantic_annotation_by_name("dining_table")
    cucumber = spawn_object_on_table(
        "ref_cucumber", Cucumber, dining_table, Scale(0.1, 0.2, 0.1)
    )
    for color in cucumber.root.visual.shapes:
        color.color = Color.GREEN()
    table: Table = world.get_semantic_annotation_by_name("table")
    orange = spawn_object_on_table("ref_apple", Orange, table, Scale(0.1, 0.1, 0.1))
    for color in orange.root.visual.shapes:
        color.color = Color.ORANGE()


def spawn_objects_to_pick(world: World):
    cooking_table: Table = world.get_semantic_annotation_by_name("cooking_table")
    cooking_table_pose = Point3(1.3, 5.3, 0.0, reference_frame=world.root)
    cucumber = spawn_object_on_table(
        "cucumber",
        Cucumber,
        cooking_table,
        Scale(0.1, 0.2, 0.1),
        robot_pose=cooking_table_pose,
    )
    for color in cucumber.root.visual.shapes:
        color.color = Color.GREEN()
    cola = spawn_object_on_table(
        "cola", Cola, cooking_table, robot_pose=cooking_table_pose
    )
    for color in cola.root.visual.shapes:
        color.color = Color.BLACK()
    banana = spawn_object_on_table(
        "banana", Banana, cooking_table, robot_pose=cooking_table_pose
    )
    for color in banana.root.visual.shapes:
        color.color = Color.ORANGE()


def pickup_object_from_table(obj: HasRootBody):
    cooking_table = PoseStamped.from_list(
        position=[1.3, 5.3, 0.0],
        orientation=[0.0, 0.0, 0.72, 0.64],
        frame=world.root,
    )
    plan = SequentialPlan(
        context,
        NavigateActionDescription(
            target_location=cooking_table,
            keep_joint_states=True,
        ),
        GiskardPickUpActionDescription(
            simulated=SIMULATED,
            object_designator=obj.root,
            arm=Arms.LEFT,
            gripper_vertical=True,
        ),
    )
    with simulated_robot:
        plan.perform()
        attach_object_to_hsrb(world=world, object_designator=obj.root),


def place_object_on_table(
    obj: HasRootBody,
    surface_to_place_on: HasSupportingSurface,
    category_of_interest: Optional[Type[SemanticAnnotation]] = None,
):
    match obj.name.name:
        case "banana":
            robot_pose = PoseStamped.from_list(
                position=[2.2, 0.9, 0.0],
                orientation=[0.0, 0.0, -0.01, 0.99],
                frame=world.root,
            )
        case "cola":
            robot_pose = PoseStamped.from_list(
                position=[3.63, 2.85, 0.0],
                orientation=[0.0, 0.0, 0.05, 0.99],
                frame=world.root,
            )
        case "cucumber":
            robot_pose = PoseStamped.from_list(
                position=[3.38, 5.19, 0.0],
                orientation=[0.0, 0.0, 0.92, 0.37],
                frame=world.root,
            )
        case _:
            robot_pose = PoseStamped.from_list()
    pose = get_pose_from_surface(
        surface=surface_to_place_on,
        obj=obj,
        category_of_interest=category_of_interest,
        robot_pose=robot_pose.to_spatial_type().to_position(),
    )
    pose_stamped = PoseStamped.from_spatial_type(pose.to_homogeneous_matrix())
    # robot_pose_description = CostmapLocation(
    #     target=pose_stamped, reachable_for=robot_view
    # )
    # SequentialPlan(context, robot_pose_description)
    # robot_pose = robot_pose_description.resolve()
    plan = SequentialPlan(
        context,
        NavigateActionDescription(target_location=robot_pose, keep_joint_states=True),
        GiskardPlaceActionDescription(
            object_designator=obj.root,
            arm=Arms.LEFT,
            target_location=pose_stamped,
            simulated=SIMULATED,
        ),
    )
    with simulated_robot:
        plan.perform()
        detach_object_from_hsrb(
            world=world,
            object_designator=obj.root,
        )


def main():
    spawn_objects_to_pick(world)
    spawn_ref_objects(world)

    table: Table = world.get_semantic_annotation_by_name("cooking_table")
    objs: List[HasRootBody] = query_get_next_object_euclidean_x_y(
        robot_view.root, table
    ).tolist()

    tables_to_look_on = [
        world.get_semantic_annotation_by_name("dining_table"),
        world.get_semantic_annotation_by_name("table"),
        world.get_semantic_annotation_by_name("lowerTable"),
    ]
    for obj in objs:
        pickup_object_from_table(obj=obj)
        surface_to_place_on: HasSupportingSurface = query_surface_of_most_similar_obj(
            obj, tables_to_look_on
        )
        # objs_on_surface = query_semantic_annotations_on_surfaces(
        #     [surface_to_place_on], world
        # ).tolist()
        # obj_on_surface = objs_on_surface[0] if objs_on_surface else None
        place_object_on_table(
            obj=obj,
            surface_to_place_on=surface_to_place_on,
        )
        with simulated_robot:
            SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH)).perform()

    all_objs = query_semantic_annotations_on_surfaces(
        supporting_surfaces=tables_to_look_on, world=world
    ).tolist()
    objs_to_pick: List[HasRootBody] = query_annotations_by_color(
        color=Color.BLACK(), objects=all_objs
    )
    for obj in objs_to_pick:
        pickup_object_from_table(obj=obj)

    return


if __name__ == "__main__":
    main()
