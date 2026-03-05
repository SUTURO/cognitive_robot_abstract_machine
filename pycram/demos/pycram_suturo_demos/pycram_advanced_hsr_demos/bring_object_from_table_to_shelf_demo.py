import logging
from typing import List

import rclpy
from suturo_resources.queries import (
    query_semantic_annotations_on_surfaces,
    query_surface_of_most_similar_obj,
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
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.place_pose import (
    get_pose_on_semantic_annotation_for_object_by_semantic_annotation,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.move_demo import move_demo
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.pickup_demo_marc import (
    pickup_demo,
)
from demos.pycram_suturo_demos.pycram_basic_hsr_demos.place_demo import place_demo
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped, GraspPose
from pycram.designators.location_designator import CostmapLocation
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot
from pycram.plan import Plan
from pycram.robot_plans import (
    ParkArmsActionDescription,
    GiskardPickUpActionDescription,
    NavigateActionDescription,
    GiskardPlaceActionDescription,
)
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
)
from semantic_digital_twin.spatial_types import Point3
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale

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


def get_pose_from_surface(surface: HasSupportingSurface, obj: HasRootBody) -> Pose:
    points = surface.sample_points_from_surface(obj)
    point = points[0] if points else Point3()
    point.z -= 0.005  # to make sure object is on table
    pose = Pose(position=point, reference_frame=points[0].reference_frame)
    return pose


def spawn_object_on_table(
    obj_name: str,
    semantic_annotation: type[HasRootBody],
    table: Table,
):
    scale = Scale(0.1, 0.1, 0.2)
    world = table._world
    with world.modify_world():
        obj = semantic_annotation.create_with_new_body_in_world(
            name=PrefixedName(obj_name), world=world, scale=scale
        )
    pose = get_pose_from_surface(surface=table, obj=obj)
    with world.modify_world():
        object_creation.move_object_to_new_pose(obj, pose.to_homogeneous_matrix())
        table.add_object(obj)


def spawn_ref_objects(world: World):
    dining_table: Table = world.get_semantic_annotation_by_name("dining_table")
    spawn_object_on_table("ref_cucumber", Cucumber, dining_table)
    table: Table = world.get_semantic_annotation_by_name("table")
    spawn_object_on_table("ref_banana", Banana, table)


def spawn_objects_to_pick(world: World):
    cooking_table: Table = world.get_semantic_annotation_by_name("cooking_table")
    spawn_object_on_table("cucumber", Cucumber, cooking_table)
    spawn_object_on_table("cola", Cola, cooking_table)
    spawn_object_on_table("banana", Banana, cooking_table)


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


def place_object_on_table(obj: HasRootBody, surface_to_place_on: HasSupportingSurface):
    pose = get_pose_from_surface(surface=surface_to_place_on, obj=obj)
    pose_stamped = PoseStamped.from_spatial_type(pose.to_homogeneous_matrix())
    plan = SequentialPlan(
        context,
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
    objs: List[HasRootBody] = query_semantic_annotations_on_surfaces(
        [table], world
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
        place_object_on_table(obj=obj, surface_to_place_on=surface_to_place_on)
        with simulated_robot:
            SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH)).perform()

    return


if __name__ == "__main__":
    main()
