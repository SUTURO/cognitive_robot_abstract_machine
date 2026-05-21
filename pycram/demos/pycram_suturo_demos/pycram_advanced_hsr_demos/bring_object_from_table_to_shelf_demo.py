import logging
from typing import List, Optional

from suturo_resources.queries import (
    query_surface_of_most_similar_obj,
    query_semantic_annotations_on_surfaces,
    query_class_by_label,
)

import semantic_digital_twin
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot, ExecutionEnvironment, simulated_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    GiskardPickUpActionDescription,
    LookAtActionDescription,
    MoveTorsoActionDescription,
    GiskardPlaceAndDetachActionDescription,
    NavigateActionDescription,
    HandoverActionDescription,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
    perceive_and_spawn_all_objects,
    move_object_to_new_pose,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.setup import setup_context
from pycram_suturo_demos.pycram_basic_hsr_demos.hri_handover import (
    take_object_from_human,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.talking_demo import TtsPublisher
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.mixins import (
    HasRootBody,
    HasSupportingSurface,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Table,
    ShelfLayer,
    Milk,
)
from semantic_digital_twin.spatial_types import Point3, HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale

logger = logging.getLogger(__name__)
logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)


def _filter_points_full_on_surface(
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


class BringObjectFromTableToShelfDemo:
    def __init__(self, context: Context, execution_type: ExecutionEnvironment):
        self._context = context
        self._execution_type = execution_type
        self._world = context.world
        self._robot_view = context.robot
        self._node = context.ros_node

        self._SIMULATED = True if execution_type == simulated_robot else False

        self._tts = TtsPublisher(
            node_name="bring_object_from_table_to_shelf_tts_publisher"
        )

        self._TABLE_NAME = "cooking_table"
        self._CUPBOARD_NAME = "cupboard_annotation"

        self._TABLE_POSE = PoseStamped.from_list(
            [
                1.210,
                5.519,
                0.0,
            ],
            [0.0, 0.0, 0.728, 0.684],
            frame=self._world.root,
        )
        self._SHELF_POSE = PoseStamped.from_list(
            [3.655, 4.639, 0.0],
            [0.0, 0.0, 0.027, 0.999],
            frame=self._world.root,
        )

        self._TORSO_THRESHOLDS = {"high": 1.5, "mid": 0.8}

        self._STARTING_POSE = PoseStamped.from_spatial_type(
            self._robot_view.root.global_pose
        )

    def run(self, *, object_to_pick: str = None):
        table: Table = self._world.get_semantic_annotation_by_name(self._TABLE_NAME)
        obj_type = query_class_by_label(object_to_pick)

        self._move_to_pose(pose=self._TABLE_POSE)

        self._tts.publish("I will try to scan for the object on the table.")
        obj = self._try_and_scan_for_object_on_table(
            object_to_pick_type=obj_type, from_table=table
        )

        if obj is None:
            self._tts.publish(
                "Object was not found after multiple tries. I will reset to the starting position."
            )
            self._reset_to_start()
            return

        self._tts.publish("I found the object and will now try to pick it up.")
        self._try_to_pick_up_else_hri(obj=obj)

        self._tts.publish("I will now move to the shelf.")
        self._move_to_pose(pose=self._SHELF_POSE)

        shelf_layers = self._world.get_semantic_annotations_by_type(ShelfLayer)
        self._scan_shelves(shelves=shelf_layers)

        surface_to_place_on: HasSupportingSurface = query_surface_of_most_similar_obj(
            obj, shelf_layers
        )
        self._tts.publish("I will try to place the object in the shelf.")
        self._place_object_on_surface(
            obj=obj,
            surface_to_place_on=surface_to_place_on,
        )

        self._move_to_pose(pose=self._SHELF_POSE)
        self._park_arms()
        self._move_torso(state=TorsoState.LOW)

        self._tts.publish(
            "I finished all tasks and will move to the starting position."
        )
        self._reset_to_start()
        self._tts.shutdown()

    def _move_to_pose(self, *, pose: PoseStamped):
        with self._execution_type:
            SequentialPlan(
                self._context, NavigateActionDescription(target_location=pose)
            ).perform()

    def _try_and_scan_for_object_on_table(
        self,
        *,
        object_to_pick_type: type,
        from_table: Table,
    ):
        offset = 0.2
        for try_count in range(3):
            # Look at surface with different offset each try
            self._look_at_surface(surface=from_table, offset=(offset * try_count))

            # Change offset directions each try
            offset = -offset

            if self._execution_type == real_robot:
                perceive_and_spawn_all_objects(self._world)

            # Check if object was spawned
            objs: List[HasRootBody] = query_semantic_annotations_on_surfaces(
                [from_table], self._world
            ).tolist()

            # Object found on table
            if object_to_pick_type in objs:
                return objs[0]

        return None

    def _look_at_point(self, *, point: Point3):
        with real_robot:
            SequentialPlan(
                self._context,
                LookAtActionDescription(
                    [
                        PoseStamped.from_spatial_type(
                            HomogeneousTransformationMatrix.from_point_rotation_matrix(
                                point=point, reference_frame=point.reference_frame
                            )
                        )
                    ]
                ),
            ).perform()

    def _look_at_surface(
        self, *, surface: HasSupportingSurface, offset: Optional[float] = None
    ):
        if offset is None or offset == 0.0:
            self._look_at_point(point=surface.global_pose.to_position())
        else:
            self._look_side_of_surface_middle(surface=surface, offset=offset)

    def _look_side_of_surface_middle(
        self, *, surface: HasSupportingSurface, offset: float
    ):
        surface_middle_point = surface.global_pose.to_translation_matrix()
        middle_point_T_robot = self._world.transform(
            surface_middle_point, self._robot_view.root
        )

        middle_point_T_robot.y += offset
        self._look_at_point(point=middle_point_T_robot.to_position())

    def _try_to_pick_up_else_hri(self, *, obj: HasRootBody):
        for try_count in range(3):
            with self._execution_type:
                did_pick_up = SequentialPlan(
                    self._context,
                    GiskardPickUpActionDescription(
                        simulated=False,
                        object_designator=obj.root,
                        arm=Arms.LEFT,
                        gripper_vertical=True,
                    ),
                ).perform()
            if did_pick_up:
                return
            self._look_at_point(point=obj.global_pose.to_position())
        self._tts.publish("Could not pick up the object. Manual handover needed.")
        self._handover_human_robot()

    def _handover_human_robot(self):
        with self._execution_type:
            SequentialPlan(
                self._context,
                HandoverActionDescription(world=self._world),
            ).perform()

            take_object_from_human()

            SequentialPlan(
                self._context,
                ParkArmsActionDescription(Arms.BOTH),
            ).perform()

    def _move_torso(self, *, state: TorsoState):
        with real_robot:
            SequentialPlan(self._context, MoveTorsoActionDescription(state)).perform()

    def _scan_shelves(self, *, shelves: List[ShelfLayer]):
        for shelf in shelves:
            z_shelf = float(shelf.global_pose.z)

            if z_shelf <= self._TORSO_THRESHOLDS["mid"]:
                self._move_torso(state=TorsoState.LOW)
            elif (
                self._TORSO_THRESHOLDS["mid"]
                <= z_shelf
                <= self._TORSO_THRESHOLDS["high"]
            ):
                self._move_torso(state=TorsoState.MID)
            else:
                self._move_torso(state=TorsoState.HIGH)

            self._look_at_surface(surface=shelf)
            if self._execution_type == real_robot:
                perceive_and_spawn_all_objects(self._world)

            objects_on_shelf = query_semantic_annotations_on_surfaces(
                [shelf], self._world
            )
            if not objects_on_shelf:
                continue
            for obj in objects_on_shelf:
                with self._world.modify_world():
                    shelf.add_object(obj)

    def _place_object_on_surface(
        self,
        *,
        obj: HasRootBody,
        surface_to_place_on: HasSupportingSurface,
    ):
        pose = self._get_pose_from_surface(
            surface=surface_to_place_on, obj=obj, closest_to_robot=True
        )
        pose_stamped = PoseStamped.from_spatial_type(pose.to_homogeneous_matrix())
        with self._execution_type:
            SequentialPlan(
                self._context,
                GiskardPlaceAndDetachActionDescription(
                    object_designator=obj.root,
                    arm=Arms.LEFT,
                    target_location=pose_stamped,
                    simulated=False,
                    ignore_orientation=True,
                ),
            )

    def _calc_closest_point_to_robot(self, *, points: List[Point3]) -> Point3:
        min_dist = float("inf")
        min_dist_point = points[0]
        for point in points:
            dist = self._robot_view.root.global_pose.to_position().euclidean_distance(
                point
            )
            min_dist_point = point if dist < min_dist else min_dist_point
            min_dist = min(dist, min_dist)
        return min_dist_point

    def _get_pose_from_surface(
        self,
        *,
        surface: HasSupportingSurface,
        obj: HasRootBody,
        closest_to_robot: bool = True,
    ) -> Pose:
        points = surface.sample_points_from_surface(obj)
        filtered_points = _filter_points_full_on_surface(
            points=points, obj=obj, surface=surface
        )

        if closest_to_robot:
            point = self._calc_closest_point_to_robot(points=filtered_points)
        else:
            point = filtered_points[0] if filtered_points else Point3()

        point.z -= 0.005  # to make sure object is on table
        pose = Pose(position=point, reference_frame=point.reference_frame)
        return pose

    def _park_arms(self):
        with self._execution_type:
            SequentialPlan(
                self._context, ParkArmsActionDescription(Arms.BOTH)
            ).perform()

    def _reset_to_start(self):
        self._park_arms()
        with self._execution_type:
            SequentialPlan(
                self._context,
                NavigateActionDescription(target_location=self._STARTING_POSE),
            )


if __name__ == "__main__":
    SIMULATED = True
    main_context, main_execution_type = setup_context(
        simulated=SIMULATED,
    )

    table: Table = main_context.world.get_semantic_annotation_by_name("cooking_table")

    with main_context.world.modify_world():
        milk = Milk.create_with_new_body_in_world(
            name=PrefixedName("milk_carton"),
            world=main_context.world,
            scale=Scale(0.1, 0.1, 0.2),
        )
    points = table.sample_points_from_surface(body_to_sample_for=milk)
    filtered_points = _filter_points_full_on_surface(
        points=points, obj=milk, surface=table
    )
    point = filtered_points[0]
    with main_context.world.modify_world():
        move_object_to_new_pose(
            semantic_annotation=milk,
            new_transform=HomogeneousTransformationMatrix.from_point_rotation_matrix(
                point=point, reference_frame=point.reference_frame
            ),
        )

    BringObjectFromTableToShelfDemo(
        context=main_context, execution_type=main_execution_type
    ).run(object_to_pick="Milk")
