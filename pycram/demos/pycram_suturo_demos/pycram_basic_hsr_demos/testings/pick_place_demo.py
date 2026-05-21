import logging

import semantic_digital_twin
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    GiskardPickUpActionDescription,
    GiskardPlaceAndDetachActionDescription,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
    perceive_and_spawn_all_objects,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    detach_object_from_hsrb,
    attach_object_to_hsrb,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.A_start_up import setup_hsrb_context
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Chips,
)
from semantic_digital_twin.world import World

logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)

logger = logging.getLogger(__name__)
rclpy_node, world, robot_view, context = setup_hsrb_context()

with real_robot:
    SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH)).perform()

perceive_and_spawn_all_objects(world)
object_to_pickup = world.get_semantic_annotations_by_type(Chips)[0]

print(f"Object: {object_to_pickup}")
object_pose = PoseStamped.from_spatial_type(object_to_pickup.global_pose)

pickup_plan = SequentialPlan(
    context,
    GiskardPickUpActionDescription(
        simulated=False,
        object_designator=object_to_pickup.root,
        arm=Arms.LEFT,
        gripper_vertical=True,
    ),
)

place_plan = SequentialPlan(
    context,
    GiskardPlaceAndDetachActionDescription(
        object_designator=object_to_pickup.root,
        arm=Arms.LEFT,
        target_location=object_pose,
        simulated=False,
        ignore_orientation=True,
    ),
)

with real_robot:
    pickup_success = pickup_plan.perform()
    if pickup_success:
        attach_object_to_hsrb(
            world=context.world, object_designator=object_to_pickup.root
        )
        place_plan.perform()
        detach_object_from_hsrb(
            world=context.world,
            object_designator=object_to_pickup.root,
        )
