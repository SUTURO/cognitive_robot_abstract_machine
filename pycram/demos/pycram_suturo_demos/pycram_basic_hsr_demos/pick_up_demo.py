import logging

import semantic_digital_twin
from pycram.datastructures.enums import Arms
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    GiskardPickUpActionDescription,
)
from pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
    perceive_and_spawn_all_objects,
)
from pycram_suturo_demos.pycram_basic_hsr_demos.A_start_up import setup_hsrb_context
from semantic_digital_twin.semantic_annotations.semantic_annotations import Cereal
from semantic_digital_twin.world import World

logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)

logger = logging.getLogger(__name__)
rclpy_node, world, robot_view, context = setup_hsrb_context()


perceive_and_spawn_all_objects(world)
print(world.bodies)
object_to_pickup = world.get_semantic_annotations_by_type(Cereal)[0]


plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    GiskardPickUpActionDescription(
        simulated=False,
        object_designator=object_to_pickup.root,
        arm=Arms.LEFT,
        gripper_vertical=True,
    ),
)

with real_robot:
    plan.perform()
