from pycram.motion_executor import simulated_robot
from pycram_suturo_demos.helper_methods_and_useful_classes.pickup_helper_methods import (
    detach_object_from_hsrb,
)
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import (
    GiskardPlaceActionDescription,
    ParkArmsActionDescription,
)

from semantic_digital_twin.world import World


def place_demo(
    *,
    simulation: bool,
    hsrb_world: World,  # parameter can be replaced
    context: Context,
    object_name: str,  # parameter can be replaced by retrieving from TCP
    place_pose: PoseStamped,  # TODO PoseStamped will be deprecated soon Point3 / smth else cant remember will be up to date then
    ignore_orientation: bool = False,
):
    robot_type = simulated_robot if simulation else real_robot
    # TODO retrieve the object from tool_frame of robot, nobody actually cares about naming the object lul
    # retrieving body of parsed object_name
    object_to_place = hsrb_world.get_body_by_name(object_name)

    # -----------------------------------------Plans
    plan = SequentialPlan(
        context,
        GiskardPlaceActionDescription(
            simulated=simulation,
            arm=Arms.LEFT,
            object_designator=object_to_place,
            target_location=place_pose,
            ignore_orientation=ignore_orientation,
        ),
    )
    park_plan = SequentialPlan(context, ParkArmsActionDescription(Arms.BOTH))
    # -----------------------------------------Execution
    with robot_type:
        plan.perform()
        detach_object_from_hsrb(
            world=hsrb_world,
            object_designator=object_to_place,
        )
        park_plan.perform()
