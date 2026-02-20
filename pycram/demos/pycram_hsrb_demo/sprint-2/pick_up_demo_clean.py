import logging

from demos.helper_methods_and_useful_classes.object_creation import perceive_and_spawn_all_objects
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import (
    Arms,
    TorsoState,
    ApproachDirection,
    VerticalAlignment,
)
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.motion_executor import real_robot
from pycram.robot_plans import (
    LookAtActionDescription,
    ParkArmsActionDescription,
    PickUpActionDescription,
)
from semantic_digital_twin.robots.hsrb import HSRB
from pycram_ros_setup import setup_ros_node

logger = logging.getLogger(__name__)

# Setup ROS node and fetch the world
node, hsrb_world = setup_ros_node()

# Setup context
context = Context(
    hsrb_world, hsrb_world.get_semantic_annotations_by_type(HSRB)[0], ros_node=node
)

# Perceive objects and spawn them
perceived_objects = perceive_and_spawn_all_objects(hsrb_world)
print(perceived_objects)

grasp = GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)


# plan1 = SequentialPlan(
#     context,
#     # LookAtActionDescription(
#     #     target=PoseStamped.from_spatial_type(
#     #         HomogeneousTransformationMatrix.from_xyz_rpy(x=1.0, y=6.22, z=0.8)
#     #     )
#     # ),
#     PickUpActionDescription(
#         arm=Arms.LEFT,
#         object_designator=hsrb_world.get_body_by_name("milk"),
#         grasp_description=grasp,
#     )
# )
#
# plan2 = SequentialPlan(
#     context,
#     ParkArmsActionDescription(arm=Arms.LEFT),
#     MoveTorsoActionDescription(TorsoState.HIGH),
# )

# Execute the plans
with real_robot:
    SequentialPlan(
        context,
        ParkArmsActionDescription(arm=Arms.LEFT),
        PickUpActionDescription(
            arm=Arms.LEFT,
            object_designator=hsrb_world.get_body_by_name("milk"),
            grasp_description=grasp,
        ),
    ).perform()

# plan1.perform()
# Uncomment to execute plan2
# plan2.perform()

# exit(0)
