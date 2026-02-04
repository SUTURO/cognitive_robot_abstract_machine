from suturo_resources.suturo_map import load_environment

from pycram.datastructures.enums import TorsoState, Arms
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import (
    MoveTorsoActionDescription,
    ParkArmsActionDescription,
    LookAtActionDescription,
)
from simulation_setup import setup_hsrb_in_environment
from pycram.external_interfaces import robokudo
import time
from time import sleep
from pycram.ros_utils.text_to_image import TextToImagePublisher

result = setup_hsrb_in_environment(load_environment=load_environment, with_viz=True)
world, robot_view, context, viz = (
    result.world,
    result.robot_view,
    result.context,
    result.viz,
)


with simulated_robot:
    text_pub = TextToImagePublisher()
    position = robokudo.query_current_human_position_in_continues()
    x = round(position.point.x)
    y = round(position.point.y)
    z = round(position.point.z)
    text_pub.publish_text(f"Point x: {x} y: {y} z: {z}")
    plan = SequentialPlan(
        context,
        ParkArmsActionDescription(Arms.BOTH),
        LookAtActionDescription(position),
    )
    plan.perform()
    robokudo.shutdown_robokudo_interface()
