import rclpy

from suturo_resources.suturo_map import load_environment

from pycram.datastructures.enums import Arms
from semantic_digital_twin.datastructures.definitions import TorsoState
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import MoveTorsoActionDescription, ParkArmsActionDescription
from simulation_setup import setup_hsrb_in_environment

rclpy.init()
result = setup_hsrb_in_environment(load_environment=load_environment, with_viz=True)
world, robot_view, context, viz = (
    result.world,
    result.robot_view,
    result.context,
    result.viz,
)

plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.HIGH),
)

with simulated_robot:
    plan.perform()
