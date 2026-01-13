from suturo_resources.suturo_map import load_environment

from giskardpy.motion_statechart.context import ExecutionContext
from pycram.datastructures.enums import TorsoState, Arms
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot

from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from semantic_digital_twin.world import World
from simulation_setup import setup_hsrb_in_environment
from pycram.robot_plans import SimplePouringActionDescription
from pycram.robot_plans import ParkArmsActionDescription
from pycram.alternative_motion_mappings import hsrb_motion_mapping

result = setup_hsrb_in_environment(load_environment=load_environment, with_viz=True)
world: World
context: ExecutionContext
viz: VizMarkerPublisher


world, robot_view, context, viz = (
    result.world,
    result.robot_view,
    result.context,
    result.viz,
)

plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    # MoveTorsoActionDescription(TorsoState.HIGH),
    # PouringActionDescription(world.get_body_by_name("milk.stl")),
    SimplePouringActionDescription(world.get_body_by_name("milk.stl"), Arms.LEFT),
)

with simulated_robot:
    plan.perform()
