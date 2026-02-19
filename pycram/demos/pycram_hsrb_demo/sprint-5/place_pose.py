import rclpy
from suturo_resources.suturo_map import load_environment

from demos.pycram_hsrb_demo.hsrb_simulation.simulation_setup import (
    setup_hsrb_in_environment,
)
from pycram.datastructures.enums import TorsoState, Arms
from pycram.designators.location_designator import SemanticCostmapLocation
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import MoveTorsoActionDescription, ParkArmsActionDescription

rclpy.init()

result = setup_hsrb_in_environment(
    load_environment=load_environment,
    with_viz=True,
    milk_xyz_rpy=(2.045, 6.401, 0.797, 0, 0, 0),
    cereal_xyz_rpy=(0.248, 1.33, 0.82, 0, 0, 0),
)
world, robot_view, context, viz = (
    result.world,
    result.robot_view,
    result.context,
    result.viz,
)

milk = world.get_body_by_name("milk.stl")

location_description = SemanticCostmapLocation(
    body=world.get_body_by_name("diningTable_body"),
    for_object=milk,
    link_is_center_link=True,
)

print(type(location_description))
print(location_description)

plan = SequentialPlan(
    context,
    ParkArmsActionDescription(Arms.BOTH),
    MoveTorsoActionDescription(TorsoState.HIGH),
)
res = location_description.resolve()
print(type(res))
for i, pose in enumerate(location_description):
    print(pose)
    if i > 3:
        break

with simulated_robot:
    plan.perform()
