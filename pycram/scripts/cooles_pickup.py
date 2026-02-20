from suturo_resources.suturo_map import load_environment

from giskardpy.motion_statechart.goals.pick_up import PickUp
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy_ros.python_interface.python_interface import GiskardWrapper
from giskardpy_ros.ros2 import rospy
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.abstract_robot import Manipulator
from semantic_digital_twin.spatial_types import Vector3, HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.derivatives import DerivativeMap
from semantic_digital_twin.world_description.connections import PrismaticConnection
from semantic_digital_twin.world_description.degree_of_freedom import (
    DegreeOfFreedomLimits,
    DegreeOfFreedom,
)
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body
from test.conftest import hsr_world_setup


rospy.init_node("pickup_test")
giskard = GiskardWrapper(node_handle=rospy.node)


msc = MotionStatechart()

map = giskard.world.root
hand = giskard.world.get_semantic_annotations_by_type(Manipulator)[0]

eistee_scale = Scale(0.05, 0.05, 0.14)

lipton_eistee = Body(
    name=PrefixedName("muh"),
    collision=ShapeCollection([Box(scale=eistee_scale)]),
    visual=ShapeCollection([Box(scale=eistee_scale)]),
)
dof_limits = DegreeOfFreedomLimits(
    lower=DerivativeMap(data=[None, -1.0, None, None]),
    upper=DerivativeMap(data=[None, 1.0, None, None]),
)
with giskard.world.modify_world():
    dof = DegreeOfFreedom(limits=dof_limits)
    giskard.world.add_degree_of_freedom(dof)
    connection = PrismaticConnection(
        dof_id=dof.id,
        parent=giskard.world.root,
        child=lipton_eistee,
        axis=Vector3.Z(reference_frame=giskard.world.root),
        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
            x=1.5,
            y=2.7,
            z=0.74 + 0.14 / 2,
        ),
    )
    giskard.world.add_connection(connection)

    scheiß_welt = load_environment()
    giskard.world.merge_world(scheiß_welt)

pickup = PickUp(
    manipulator=hand, object_geometry=lipton_eistee, simulated_execution=False
)
msc.add_node(pickup)
msc.add_node(EndMotion.when_true(pickup))
giskard.execute(msc)

rospy.shutdown()
