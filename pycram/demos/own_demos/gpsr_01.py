import os

from absl.logging import exception

from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.reasoning.world_reasoner import WorldReasoner
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Container
from semantic_digital_twin.adapters.procthor.procthor_semantic_annotations import Milk, Bowl, Spoon
from semantic_digital_twin.spatial_types import TransformationMatrix
from semantic_digital_twin.world_description.connections import FixedConnection

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TorsoState, Arms
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import MoveTorsoActionDescription, TransportActionDescription
from pycram.robot_plans import ParkArmsActionDescription
from pycram.testing import setup_world

from pycram.datastructures.dataclasses import Color
import tempfile

import numpy as np
from pycram.robot_plans import *

import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, PoseStamped as PoseSta
import nlp_gpsr
from src.pycram.external_interfaces import nav2_move

#DEBUG
test_response = ["take", "bowl", "food", "blue", "", "", "me", "kitchen"]

# positions
kitchen_placeholder = PoseStamped.from_list([1,1,1], [0,0,0,1])
living_room_placeholder = PoseStamped.from_list([3.56,4.06,0], [0,0,0,1])

# for creating objects with unique names
global object_name_iteration

# --------------------------------------------------------------------------------------
world = setup_world()

# test objects
spoon = STLParser(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "objects", "spoon.stl")).parse()
bowl = STLParser(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "objects", "bowl.stl")).parse()

"""
The following block shows how objects can be merged into the world model and connected
to other world elements. Left in comments because it doesn`t work right now and I don`t know how to solve it.


with world.modify_world():
    world.merge_world_at_pose(bowl, TransformationMatrix.from_xyz_quaternion(2.4, 2.2, 1, reference_frame=world.root))
    connection = FixedConnection(parent=world.get_body_by_name("cabinet10_drawer_top"), child=spoon.root)
    world.merge_world(spoon, connection)

try:
    import rclpy

    rclpy.init()
    from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher

    v = VizMarkerPublisher(world, rclpy.create_node("viz_marker"))
except ImportError:
    pass

toya = HSRB.from_world(world)
context = Context.from_world(world)
"""
# ---------------------------------------------------------------------------------------------

"""
Processes the NLP module output and dispatches the correct action.

    Expected format for `lst`:
        [intent, item, item_entity, item_property, item_action, item_number, to_who, location]

    Parameters:
        lst (list[Any]): Parsed response from NLP module.

    Behavior:
        - Checks the intent (lst[0])
        - Calls appropriate task function

"""
# response = [intent, item, item_entity, item_property, item_action, item_number, to_who, location]
def process_response(lst: list[Any]):
    if len(lst) != 8:
        print("check if nlp_gpsr.response was changed, otherwise something went wrong" + str(len(lst)))
    else:
        match lst[0]:
            case "take":
                print("case take")
                take_obj_from_plcmt(lst[7], lst[1]),
            # MILESTONE 1
            case "Guide":
                driveTo(lst[7])
            case _:
                print("No function for this intent")  # Default case



#------------------------Methods for solving tasks------------------------------------------------------------------------
"""
Task: Take an object from a specified location.

    Parameters:
        location (String): Location keyword ("kitchen", "living room", etc.)
        obj (String): Object name as string.

    Notes:
        - Creates a unique object name via global counter.
        - Uses STLParser to load object model.
        - TODO: Does not yet execute pick action (future work).

"""
def take_obj_from_plcmt(location: String, obj: String):
    global object_name_iteration
    object_name_iteration += 1
    if location != "":
        obj_pose = _location_from_string(location)
        print(obj_pose)
    else:
        # TO-DO Ask for location
        PoseStamped.from_list([0,0,0], [0,0,0,1]) # DEBUG

    # creates object from class
    finished_object = STLParser(os.path.join(os.path.dirname(__file__), "..", "..", "resources", obj.__str__(), obj.__str__() + "spoon.stl")).parse()
    # TODO einbindung der obj_pose

    print("Finished Object: " + finished_object.__str__())

"""
Navigates the robot to a specified location.

Parameters:
    location (String): Target location keyword.

Behavior:
    - Resolves location to a pose.
    - Starts Nav2 navigation to the position.
    
"""
def driveTo(location: String):
    goal = _location_from_string(location)
    start_nav(goal.position.x, goal.position.y)
    print(goal)


#----------------------------HELPER METHODS------------------------------------------------------------------
"""
Converts a location keyword into a PoseStamped placeholder.

    Parameters:
        location (String): "kitchen", "living room", etc.

    Returns:
        PoseStamped: Predefined placeholder pose.

    Raises:
        Logs an exception for unknown locations.
"""
def _location_from_string(location: String):
    match location:
        case "kitchen":
            return kitchen_placeholder
        case "living room":
            return living_room_placeholder
        case _:
            exception("unknown location")
            # return PoseStamped.from_list([0,0,0], [0,0,0,1])

"""
helper method
find object class from string, koennen wir spaeter von knowledge klauen

SOLLTE NICHT MEHR GEBRAUCHT WERDEN IN NEUEM PYCRAM
"""
def _find_class_from_string(obj: String):
    match obj:
        case "bowl":
            return None
            """
            case "bowl":
                return Bowl
            case "milk":
                return Milk
            case "spoon":
                return Spoon
            case "cereal":
                return Cereal
            """
        case _:
            return None


"""
finds an instance from string

def _find_instance_from_string(obj: String):
    for cls in ontology.individuals():
        if cls.name == obj:
            return cls
    return None
"""



"""
kopiert von Ansgar, angepasst, dass x und y Koordinate übergeben werden,
PoseSta, da nicht der pycram datatype benutzt wird wie im restlichen file, sondern der von geometry_msgs


Starts Nav2 navigation to the specified XY position.

    Parameters:
        pos_x (float): X coordinate in map frame
        pos_y (float): Y coordinate in map frame

    Behavior:
        - Creates a geometry_msgs/PoseStamped
        - Calls Nav2 wrapper `nav2_move.start_nav_to_pose`
"""
def start_nav(pos_x: float, pos_y: float):
    pos = Pose(position=Point(x=pos_x, y=pos_y))
    pos_stamped = PoseSta()
    pos_stamped.header.frame_id = "map"
    pos_stamped.pose = pos
    nav2_move.start_nav_to_pose(pos_stamped)


#-----------------------------------------------------------------------------------------------------------------------

# MAIN CODE
"""
Main execution loop:
        - Initializes ROS2
        - Creates NLP node
        - Waits for spoken commands
        - Processes received NLP responses
"""
if __name__ == '__main__':
    rclpy.init()

    start_nav(3, 3)
    """
    sleep(3)

    node = nlp_gpsr.NLP_GPSR()
    while rclpy.ok():
        object_name_iteration = 0
        wait = True
        resp = node.talk_nlp()


        while wait:
            #process_response(test_response)
            #sleep(7)
            if resp:
                print("Got response in gpsr_01.py: ", resp)
                process_response(resp)
                wait = False
                resp = []
            else:
                print("no response yet")
                wait = False


    """
