import os
from xml.etree.ElementTree import tostring

from absl.logging import exception
from breezy.switch import switch
from mercurial.repoview import newtype

from rclpy.node import Node

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

from pycram.world_concepts.world_object import Object

from pycram.external_interfaces import tmc

from pycram.robot_plans.actions.composite import searching
# from suturo_resources import queries_rody as queries, suturo_map_rody as suturo_map TODO: Implement mit rodys shit

from pycram.datastructures.dataclasses import Color
import tempfile

import numpy as np
from pycram.robot_plans import *

import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, PoseStamped as PoseSta
import nlp_gpsr
from pycram.external_interfaces import nav2_move

from typing import Optional

#DEBUG
test_response = ["take", "bowl", "food", "blue", "", "", "me", "kitchen"]
milestone1_response =["Guide", "", "", "", "", "", "", "living room"]

last_affirmation = False

# positions
kitchen_placeholder = PoseStamped.from_list([1,1,1], [0,0,0,1])
living_room_placeholder = PoseStamped.from_list([3.56,4.06,0], [0,0,0,1])

milk_position = PoseStamped.from_list([0.5, 0.5, 0.5], [0, 0, 0, 1]) # TODO, find the actualy position


# for creating objects with unique names
global object_name_iteration

# --------------------------------------------------------------------------------------
# world = setup_world()
world2 = suturo_map.load_environment()

# test objects
spoon = STLParser(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "objects", "spoon.stl")).parse()
bowl = STLParser(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "objects", "bowl.stl")).parse()

# Can from there on be used with a normal tts_pub.say("bliblo")
tts_pub = tmc.TextToSpeechPublisher()

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
    global last_affirmation
    if len(lst) != 8:
        print("check if nlp_gpsr.response was changed, otherwise something went wrong" + str(len(lst)))
    else:
        match lst[0]:
            case "take":
                print("case take")
                take_obj_from_plcmt(lst[7], lst[1]),
            # MILESTONE 1
            case "Navigation":
                print("case navigation")
                driveToLocation(lst[7])
            case "affirm":
                last_affirmation = True
            case "deny":
                last_affirmation = False
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
def driveToLocation(location: String):
    goal = _location_from_string(location)
    start_nav(goal.position.x, goal.position.y)
    print(goal)

def driveToObject(object: String):
    goal = _find_obj_location(object)
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
def _location_from_string(location: String) -> PoseStamped:
    match location:
        case "living room":
            liv_room = queries.query_living_room_area(world2)
            liv_pose = PoseStamped().from_list(position=liv_room)
            return liv_pose #living_room_placeholder
        case "office":
            off_room = queries.query_office_area(world2)
            off_pose = PoseStamped().from_list(position=off_room)
            return off_pose
        case "kitchen":
            kit_room = queries.query_kitchen_area(world2)
            kit_pose = PoseStamped().from_list(position=kit_room)
            return kit_pose
        case "bedroom":
            bed_room =queries.query_bed_room_area(world2)
            bed_pose = PoseStamped().from_list(position=bed_room)
            return bed_pose
        case _:
            exception("unknown location")
            # return PoseStamped.from_list([0,0,0], [0,0,0,1])

def _find_obj_location(object: String = "") -> PoseStamped:
    object_pose = PoseStamped.from_list(position=(queries.query_object(suturo_map))) # TODO THIS METHOD DOES NOT EXIST, IT NEEDS TO BE FIXED, ELSE SWITCH CASE

    obj_pose = PoseStamped.from_list([0, 0, 0], [0, 0, 0, 1])
    match object:
        case "bowl":
            obj_pose = PoseStamped.from_list(position=(suturo_map.query_bowl()))
        case "milk":
            obj_pose = PoseStamped.from_list(position=(suturo_map.query_milk()))
        case "spoon":
            obj_pose = PoseStamped.from_list(position=(suturo_map.query_spoon()))
        case _:
            obj_pose = PoseStamped.from_list(position=(suturo_map.query_bowl()))
    return obj_pose

    return _find_class_from_string(obj)

"""
helper method
find object class from string, koennen wir spaeter von knowledge klauen

SOLLTE NICHT MEHR GEBRAUCHT WERDEN IN NEUEM PYCRAM
"""
def _find_obj_location(obj: String):
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

# TODO REFEREE interaction implementation
def referee_interaction() -> None:
    tts_pub.say("Hello, I am Toya. Nice to meet you. How may I help you today?")

    # TODO NLP pipeline here
    talkback_response = "" # ADD HERE NLP RESPONSE

    # retrieve task from the message type
    task = "" # for example describe

    # retrieve all other needed items
    obj_name = "" # if contains
    location = String() # if contains, else blank or empty string


    match task:
        case "describe":
            describing(obj_name, location)
        case "find":
            raise NotImplementedError("This feature is not implemented yet")
        case "tell":
            raise NotImplementedError("This feature is not implemented yet")
        case _:
            tts_pub.say("Sorry, I did not understand your request. Please try again.")


def check_affirm(sentence, nlp_node : nlp_gpsr.NLP_GPSR) :
    """
    gets nlp input and asks if its correct
    """
    if not isinstance(sentence, list):
        print("no response yet")
        return False

    sleep(2)
    print("Did I understand correctly? You want me to " + sentence.__str__())
    affirmation = nlp_node.talk_nlp()

    process_response(affirmation)

    sleep(1)

    print("affirmation: " + last_affirmation.__str__())
    return last_affirmation


"""
The base workflow is:
1. Navigate to desired location
2. Talk to human
3. Execute action
4. Talk back to human

What i need:
    Navigation [X]
        NavigateActionDescription or MoveTCPActionDescription for robot movement
        PoseStamped object with target coordinates and reference frame
        Environment/map knowledge for path planning
        Collision avoidance or world model setup

    Talk to Human (Initial)
        [X] Speech synthesis system or TTS (Text-to-Speech) interface
        TalkingMotionDescription or custom speech action
        Predefined message or dynamic text content
        Audio output device/speaker on robot

    Execute Action
        Task-specific action descriptions:
            PickUpActionDescription for grasping
            PlaceActionDescription for placing
            TransportActionDescription for moving objects
            OpenActionDescription/CloseActionDescription for containers
        Arms enum to specify which arm(s) to use
        Object designators for manipulation targets
        Motion planning and execution capabilities

    Talk Back to Human
        TTS system (same as initial talking)
        Feedback message content (success/failure/status)
        TalkingMotionDescription or speech interface
        Audio confirmation output

    Overall System Requirements
        Context for execution state management
        SequentialPlan to orchestrate all steps
        simulated_robot context manager for testing
        Error handling and recovery behaviors
        World setup using setup_world() or equivalent
"""
# TODO: Describtion implementation. Curr.
def describing(obj_name, room: Optional[String] = "") -> Description:

    if room != "":
        findWithoutRoom(obj_name)
    else:
        findWithoutRoom(obj_name, room)
    pass

def findWithoutRoom(obj_name):
    raise NotImplementedError("This feature is not implemented yet")

def findWithRoom(obj_name, room: Optional[String] = ""):
    tts_pub.say("I will now go to " + str(room))
    driveToLocation(room)
    sleep(2)
    SearchAction()
    tts_pub.say("I will now pick up the " + str(obj_name))
    try_pickup(suturo_map.query_object(obj_name))

def try_pickup(object: Object):
    if obj_name == "milk":
        pickup_milk(robot, milk_position, grasp)


    raise NotImplementedError("This feature is not implemented yet")


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
    #rclpy.init() # rausgenommen weil das schon in knowledge existiert TODO: testen ob nlp noch klappt

    #process_response(milestone1_response)

    # start_nav(3, 3)
    sleep(3)

    node = nlp_gpsr.NLP_GPSR()
    while rclpy.ok():
        object_name_iteration = 0
        wait = True
        resp = node.talk_nlp()
        print("Got response in gpsr_01.py: ", resp)
        process_response(resp)
        sleep(10)
        """
        if check_affirm(resp, node):
            print("Got response in gpsr_01.py: ", resp)
            process_response(resp)
            sleep(10)
        """

