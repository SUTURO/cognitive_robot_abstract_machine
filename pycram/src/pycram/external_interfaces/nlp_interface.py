import ast
import json
import time
from logging import exception
from typing import List, Dict, Any
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from abc import ABC, abstractmethod

from sympy import false

from pycram.datastructures.enums import ImageEnum

from time import sleep



callback = False

class NlpInterface(ABC):

    last_output = []
    last_confirmation = []
    timeout : int = 15

    def __init__(self):
        self.node = NlpNode()

    # TODO: für die anderen Gruppen functions vorschreiben zum Filtern
    @abstractmethod
    def filter_response(self, response : list[Any], challenge : str):
        match challenge:
            case "GPSR":
                NotImplemented()

        NotImplemented()

    def get_and_check_input(self, tries : int):
        for i in range(0, tries):
            self.start_nlp()
            sleep(2)
            if self.confirm_last_response():
                return self.last_output

        return None

    # TODO: Funktion, die bereits eine start_nlp/confirm_last_response Schleife erzeugt, nimmt parameter n für die Anzahl versuche entgegen


    # starts input and saves output to last_output
    def start_nlp(self):
        self.last_output = NlpNode.talk_nlp(self.node, timeout=self.timeout)

    def confirm_last_response(self):
        # TODO: replace print with talking function
        for i in range (0, 10):
            print("Did I understand correctly, you want me to " + self.last_output[0])
            self.last_confirmation = NlpNode.talk_nlp(self.node, timeout=self.timeout)
            # TODO: check mit nlp, ob affirm und deny für alle trainingsmodelle (alle challenges) gilt
            if self.last_confirmation[1] == "affirm":
                return True
            elif self.last_confirmation[1] == "deny":
                return False
            else:
                # TODO: put in talking function
                print("Sorry, I couldn't understand you, let's try again.")

        return false
        
        
"""
ROS2 node that interfaces with a speech/NLP system for the GPSR challenge.

Responsibilities:
-----------------
- Publish a trigger message to start the speech recognition/NLP pipeline.
- Subscribe to the processed NLP output.
- Parse NLP output into a structured Python list.
- Expose a blocking `talk_nlp()` call that waits for NLP results.
"""


class NlpNode(Node):
    def __init__(self):

        # rclpy.init()
        super().__init__('nlp_gpsr')

        # Publisher
        self.nlp_pub = self.create_publisher(
            String,
            '/startListener',
            10
        )

        # Subscriber
        self.sub_nlp = self.create_subscription(
            String,
            'nlp_out',
            self._data_callback,
            10
        )
        self.response = None
        self.callback = False

    """
    Attempts to parse incoming NLP output as JSON.

    Parameters:
        data (str): JSON string from NLP module.

    Returns:
       dict | None: Parsed JSON or None on failure.
    """

    def parse_nlp_response(self, data: str):
        print(data)
        try:
            return json.loads(data)
        except:
            rclpy.logwarn("Failed to parse NLP")
            return None

    def _data_callback(self, data):
        """
        ROS2 subscriber callback.

        Called when NLP output is published on 'nlp_out'.
        Extracts structured information and stores it in `self.response`.

        Parameters:
            data (std_msgs.msg.String): The incoming NLP data.
        """
        self.parse_json_string(data.data)
        self.callback = True
        ##### EXAMPL SENTENCE ####
        # {"sentence": "Please bring the object to the kitchen counter .",
        # "intent": "Transporting",
        # "entities":
        # [{"role": "Item", "value": "object", "entity": "Transportable", "propertyAttribute": [], "actionAttribute": [], "numberAttribute": []},
        # {"role": "Destination", "value": "kitchen counter", "entity": "DesignedFurniture", "propertyAttribute": [], "actionAttribute": [], "numberAttribute": []}]}

    def parse_json_string(self, json_string: str):
        """
        Parses NLP JSON output into a standard list format used by the robot logic.

        Parameters:
            json_string (str): Raw JSON string from NLP.

        Output Format:
            response = [
                sentence,
                intent,
                entities[entity_elem[]]
            ], mit
            entity_elem = [role, value, entity, propertyAttributes[], actionAttributes[], numberAttributes[]

        Notes:
            - Missing entities default to empty strings.
            - Handles parsing errors gracefully.
        """
        global sentence, intent, entities
        print(json_string)
        try:
            parsed = json.loads(json_string)
            sentence = parsed['sentence']
            intent = parsed.get('intent')
            entities = parsed.get('entities')

            entity_elems = []
            for entity in entities:
                entity_elem = [entity.get('role'), entity.get('value'), entity.get('entity'), 
                               entity.get('propertyAttribute'),entity.get('actionAttribute'), 
                               entity.get('numberAttribute')]

                entity_elems.append(entity_elem)

            self.response = [sentence, intent, entity_elems]
            # TODO: remove this
            print(self.response)
        except (ValueError, SyntaxError, IndexError) as e:
            # TODO: das schöner machen
            print(f"Error parsing string: {e}")



    def talk_nlp(self, timeout : int):
        """
        Triggers the NLP system and waits synchronously for a response.

        Parameters:
            timeout (int): Maximum waiting time in seconds.

        Returns:
            list | None:
                - Parsed NLP response list if successful
                - None if no response arrives within timeout
        """
        sleep(4)

        self._start_listening()

        executor = SingleThreadedExecutor()
        executor.add_node(self)

        start_time = time.time()
        while not self.response and (time.time() - start_time < timeout):
            executor.spin_once(timeout_sec=0.1)
        if self.response:
            print("Received response:", self.response)
            resp = self.response
            self.response = None
            return resp
        else:
            print("No response received within timeout")
            return None

    def _start_listening(self):
        """
        Sends a signal to the NLP system to start listening for speech.

        Behavior:
            - Publishes a blank string on /startListener
            - This begins the external NLP pipeline
        """
        print("NLP start")
        msg = String()
        msg.data = ""  # entspricht "{data: ''}" in CLI

        # send message once
        self.nlp_pub.publish(msg)
        self.get_logger().info(f"Publishing once: {msg}")
        print("speak now: ..................")



