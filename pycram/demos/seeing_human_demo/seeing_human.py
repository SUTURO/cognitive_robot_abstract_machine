import ast
import json
import time
from typing import List, Dict, Any
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from pycram.datastructures.enums import ImageEnum

from time import sleep

import logging
from pycram.external_interfaces import tmc
from semantic_digital_twin.robots.hsrb import HSRB

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from pycram.external_interfaces import robokudo

logger = logging.getLogger(__name__)
_ros2_node = Node("robokudo_interface")


def saw_human():
    logger.info("Saw Human")


def checking_for_human():
    """
    Returns:
    Cheking if a human is in front of the camara.
    """
    try:
        human = robokudo.query_human
        print(human)
    except:
        logger.info("Error")


'''
class Perception_Interface(Node):
    def init(self):

        # rclpy.init()
        super().init('perception_interface')

        # Publisher
        self.nlp_pub = self.create_publisher(
            String,
            '/robokudo/query',
            10
        )

        # Subscriber
        self.sub_nlp = self.create_subscription(
            String,
            'robokudo_msgs/action/Query',
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
        Extracts structured information and stores it in self.response.

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
'''
