import ast
import json
import time
from typing import List, Dict, Any
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from pycram.datastructures.enums import ImageEnum

from time import sleep

# fast komplett übernommen von give me a hand, umgeschrieben für ROS2

confirmation = []
callback = False
timeout = 15



"""
ROS2 node that interfaces with a speech/NLP system for the GPSR challenge.

Responsibilities:
-----------------
- Publish a trigger message to start the speech recognition/NLP pipeline.
- Subscribe to the processed NLP output.
- Parse NLP output into a structured Python list.
- Expose a blocking `talk_nlp()` call that waits for NLP results.
"""
class NLP_GPSR(Node):
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
                intent,
                item,
                item_entity,
                item_property,
                item_action,
                item_number,
                to_who,
                destination
            ]

        Notes:
            - Missing entities default to empty strings.
            - Handles parsing errors gracefully.
        """
        global intent, item, item_entity, item_property, item_action, item_number, to_who, destination
        print(json_string)
        try:
            parsed = json.loads(json_string)
            intent = parsed.get('intent')
            entities = parsed.get('entities')
            for entity in entities:
                item = ""
                item_entity = ""
                item_property = ""
                item_action = ""
                item_number = ""
                to_who = ""
                destination = ""
                if entity.get('role') == "Item":
                    item = entity.get('value')
                    item_entity = entity.get('entity')
                    item_property = entity.get('propertyAttribute')
                    item_action = entity.get('actionAttribute')
                    item_number = entity.get('numberAttribute')
                elif entity.get('role') == "BeneficiaryRole":
                    to_who = entity.get('value')
                elif entity.get('role') == "Location":
                    destination = entity.get('value')


                self.response = [intent, item, item_entity, item_property, item_action, item_number, to_who, destination]
        except (ValueError, SyntaxError, IndexError) as e:
            print(f"Error parsing string: {e}")
            #self.response = ["Transporting", "long table"]


    def talk_nlp(self, timeout=15):
        """
        Triggers the NLP system and waits synchronously for a response.

        Parameters:
            timeout (int): Maximum waiting time in seconds.

        Returns:
            list | None:
                - Parsed NLP response list if successful
                - None if no response arrives within timeout
        """

        sleep(2)

        self._start_listening()

        executor = SingleThreadedExecutor()
        executor.add_node(self)

        start_time = time.time()
        while not self.callback and (time.time() - start_time < timeout):
            executor.spin_once(timeout_sec=0.1)
        if self.callback:
            print("Received response:", self.response)
            return self.response
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

        # Nachricht einmal senden
        self.nlp_pub.publish(msg)
        self.get_logger().info(f"Publishing once: {msg}")
        print("speak now: ..................")

        # kurze Pause, um sicherzugehen, dass Nachricht verschickt wird
        # rclpy.spin_once(self.node, timeout_sec=0.5)

# ======================================================================

def main():
    """
    Entry point for standalone testing.

    - Initializes ROS2
    - Instantiates NLP node
    - Requests NLP input once
    - Prints parsed result
    """

    rclpy.init()
    nlp = NLP_GPSR()
    response = nlp.talk_nlp()
    print("Final response:", response)
    nlp.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()