# Standard library imports
import json
import time
from time import sleep
from typing import Any
from abc import ABC, abstractmethod

# ROS2 related imports
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


# TODO: replace print with talking function


class NlpInterface(ABC):
    """
    Abstract base class defining a generic NLP interface.

    This class provides:
    - Storage for the last NLP output
    - A confirmation mechanism for user commands
    - An abstract method for filtering NLP responses
    """

    def __init__(self):
        # Initialize the ROS2 NLP node
        self.node = NlpNode()

        """
        stores the last NLP output
        """
        self.last_output = []

        """
        if multiple outputs are expected, all outputs from one input
        """
        self.all_last_outputs : list[list[Any]] = []

        """
        Stores the last confirmation result 
        (affirm / deny)
        """
        self.last_confirmation = []

        """
        Timeout (in seconds) for waiting for NLP responses, default: 15
        """
        self.timeout: int = 15

    # TODO: write functions for all challenges
    """
    List of intents and roles
        ---Intent List:---
        go 
        follow
        guide
        take
        place
        deliver
        talk
        answer
        meet
        greet
        remember
        describe
        count
        offer
        accompany
        affirm
        deny
        receptionist
        order

        ---Roles:---
        NatrualPerson = Person,  
        drink = Drink, 
        food = Food, 
        Room = Location, 
        DesignedFurniture = Furniture, 
        Interest = Hobby, 
        Clothing = Clothes, 
        Transportable = Item
    """

    @abstractmethod
    def filter_response(self, response: list[Any], challenge: str):
        """
        Filters and post-processes the NLP response depending on the challenge.

        Parameters:
            response (list[Any]): Parsed NLP response
            challenge (str): Challenge type (e.g. GPSR)
        """

        """ draft start
        match challenge:
            case "GPSR":
                raise NotImplementedError()
            """

        raise NotImplementedError()

    def input_confirmation_loop(self, tries: int):
        """
        Repeatedly asks the user for a command and confirms it.

        Behavior:
            - Ask for user input via NLP
            - Ask for confirmation
            - Accept input on affirmation
            - Retry on denial or retry on no response

        Parameters:
            tries (int): Maximum number of attempts

        Returns:
            list: Last accepted NLP output
        """
        for tries in range(0, tries):
            print("Say what you want me to do:")

            # Start NLP listening and store result
            self.start_nlp()

            # If no NLP output was received, retry
            if self.last_output is None:
                continue

            sleep(1)

            print("Confirm, what you want me to do:")
            # Ask for confirmation
            conf = self.confirm_last_response()
            sleep(1)

            # Exit loop if user confirms
            if conf:
                break

        return self.last_output

    def start_nlp(self):
        """
        Starts the NLP process and stores the result in last_output and all_last_outputs.
        """
        (self.last_output, self.all_last_outputs) = self.node.talk_nlp(timeout=self.timeout)

    def confirm_last_response(self):
        """
        Asks the user to confirm the previously understood command. If there is no previous command,
        return false.

        Returns:
            bool | None:
                True  -> User affirmed
                False -> User denied
                None  -> Counts  as denied
        """
        if self.last_output is None:
            return False

        # Ask user for confirmation
        print("Did I understand correctly, you want me to " + self.last_output[0])

        # Listen for confirmation response
        self.last_confirmation = NlpNode.talk_nlp(self.node, timeout=self.timeout)
        if self.last_confirmation is None:
            return False
        elif self.last_confirmation[1] == "affirm":
            return True
        elif self.last_confirmation[1] == "deny":
            return False

        # Fallback return value
        return False


class NlpNode(Node):
    """
    ROS2 node that interfaces with a speech/NLP system.

    Responsibilities:
    -----------------
    - Publish a trigger message to start the speech recognition/NLP pipeline.
    - Subscribe to the processed NLP output.
    - Parse NLP output into a structured Python list.
    - Expose a blocking `talk_nlp()` call that waits for NLP results.
    """

    def __init__(self):

        # Initialize ROS2 node with name "nlp"
        super().__init__('nlp')

        # Publisher to trigger NLP listening
        self.nlp_pub = self.create_publisher(
            String,
            '/startListener',
            10
        )

        # Subscriber to receive NLP output
        self.sub_nlp = self.create_subscription(
            String,
            'nlp_out',
            self._data_callback,
            10
        )
        # Stores the parsed NLP response
        self.response = None

        # Indicates whether a callback was received
        self.callback = False


    def parse_nlp_response(self, data: str):
        """
        Attempts to parse incoming NLP output as JSON.

        Parameters:
            data (str): JSON string from NLP module.

        Returns:
           dict | None: Parsed JSON or None on failure.
        """

        self.get_logger().info(f"NLP output: {data}")
        try:
            return json.loads(data)
        except json.JSONDecodeError as e:
            self.get_logger().warning(f"decode failed: {e}")
        except KeyError as e:
            self.get_logger().warning(f"missing key: {e}")
        except TypeError as e:
            self.get_logger().warning(f"Invalid type while parsing: {e}")

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
        ##### EXAMPLE SENTENCE ####
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
        try:
            parsed = json.loads(json_string)
            sentence = parsed['sentence']
            intent = parsed.get('intent')
            entities = parsed.get('entities')

            entity_elems = []
            for entity in entities:
                entity_elem = [entity.get('role'), entity.get('value'), entity.get('entity'),
                               entity.get('propertyAttribute'), entity.get('actionAttribute'),
                               entity.get('numberAttribute')]

                entity_elems.append(entity_elem)

            self.response = [sentence, intent, entity_elems]
        except (ValueError, SyntaxError, IndexError) as e:
            self.get_logger().error(f"Error parsing string: {e}")

    def talk_nlp(self, timeout: int):
        """
        Triggers the NLP system and waits synchronously for a response.

        Parameters:
            timeout (int): Maximum waiting time in seconds.

        Returns:
            list | None:
                - Parsed NLP response list if successful and all outputs in 1.5 seconds
                - None if no response arrives within timeout
        """
        # Initial delay before triggering NLP
        sleep(1)

        # Trigger NLP listening
        self._start_listening()

        # Use a single-threaded executor to spin the node
        executor = SingleThreadedExecutor()
        executor.add_node(self)

        start_time = time.time()

        # Wait until a response arrives or timeout is reached
        while not self.response and (time.time() - start_time < timeout):
            executor.spin_once(timeout_sec=0.1)

        all_out : list[list[Any]] = []

        if self.response:
            # sometimes multiple outputs, have to wait to ensure all of them are received
            old_res = self.response
            all_out.append(self.response)
            second_start_time = time.time()
            while (time.time() - second_start_time) < 2:
                new_res = self.response
                if new_res != old_res:
                    res = self.response
                    all_out.append(res)
                    print (self.response)
                    old_res = new_res
                executor.spin_once(timeout_sec=0.05)
            print(all_out)

            self.get_logger().info(f"Received response: {self.response}")

            # Save and reset response
            resp = self.response
            self.response = None
            return resp, all_out
        else:
            self.get_logger().info(f"No response received within timeout")
            print("Sorry, I couldn't understand.")
            return None

    def _start_listening(self):
        """
        Sends a signal to the NLP system to start listening for speech.

        Behavior:
            - Publishes a blank string on /startListener
            - This begins the external NLP pipeline
        """
        self.get_logger().info(f"NLP start")
        msg = String()
        msg.data = ""  # entspricht "{data: ''}" in CLI

        # send message once
        self.nlp_pub.publish(msg)
        self.get_logger().info(f"Publishing once: {msg}")
        print("speak now: ..................")


