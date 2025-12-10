from typing_extensions import Optional
from typing import Callable

# from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.publisher import Publisher
from rclpy.node import Node

from tmc_voice_msgs.msg import Voice
from ..ros.ros2.publisher import create_publisher

import logging
logger = logging.getLogger(__name__)

import rclpy
from tmc_voice_msgs.msg import Voice
from ..ros.ros2.publisher import create_publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy

class TextToSpeechPublisher():
    # Surprise its' initializations
    is_init = False

    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    tts_node: Node
    tts_pub: Publisher
    @staticmethod
    def init_talk_interface(func: Callable) -> Callable:


        """Ensures initialization of the talk interface before function execution."""
        def wrapper(*args, **kwargs):

            # Check if the interface is already initialized
            if TextToSpeechPublisher.is_init:
                return func(*args, **kwargs)
            try:
                from tmc_voice_msgs.msg import Voice        # This import is only needed if you intend to use voice
            except ImportError:
                logger.warning("Failed to import tmc_voice_msgs - package may not be installed")
                return None
            TextToSpeechPublisher.is_init = True

            rclpy.init()
            TextToSpeechPublisher.tts_node = rclpy.create_node("tts")
            TextToSpeechPublisher.tts_pub = create_publisher("/talk_request", Voice, TextToSpeechPublisher.tts_node, TextToSpeechPublisher.qos)

            logger.info("Successfully initialized tmc interface")

            return func(*args, **kwargs)
        return wrapper

    def get_tts_node(self)-> Node:
        return TextToSpeechPublisher.tts_node

    @init_talk_interface
    def say(self, content : Optional[str] = ""):
        """

        :param content: The message to be spoken
        :return:
        """

        msg = Voice()

        msg.language = 1
        msg.sentence = content

        TextToSpeechPublisher.tts_pub.publish(msg)
        return