from typing import Optional
import logging
import rclpy
import time
from std_msgs.msg import Int32
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy

logger = logging.getLogger(__name__)


class ImageSwitchPublisher:
    """
    Publishes a new image to the display of HSR.

    create new Publisher like this:
    img_pub = ImageSwitchPublisher()

    publish new image like this:
    text_pub.publish_image(0)
    """
    def __init__(self, topic_name: str = "/media_switch_topic"):
        self.is_init = False
        self.topic_name = topic_name
        self.node: Optional[Node] = None
        self.publisher: Optional[Publisher] = None
        self._init_interface()


    def _init_interface(self):
        """
        Initializes the ROS node and publisher once.
        """
        if self.is_init:
            return

        if not rclpy.ok():
            rclpy.init()

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.node = rclpy.create_node("image_switch_publisher_node")
        self.publisher = self.node.create_publisher(Int32, self.topic_name, reliable_qos)

        # initialize screen with default picture of suturo logo
        for i in range(3):
            self.publish_image(0)

        self.is_init = True

        # wait a few seconds to ensure the screen is initialized
        time.sleep(2)

        logger.info("ImageSwitchPublisher initialized")


    def publish_image(self, image_number: int):
        """
        Publishes a new image to the display of HSR.
        numbers are mapped to pictures on the hsr

        existing images:
        0 = default Suturo Logo
        1 = image of happy robot
        2 = cute expression
        3 = happy expression
        TODO: add Enums
        """

        msg = Int32()
        msg.data = image_number
        for _ in range(3):
            self.publisher.publish(msg)
        time.sleep(1)

        logger.info("Published new text to image display")
