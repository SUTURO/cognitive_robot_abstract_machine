from typing import Optional
import logging
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

logger = logging.getLogger(__name__)


class TextToImagePublisher:
    """Publishes a text message to the display of HSR.

    Usage:
        text_pub = TextToImagePublisher(node=my_node)
        text_pub.publish_text("Hello my name is Toya")

    If `node` is not provided, this class creates its own node.
    """

    def __init__(
        self,
        topic_name: str = "/head_display/text_to_image",
        node: Optional[Node] = None,
    ):
        self.is_init = False
        self.topic_name = topic_name
        self.node: Optional[Node] = node
        self.publisher: Optional[Publisher] = None
        self._init_interface()

    def _sleep_ros(self, seconds: float) -> None:
        """ROS2-friendly sleep that keeps the executor responsive."""
        if not self.node:
            return
        rclpy.spin_once(self.node, timeout_sec=float(seconds))

    def _init_interface(self):
        """Initializes the ROS node and publisher once."""
        if self.is_init:
            return

        if not rclpy.ok():
            rclpy.init()

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        if self.node is None:
            self.node = rclpy.create_node("text_publisher_node")

        self.publisher = self.node.create_publisher(
            String, self.topic_name, reliable_qos
        )

        # prime the display / transport
        for _ in range(3):
            self.publish_text("")

        self.is_init = True

        # wait a bit to ensure the screen is initialized
        self._sleep_ros(2)

        try:
            self.node.get_logger().info("TextImagePublisher initialized")
        except Exception:
            logger.info("TextImagePublisher initialized")

    def publish_text(self, text: str):
        """Publishes a text message to the display of HSR."""
        if self.publisher is None:
            self._init_interface()
        if self.publisher is None:
            return

        msg = String()
        msg.data = text
        for _ in range(3):
            self.publisher.publish(msg)

        self._sleep_ros(1)

        try:
            self.node.get_logger().info("Published new text to image display")
        except Exception:
            logger.info("Published new text to image display")
