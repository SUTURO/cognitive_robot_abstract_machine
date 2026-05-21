import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TtsPublisher:
    def __init__(self, node_name="tts_text_publisher", topic_name="/tts_text"):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self.publisher = self.node.create_publisher(String, topic_name, 10)

    def publish(self, text: str):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.node.get_logger().info(f"Published: '{text}'")
        # kurz spin, damit Nachricht sicher rausgeht
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    tts = TtsPublisher()
    tts.publish("Hallo i am toya")
    tts.shutdown()
