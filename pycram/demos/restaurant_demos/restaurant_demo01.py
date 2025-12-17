import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RestaurantDemoListener(Node):

    def __init__(self):
        super().__init__("restaurant_demo_listener")

        self.options = {
            "one": 1,
            "two": 2,
            "three": 3,
            "four": 4,
            "five": 5,
            "six": 6,
            "seven": 7,
            "eight": 8,
            "nine": 9,
            "ten": 10,
            "1": 1,
            "2": 2,
            "3": 3,
            "4": 4,
            "5": 5,
            "6": 6,
            "7": 7,
            "8": 8,
            "9": 9,
            "10": 10,
        }

        self.done = False
        self.nlp_pub = self.create_publisher(String, "/startListener", 16)
        self.sub_nlp = self.create_subscription(String, "nlp_out", self.data_cb, 16)
        self.get_logger().info("started listener")

    def data_cb(self, msg):
        """Callback for incoming NLP-Data"""
        print("=== NEW ORDER RECEIVED ===")
        response = self.parse_json_string(msg.data)
        self.print_order(response)
        print("================================")

        intent, order = response
        if intent == "Order" and order:
            self.get_logger().info("Order received, shutting down node.")
            self.done = True

    def parse_json_string(self, json_string: str):
        """Parsing NLP JSON to Intent and Order"""
        try:
            parsed = json.loads(json_string)
            intent = parsed.get("intent", "unknown")
            entities = parsed.get("entities", [])
            order = []

            if intent == "Order":
                for entity in entities:
                    item = entity.get("value", "unknown")
                    amount_list = entity.get("numberAttribute", [])
                    amount_raw = amount_list[0] if amount_list else 1
                    num = self.options.get(str(amount_raw).lower(), 1)
                    order.append((item, num))

            return intent, order

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON error: {e}")
            return "error", []

    def print_order(self, response):
        """Returns the order in the terminal"""
        intent, order = response

        if order:
            # Bestätigungssatz mit allen Items
            item_strings = [f"{amount}x {item}" for item, amount in order]
            items_text = "and ".join(item_strings)
            print(f"So you would like to {intent}: {items_text}")
        else:
            print("no items found")

        print("?")  # Bestätigungsfrage

    def start_listening(self):
        """sends 'start listening' message"""
        msg = String()
        msg.data = "start listening"
        self.nlp_pub.publish(msg)
        self.get_logger().info("started listening")


def main(args=None):
    rclpy.init(args=args)
    node = RestaurantDemoListener()

    try:
        rclpy.spin_once(node, timeout_sec=2.0)
        node.start_listening()

        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info("end listening")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
