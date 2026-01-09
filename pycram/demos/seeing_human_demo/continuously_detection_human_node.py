import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pycram.external_interfaces import robokudo
from time import sleep


class CONTINUOUS_HUMAN_DETECTION(Node):
    def __init__(self):

        super().__init__("continous_human_detection")

        self.sub_chd = self.create_subscription(
            String, "/human_pose", self._data_callback, 10
        )

    def _data_callback(self, data):
        print(data)
        # self.parse_json_string(data.data)

    def parse_json_string(self, json_string: str):
        print(json_string)
        try:
            parsed = json.loads(json_string)
            point = parsed.get("point")
        except () as e:
            print(f"Error parsing string:{e}")


# ----------------------------------------------------------------


def main():
    """Testing"""
    rclpy.init()
    chd = CONTINUOUS_HUMAN_DETECTION()
    sleep(10)
    robokudo.cancel_goal()
    chd.destroy_node()
    rclpy.shutdown()
