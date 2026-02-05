#!/usr/bin/env python3
import rclpy
from rclpy import Future
from rclpy.node import Node
from rclpy.action import ActionClient

from robokudo_msgs.action import Query


class RobokudoActionClient(Node):

    def __init__(self):
        super().__init__("robokudo_action_client")
        self._action_client = ActionClient(self, Query, "/robokudo/query")

    def send_query(self, order):
        """
        Reproduziert:
        ros2 action send_goal /robokudo/query robokudo_msgs/action/Query "{obj: {type: 'human'}}"
        """
        goal_msg = Query.Goal()

        # Set Query object to order
        goal_msg.obj.type = order
        self._action_client.wait_for_server()

        self.get_logger().info("Send Query: obj.type = 'human' ...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(result)
        rclpy.shutdown()


def main():
    node = RobokudoActionClient()
    node.send_query("human")
    # rclpy.spin_until_future_complete(node, )
    # Selection of empty Query or human
    """
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "human":
        
    else:
        rclpy.spin_until_future_complete(node, node.send_empty_query())
    """
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
