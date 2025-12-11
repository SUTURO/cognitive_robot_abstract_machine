#!/usr/bin/env python3
import rclpy
from rclpy import Future
from rclpy.node import Node
from rclpy.action import ActionClient

from robokudo_msgs.action import Query


class RobokudoQueryClient(Node):

    def __init__(self):
        super().__init__("robokudo_query_client")
        self._client = ActionClient(self, Query, "/robokudo/query")

    async def send_empty_query(self):
        """
        Reproduziert:
        ros2 action send_goal /robokudo/query robokudo_msgs/action/Query
        """
        self._client.wait_for_server()
        goal_msg = Query.Goal()
        self.get_logger().info("Send *empty Query* ...")
        send_goal_future = self._client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected!")
            return

        result = await goal_handle.get_result_async()
        self.get_logger().info(f"Got result: {result.result}")

    async def send_human_query(self) -> Future:
        """
        Reproduziert:
        ros2 action send_goal /robokudo/query robokudo_msgs/action/Query "{obj: {type: 'human'}}"
        """
        self._client.wait_for_server()
        goal_msg = Query.Goal()

        # Set Query object human
        goal_msg.obj.type = "human"

        self.get_logger().info("Send Query: obj.type = 'human' ...")
        send_goal_future = self._client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected!")
            return

        result = await goal_handle.get_result_async()
        self.get_logger().info(f"Got result: {result.result}")


def main():
    rclpy.init()
    node = RobokudoQueryClient()

    # Selection of empty Query or human
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "human":
        rclpy.spin_until_future_complete(node, node.send_human_query())
    else:
        rclpy.spin_until_future_complete(node, node.send_empty_query())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
