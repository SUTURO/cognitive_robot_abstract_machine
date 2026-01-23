import rclpy

from rclpy.action import ActionClient
from rclpy.lifecycle import Node
from tmc_control_msgs.action import GripperApplyEffort


class GripperActionClient(Node):
    def __init__(self):
        super().__init__("gripper_action_client")
        self._action_client = ActionClient(
            self, GripperApplyEffort, "/gripper_controller/grasp"
        )

    def send_goal(self, effort):
        """
        Sendet ein Ziel (Goal) an den Gripper Action Server
        :param effort: Der Effort-Wert für den Gripper (positiv für öffnen, negativ für schließen)
        """
        goal_msg = GripperApplyEffort.Goal()
        goal_msg.effort = effort  # Setze den Effort-Wert

        # Sende das Ziel an den Action-Server und warte auf eine Antwort
        self.get_logger().info(f"Sending goal with effort: {effort}")
        self._action_client.wait_for_server()

        # Sende das Ziel und erhalte eine Antwort
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback, wenn der Action-Server eine Antwort auf das Ziel sendet.
        """
        result = future.result()
        if result.accepted:
            self.get_logger().info("Goal accepted by the action server.")
        else:
            self.get_logger().error("Goal rejected by the action server.")

    def feedback_callback(self, feedback):
        """
        Callback, um Feedback vom Action-Server zu erhalten.
        """
        self.get_logger().info(f"Feedback received: {feedback.feedback}")
