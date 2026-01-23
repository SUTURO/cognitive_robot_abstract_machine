import rclpy

from pycram.external_interfaces.tmc import GripperActionClient


def main(args=None):
    rclpy.init(args=args)

    # Erstelle den Action-Client
    action_client = GripperActionClient()

    # Beispiel: Gripper öffnen
    action_client.send_goal(effort=0.8)

    # Beispiel: Gripper schließen
    # action_client.send_goal(effort=-0.8)

    rclpy.spin(action_client)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
