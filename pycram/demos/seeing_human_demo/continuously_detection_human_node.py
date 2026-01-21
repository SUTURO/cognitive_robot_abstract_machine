import rclpy
from pycram.external_interfaces import robokudo
from time import sleep
from pycram.ros_utils.text_to_image import TextToImagePublisher


def main():
    """Testing"""
    text_pub = TextToImagePublisher()
    # Send goal
    position = robokudo.query_current_human_postion_in_continues()
    sleep(5)
    if position is not None:
        x = position.point.x
        y = position.point.y
        z = position.point.z
        print(x)
        text_pub.publish_text(f"Point x: {x} y: {y} z: {z}")
    else:
        text_pub.publish_text("No Human seen.")
    sleep(5)
    position = robokudo.query_current_human_postion_in_continues()
    sleep(5)
    if position is not None:
        x = position.point.x
        y = position.point.y
        z = position.point.z
        print(x)
        text_pub.publish_text(f"Point x: {x} y: {y} z: {z}")
    else:
        text_pub.publish_text("No Human seen.")
    sleep(5)
    position = robokudo.query_current_human_postion_in_continues()
    sleep(5)
    if position is not None:
        x = position.point.x
        y = position.point.y
        z = position.point.z
        print(x)
        text_pub.publish_text(f"Point x: {x} y: {y} z: {z}")
    else:
        text_pub.publish_text("No Human seen.")

    # Close every think
    robokudo.shutdown_robokudo_interface()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
