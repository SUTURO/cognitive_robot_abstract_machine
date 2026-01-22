import rclpy

from rclpy.node import Node

import pycram
import numpy as np
from pycram.datastructures.enums import ImageEnum, Arms
from pycram.external_interfaces import robokudo
from pycram.external_interfaces import nav2_move
from pycram.failures import SensorMonitoringCondition
from pycram.robot_plans import (
    MoveJointsMotion,
    MoveTorsoAction,
    NavigateAction,
    LookAtAction,
    ParkArmsAction,
)
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor
from pycram.failures import SensorMonitoringCondition
from pycram.ros_utils.text_to_image import TextToImagePublisher

###########################################################################
# import tf
# from tf.transformations import quaternion_matrix
from collections import OrderedDict
from std_msgs.msg import String
from pycram.datastructures.pose import Pose
from pycram.process_module import real_robot

###########################################################################

# Initialize the necessary components
rclpy.init()
_demo_node = Node("restaurant_demo02")


def _sleep_ros(seconds: float) -> None:
    """ROS2-friendly sleep that keeps the executor responsive."""
    rclpy.spin_once(_demo_node, timeout_sec=float(seconds))


(
    tf_listener,
    marker,
    world,
    v,
    text_to_speech_publisher,
    image_switch_publisher,
    move,
    robot,
    kitchen,
) = startup()

text_to_img_publisher = TextToImagePublisher()

_demo_node.get_logger().info("Waiting for action server")
_demo_node.get_logger().info("You can start your demo now")
response = [None, None]
stopped = False
#
callback = False
# pub_nlp = rospy.Publisher('/startListener', String, queue_size=16)
# nlp = NLPRestaurant()
# nlp_test = NLPRestaurant_refactord()
odom_response = None
# pose_dict = OrderedDict()
moving = False
stuck = False

###########################################################################

# Initialize global variable
global human_pose
last_position = None
stuck_time = _demo_node.get_clock().now()
human_pose = None
timeout = 25
customers = list()
global customerCounter
customerCounter = 0
fts = ForceTorqueSensor(robot_name="hsrb")

global kitchen_pose


def look_around(increase: float):
    """
    Make robot look continuously from left to right. Stops if a human is perceived.
    MoveJointMotion because without it the pose that perception returns is wrong.
    :param: increase: The increments in which Toya should look around.
    """

    global human_pose
    human_pose = None

    x = -0.5
    while x <= 1:

        MoveJointsMotion(["head_pan_joint"], [x]).perform()
        try:
            human_pose = robokudo.query_waving_human()
        except pycram.failures.PerceptionObjectNotFound:
            print("oh no, no waving human was found")
        if human_pose:
            # austauschen mit talking-motion
            print("found human")
            break

        x += increase
        if x == 1:
            x = -0.5


def monitor_func():
    """
    monitors force torque sensor of robot and throws
    Condition if a significant force is detected (e.g. the gripper is pushed down)
    """
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        _demo_node.get_logger().warn("sensor exception")
        return SensorMonitoringCondition

    return False


def confirmation():
    """
    The robot will wait until its hand is pushed down and then scan the
    environment for a human
    """
    try:

        MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()

        image_switch_publisher.pub_now(ImageEnum.PUSHBUTTONS.value)
        # plan = Code(lambda: rospy.sleep(1)) * 999999 >> Monitor(monitor_func)
        # plan.perform()
    except SensorMonitoringCondition:
        print("done")
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        _sleep_ros(2)
        return


def laser_callback(msg):
    global stuck
    min_dist = np.min(msg.ranges)
    if moving:
        if min_dist < 0.3:
            _demo_node.get_logger().warn("Obstacle too close")
            if stuck:
                print("There is an obstacle in my way.")
                # TalkingMotion("There is an obstacle in my way.").perform()
                _sleep_ros(2)


def cmd_vel_callback(msg):
    global stuck
    if moving:
        if abs(msg.linear.x) < 0.00 and abs(msg.angular.z) < 0.00:
            _demo_node.get_logger().warn("No Movement detected")
            stuck = True


def demo(step: int):
    # rospy.Subscriber("/hsrb/odom", Odometry, cmd_vel_callback)
    # rospy.Subscriber("/hsrb/base_scan", LaserScan, laser_callback)
    global customer, customerCounter, kitchen_pose, human_pose
    with real_robot:

        talk = True
        start_pose = robot.get_pose()
        kitchen_pose = start_pose
        MoveJointsMotion(["wrist_flex_joint"], [-1.6]).perform()
        image_switch_publisher.pub_now(ImageEnum.HI.value)
        _sleep_ros(2)

        if len(customers) == 0:
            # TalkingMotion("start restaurant demo").perform()
            _sleep_ros(2)
            # TalkingMotion("Please push down my gripper to start the demo ").perform()
            image_switch_publisher.pub_now(ImageEnum.PUSHBUTTONS.value)

            try:
                plan = Code(lambda: _sleep_ros(1)) * 99999999 >> Monitor(monitor_func)
                plan.perform()
            except SensorMonitoringCondition:
                image_switch_publisher.pub_now(ImageEnum.HI.value)

        if step <= 0:
            MoveJointsMotion(["head_pan_joint"], [0.0]).perform()
            MoveJointsMotion(["head_tilt_joint"], [0.0]).perform()
            config_for_placing = {
                "arm_lift_joint": -1,
                "arm_flex_joint": -0.16,
                "arm_roll_joint": -0.0145,
                "wrist_flex_joint": -1.417,
                "wrist_roll_joint": 0.0,
            }
            pakerino(config=config_for_placing)
            MoveTorsoAction([0.2]).resolve().perform()
            ParkArmsAction([Arms.LEFT]).resolve().perform()
            # TalkingMotion("Please wave, so that I can perceive you").perform()

        if step <= 1:
            image_switch_publisher.pub_now(ImageEnum.WAVING.value)
            annotator = get_used_annotator_list(Demos.RESTAURANT, as_topic_names=False)

            isp = ImageSendPublisher(sub_topic=annotator[0])
            isp.activate_subscriber()
            _sleep_ros(2)
            look_around(0.5)
            MoveTorsoAction([0]).resolve().perform()

            if human_pose is not None:
                # Changes image to the results of perception

                image_switch_publisher.pub_now(ImageEnum.PERCEPTION_RESULT.value)
                _sleep_ros(2)
                drive_pose = transform_camera_to_x(human_pose, "head_rgbd_sensor_link")
                adjusted_pose = set_pose_in_front(drive_pose, 0.8)
                print(drive_pose)

                customerCounter += 1

                # customer = CustomerDescription(customerCounter, adjusted_pose)
                customer.set_pose(adjusted_pose)
                customers.append(customer)

            marker.publish(
                Pose.from_pose_stamped(drive_pose),
                color=[1, 1, 0, 1],
                name="human_waving_pose",
            )
            marker.publish(
                Pose.from_pose_stamped(adjusted_pose),
                color=[1, 0, 0, 1],
                name="adjusted_pose",
            )
            _sleep_ros(2.5)
            move.pub_now(adjusted_pose)
            # plan = Code(move.pub_now(navpose=drive_pose) | lol(drive_pose))
            # plan.perform()

            _sleep_ros(1)

        if step <= 2:  # Order step
            image_switch_publisher.pub_now(ImageEnum.ORDER.value)
            MoveTorsoAction([0.1]).resolve().perform()
            LookAtAction([Pose([robot.pose.position.x, robot.pose.position.y, 0.8])])
            _sleep_ros(1)
            # Timmi = CustomerDescription(id=1, pose=start_pose)
            # customer = Timmi
            # nlp_test.get_order(customer=customer)
            print(customer.order)
            _sleep_ros(2)
            if customer.order is not None:
                print("not implemented yet")
                # nlp_test.confirm_order(customer=customer)
        if step <= 3:  # Drive back step
            # TalkingMotion("I will drive back now and return with your order").perform()
            _sleep_ros(2.5)

            change_o = nav2_move.change_orientation(robot.get_pose())
            NavigateAction([change_o]).resolve().perform()
            _sleep_ros(2)
            image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
            MoveTorsoAction([0]).resolve().perform()
            _sleep_ros(2)

            order_kitchen_pose = nav2_move.change_orientation(kitchen_pose)

            move.pub_now(navpose=kitchen_pose)
            NavigateAction([order_kitchen_pose]).resolve().perform()

            _sleep_ros(2.5)
            print("order", customer.order)
            if len(customer.order) == 1:
                # TalkingMotion(f"Please prepare the order {customer.order[0][1]} {customer.order[0][0]}").perform()
                text_to_img_publisher.publish_text(
                    f"The order: {customer.order[0][1]} {customer.order[0][0]}"
                )
                _sleep_ros(2)
                image_switch_publisher.pub_now(ImageEnum.GENERATED_TEXT.value)
            elif len(customer.order) >= 2:
                # TalkingMotion("Please prepare the following order").perform()
                txt_order = ""
                for n in customer.order:
                    # TalkingMotion(f"{n[1]} {n[0]} ").perform()
                    _sleep_ros(1)
                    txt_order += f" {n[1]} {n[0]} " + "\n"
                text_to_img_publisher.publish_text(txt_order)
                _sleep_ros(2)
                image_switch_publisher.pub_now(ImageEnum.GENERATED_TEXT.value)
            # TalkingMotion("Please put the order into the tray in my gripper").perform()
            _sleep_ros(2)
            # TalkingMotion(f"Please push down my gripper, if the order is prepared and my display changed").perform()
            _sleep_ros(1)

            confirmation()

            _sleep_ros(3)
            # TalkingMotion("I will bring the order to the customer now").perform()
        if step <= 4:
            kitchen_to_cust_orientation = nav2_move.change_orientation(
                order_kitchen_pose
            )
            NavigateAction([kitchen_to_cust_orientation]).resolve().perform()
            _sleep_ros(2.5)
            move.pub_now(navpose=customer.pose)
            _sleep_ros(2.5)
            # TalkingMotion("Here is your order. Please take it out of my tray").perform()
            _sleep_ros(2)
            # TalkingMotion("Please push down my gripper, if you took your order and my display changed").perform()
            _sleep_ros(2)
            confirmation()
            # TalkingMotion("I will drive back now to search for new customers").perform()
            _sleep_ros(1)
            cust_to_kitchen_orentation = nav2_move.change_orientation(robot.get_pose())
            NavigateAction([cust_to_kitchen_orentation]).resolve().perform()
            move.pub_now(navpose=kitchen_pose)
            while len(customers) <= 2:
                print(len(customers))
                demo(0)


demo(0)
