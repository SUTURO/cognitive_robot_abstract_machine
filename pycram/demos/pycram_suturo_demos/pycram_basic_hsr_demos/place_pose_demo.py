import rclpy

from demos.pycram_suturo_demos.helper_methods_and_useful_classes.place_pose import (
    get_pose_for_object_on_body,
)
from demos.pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
    robot_setup,
)


def simulation_demo():
    rclpy.init()

    setup_result = robot_setup(
        simulation=True, with_objects=True, with_perception=False
    )
    world, robot_view, context = (
        setup_result.world,
        setup_result.robot_view,
        setup_result.context,
    )

    location = get_pose_for_object_on_body(
        for_object=world.get_body_by_name("milk.stl"),
        body=world.get_body_by_name("diningTable_body"),
        world=world,
        link_is_center_link=True,
    )
    print(location)


# Not tested yet
def real_demo():
    # Not sure if real robot need rclpy.init(). Uncomment if needed.
    # rclpy.init()

    setup_result = robot_setup(
        simulation=False, with_objects=True, with_perception=False
    )
    world, robot_view, context = (
        setup_result.world,
        setup_result.robot_view,
        setup_result.context,
    )

    location = get_pose_for_object_on_body(
        for_object=world.get_body_by_name("milk.stl"),
        body=world.get_body_by_name("diningTable_body"),
        world=world,
        link_is_center_link=True,
    )
    print(location)


if __name__ == "__main__":
    simulation_demo()
