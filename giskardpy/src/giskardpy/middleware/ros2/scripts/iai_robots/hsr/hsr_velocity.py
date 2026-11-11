from datetime import timedelta

from giskardpy.middleware.ros2.giskard import Giskard
from giskardpy.middleware.ros2.scripts.iai_robots.hsr.configs import (
    HSRVelocityInterface,
)
from giskardpy.middleware.ros2.server_config import GiskardServerConfig
from giskardpy.qp.qp_controller_config import QPControllerConfig
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification


def create_giskard() -> Giskard:
    """
    Build a Giskard that commands the HSR, described by the description file
    :class:`~semantic_digital_twin.robots.hsrb.HSRB` declares.
    """
    return Giskard(
        world_source=WorldSpecification(
            robots=[RobotSpecification.specify_real_robot(HSRB)]
        ),
        robot_interface_config=HSRVelocityInterface(),
        server_config=GiskardServerConfig(debug_mode=False),
        qp_controller_config=QPControllerConfig(
            # reproduces the braking of the former prediction_horizon=15
            target_frequency=40,
            braking_time=timedelta(seconds=0.35),
        ),
    )


def main() -> None:
    """
    Run the Giskard :func:`create_giskard` builds as a ros node until ROS shuts down.
    """
    Giskard.launch(create_giskard)


if __name__ == "__main__":
    main()
