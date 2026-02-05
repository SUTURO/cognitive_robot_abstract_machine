
"""
Base initialization module for Toyota HSR pickup actions.
Initializes the world, robot, and all necessary components for performing pickup tasks.
"""

import rclpy
from suturo_resources.suturo_map import load_environment
from demos.pycram_suturo_demo.simulation_setup import setup_hsrb_in_environment

from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms
from pycram.process_module import simulated_robot
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.world import World
from semantic_digital_twin.reasoning.world_reasoner import WorldReasoner

# Removed this import - it's causing the issue

class HSRPickupInitializer:
    """
    Handles initialization of the Toyota HSR robot for pickup operations.
    """

    def __init__(self, with_viz: bool = True):
        """
        Initialize the HSR robot in the environment.

        Args:
            with_viz: Whether to enable visualization (default: True)
        """
        self.world = None
        self.robot_view = None
        self.context = None
        self.hsrb = None
        self.viz = None
        self.with_viz = with_viz

        self._initialize_ros()
        self._setup_environment()
        self._setup_robot()
        self._run_world_reasoning()

    def _initialize_ros(self):
        """Initialize ROS if not already initialized."""
        if not rclpy.ok():
            rclpy.init()

    def _setup_environment(self):
        """Setup the environment and robot."""
        result = setup_hsrb_in_environment(
            load_environment=load_environment,
            with_viz=self.with_viz
        )

        self.world = result.world
        self.robot_view = result.robot_view
        self.context = result.context
        self.viz = result.viz

    def _setup_robot(self):
        """Setup the HSRB semantic robot description."""
        self.hsrb = HSRB.from_world(self.world)

    def _run_world_reasoning(self):
        """Run world reasoning to establish semantic relationships."""
        with self.world.modify_world():
            world_reasoner = WorldReasoner(self.world)
            world_reasoner.reason()

    def get_arm(self):
        """
        Get the robot's arm.

        Returns:
            The arm component of the HSR robot
        """
        return list(self.hsrb.arms)[0] if self.hsrb.arms else None

    def get_gripper(self):
        """
        Get the robot's gripper (end effector).

        Returns:
            The gripper component of the HSR robot
        """
        arm = self.get_arm()
        return arm.manipulator if arm else None

    def get_neck(self):
        """
        Get the robot's neck for perception.

        Returns:
            The neck component of the HSR robot
        """
        return list(self.hsrb.necks)[0] if self.hsrb.necks else None

    def get_torso(self):
        """
        Get the robot's torso for height adjustment.

        Returns:
            The torso component of the HSR robot
        """
        return list(self.hsrb.torsos)[0] if self.hsrb.torsos else None

    def get_arm_enum(self):
        """
        Get the arm enumeration value for action descriptions.

        Returns:
            Arms.LEFT (HSR uses LEFT arm designation)
        """
        return Arms.LEFT

    def get_cameras(self):
        """
        Get all cameras available on the robot.

        Returns:
            dict: Dictionary with camera names and camera objects
        """
        cameras = {}

        arm = self.get_arm()
        if arm and arm.sensors:
            for sensor in arm.sensors:
                cameras[f"hand_camera"] = sensor

        neck = self.get_neck()
        if neck and neck.sensors:
            for i, sensor in enumerate(neck.sensors):
                cameras[f"head_camera_{i}"] = sensor

        return cameras

    def print_status(self):
        """Print initialization status and available components."""
        print("=" * 60)
        print("Toyota HSR Pickup Initialization")
        print("=" * 60)
        print(f"✓ Robot Name: {self.hsrb.name}")
        print(f"✓ Arm: {self.get_arm().name if self.get_arm() else 'None'}")
        print(f"✓ Gripper: {self.get_gripper().name if self.get_gripper() else 'None'}")
        print(f"✓ Neck: {self.get_neck().name if self.get_neck() else 'None'}")
        print(f"✓ Torso: {self.get_torso().name if self.get_torso() else 'None'}")
        print(f"✓ Cameras: {len(self.get_cameras())}")
        print(f"✓ World Entities: {len(list(self.world.kinematic_structure_entities))}")
        print(f"✓ Visualization: {'Enabled' if self.viz else 'Disabled'}")
        print("=" * 60)
        print("Ready for pickup actions!")
        print("=" * 60)


def initialize_hsr_for_pickup(with_viz: bool = True) -> HSRPickupInitializer:
    """
    Convenience function to initialize the HSR robot for pickup actions.

    Args:
        with_viz: Whether to enable visualization (default: True)

    Returns:
        HSRPickupInitializer: Initialized robot setup
    """
    return HSRPickupInitializer(with_viz=with_viz)


if __name__ == "__main__":
    # Initialize the robot and environment
    hsr_setup = initialize_hsr_for_pickup(with_viz=True)

    # Print status
    hsr_setup.print_status()

    # Access components for pickup actions
    world = hsr_setup.world
    context = hsr_setup.context
    hsrb = hsr_setup.hsrb
    arm = hsr_setup.get_arm()
    gripper = hsr_setup.get_gripper()
    arm_enum = hsr_setup.get_arm_enum()

    # Example: Use simulated_robot context for executing actions
    # with simulated_robot:
    #     from pycram.robot_plans import ParkArmsActionDescription
    #     plan = ParkArmsActionDescription(Arms.BOTH).resolve(context)
    #     plan.perform()