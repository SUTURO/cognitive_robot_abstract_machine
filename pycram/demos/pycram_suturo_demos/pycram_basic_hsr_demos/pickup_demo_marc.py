# import logging
# from typing import Any
#
# import rclpy
# from rclpy.logging import get_logger
#
# import semantic_digital_twin.exceptions
#
# from pycram.datastructures.enums import Arms, VerticalAlignment, ApproachDirection
# from pycram.datastructures.grasp import GraspDescription
# from pycram.language import SequentialPlan
# from pycram.motion_executor import real_robot, simulated_robot, ExecutionEnvironment
# from pycram.robot_plans import (
#     ParkArmsActionDescription,
#     PickUpActionDescription,
# )
# from pycram_suturo_demos.helper_methods_and_useful_classes.robot_setup import (
#     robot_setup,
# )
# from pycram_suturo_demos.pycram_basic_hsr_demos.start_up import setup_hsrb_context
# from semantic_digital_twin.robots.abstract_robot import Manipulator, ParallelGripper
# from semantic_digital_twin.world_description.world_entity import Body
#
# logger = logging.getLogger(__name__)
# logging.getLogger(semantic_digital_twin.world.__name__).setLevel(logging.WARN)
#
#
# rclpy_node, world, robot_view, context = setup_hsrb_context()
#
#
# #
# # # ------------------------ BASE-DEFINITIONS
# # logger = get_logger(__name__)
# #
# # SIMULATED: bool = False
# # with_perception: bool = False
# # object_name: str = ""
# #
# # robot_type: ExecutionEnvironment = simulated_robot if SIMULATED else real_robot
# #
# result = robot_setup(SIMULATED)
#
# hsrb_world, robot_view, context, manipulator = (
#     result.world,
#     result.robot_view,
#     result.context,
#     result.manipulator,
# )
# #
# #
# manipulator: ParallelGripper
#
# grasp: GraspDescription = GraspDescription(
#     approach_direction=ApproachDirection.FRONT,
#     vertical_alignment=VerticalAlignment.NoAlignment,
#     manipulator=manipulator,
#     rotate_gripper=False,
# )
#
#
# #
# #
# def try_get_object_to_pickup(world, object_name_method) -> Body | None:
#     try:
#         object_to_pickup_method = hsrb_world.get_body_by_name(object_name_method)
#         logger.info(f"picking up object with name '{object_name_method}'")
#     except semantic_digital_twin.exceptions.WorldEntityNotFoundError:
#         object_to_pickup_method = None
#         logger.error(f"No object with name '{object_name_method}' found")
#     return object_to_pickup_method
#
#
# # -------------------------------- DETERMIN OBJECT_TO_PICKUP
# if SIMULATED:
#     object_name: str = "milk.stl"
#     object_to_pickup = try_get_object_to_pickup(hsrb_world, object_name)
# else:
#     if with_perception:
#         from demos.pycram_suturo_demos.helper_methods_and_useful_classes.object_creation import (
#             perceive_and_spawn_all_objects,
#         )
#
#         perceived_objects: dict[Any, Any] = perceive_and_spawn_all_objects(hsrb_world)
#         logger.info(f"perceived following objects: '{perceived_objects}'")
#     object_to_pickup = try_get_object_to_pickup(hsrb_world, object_name)
# logger.info(f"object_to_Pickup: '{object_to_pickup}'")
#
#
# # -------------------------------- PLANNING
# if object_to_pickup is not None:
#     plan = SequentialPlan(
#         context,
#         ParkArmsActionDescription(Arms.BOTH),
#         PickUpActionDescription(
#             object_designator=object_to_pickup, arm=Arms.LEFT, grasp_description=grasp
#         ),
#         ParkArmsActionDescription(Arms.BOTH),
#     )
# else:
#     plan = SequentialPlan(
#         context,
#         ParkArmsActionDescription(Arms.BOTH),
#     )
#
# # ------------------------ EXECUTION
# with simulated_robot:
#     plan.perform()
#
# rclpy.shutdown()
