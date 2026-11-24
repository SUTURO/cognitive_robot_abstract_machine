#!/usr/bin/env python
"""
The check CI runs on the door to sofa demo.
"""

import traceback


def main() -> None:
    """
    Run the door to sofa demo with a person on the right seat place, in kinematic
    simulation and in MuJoCo without a viewer, and exit non-zero with a traceback on
    failure, if any other seat place than the left one is reported free, if the camera
    has lost the sofa after the drive looking at it, or if the floors collide in MuJoCo.
    """
    try:
        import demo
        import look_while_driving_demo
        from coraplex.plans.executors import (
            KinematicallySimulatedPlanExecutor,
            PhysicallySimulatedPlanExecutor,
        )
        from krrood_spatial_math.spatial_types import Vector3
        from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat
        from semantic_digital_twin.semantic_annotations.semantic_annotations import (
            Floor,
        )

        demonstration = demo.DoorToSofaDemonstration(headless=True)
        world = demonstration.build_simulated_world()
        demonstration.build_simulation(world)
        if any(
            floor.root.collision
            for floor in world.get_semantic_annotations_by_type(Floor)
        ):
            raise AssertionError("The floors collide in MuJoCo")

        looking_demonstration = (
            look_while_driving_demo.LookAtSofaWhileDrivingDemonstration(headless=True)
        )
        world = looking_demonstration.run()
        [robot] = world.get_semantic_annotations_by_type(
            looking_demonstration.used_robot
        )
        camera = robot.get_default_camera()
        camera_P_sofa = world.transform(
            looking_demonstration.look_target(world=world), camera.root
        ).position
        camera_V_sofa = Vector3(*camera_P_sofa.to_np()[:3], reference_frame=camera.root)
        if float(camera.viewing_direction.angle_between(camera_V_sofa)) > 0.05:
            raise AssertionError("The camera does not look at the sofa after the drive")

        for executor_type in (
            KinematicallySimulatedPlanExecutor,
            PhysicallySimulatedPlanExecutor,
        ):
            free_seats = SofaSeat.free_in(
                demo.main(
                    executor_type=executor_type,
                    occupied_seats=[SofaSeat.RIGHT],
                    headless=True,
                )
            )
            if free_seats != [SofaSeat.LEFT]:
                raise AssertionError(f"Expected only LEFT to be free, got {free_seats}")
    except Exception:
        traceback.print_exc()
        exit(1)


if __name__ == "__main__":
    main()
