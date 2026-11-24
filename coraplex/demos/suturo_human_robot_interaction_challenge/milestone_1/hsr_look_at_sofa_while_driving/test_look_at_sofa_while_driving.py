#!/usr/bin/env python
"""
The check CI runs on the look at sofa while driving demo.
"""

import traceback


def main() -> None:
    """
    Run the demo in MuJoCo without a viewer and exit non-zero with a traceback on
    failure or if the camera has lost the sofa after the drive.
    """
    try:
        import look_at_sofa_while_driving_in_mujoco
        from krrood_spatial_math.spatial_types import Vector3

        demonstration = (
            look_at_sofa_while_driving_in_mujoco.LookAtSofaWhileDrivingDemonstration(
                headless=True
            )
        )
        world = demonstration.run()
        [robot] = world.get_semantic_annotations_by_type(demonstration.used_robot)
        camera = robot.get_default_camera()
        camera_P_sofa = world.transform(
            demonstration.look_target(world=world), camera.root
        ).position
        camera_V_sofa = Vector3(*camera_P_sofa.to_np()[:3], reference_frame=camera.root)
        if float(camera.viewing_direction.angle_between(camera_V_sofa)) > 0.05:
            raise AssertionError("The camera does not look at the sofa after the drive")
    except Exception:
        traceback.print_exc()
        exit(1)


if __name__ == "__main__":
    main()
