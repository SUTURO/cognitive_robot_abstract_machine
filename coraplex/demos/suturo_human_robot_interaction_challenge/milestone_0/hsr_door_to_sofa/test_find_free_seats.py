#!/usr/bin/env python
"""
The check CI runs on the door to sofa demo.
"""

import traceback


def main() -> None:
    """
    Run the door to sofa demo with a person on the right seat place, in kinematic
    simulation and in MuJoCo without a viewer, and exit non-zero with a traceback on
    failure, if any other seat place than the left one is reported free or if one look
    does not report the empty sofa free on both seats.
    """
    try:
        import free_seats_demonstration
        from coraplex.plans.executors import (
            KinematicallySimulatedPlanExecutor,
            ForwardDrivingPlanExecutor,
        )
        from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat

        for executor_type in (
            KinematicallySimulatedPlanExecutor,
            ForwardDrivingPlanExecutor,
        ):
            free_seats = SofaSeat.free_in(
                free_seats_demonstration.main(
                    free_seats_demonstration.DoorToSofaDemonstration(
                        executor_type=executor_type,
                        occupied_seats=[SofaSeat.RIGHT],
                        headless=True,
                    )
                )
            )
            if free_seats != [SofaSeat.LEFT]:
                raise AssertionError(f"Expected only LEFT to be free, got {free_seats}")

        free_seats = SofaSeat.free_in(
            free_seats_demonstration.main(
                free_seats_demonstration.OneLookDoorToSofaDemonstration()
            )
        )
        if free_seats != list(SofaSeat):
            raise AssertionError(f"Expected every seat to be free, got {free_seats}")
        if (
            free_seats_demonstration.FreeSeatSentence.describe([])
            != free_seats_demonstration.FreeSeatSentence.NONE
        ):
            raise AssertionError("A full sofa is not described as full")
    except Exception:
        traceback.print_exc()
        exit(1)


if __name__ == "__main__":
    main()
