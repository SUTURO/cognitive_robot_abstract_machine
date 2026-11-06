#!/usr/bin/env python
import traceback


def main() -> None:
    """
    Run the door to sofa demo with a person on the right seat place, in kinematic
    simulation and in MuJoCo without a viewer, and exit non-zero with a traceback on
    failure or if any other number than one free seat place is reported.
    """
    try:
        import demo
        from coraplex.plans.executors import (
            KinematicallySimulatedPlanExecutor,
            PhysicallySimulatedPlanExecutor,
        )
        from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat

        for executor_type in (
            KinematicallySimulatedPlanExecutor,
            PhysicallySimulatedPlanExecutor,
        ):
            number_of_free_seats = demo.main(
                executor_type=executor_type,
                occupied_seats=[SofaSeat.RIGHT],
                headless=True,
            )
            if number_of_free_seats != len(SofaSeat) - 1:
                raise AssertionError(
                    f"Expected one free seat, got {number_of_free_seats}"
                )
    except Exception:
        traceback.print_exc()
        exit(1)


if __name__ == "__main__":
    main()
