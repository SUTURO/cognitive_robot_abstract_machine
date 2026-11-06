#!/usr/bin/env python
import traceback


def main() -> None:
    """
    Run the free seats demo with a person on the right seat place, in kinematic
    simulation and in MuJoCo without a viewer, and exit non-zero with a traceback on
    failure or if any other seat place than the left one is reported free.
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
            free_seats = demo.main(
                executor_type=executor_type,
                occupied_seats=[SofaSeat.RIGHT],
                headless=True,
            )
            if free_seats != [SofaSeat.LEFT]:
                raise AssertionError(f"Expected only LEFT to be free, got {free_seats}")
    except Exception:
        traceback.print_exc()
        exit(1)


if __name__ == "__main__":
    main()
