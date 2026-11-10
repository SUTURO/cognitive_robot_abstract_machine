#!/usr/bin/env python
import traceback


def main() -> None:
    """
    Run the door to sofa demo with a person on the right seat place, in kinematic
    simulation and in MuJoCo without a viewer, and exit non-zero with a traceback on
    failure, if any other number than one free seat place is reported or if the floors
    collide in MuJoCo, where the wheels would catch on the edge between two floors.
    """
    try:
        import demo
        from coraplex.plans.executors import (
            KinematicallySimulatedPlanExecutor,
            PhysicallySimulatedPlanExecutor,
        )
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
