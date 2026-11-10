#!/usr/bin/env python
"""
The check CI runs on the door to sofa demo.
"""

import traceback


def main() -> None:
    """
    Run the door to sofa demo with a person on the right seat place, in kinematic
    simulation and in MuJoCo without a viewer, and exit non-zero with a traceback on
    failure, if any other seat place than the left one is reported free or if the floors
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
            world = demo.main(
                executor_type=executor_type,
                occupied_seats=[SofaSeat.RIGHT],
                headless=True,
            )
            if SofaSeat.free_in(world) != [SofaSeat.LEFT]:
                raise AssertionError(
                    f"Expected only LEFT to be free, got {SofaSeat.free_in(world)}"
                )
    except Exception:
        traceback.print_exc()
        exit(1)


if __name__ == "__main__":
    main()
