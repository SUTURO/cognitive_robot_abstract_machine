#!/usr/bin/env python
"""
The check CI runs on the kitchen to sofa demo.
"""

import traceback


def main() -> None:
    """
    Run the kitchen to sofa demo in kinematic simulation, its perception answered by the
    world model since CI has neither RoboKudo nor the recordings, and exit non-zero with
    a traceback on failure or if the empty sofa is not reported free on both seats.
    """
    try:
        import demo
        from semantic_digital_twin.predefined_maps.sofa_seats import SofaSeat

        free_seats = SofaSeat.free_in(demo.main())
        if free_seats != list(SofaSeat):
            raise AssertionError(f"Expected every seat to be free, got {free_seats}")
        if demo.FreeSeatSentence.describe([]) != demo.FreeSeatSentence.NONE:
            raise AssertionError("A full sofa is not described as full")
    except Exception:
        traceback.print_exc()
        exit(1)


if __name__ == "__main__":
    main()
