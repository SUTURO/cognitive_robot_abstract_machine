#!/usr/bin/env python
"""
Runs the HSR kitchen navigation demo without a viewer, failing with a traceback.
"""

from __future__ import annotations

import traceback


def main() -> None:
    """
    Run the HSR MuJoCo kitchen navigation demo without a viewer and exit non-zero with a
    traceback on failure.
    """
    try:
        import demo_in_kitchen

        demo_in_kitchen.main(headless=True)
    except Exception:
        traceback.print_exc()
        exit(1)


if __name__ == "__main__":
    main()
