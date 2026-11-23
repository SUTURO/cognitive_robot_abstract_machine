"""
The kitchen to sofa demo in simulation: a kinematically simulated HSR whose view of the
sofa is a recording of the real HSR's head camera, perceived by RoboKudo.

Start RoboKudo first, then run it from this folder with the recording's folder::

    python rosbag_demo.py ~/workspace/src/kinect/07
"""

from __future__ import annotations

import argparse
from pathlib import Path

from camera_recording import RecordedKitchenToSofaDemonstration
from demo import main

# %% running it

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Drive a simulated HSR from the kitchen to the sofa, perceive the sofa in a recording and print which of its seats are free."
    )
    parser.add_argument(
        "recording",
        type=Path,
        help="folder of the ros2 bag recording of the HSR's head camera",
    )
    main(
        RecordedKitchenToSofaDemonstration(
            recording=parser.parse_args().recording.expanduser()
        )
    )
