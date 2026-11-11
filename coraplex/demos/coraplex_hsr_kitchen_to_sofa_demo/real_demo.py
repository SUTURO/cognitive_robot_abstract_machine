"""
The kitchen to sofa demo carried out by the real HSR, which Pumas drives to the sofa and
which says the free seats aloud.

Start AMCL and Pumas on the map ``apartment_master1``, Giskard and RoboKudo, put the
robot in the kitchen, then run it from this folder::

    source ~/ros_clean_env.sh
    python real_demo.py
"""

from __future__ import annotations

from demo import FreeSeatAnnouncement, main
from pumas_navigation import PumasKitchenToSofaDemonstration

# %% running it

if __name__ == "__main__":
    main(PumasKitchenToSofaDemonstration(), FreeSeatAnnouncement(speak=True))
