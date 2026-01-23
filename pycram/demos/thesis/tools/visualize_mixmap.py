from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Protocol, Tuple, Optional, Iterable

import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.geometry import Cylinder, Box, Scale


def clearance_grid(volume: ContainerVolumeModel, z_mix: float, step: float):
    hx, hy = volume.inner_bounds_xy()
    xs = np.arange(-hx, hx + 1e-9, step, dtype=float)
    ys = np.arange(-hy, hy + 1e-9, step, dtype=float)
    grid = np.empty((len(ys), len(xs)), dtype=float)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            grid[j, i] = float(
                volume.distance_inside(
                    np.array([float(x), float(y), float(z_mix)], dtype=float)
                )
            )
    return xs, ys, grid
