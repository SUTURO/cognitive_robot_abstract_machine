"""
Restriction of the detected humans to the region a query asks about.
"""

from __future__ import annotations

import numpy as np
import open3d
from py_trees.common import Status
from semantic_digital_twin.world_description.world_entity import Region

import robokudo.world as rk_world
from robokudo.annotators.core import BaseAnnotator
from robokudo.annotators.exceptions import LocationNotInWorld
from robokudo.cas import CASViews
from robokudo.types.scene import HumanHypothesis
from robokudo.utils.annotator_helper import get_world_to_camera_transform_matrix
from robokudo.utils.region import region_obb_in_camera_coordinates

# %% occupants of the queried region


class RegionOccupantFilter(BaseAnnotator):
    """
    Removes every human hypothesis from the CAS whose cloud is not centered within the
    region of the runtime world the query's location names.

    A query that names no location leaves every human hypothesis in the CAS.
    """

    def __init__(self, name: str = "RegionOccupantFilter") -> None:
        super().__init__(name=name)

    def update(self) -> Status:
        """
        :return: :attr:`Status.SUCCESS` once the humans outside the region are removed.
        :raises LocationNotInWorld: If the runtime world holds no region of the queried
            location.
        """
        cas = self.get_cas()
        location = cas.get(CASViews.QUERY).obj.location
        if location == "":
            return Status.SUCCESS

        world = rk_world.world_instance()
        regions = {
            str(region.name): region
            for region in world.get_kinematic_structure_entity_by_type(Region)
        }
        if location not in regions:
            raise LocationNotInWorld(location)

        region_box = region_obb_in_camera_coordinates(
            world, regions[location], get_world_to_camera_transform_matrix(cas)
        )
        for hypothesis in cas.filter_annotations_by_type(HumanHypothesis):
            centroid = open3d.utility.Vector3dVector(
                np.asarray(hypothesis.points.get_center()).reshape(1, 3)
            )
            if region_box.get_point_indices_within_bounding_box(centroid):
                continue
            cas.annotations.remove(hypothesis)
        return Status.SUCCESS
