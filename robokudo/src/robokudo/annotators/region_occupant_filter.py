"""
Restriction of the detected humans to the occupants of the region a query asks about.
"""

from __future__ import annotations

import numpy as np
import open3d
from py_trees.common import Status
from semantic_digital_twin.world_description.world_entity import Region

import robokudo.world as rk_world
from robokudo.annotators.core import BaseAnnotator
from robokudo.annotators.exceptions import LocationNotInWorld
from robokudo.annotators.seat_detection import SeatOccupantRule
from robokudo.cas import CASViews
from robokudo.types.scene import HumanHypothesis
from robokudo.utils.annotator_helper import get_world_to_camera_transform_matrix
from robokudo.utils.region import region_obb_in_camera_coordinates

# %% occupants of the queried region


class RegionOccupantFilter(BaseAnnotator):
    """
    Removes every human hypothesis from the CAS that does not occupy the region of the
    runtime world the query's location names: whose cloud is not centered within it, or
    who is no occupant by :attr:`Descriptor.Parameters.occupant_rule`.

    A query that names no location leaves every human hypothesis in the CAS.
    """

    class Descriptor(BaseAnnotator.Descriptor):
        """
        Descriptor for the RegionOccupantFilter.
        """

        class Parameters:
            """
            Parameters for the RegionOccupantFilter.
            """

            def __init__(self) -> None:
                """
                Set every parameter to its default.
                """
                self.occupant_rule: SeatOccupantRule = SeatOccupantRule()
                """
                The rule that decides which humans in the region occupy it.
                """

    def __init__(
        self,
        name: str = "RegionOccupantFilter",
        descriptor: RegionOccupantFilter.Descriptor | None = None,
    ) -> None:
        """
        :param name: Name of the annotator in the pipeline.
        :param descriptor: Configuration of the annotator, the default one if None.
        """
        super().__init__(name=name, descriptor=descriptor)

    def update(self) -> Status:
        """
        :return: :attr:`Status.SUCCESS` once the humans not occupying the region are
            removed.
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
        occupant_rule = self.descriptor.parameters.occupant_rule
        for hypothesis in cas.filter_annotations_by_type(HumanHypothesis):
            centroid = open3d.utility.Vector3dVector(
                np.asarray(hypothesis.points.get_center()).reshape(1, 3)
            )
            if region_box.get_point_indices_within_bounding_box(
                centroid
            ) and occupant_rule.is_occupant(hypothesis):
                continue
            cas.annotations.remove(hypothesis)
        return Status.SUCCESS
