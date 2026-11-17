"""
Reports what perception found out about a cloth to the T-shirt in the semantic digital
twin.
"""

from __future__ import annotations

from py_trees.common import Status

from robokudo import world
from robokudo.annotators.core import BaseAnnotator
from robokudo.annotators.exceptions import TShirtNotUnique
from robokudo.cas import CAS
from robokudo.types.cloth import (
    ClothFoldAnnotation,
    ClothGraspPointsAnnotation,
    ClothHypothesis,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import TShirt

# %% annotator


class ClothStateConnector(BaseAnnotator):
    """
    Records the grasp points and the fold of the cloth hypothesis on the one T-shirt in
    RoboKudo's world, so that the T-shirt mirrors the latest perception: what the
    hypothesis is not annotated with is cleared.
    """

    def __init__(
        self,
        name: str = "ClothStateConnector",
        descriptor: BaseAnnotator.Descriptor | None = None,
    ) -> None:
        super().__init__(name, descriptor)

    def update(self) -> Status:
        """
        Record the grasp points and the fold of each cloth hypothesis on the T-shirt.

        :raises TShirtNotUnique: If the world does not hold exactly one T-shirt.
        """
        hypotheses = self.get_cas().filter_annotations_by_type(ClothHypothesis)
        if len(hypotheses) == 0:
            return Status.SUCCESS

        perception_world = world.world_instance()
        t_shirts = perception_world.get_semantic_annotations_by_type(TShirt)
        if len(t_shirts) != 1:
            raise TShirtNotUnique(t_shirt_count=len(t_shirts))
        t_shirt = t_shirts[0]

        with perception_world.modify_world():
            for hypothesis in hypotheses:
                grasp_points = CAS.filter_by_type(
                    ClothGraspPointsAnnotation, hypothesis.annotations
                )
                folds = CAS.filter_by_type(ClothFoldAnnotation, hypothesis.annotations)
                t_shirt.update_grasp_points(
                    grasp_points[-1].grasp_points if len(grasp_points) > 0 else None
                )
                t_shirt.update_fold(folds[-1].fold if len(folds) > 0 else None)
        return Status.SUCCESS
