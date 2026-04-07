from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Type

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import (
    Body,
    SemanticAnnotation,
)

logger = logging.getLogger(__name__)

"""use example:

# python
mapper = PerceptionSemanticAnnotationMapper.from_json_file(
    "pycram/resources/lists/mapping_perception_semantic-annotations.json"
)

# after you have created/inserted `body` into `world`
mapper.annotate_on_creation(world=world, body=body, perceived_name=perception_label)
"""


@dataclass(frozen=True)
class PerceptionSemanticAnnotationMapper:
    """
    Maps a perception label (string) to a Semantic Digital Twin semantic annotation class name
    and attaches the resulting semantic annotation to a freshly created Body.

    Mapping format (JSON):
        {
          "apple": "Apple",
          "milk_jumbo_pack_voll": "Milk"
        }
    """

    mapping: dict[str, str]

    @classmethod
    def from_json_file(
        cls, mapping_path: str | Path
    ) -> "PerceptionSemanticAnnotationMapper":
        mapping_path = Path(mapping_path)
        with mapping_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError(
                f"Expected JSON object at top-level in {mapping_path}, got {type(data).__name__}"
            )
        # Ensure keys/values are strings
        mapping: dict[str, str] = {}
        for k, v in data.items():
            if not isinstance(k, str) or not isinstance(v, str):
                raise ValueError(
                    f"Invalid mapping entry {k!r}: {v!r}. Expected str->str mapping."
                )
            mapping[k] = v
        return cls(mapping=mapping)

    def mapped_class_name(self, perceived_name: str) -> Optional[str]:
        """
        Returns the semantic annotation class name for a perceived label, or None if unmapped.
        """
        return self.mapping.get(perceived_name)

    def resolve_annotation_class(self, class_name: str) -> Type[SemanticAnnotation]:
        """
        Resolves a semantic annotation class name like 'Apple' to the actual class object.

        By default this looks in semantic_digital_twin.semantic_annotations.semantic_annotations.
        Extend this if you keep custom annotation classes elsewhere.
        """
        from semantic_digital_twin.semantic_annotations import (
            semantic_annotations as sem_ann_module,
        )

        cls_obj = getattr(sem_ann_module, class_name, None)
        if cls_obj is None:
            raise LookupError(
                f"Semantic annotation class '{class_name}' not found in "
                f"semantic_digital_twin.semantic_annotations.semantic_annotations"
            )
        if not isinstance(cls_obj, type) or not issubclass(cls_obj, SemanticAnnotation):
            raise TypeError(
                f"Resolved '{class_name}' but it is not a SemanticAnnotation subclass: {cls_obj!r}"
            )
        return cls_obj

    def annotate_on_creation(
        self,
        *,
        world: World,
        body: Body,
        perceived_name: str,
        annotation_name: Optional[str] = None,
        skip_duplicates: bool = True,
    ) -> Optional[SemanticAnnotation]:
        """
        Attach the mapped semantic annotation to 'body' in 'world'.

        Returns the created annotation, or None if perceived_name is not mapped.
        """
        class_name = self.mapped_class_name(perceived_name)
        if not class_name:
            logger.debug(
                "No semantic-annotation mapping for perceived object '%s'",
                perceived_name,
            )
            return None

        ann_cls = self.resolve_annotation_class(class_name)

        # Choose a stable, readable name. PrefixedName uniqueness is handled via its internal id.
        ann_name = PrefixedName(annotation_name or f"{perceived_name}_annotation")

        annotation = ann_cls(root=body, name=ann_name)

        with world.modify_world():
            world.add_semantic_annotation(annotation, skip_duplicates=skip_duplicates)

        logger.info(
            "Annotated perceived '%s' (body=%s) as %s",
            perceived_name,
            getattr(body, "name", body),
            ann_cls.__name__,
        )
        return annotation
