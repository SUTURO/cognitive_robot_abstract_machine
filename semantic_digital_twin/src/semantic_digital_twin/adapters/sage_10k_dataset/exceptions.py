from dataclasses import dataclass
from typing import TYPE_CHECKING

from krrood.utils import DataclassException

if TYPE_CHECKING:
    from semantic_digital_twin.adapters.sage_10k_dataset.schema import Sage10kScene
