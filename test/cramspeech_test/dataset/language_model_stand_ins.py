"""
Stand-ins for the language model of the plan model, so tests run without a trained
model.
"""

from __future__ import annotations

# %% Imports
from dataclasses import dataclass

from cramspeech.inference.plan_model import CompletionKey
from typing_extensions import Any


# %% Language model
@dataclass
class LanguageModelWritingText:
    """
    Takes the place of :class:`llama_cpp.Llama`: writes the same text for every prompt.
    """

    text: str
    """
    What the model writes.
    """

    def create_completion(self, prompt: str, **settings: Any) -> dict[str, Any]:
        """
        :return: :attr:`text` as the one choice of a completion, in the shape llama.cpp
            returns it.
        """
        return {CompletionKey.CHOICES: [{CompletionKey.TEXT: self.text}]}
