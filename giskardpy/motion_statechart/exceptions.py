from dataclasses import dataclass

from typing_extensions import Any


class MotionStatechartError(Exception):
    pass


class EmptyMotionStatechartError(MotionStatechartError):
    def __init__(self):
        super().__init__("MotionStatechart is empty.")


@dataclass
class NodeNotFoundError(MotionStatechartError):
    name: str

    def __post_init__(self):
        super().__init__(f"Node '{self.name}' not found in MotionStatechart.")


@dataclass
class NotInMotionStatechartError(MotionStatechartError):
    name: str

    def __post_init__(self):
        super().__init__(
            f"Operation can't be performed because node '{self.name}' does not belong to a MotionStatechart."
        )


@dataclass
class InvalidConditionError(MotionStatechartError):
    expression: Any

    def __post_init__(self):
        super().__init__(
            f"Invalid condition: {self.expression}. Did you forget '.observation_variable'?"
        )
