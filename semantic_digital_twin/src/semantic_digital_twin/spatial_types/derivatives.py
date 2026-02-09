from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Self

from typing_extensions import Generic, TypeVar, List, Optional, Dict, Any

from krrood.adapters.json_serializer import (
    SubclassJSONSerializer,
    DataclassJSONSerializer,
)

T = TypeVar("T")


class Derivatives(IntEnum):
    """
    Enumaration of interpretation for the order of derivativeson the spatial positions
    """

    position = 0
    velocity = 1
    acceleration = 2
    jerk = 3
    snap = 4
    crackle = 5
    pop = 6

    @classmethod
    def range(cls, start: Derivatives, stop: Derivatives, step: int = 1):
        """
        Includes stop!
        """
        return [item for item in cls if start <= item <= stop][::step]


@dataclass
class DerivativeMap(Generic[T]):
    """
    A container class that maps derivatives (position, velocity, acceleration, jerk) to values of type T.

    This class provides a structured way to store and access different orders of derivatives.
    Each derivative order can hold a value of type T or None.
    """

    position: Optional[T] = None
    velocity: Optional[T] = None
    acceleration: Optional[T] = None
    jerk: Optional[T] = None
    snap: Optional[T] = None
    crackle: Optional[T] = None
    pop: Optional[T] = None

    @classmethod
    def from_data(cls, data: List[Optional[T]]) -> DerivativeMap[T]:
        return cls(*data)

    @property
    def data(self) -> List[Optional[T]]:
        """
        :return: A list of all derivative values.
        """
        return [
            self.position,
            self.velocity,
            self.acceleration,
            self.jerk,
            self.snap,
            self.crackle,
            self.pop,
        ]

    def __hash__(self):
        return hash(tuple(self.data))

    def _broadcast_callable(self, operand: Callable[[Optional[T]], T]) -> Self:
        """
        Apply a callable to each derivative value and return a new instance with the resulting values.

        :param operand: The callable to apply. Make sure it can deal with None values.
        :return: The new instance with the resulting values.
        """
        return type(self)(
            operand(self.position),
            operand(self.velocity),
            operand(self.acceleration),
            operand(self.jerk),
            operand(self.snap),
            operand(self.crackle),
            operand(self.pop),
        )

    def __mul__(self, other: float) -> DerivativeMap[T]:
        return self._broadcast_callable(lambda v: v * other if v is not None else None)

    def __add__(self, other: float) -> DerivativeMap[T]:
        return self._broadcast_callable(lambda v: v + other if v is not None else None)

    def __setitem__(self, key: Derivatives, value: T):
        """
        Set an attribute using the `Derivatives` Enum.

        :param key: The derivative to set.
        :param value: The value to set.
        """
        assert hasattr(self, key.name)
        self.__setattr__(key.name, value)

    def __getitem__(self, item: Derivatives) -> T:
        """
        Get an attribute using the `Derivatives` Enum.

        :param item: The derivative.
        :return: The value.
        """
        assert hasattr(self, item.name)
        return self.__getattribute__(item.name)
