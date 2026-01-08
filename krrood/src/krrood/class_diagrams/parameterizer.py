from dataclasses import fields, is_dataclass
from datetime import datetime
from enum import Enum
from typing import Type, List, Union, Sequence, get_origin, get_args, get_type_hints

from random_events.set import Set
from random_events.variable import Continuous, Integer, Symbolic, Variable
from probabilistic_model.probabilistic_circuit.rx.helper import fully_factorized
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit,
)


class Parameterizer:
    """
    Parameterizer for extracting probabilistic variables from wrapped dataclass schemas.

    The Parameterizer operates on *dataclass types*, not instances.
    This ensures plan-agnostic, declarative variable extraction suitable for
    probabilistic reasoning and learning.
    """

    def __call__(self, wrapped_class: Type) -> List[Variable]:
        """
        Extract variables from a wrapped dataclass schema.

        :param wrapped_class: Dataclass type defining parameter schema
        :return: List of random_events Variables
        """
        if not is_dataclass(wrapped_class):
            raise TypeError(
                f"Parameterizer expects a dataclass type, got {wrapped_class}"
            )

        return self._parameterize_class(
            wrapped_class,
            wrapped_class.__name__,
        )

    def _parameterize_class(self, cls: Type, prefix: str) -> List[Variable]:
        """
        Recursively extract variables from a dataclass.
        """
        variables: List[Variable] = []
        type_hints = get_type_hints(cls)

        for field in fields(cls):
            field_type = type_hints[field.name]
            qualified_name = f"{prefix}.{field.name}"
            variables.extend(self._parameterize_type(field_type, qualified_name))

        return variables

    def _parameterize_type(self, typ: Type, prefix: str) -> List[Variable]:
        """
        Convert a type annotation into random event variables.
        """
        variables: List[Variable] = []
        unsupported_types = (datetime,)
        if typ in unsupported_types:
            return []

        origin = get_origin(typ)
        args = get_args(typ)

        if origin is Union:
            typ = next(a for a in args if a is not type(None))
        origin = get_origin(typ)
        args = get_args(typ)
        if origin in (list, List, Sequence):
            typ = args[0]

        if is_dataclass(typ):
            for f in fields(typ):
                variables.extend(
                    self._parameterize_type(
                        get_type_hints(typ)[f.name],
                        f"{prefix}.{f.name}",
                    )
                )

        elif issubclass(typ, bool):
            variables.append(Symbolic(prefix, Set.from_iterable([True, False])))

        elif issubclass(typ, Enum):
            variables.append(Symbolic(prefix, Set.from_iterable(list(typ))))

        elif issubclass(typ, int):
            variables.append(Integer(prefix))

        elif issubclass(typ, float):
            variables.append(Continuous(prefix))

        else:
            raise NotImplementedError(f"No variable mapping for type {typ}")

        return variables

    def create_fully_factorized_distribution(
        self,
        variables: List[Variable],
    ) -> ProbabilisticCircuit:
        """
        Create a fully factorized probabilistic circuit over the given variables.
        """
        return fully_factorized(
            variables,
            means={v: 0.0 for v in variables if isinstance(v, Continuous)},
            variances={v: 1.0 for v in variables if isinstance(v, Continuous)},
        )
