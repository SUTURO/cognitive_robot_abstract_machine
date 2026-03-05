from __future__ import annotations

import enum
import typing
from dataclasses import dataclass, field
from functools import cached_property
from typing import Dict, Type, Optional

import numpy as np
from typing_extensions import Any, List, get_args

import random_events.variable
from krrood.adapters.json_serializer import list_like_classes, leaf_types
from krrood.patterns.callable_with_kwargs import HasFactoryAndKwargs
from krrood.class_diagrams.utils import get_type_hint_of_keyword_argument
from krrood.entity_query_language.core.base_expressions import SymbolicExpression
from krrood.entity_query_language.core.mapped_variable import (
    MappedVariable,
    CanBehaveLikeAVariable,
)
from krrood.entity_query_language.factories import variable, and_
from krrood.entity_query_language.query.match import UnderspecifiedVariable
from krrood.parametrization.random_events_translator import (
    WhereExpressionToRandomEventTranslator,
)
from random_events.product_algebra import Event
from random_events.set import Set
from random_events.variable import variable_from_name_and_type


@dataclass
class UnderspecifiedFactory:
    """
    This behaves like a factory for underspecified method calls where Ellipsis indicates the missing parameters.
    """

    statement: UnderspecifiedVariable
    """
    The UnderspecifiedVariable that contains the ellipsis statements.
    """

    @cached_property
    def flat_variables(
        self,
    ) -> List[ParametrizationVariable]:
        """
        :return: A dictionary mapping all variables mentioned in the CallableAndKwargs to their values.
        """
        result = []

        symbolic_access_for_kwargs = variable(self.statement.factory, [])
        symbolic_access_for_where = variable(self.statement.factory, [])
        symbolic_access = ParametrizationVariable(
            variable_in_kwargs=symbolic_access_for_kwargs,
            variable_in_where_conditions=symbolic_access_for_where,
            type_hint=None,
            value=None,
        )

        self._flat_variables(self.statement, symbolic_access, result)
        return result

    @classmethod
    def _flat_variables(
        cls,
        current_statement: UnderspecifiedVariable,
        symbolic_access: ParametrizationVariable,
        result: List,
    ):
        """
        Recursively extract all variables and their values from the CallableAndKwargs.

        :param symbolic_access: The current symbolic access to the variables.
        :param result: The dictionary to store the extracted variables and their values.
        """
        symbolic_access.variable_in_kwargs = symbolic_access.variable_in_kwargs.kwargs
        for key, value in current_statement.kwargs.items():

            # update access patterns
            current_symbolic_access = symbolic_access.apply_attribute_access(key)
            current_symbolic_access._update_type_hint(
                get_type_hint_of_keyword_argument(current_statement.factory, key)
            )

            if isinstance(value, list_like_classes):
                # handle list like classes by wrapping the index access
                for index, element in enumerate(value):

                    # update access pattern for the current index
                    current_symbolic_access_index = (
                        current_symbolic_access.apply_index_access(index)
                    )
                    current_symbolic_access._update_type_hint(
                        (
                            get_type_hint_of_keyword_argument(
                                current_statement.factory, key
                            )
                        )
                    )  # get the type hint from the signature of the function
                    if isinstance(element, UnderspecifiedVariable):
                        cls._flat_variables(
                            element,
                            current_symbolic_access_index,
                            result,
                        )
                    else:
                        current_symbolic_access.is_leaf = True
                        current_symbolic_access.value = element
                        result.append(current_symbolic_access)

            elif isinstance(value, UnderspecifiedVariable):
                cls._flat_variables(
                    value,
                    current_symbolic_access,
                    result,
                )
            else:
                current_symbolic_access.is_leaf = True
                current_symbolic_access.value = value
                result.append(current_symbolic_access)

    def apply_assignments(self, bindings: Dict[ParametrizationVariable, Any]):
        """
        Apply the given bindings to the CallableAndKwargs instance.
        This updates the `kwarg` attribute in-place.

        :param bindings: A dictionary mapping symbolic accesses from this instance to their values.
        """
        for variable, value in bindings.items():
            variable.variable_in_kwargs._set_external_instance_value_(
                self.statement, value
            )


@dataclass
class ParametrizationVariable:
    """
    Grouping of variables that appear in different expressions but mean the same thing.
    """

    variable_in_kwargs: CanBehaveLikeAVariable
    """
    The variable in the kwargs of the factory used for constructing the final object.
    This one is guaranteed to appear.
    """

    variable_in_where_conditions: CanBehaveLikeAVariable
    """
    The variable in the where conditions. This one is very likely to exist. 
    """

    value: Any = None
    """
    The value this variable has in the Kwargs"""

    type_hint: Optional[Type] = None
    """
    The type hint of the variable extracted from the signature of the factory.
    """

    is_leaf: bool = False
    """
    Whether this variable is a leaf variable.
    A leaf variable is a variable where its value was not an `underspecified` statement.
    """

    def __hash__(self):
        return hash(self.variable_in_kwargs)

    @property
    def name(self) -> str:
        return self.variable_in_where_conditions._name_

    def _update_type_hint(self, type_hint: Type):
        if args := get_args(type_hint):
            self.type_hint = args[0]
        else:
            self.type_hint = type_hint

    def apply_attribute_access(self, attribute: str):
        """
        Apply attribute access to the EQL variables.
        :param attribute: The namee of the attribute
        :return: A new ParametrizationVariable with the attribute access applied.
        """
        if self.is_leaf:
            raise ValueError("Cannot apply attribute access to leaf variable")
        return self.__class__(
            variable_in_kwargs=self.variable_in_kwargs[attribute],
            variable_in_where_conditions=getattr(
                self.variable_in_where_conditions, attribute
            ),
            value=None,
            type_hint=None,
            is_leaf=False,
        )

    def apply_index_access(self, index: Any):
        """
        Apply index access to the EQL variables.
        :param index: The index to access
        :return: A new ParametrizationVariable with the index access applied.
        """
        if self.is_leaf:
            raise ValueError("Cannot apply index access to leaf variable")
        return self.__class__(
            variable_in_kwargs=self.variable_in_kwargs[index],
            variable_in_where_conditions=self.variable_in_where_conditions[index],
            value=None,
            type_hint=None,
            is_leaf=False,
        )

    @cached_property
    def random_events_variable(self) -> Optional[random_events.variable.Variable]:
        if not self.is_leaf:
            return None

        if isinstance(self.value, SymbolicExpression):
            return random_events.variable.Symbolic(
                self.name,
                Set.from_iterable(self.value.tolist()),
            )
        if not issubclass(self.type_hint, leaf_types + (enum.Enum,)):
            return None
        return variable_from_name_and_type(self.name, self.type_hint)

    def __repr__(self):
        return f"{self.name}: {self.type_hint} = {self.value}"


@dataclass
class UnderspecifiedParameters:
    """
    A class that extracts all necessary information from an UnderspecifiedVariable and binds it together.
    Instances of this can be used to parameterize objects with underspecified variables using generative models.
    """

    statement: UnderspecifiedVariable
    """
    The UnderspecifiedVariable to extract information from.
    """

    _random_event_compiler: Optional[WhereExpressionToRandomEventTranslator] = field(
        init=False
    )
    """
    The translator that extracts a random event from the where conditions.
    Only exists if the statement has a where condition.
    """

    factory: UnderspecifiedFactory = field(init=False)
    """
    The factory for constructing new objects from samples.
    """

    truncation_event: Optional[Event] = field(init=False, default=None)
    """
    The where condition as random event.
    Only exists if the statement has a where condition.
    """

    def __post_init__(self):
        self.factory = UnderspecifiedFactory(self.statement)

        self._random_event_compiler = WhereExpressionToRandomEventTranslator(
            and_(*self.statement._where_expressions), self.factory.flat_variables
        )
        if self.statement._where_expressions:
            self.truncation_event = self._random_event_compiler.translate()

    @property
    def assignments_for_conditioning(
        self,
    ) -> Dict[random_events.variable.Variable, Any]:
        """
        :return: A dictionary that contains all facts from the statement and that can be directly used for
        conditioning a probabilistic model.
        """
        return {
            v.random_events_variable: v.value
            for v in self._random_event_compiler.leaf_variables_with_topology
            if v.value is not None and isinstance(v.value, leaf_types)
        }

    @property
    def random_event_variables(self) -> List[random_events.variable.Variable]:
        """
        :return: A list of all random event variables that are used in the statement.
        """
        return [
            v.random_events_variable
            for v in self.factory.flat_variables
            if v.random_events_variable is not None
        ]

    def get_variable_by_name(self, name: str) -> ParametrizationVariable:
        [result] = [v for v in self.factory.flat_variables if v.name == name]
        return result

    def create_assignment_from_variables_and_sample(
        self,
        variables: typing.Iterable[random_events.variable.Variable],
        sample: np.ndarray,
    ) -> Dict[ParametrizationVariable, Any]:
        """
        Create an assignment dictionary that can be used to construct a new object from a sample.
        :param variables: The variables from a probabilistic model.
        :param sample: A sample from the same model-
        :return: A dictionary that can be used to construct a new object from a sample.
        """

        result = {}
        for variable_, value in zip(variables, sample):
            parametrization_variable = self.get_variable_by_name(variable_.name)

            if not variable_.is_numeric:
                [value] = [
                    domain_value.element
                    for domain_value in variable_.domain
                    if hash(domain_value) == value
                ]
            else:
                value = value.item()
            result[parametrization_variable] = value

        return result
