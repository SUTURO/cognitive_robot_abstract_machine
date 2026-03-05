from copy import deepcopy
from dataclasses import dataclass, field

from typing_extensions import Callable, Dict, Any, Generic, TypeVar, Self, Union

from krrood.adapters.json_serializer import list_like_classes

T = TypeVar("T")


@dataclass
class HasFactoryAndKwargs(Generic[T]):
    """
    Mixing containing a hierarchy of factories and their keyword arguments.
    """

    factory: Callable[..., T]
    """
    The factory function to construct `T` with the keyword arguments.
    """

    kwargs: Dict[str, Any] = field(default_factory=dict, kw_only=True)
    """
    The keyword arguments to pass to the factory.
    """

    def construct_instance(self):
        """
        Construct a python object from the CallableAndKwargs instance.

        ..note:: This method may work with ellipsis, but it's not guaranteed to work with all types.

        :return: The constructed object.
        """
        constructed_kwargs = {}
        for key, value in self.kwargs.items():
            if isinstance(value, list_like_classes):
                constructed_kwargs[key] = type(value)(
                    (
                        element.construct_instance()
                        if isinstance(element, HasFactoryAndKwargs)
                        else element
                    )
                    for element in value
                )

            elif isinstance(value, HasFactoryAndKwargs):
                constructed_kwargs[key] = value.construct_instance()
            else:
                constructed_kwargs[key] = value
        return self.factory(**constructed_kwargs)

    def __deepcopy__(self, memo):
        return self.__class__(
            self.factory,
            kwargs={name: deepcopy(value) for name, value in self.kwargs.items()},
        )
