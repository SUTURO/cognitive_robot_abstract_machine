from __future__ import annotations

from dataclasses import dataclass
from typing import Type, Iterable, Tuple, Union

"""
Utilities for hashing, rendering, and general helpers used by the
symbolic query engine.
"""
import codecs
import itertools
import os
import re
from subprocess import check_call
from tempfile import NamedTemporaryFile

try:
    import six
except ImportError:
    six = None

try:
    from graphviz import Source
except ImportError:
    Source = None

from typing_extensions import Callable, Set, Any, Optional, List


class IDGenerator:
    """
    A class that generates incrementing, unique IDs and caches them for every object this is called on.
    """

    _counter = 0
    """
    The counter of the unique IDs.
    """

    # @lru_cache(maxsize=None)
    def __call__(self, obj: Any) -> int:
        """
        Creates a unique ID and caches it for every object this is called on.

        :param obj: The object to generate a unique ID for, must be hashable.
        :return: The unique ID.
        """
        self._counter += 1
        return self._counter


def lazy_iterate_dicts(dict_of_iterables):
    """Generator that yields dicts with one value from each iterable"""
    for values in zip(*dict_of_iterables.values()):
        yield dict(zip(dict_of_iterables.keys(), values))


def generate_combinations(generators_dict):
    """Yield all combinations of generator values as keyword arguments"""
    for combination in itertools.product(*generators_dict.values()):
        yield dict(zip(generators_dict.keys(), combination))


def filter_data(data, selected_indices):
    data = iter(data)
    prev = -1
    encountered_indices = set()
    for idx in selected_indices:
        if idx in encountered_indices:
            continue
        encountered_indices.add(idx)
        skip = idx - prev - 1
        data = itertools.islice(data, skip, None)
        try:
            yield next(data)
        except StopIteration:
            break
        prev = idx


def make_list(value: Any) -> List:
    """
    Make a list from a value.

    :param value: The value to make a list from.
    """
    return list(value) if is_iterable(value) else [value]


def is_iterable(obj: Any) -> bool:
    """
    Check if an object is iterable.

    :param obj: The object to check.
    """
    return hasattr(obj, "__iter__") and not isinstance(
        obj, (str, type, bytes, bytearray)
    )


def make_tuple(value: Any) -> Any:
    """
    Make a tuple from a value.
    """
    return tuple(value) if is_iterable(value) else (value,)


def make_set(value: Any) -> Set:
    """
    Make a set from a value.

    :param value: The value to make a set from.
    """
    return set(value) if is_iterable(value) else {value}


@dataclass(eq=False)
class ALL:
    """
    Sentinel that compares equal to any other value.

    This is used to signal wildcard matches in hashing/containment logic.
    """

    def __eq__(self, other):
        """Always return True."""
        return True

    def __hash__(self):
        """Hash based on object identity to remain unique as a sentinel."""
        return hash(id(self))


All = ALL()
