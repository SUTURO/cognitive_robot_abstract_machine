from dataclasses import is_dataclass
import inspect
import sys

import semantic_digital_twin.semantic_annotations.semantic_annotations as sem_annotations

"""Helper functions for resolving semantic object classes from strings"""


# Faster lookup method:
# If the target dataclass is known to live inside a specific module
# (e.g. semantic_annotations), search only that module.
def _find_dataclass_by_name(module, class_name: str):
    """
    Search a given module for a dataclass with a specific name.

    :param module: Python module to inspect
    :param class_name: Name of the dataclass to find
    :return: Dataclass type if found, otherwise None
    """
    for _, obj in inspect.getmembers(module, inspect.isclass):
        if obj.__name__ == class_name and is_dataclass(obj):
            return obj
    return None


# Slower lookup method:
# Used when the module of the dataclass is unknown.
# Iterates over all currently loaded Python modules.
def _find_dataclass_global(class_name: str):
    """
    Search all loaded modules for a dataclass with a specific name.

    This is significantly slower than a module-local lookup and should
    only be used as a fallback.

    :param class_name: Name of the dataclass to find
    :return: Dataclass type if found, otherwise None
    """
    for module in list(sys.modules.values()):
        if not module:
            continue
        try:
            for _, obj in inspect.getmembers(module, inspect.isclass):
                if obj.__name__ == class_name and is_dataclass(obj):
                    return obj
        except Exception:
            # Some modules may raise exceptions when inspected;
            # these are safely ignored.
            pass
    return None


def get_object_class_from_string(string: str):
    """
    Resolve a semantic dataclass from a string representation.

    The string is capitalized to match class naming conventions
    (e.g. "cola" -> "Cola").

    First tries a fast lookup in the semantic_annotations module.
    Falls back to a global search if not found.

    :param string: Raw entity string from NLP
    :return: Dataclass type or None
    """

    # Try fast, module-local lookup first
    if _find_dataclass_by_name(sem_annotations, string.capitalize()) is not None:
        return _find_dataclass_by_name(sem_annotations, string.capitalize())
    else:
        # Fallback: search all loaded modules
        return _find_dataclass_global(string.capitalize())


def main():
    # Example usage / debug output
    print(f"find dataclass for Cola: {_find_dataclass_global("Cola")}")


if __name__ == "__main__":
    main()
