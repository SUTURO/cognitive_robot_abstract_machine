from typing import Any, Optional

import rclpy
from debian.debtags import output
from uaclient.contract_data_types import Override
from dataclasses import dataclass, field, is_dataclass
import inspect
import sys

import semantic_digital_twin.semantic_annotations.semantic_annotations as sem_annotations

# if you know model - faster
def _find_dataclass_by_name(module, class_name: str):
    for _, obj in inspect.getmembers(module, inspect.isclass):
        if obj.__name__ == class_name and is_dataclass(obj):
            return obj
    return None

# if you don't know model - slower
def _find_dataclass_global(class_name: str):
    for module in list(sys.modules.values()):
        if not module:
            continue
        try:
            for _, obj in inspect.getmembers(module, inspect.isclass):
                if obj.__name__ == class_name and is_dataclass(obj):
                    return obj
        except Exception:
            pass
    return None



def get_object_class_from_string(string: str):
    if _find_dataclass_by_name(sem_annotations, string.capitalize()) is not None:
        return _find_dataclass_by_name(sem_annotations, string.capitalize())
    else:
        return _find_dataclass_global(string.capitalize())





def main():
    print(f"find dataclass for Cola: {_find_dataclass_global("Cola")}")

if __name__ == "__main__":
    main()