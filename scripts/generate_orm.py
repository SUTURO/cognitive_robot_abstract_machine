import builtins
import os
from enum import Enum

from ormatic.dao import AlternativeMapping
from ormatic.ormatic import ORMatic
from ormatic.utils import classes_of_module, recursive_subclasses


import krrood.lubm
import krrood.generator


# create of classes that should be mapped
classes = set(recursive_subclasses(AlternativeMapping))
classes |= set(classes_of_module(krrood.lubm))
classes |= {krrood.generator.Dataset}

# remove classes that should not be mapped
# classes -= {
# }
classes -= set(recursive_subclasses(Enum))
classes -= set(recursive_subclasses(Exception))


def generate_orm():
    """
    Generate the ORM classes for the pycram package.
    """
    # Create an ORMatic object with the classes to be mapped
    ormatic = ORMatic(list(classes))

    # Generate the ORM classes
    ormatic.make_all_tables()

    path = os.path.abspath(os.path.join(os.getcwd(), "..", "src", "krrood", "orm"))
    with builtins.open(os.path.join(path, "ormatic_interface.py"), "w") as f:
        ormatic.to_sqlalchemy_file(f)


if __name__ == "__main__":
    generate_orm()
