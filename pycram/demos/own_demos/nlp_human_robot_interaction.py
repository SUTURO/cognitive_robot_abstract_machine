import logging
from typing import Any, Optional

import rclpy

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_helper_methods import get_object_class_from_string as get_obj


from pycram.external_interfaces.nlp_interface import NlpInterface
from semantic_digital_twin.semantic_annotations.semantic_annotations import Food, Drink
from semantic_digital_twin.world_description.world_entity import Human


class HriHuman(Human):
    """
    Extension of the Human world entity for Human–Robot Interaction (HRI).

    Stores personal preferences, orders, and other user-related information
    extracted from NLP responses.

    attributes:
    - name (inherited from Human)
    - favourite robot
    - favourite drink (NOT inherited from human, different data types)
    - favourite food
    - order
    - hobby

    each attribute is saved as (Optional[str], Optional[<DataType SemanticAnnotation>]),
    DataType SemanticAnnotation only if it exists, else None.
    """
    # favourite drink, food and the order are saved both as
    # raw strings (from NLP) and as semantic object types
    def __init__(self):
        super().__init__()

        # theoretically useless (for now), emotionally very important
        self.favourite_robot = "Toya"

        # Tuple: (entity name as string, semantic Drink object)
        self.hri_favourite_drink = (Optional[str], Optional[Drink])

        # Tuple: (entity name as string, semantic Food object)
        self.favourite_food = (Optional[str], Optional[Food])

        # Order structure:
        # [0] list of raw entity strings
        # [1] list of Food objects
        # [2] list of Drink objects
        self.order = ([Optional[str]], [Optional[Food]], [Optional[Drink]])

        # User hobby as plain text
        self.hobby = Optional[str]

    def debug(self):
        """Print all stored HRI-relevant information for debugging."""
        print(
            f"name: {self.name}, \n"
            f"favourite robot: {self.favourite_robot}, \n",
            f"favourite drink: {self.hri_favourite_drink}, \n",
            f"favourite food: {self.favourite_food}, \n",
            f"order: {self.order}, \n",
            f"hobby: {self.hobby}"
        )


# there are different Rasa Models for each challenge, the Models are written behind the case, option for later: use
# string challenge, if some intents are the same for different challenges, but should behave different in hri
def process_response(responses: list[list[Any]], challenge: str, person: HriHuman):
    """
    Process NLP responses and update the HriHuman instance accordingly.

    :param responses: NLP outputs (intent + extracted entities)
    :param challenge: Name of the current challenge (currently unused)
    :param person: HriHuman object to update
    """
    for response in responses:
        match response[1]:  # intent
            case "Order":  # Model: Restaurant
                if len(response[2]) == 0:
                    print("No roles found.")

                repeat = 1
                for elem in response[2]:
                    # elem structure:
                    # [0] role (Food / Drink)
                    # [1] value
                    # [5] quantity modifier (e.g. "two", "three")
                    if elem[0] == 'Food' or elem[0] == 'Drink':
                        item = elem[1]

                        # Handle quantity words
                        if elem[5] == ['two']:
                            repeat = 2
                        if elem[5] == ['three']:
                            repeat = 3

                        # Add item(s) to the order
                        for i in range(0, repeat):
                            print(repeat)
                            person.order[0].append(item)  # entity value
                            if elem[0] == 'Food':
                                person.order[1].append(get_obj(elem[1]))
                            if elem[0] == 'Drink':
                                person.order[2].append(get_obj(elem[1]))

            case "Receptionist":  # Model: Receptionist
                if len(response[2]) == 0:
                    logging.warning(f"No roles found in response: {response}")

                # Process extracted entities
                for elem in response[2]:
                    match elem[0]:  # role
                        case "Drink":
                            # Save favourite drink as (string, semantic object)
                            person.hri_favourite_drink = (elem[1], get_obj(elem[1]))  # elem[1]: value

                        case "Food":
                            # Save favourite food as (string, semantic object)
                            person.favourite_food = (elem[1], get_obj(elem[1]))  # elem[1]: value

                        case "Person":
                            # Assign human name with semantic prefix
                            person.name = PrefixedName(elem[1])

                        case "Hobby":
                            # Save hobby
                            person.hobby = elem[1]

                        case _:
                            logging.warning(f'Role not implemented in hri: {elem}')

            case _:
                logging.warning(f'Intent not implemented in hri: {response[1]}')


class HriNlpInterface(NlpInterface):
    """
    NLP interface wrapper for HRI use cases.
    """

    def __init__(self):
        super().__init__()

    # Required by the interface but unused here because
    # we need direct access to the human object
    def filter_response(self, response: list[Any], challenge: str):
        raise NotImplementedError


def main():
    # Start NLP system
    nlp = HriNlpInterface()
    nlp.start_nlp()

    # Create HRI human representation
    person = HriHuman()

    # Process all collected NLP outputs
    process_response(responses=nlp.all_last_outputs, challenge="", person=person)

    # Debug output
    print(person.debug())
    print(nlp.all_last_outputs)
    # TODO nlp.input_confirmation_loop(2)


if __name__ == "__main__":
    rclpy.init()
    main()