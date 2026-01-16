from typing import Any, Optional

import rclpy

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_helper_methods import get_object_class_from_string as get_obj


from pycram.external_interfaces.nlp_interface import NlpInterface
from semantic_digital_twin.semantic_annotations.semantic_annotations import Food, Drink
from semantic_digital_twin.world_description.world_entity import Human
import semantic_digital_twin.semantic_annotations.semantic_annotations as sem_annotations


class HriHuman(Human):
    # favourite drink, food and the order are saved as string and as objectTypes: (string, ObjectType)
    def __init__(self):
        super().__init__()
        # theoretically useless, emotionally very important
        self.favourite_robot = "Toya"

        self.hri_favourite_drink = (Optional[str], Optional[Drink])

        self.favourite_food = (Optional[str], Optional[Food])

        self.order = ([Optional[str]], [Optional[Food]], [Optional[Drink]])

        self.hobby = Optional[str]

    def debug(self):
        print(
            f"name: {self.name}, \n"
            f"favourite robot: {self.favourite_robot}, \n",
            f"favourite drink: {self.hri_favourite_drink}, \n",
            f"favourite food: {self.favourite_food}, \n",
            f"order: {self.order}, \n",
            f"hobby: {self.hobby}"
        )


# there are different Rasa Models, the Models are written behind the case
def process_response(responses: list[list[Any]], challenge: str, person: HriHuman):
    for response in responses:
        match response[1]:  # intent
            case "Order": # Model: Restaurant
                if len(response[2]) == 0:
                    print("No roles found.")
                repeat = 1
                for elem in response[2]:
                    if elem[0] == 'Food' or elem[0] == 'Drink':
                        item = elem[1]
                        if elem[5] == ['two']:
                            repeat = 2
                        if elem[5] == ['three']:
                            repeat = 3

                        for i in range(0, repeat):
                            print(repeat)
                            person.order[0].append(item) # entity value
                            if elem[0] == 'Food':
                                person.order[1].append(get_obj(elem[1]))
                            if elem[0] == 'Drink':
                                person.order[2].append(get_obj(elem[1]))

            case "Receptionist": # Model: Receptionist
                if len(response[2]) == 0:
                    print("No roles found.")
                    raise NotImplementedError
                # elem: the entity
                for elem in response[2]:
                    match elem[0]: # role
                        case "Drink":
                            person.hri_favourite_drink = (elem[1], get_obj(elem[1])) # elem[1]: value
                        case "Food":
                            person.favourite_food = (elem[1], get_obj(elem[1])) # elem[1]: value
                        case "Person":
                            person.name = PrefixedName(elem[1])
                        case "Hobby":
                            person.hobby = elem[1]
                        case _:
                            print("cool, dont know why I need this info.")
            case _:
                raise NotImplementedError



class HriNlpInterface(NlpInterface):

    def __init__(self):
        super().__init__()

    # has to be initialized for Interface, isn't used here because we also need to input human
    def filter_response(self, response: list[Any], challenge: str):
        raise NotImplementedError

def main():
    nlp = HriNlpInterface()
    nlp.start_nlp()

    person = HriHuman()
    process_response(responses=nlp.all_last_outputs, challenge="", person=person)

    print(person.debug())
    print(nlp.all_last_outputs)
    # nlp.input_confirmation_loop(2)

if __name__ == "__main__":
    rclpy.init()
    main()