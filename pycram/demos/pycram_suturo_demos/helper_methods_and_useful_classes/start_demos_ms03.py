from typing import Any

import rclpy
from debian.debtags import output

from pycram.external_interfaces.nlp_interface import NlpInterface, FilterOptions


class NlpInterfaceDemoStartM3(NlpInterface):

    def __init__(self):
        super().__init__()

    def filter_response(self, response: list[Any], filter_for: FilterOptions):
        """
                Filters and post-processes the NLP response depending on the challenge.

                WARNING for all_last_outputs: This method is to process one response, if multiple responses are possible and you work with
                all_last_outputs remember to filter each response separately!

                Parameters:
                    response (list[Any]): Parsed NLP response
                    filter_for (FilterOptions): what attribute to filter for
                """
        if response is None:
            return None

        if not response:
            return None

        if filter_for is None:
            return response

        match filter_for:
            case FilterOptions.SENTENCE:
                return response[0]

            case FilterOptions.INTENT:
                return response[1]

            case _:
                elems = []
                for elem in response[2]:
                    if elem[0] == filter_for.value:
                        print(("AAAAAAAAAAAAAAH"))
                        elems.append(elem[1])

                if elems:
                    return elems
                else:
                    return None


def main():
    nlp = NlpInterfaceDemoStartM3()
    nlp.start_nlp()
    #nlp.input_confirmation_loop(2)
    resp = nlp.last_output

    print(resp)

    match nlp.filter_response(resp, FilterOptions.INTENT):
        case 'seating':
            if "waving" in resp[2][0][4]:
                print("start seating here")
        case 'deliver':
            re = nlp.filter_response(resp, FilterOptions.FURNITURE)
            item = nlp.filter_response(resp, FilterOptions.ITEM)
            if re is None:
                print("Oh no")
            if len(re) == 1:
                print(f"Start Challenge 'Bring me object {item[0]} from the {re[0]}.'")
            elif len(re) == 2:
                print(f"Start Challenge 'Bring object {item[0]} from the {re[0]} to the {re[1]}.'")
            else:
                print("Oh no")
        case 'open':
            print("Start Challenge 'Open the door.'")


    print(f"last intent: {nlp.filter_response(nlp.last_output, FilterOptions.INTENT)}")
    print(f"last output: {nlp.last_output}")
    print(f"complete output: {nlp.all_last_outputs}")
    print(f"last confirmation: {nlp.last_confirmation}")


if __name__ == "__main__":
    rclpy.init()
    main()
