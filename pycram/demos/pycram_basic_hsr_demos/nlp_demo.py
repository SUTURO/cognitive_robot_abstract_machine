from typing import Any

import rclpy
from debian.debtags import output

from pycram.external_interfaces.nlp_interface import NlpInterface, FilterOptions


class NlpInterfaceTest(NlpInterface):

    def __init__(self):
        super().__init__()

    def filter_response(self, response : list[Any], filter_option : FilterOptions):
        return super().filter_response(response, filter_option)


def main():
    nlp = NlpInterfaceTest()
    nlp.input_confirmation_loop(2)
    print(f"last intent: {nlp.filter_response(nlp.last_output, FilterOptions.INTENT)}")
    print(f"last output: {nlp.last_output}")
    print(f"complete output: {nlp.all_last_outputs}")
    print(f"last confirmation: {nlp.last_confirmation}")

if __name__ == "__main__":
    rclpy.init()
    main()