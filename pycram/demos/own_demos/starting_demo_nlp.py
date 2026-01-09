from typing import Any

import rclpy
from debian.debtags import output
import gpsr_01
from pycram.external_interfaces.nlp_interface import NlpInterface

class NlpInterfaceStartingDemo(NlpInterface):

    def __init__(self):
        super().__init__()

    def filter_response(self, response : list[Any], challenge : str):
        # TODO: role einbinden, damit challenge ausgegeben wird
        return [response[1]]
        print("heehee")


def main():
    nlp = NlpInterfaceStartingDemo()
    nlp.input_confirmation_loop(4)
    if nlp.last_output is None:
        print("Oh no :((")

    # TODO: füge alle funktionen der demos ein
    match nlp.filter_response(nlp.last_output, ""):
        case "Receptionist":
            print("Start demo here")
        case "GPSR":
            gpsr_01.main()
        case "...":
            print("Start demo here")
        case _:
            print("Understood the command wrong")

if __name__ == "__main__":
    rclpy.init()
    main()