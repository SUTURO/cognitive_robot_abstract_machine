from typing import Any

import rclpy

from pycram.external_interfaces.nlp_interface import NlpInterface

class NlpInterfaceGPSR(NlpInterface):
    def __init__(self):
        super().__init__()

    def filter_response(self, response : list[Any], challenge : str):
        print("heehee")


def main():
    nlp = NlpInterfaceGPSR()
    nlp.start_nlp()
    print(nlp.confirm_last_response().__str__())

if __name__ == "__main__":
    rclpy.init()
    main()