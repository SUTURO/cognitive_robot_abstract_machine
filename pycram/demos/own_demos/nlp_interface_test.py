from abc import ABC
from typing import Any

import rclpy

from nlp_interface import NlpInterface

class NlpInterfaceTest(NlpInterface):
    def __init__(self):
        super().__init__()

    def filter_response(self, response : list[Any]):
        print("heehee")


def main():
    nlp = NlpInterfaceTest()
    nlp.start_nlp()
    print(nlp.confirm_last_response().__str__())

if __name__ == "__main__":
    rclpy.init()
    main()