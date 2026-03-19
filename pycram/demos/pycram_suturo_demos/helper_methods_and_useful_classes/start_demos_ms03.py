from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pycram.external_interfaces.nlp_interface import NlpInterface, FilterOptions


class TalkingNode(Node):
    """
    ROS2 node that interfaces with a speech/NLP system.

    Responsibilities:
    -----------------
    - Publish a trigger message to start the speech recognition/NLP pipeline.
    - Subscribe to the processed NLP output.
    - Parse NLP output into a structured Python list.
    - Expose a blocking `talk_nlp()` call that waits for NLP results.
    """

    def __init__(self):

        # Initialize ROS2 node with name "talking"
        super().__init__('talking')

        # Publisher to let Toya talk
        self.talk_pub = self.create_publisher(
            String,
            '/tts_text',
            10
        )

    def pub(self, text: str):
        msg = String()
        msg.data = text
        self.get_logger().info(f"Publishing: {text}")

        self.talk_pub.publish(msg)



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

    talk = TalkingNode()

    #print(resp)

    match nlp.filter_response(resp, FilterOptions.INTENT):
        case 'seating':
            if "waving" in resp[2][0][4]:
                print("start 'Tell the waving person where he/she can sit'")
                talk.pub("I will go and show them where to sit.")
        case 'lookup':
            where = resp[2][0][1]
            print(f"Start Challenge 'Is there something on the {where}.'")
            talk.pub(f"I will look if there is something on the {where}.")
        case 'deliver':
            re = nlp.filter_response(resp, FilterOptions.FURNITURE)
            item = nlp.filter_response(resp, FilterOptions.ITEM)
            if re is None:
                print("Oh no")
            if len(re) == 1:
                print(f"Start Challenge 'Bring me object {item[0]} from the {re[0]}.'")
                talk.pub(f"I will drive to the {re[0]} now and get the object {item[0]}.")
            elif len(re) == 2:
                print(f"Start Challenge 'Bring object {item[0]} from the {re[0]} to the {re[1]}.'")
                talk.pub(f"I will drive to the {re[0]} now and get the object {item[0]} and bring it to the {re[1]}.")
            else:
                print("Oh no")
        case 'open':
            print("Start Challenge 'Open the door.'")
            talk.pub(f"I will go and open the door now.")


    #print(f"last intent: {nlp.filter_response(nlp.last_output, FilterOptions.INTENT)}")
    #print(f"last output: {nlp.last_output}")
    #print(f"complete output: {nlp.all_last_outputs}")
    #print(f"last confirmation: {nlp.last_confirmation}")


if __name__ == "__main__":
    rclpy.init()
    main()
