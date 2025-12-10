from pycram.external_interfaces import tmc
import rclpy
import nlp_gpsr
from semantic_digital_twin.robots.hsrb import HSRB

from std_msgs.msg import String

tts = tmc.TextToSpeechPublisher()
nlp_node = nlp_gpsr.NLP_GPSR()
nlp_node.talk_nlp()

"""
This is a demo for the TextToSpeechPublisher

The process step is structured as follows:
0. Custom input
1. Welcoming and recognising human (On arm push)
2. talking before movement to tell what its´ about to do
3. talk back that it now reached its goal
4. Talk back, that she will now go back to its starting point

param
"""
def talking_process(process_step : int = 0, voice_input : str = ""):
    if process_step == 0:
      tts.say(voice_input)
    elif process_step == 1:
        tts.say("Hello, I´m Toya. Great to meet you. How can I help you today?")
        # Here it should also be published to the Screen
    elif process_step == 2:
        tts.say("")
        # Here it should also be published to the Screen
    elif process_step == 3:
        tts.say()
        # Here it should also be published to the Screen
    elif process_step == 4:
        tts.say("I´m sorry, I did not properly understand your request.")
        # Here it should also be published to the Screen

def affirmation_process(voice_input : str = ""):
    tts.say(voice_input)
    tts.say("Did I understand you correctly?")
    nlp_node.parse_json_string() # where do i extract the yess here lol
