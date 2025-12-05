from pycram.external_interfaces import tmc
import rclpy


if __name__ == '__main__':
    while True:
        tts = tmc.TextToSpeechPublisher()
        tts.say("Beans or not to Beans")
