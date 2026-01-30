import rclpy

from nlp_human_robot_interaction import *
from time import sleep


def main():
    sleep(3)

    print("Hello, please introduce yourself, who are you and what is your favorite drink?")
    nlp = HriNlpInterface()
    sleep(2)
    nlp.start_nlp()

    # Create HRI human representation
    person1 = HriHuman()

    # Process all collected NLP outputs
    process_response(responses=nlp.all_last_outputs, challenge="", person=person1)

    print(f"Hello {person1.name}, I also like to drink {person1.hri_favourite_drink[1]},")
    print(f"especially after a nice round of hobby horsing! What is your hobby?")

    sleep(2)

    nlp.start_nlp()
    process_response(responses=nlp.all_last_outputs, challenge="", person=person1)

    print(f"{person1.hobby} seems fun! It was nice meeting you, see you soon!")


if __name__ == "__main__":
    rclpy.init()
    main()