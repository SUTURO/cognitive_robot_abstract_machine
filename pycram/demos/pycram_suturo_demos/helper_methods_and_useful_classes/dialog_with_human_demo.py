import rclpy

from nlp_human_robot_interaction import *
from time import sleep


def main():
    talking = TalkingNode()
    sleep(15)
    # print("-------------------------------------------------------------------------------------------------")
    # print("Hello, please introduce yourself, who are you and what is your favorite drink?")
    talking.pub("Hello, please introduce yourself, who are you and what is your favorite drink")

    nlp = HriNlpInterface()
    sleep(3)
    nlp.start_nlp()

    # Create HRI human representation
    person1 = HriHuman()

    # Process all collected NLP outputs
    process_response(responses=nlp.all_last_outputs, challenge="", person=person1)

    # print(f"Hello {person1.name}, I also like to drink {person1.hri_favourite_drink[0]},")
    # print(f"especially after a nice round of hobby horsing! What is your hobby?")
    talking.pub(f"Hello {person1.name}, I also like to drink {person1.hri_favourite_drink[0]}, especially after a nice round of hobby horsing! What is your hobby?")

    sleep(4)

    nlp.start_nlp()
    process_response(responses=nlp.all_last_outputs, challenge="", person=person1)

    #print(f"{person1.hobby} seems fun! It was nice meeting you, see you soon!")
    #print("-------------------------------------------------------------------------------------------------")
    talking.pub(f"{person1.hobby} seems fun! It was nice meeting you, see you soon!")

    print(person1.debug())

if __name__ == "__main__":
    rclpy.init()
    main()