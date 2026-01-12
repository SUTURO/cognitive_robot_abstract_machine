from typing import Any

import rclpy
from debian.debtags import output
#import gpsr_01
from pycram.external_interfaces.nlp_interface import NlpInterface
import subprocess
import socket
import psutil
from time import sleep



class NlpInterfaceStartingDemo(NlpInterface):

    def __init__(self):
        super().__init__()

    def filter_response(self, response : list[Any], challenge : str):
        # TODO: role einbinden, damit challenge ausgegeben wird
        return response[1]

def is_port_in_use(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(('0.0.0.0', port)) == 0

def end_process(procs : list[subprocess.Popen]):
    for proc in procs:
        try:
            parent = psutil.Process(proc.pid)
            children = parent.children(recursive=True)
            for child in children:
                print(f"Killing child {child.pid}")
                child.kill()
            parent.kill()
            parent.wait(timeout=5)
            print("PROCESS TREE TERMINATED")
        except psutil.NoSuchProcess:
            print("Process already terminated")


def main():
    """
    Starts demos and rasa and whisper for our demos

    Note: If interrupted too early, processes might not be terminated, if possible, terminate when asked, else see DEBUG

    DEBUG: if port 5005 is blocked, in terminal:
            lsof -i :5005
    take PID and:
            kill <PID>
    """
    nlp = NlpInterfaceStartingDemo()

    # DEBUG
    # nlp.last_output = ["", "GPSR"]

    while True:
        # starts NLP Pipeline for demo start
        proc = subprocess.Popen('bash -i -c "nlp_rasa_start"', shell=True)
        sleep(45)

        proc2 = subprocess.Popen('bash -i -c "nlp_whisper_start"', shell=True)
        sleep(20)

        nlp.input_confirmation_loop(4)

        end_process([proc, proc2])

        # waits until rasa port is free again
        while is_port_in_use(5005):
            print("Port 5005 is still in use, waiting...")
            sleep(3)

        if nlp.last_output is None:
            print("Oh no :((")

        # starts rasa with model for challenge
        match nlp.filter_response(nlp.last_output, ""):
            case 'Receptionist':
                proc = subprocess.Popen('bash -i -c "nlp_rasa_receptionist_start"', shell=True) # befehle sind alias befehle
            case 'GPSR':
                proc = subprocess.Popen('bash -i -c "nlp_rasa_start"', shell=True)
                #gpsr_01.main()
            case 'Restaurant':
                proc = subprocess.Popen('bash -i -c "nlp_rasa_restaurant_start"', shell=True)
            case _:
                NotImplemented()
        sleep(45)

        proc2 = subprocess.Popen('bash -i -c "nlp_whisper_start"', shell=True)
        sleep(20)

        match nlp.filter_response(nlp.last_output, ""):
            case 'Receptionist':
                NotImplemented()
            case 'GPSR':
                NotImplemented()
                # gpsr_01.main()
            case 'Restaurant':
                NotImplemented()
            case _:
                NotImplemented()

        end_process([proc, proc2])

        while is_port_in_use(5005):
            print("Port 5005 is still in use, waiting...")
            sleep(3)

        print("If you want to stop, interrupt now")
        i = 5
        for i in range (5, 0):
            print(i)
            sleep(1)



if __name__ == "__main__":
    rclpy.init()
    main()