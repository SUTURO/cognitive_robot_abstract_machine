from typing import Any

import rclpy
#import gpsr_01
from pycram.external_interfaces.nlp_interface import NlpInterface
import subprocess
import socket
import psutil
from time import sleep
import time
import requests



class NlpInterfaceStartingDemo(NlpInterface):

    def __init__(self):
        super().__init__()

    def filter_response(self, response : list[Any], challenge : str):
        # TODO: role einbinden, damit challenge ausgegeben wird
        print(response)
        return response[2][0][1] # should return value from entity_elem

def is_port_in_use(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(('0.0.0.0', port)) == 0

def wait_for_rasa(
    url="http://localhost:5005/status",
    timeout=120,
    interval=1
):
    start = time.time()
    while time.time() - start < timeout:
        try:
            r = requests.get(url, timeout=1)
            if r.status_code == 200:
                print("Rasa is ready")
                return True
        except requests.exceptions.RequestException:
            pass
        time.sleep(interval)

    raise TimeoutError("Rasa did not start in time")

def wait_for_whisper(proc, keyword="started", timeout=60):
    start = time.time()
    while time.time() - start < timeout:
        line = proc.stdout.readline()
        if keyword in line:
            print(f"Found '{keyword}'")
            return True
    raise TimeoutError("Whisper did not start")


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

    Don't want to automate this, because accidentally killing wrong processes is not that good ._.
    DEBUG: if port 5005 is blocked, in terminal:
            lsof -i :5005
    take PID and:
            kill <PID>
    """
    nlp = NlpInterfaceStartingDemo()


    # DEBUG
    # nlp.last_output = ["Start", "", ["", "GPSR"]]

    while True:
        # starts NLP Pipeline for demo start
        proc = subprocess.Popen('bash -i -c "nlp_rasa_demo_start"', shell=True)
        wait_for_rasa()

        proc2 = subprocess.Popen('bash -i -c "nlp_whisper_start"', shell=True, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True)
        wait_for_whisper(proc2)

        sleep(3)

        #nlp.input_confirmation_loop(4)
        nlp.start_nlp()

        print(nlp.last_output)

        # if intent wasn't understood right even after confirmation and repeating n-times
        if nlp.last_output[1] != "Start" and nlp.last_output[0] != "Change":
            raise NotImplementedError("NLP Intent should not happen.")

        end_process([proc, proc2])

        # waits until rasa port is free again
        while is_port_in_use(5005):
            print("Port 5005 is still in use, waiting...")
            sleep(2)

        if nlp.last_output is None:
            print("Oh no :((")

        # starts rasa with model for challenge
        match nlp.filter_response(nlp.last_output, ""):
            case 'Receptionist':
                proc = subprocess.Popen('bash -i -c "nlp_rasa_receptionist_start"', shell=True) # befehle sind alias befehle
            case 'GPSR':
                proc = subprocess.Popen('bash -i -c "nlp_rasa_gpsr_start"', shell=True)
                #gpsr_01.main()
            case 'Restaurant':
                proc = subprocess.Popen('bash -i -c "nlp_rasa_restaurant_start"', shell=True)
            case _:
                NotImplemented()
        wait_for_rasa()
        proc2 = subprocess.Popen('bash -i -c "nlp_whisper_start"', shell=True, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True)
        wait_for_whisper(proc2)

        match nlp.filter_response(nlp.last_output, ""):
            case 'Receptionist':
                print("Insert code here.......")
            case 'GPSR'|'GPS'|'GPS R':
                print("Insert GPSR code here.......")
                # gpsr_01.main()
            case 'Restaurant':
                print("Insert code here.......")
            case _:
                print("couldn't understand your input")

        end_process([proc, proc2])

        while is_port_in_use(5005):
            print("Port 5005 is still in use, waiting...")
            sleep(2)

        print("If you want to stop, interrupt now")
        number = 5
        for i in range (5, 0):
            print(number)
            number -= 1
            sleep(2)



if __name__ == "__main__":
    rclpy.init()
    main()