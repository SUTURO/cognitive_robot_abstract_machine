from typing import Any

import rclpy

# import gpsr_01 # Uncomment if GPSR module is needed
from pycram.external_interfaces.nlp_interface import NlpInterface
import subprocess
import socket
import psutil
from time import sleep
import time
import requests


class NlpInterfaceStartingDemo(NlpInterface):
    """
    Subclass of the NLP interface for starting demos.
    """

    def __init__(self):
        super().__init__()

    def filter_response(self, response: list[Any], challenge: str):
        """
        Extract a specific response value from the NLP output.

        Args:
            response (list[Any]): The NLP output list.
            challenge (str): Placeholder for challenge-based filtering (not used yet).

        Returns:
            The challenge name.
        """
        return response[2][0][1]  # should return value from entity_elem


def is_port_in_use(port):
    """
    Check if a TCP port is currently in use on localhost.

    Args:
        port (int): Port number to check.

    Returns:
        bool: True if port is in use, False otherwise.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(("0.0.0.0", port)) == 0


def wait_for_rasa(url="http://localhost:5005/status", timeout=120, interval=1):
    """
    Poll Rasa server until it becomes available or timeout occurs.

    Args:
        url (str): The URL to poll for Rasa status.
        timeout (int): Maximum seconds to wait.
        interval (int): Seconds between requests.

    Raises:
        TimeoutError: If Rasa does not respond within timeout.
    """
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
    """
    Wait for a Whisper process to output a specific keyword, signaling readiness.

    Args:
        proc (subprocess.Popen): Whisper subprocess.
        keyword (str): Keyword to search for in stdout.
        timeout (int): Maximum seconds to wait.

    Raises:
        TimeoutError: If keyword not found within timeout.
    """
    start = time.time()
    while time.time() - start < timeout:
        line = proc.stdout.readline()
        if keyword in line:
            print(f"Found '{keyword}'")
            return True
    raise TimeoutError("Whisper did not start")


def end_process(procs: list[subprocess.Popen]):
    """
    Safely terminate a list of subprocesses and all their child processes.

    Args:
        procs (list[subprocess.Popen]): List of running subprocesses.
    """
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
    Main loop to start demo and challenge NLP pipelines safely.

    Each iteration:
        1. Starts the demo Rasa and Whisper pipelines.
        2. Runs NLP input/confirmation logic.
        3. Waits for port 5005 to become free.
        4. Starts challenge Rasa and Whisper pipelines based on the demo response.
        5. Runs challenge logic.

    Each process lifecycle is wrapped in try/finally to guarantee cleanup,
    preventing hanging processes or blocked ports even in case of exceptions or Ctrl+C.
    ---------------------------------------------------------------------------------

    Note: If interrupted too early, processes might not be terminated, if possible, terminate when asked, else see DEBUG

    Don't want to automate this, because accidentally killing wrong processes is not that good ._.

    DEBUG: if port 5005 is blocked, in terminal:
            lsof -i :5005
    take PID and:
            kill <PID>
    """
    proc = None
    proc2 = None

    nlp = NlpInterfaceStartingDemo()

    # DEBUG
    # nlp.last_output = ["Start", "", ["", "GPSR"]]

    try:
        while True:
            # ----------------- DEMO PIPELINE -----------------
            proc = proc2 = None
            try:
                # Start demo Rasa server
                proc = subprocess.Popen('bash -i -c "nlp_rasa_start"', shell=True)
                wait_for_rasa()

                # Start demo Whisper process
                proc2 = subprocess.Popen(
                    'bash -i -c "nlp_whisper_start"',
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                wait_for_whisper(proc2)

                sleep(3) # Give processes some buffer time - needed, otherwise doesn't work sometimes

                # gets nlp input
                # nlp.input_confirmation_loop(4)
                nlp.start_nlp()

                print(nlp.last_output)
            finally:
                # Ensure all demo processes are terminated after this block
                if proc is not None:
                    end_process([proc])
                if proc2 is not None:
                    end_process([proc2])

            # Validate demo output

            if nlp.last_output is None:
                print("Oh no :((")

            if nlp.last_output[1] != "Start" and nlp.last_output[0] != "Change":
                raise NotImplementedError("NLP Intent should not happen.")

            # Wait until Rasa port is free before starting challenge
            while is_port_in_use(5005):
                print("Port 5005 is still in use, waiting...")
                sleep(2)

            # ----------------- CHALLENGE PIPELINE -----------------
            proc = proc2 = None
            try:
                # Start challenge Rasa based on demo NLP response
                match nlp.filter_response(nlp.last_output, ""):
                    case "Receptionist":
                        proc = subprocess.Popen(
                            'bash -i -c "nlp_rasa_receptionist_start"', shell=True
                        )  # alias terminal commands
                    case "GPSR":
                        proc = subprocess.Popen('bash -i -c "nlp_rasa_gpsr_start"', shell=True)
                    case "Restaurant":
                        proc = subprocess.Popen(
                            'bash -i -c "nlp_rasa_restaurant_start"', shell=True
                        )
                    case _:
                        raise NotImplementedError()
                wait_for_rasa()

                # Start challenge Whisper process
                proc2 = subprocess.Popen(
                    'bash -i -c "nlp_whisper_start"',
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                wait_for_whisper(proc2)

                # Run the challenge logic depending on NLP response
                match nlp.filter_response(nlp.last_output, ""):
                    case "Receptionist":
                        print("Insert code here.......")
                    case "GPSR" | "GPS" | "GPS R":
                        print("Insert GPSR code here.......")
                        # gpsr_01.main()
                    case "Restaurant":
                        print("Insert code here.......")
                    case _:
                        print("couldn't understand your input")
            finally:
                # Ensure all challenge processes are terminated
                if proc is not None:
                    end_process([proc])
                if proc2 is not None:
                    end_process([proc2])

            # Wait until port is free before next iteration
            while is_port_in_use(5005):
                print("Port 5005 is still in use, waiting...")
                sleep(2)


    except KeyboardInterrupt as e:
        print("User interrupted execution. Exiting safely...")
    finally:
        # Final cleanup in case any process remains
        if proc is not None:
            end_process([proc])
        if proc2 is not None:
            end_process([proc2])


if __name__ == "__main__":
    rclpy.init()
    main()
