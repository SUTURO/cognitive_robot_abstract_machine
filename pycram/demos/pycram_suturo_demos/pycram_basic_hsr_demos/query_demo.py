import rclpy
from pycram.external_interfaces.robokudo import (
    query_human,
    send_query,
    shutdown_robokudo_interface,
)
from time import sleep

# Start challange then send
rclpy.init()
sleep(2)
human = send_query(obj_type="human", attributes=["waving"])
print("Human:")
print(human)
shutdown_robokudo_interface()
rclpy.shutdown()

# looking into new answers from Perception adding more later
