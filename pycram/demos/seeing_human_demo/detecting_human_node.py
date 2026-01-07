import rclpy
from pycram.external_interfaces import robokudo

rclpy.init()
location = robokudo.query_human()
if location is None:
    print("Error or No Human seen")
else:
    print(f"Human seen at: {location}")
robokudo.shutdown_robokudo_interface()
rclpy.shutdown()
