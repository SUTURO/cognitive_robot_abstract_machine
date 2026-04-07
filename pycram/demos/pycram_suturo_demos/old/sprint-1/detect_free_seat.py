import rclpy
from pycram.external_interfaces import robokudo

rclpy.init()
result = robokudo.query_specific_region("sofa")
print(result)
robokudo.shutdown_robokudo_interface()
rclpy.shutdown()
