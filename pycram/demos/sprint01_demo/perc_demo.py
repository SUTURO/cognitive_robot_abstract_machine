import rclpy
from pycram.external_interfaces import robokudo

rclpy.init()
result = robokudo.query_object_str("beverage")
processed_results = []
if result and result.res:
    for obj in result.res:
        if obj.description and obj.pose:
            description = obj.description[0]
            position = obj.pose[0].pose.position
            processed_results.append({"description": description, "position": position})

print(processed_results)
robokudo.shutdown_robokudo_interface()
rclpy.shutdown()
