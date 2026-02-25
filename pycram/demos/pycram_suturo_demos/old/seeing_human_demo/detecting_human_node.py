import rclpy
from pycram.external_interfaces import robokudo

rclpy.init()
result = robokudo.send_query("human")
# result = robokudo.query_postion_closest_object()
print(result)
# result = robokudo.send_query()
# if result is None:
#    print("Error")
# else:
#    print(f"Result got location: {result}")
#    for r in result.res:
#        for p in r.pose:
#            print(f"Pose: {p.pose}")
robokudo.shutdown_robokudo_interface()
rclpy.shutdown()
