import rclpy
from pycram.external_interfaces import robokudo

rclpy.init()
result = robokudo.query_object_str("beverage")
# result = robokudo.query_all_objects()
processed_results = []
if result and result.res:
    for obj in result.res:
        if obj.description and obj.pose:
            description = obj.description[0]
            if "colabottle" in description.lower():
                position = obj.pose[0].pose.position
                processed_results.append(
                    {"description": description, "position": position}
                )

if processed_results:
    print(processed_results)
else:
    print("Das gewünschte Objekt wurde nicht gefunden.")
robokudo.shutdown_robokudo_interface()
rclpy.shutdown()
