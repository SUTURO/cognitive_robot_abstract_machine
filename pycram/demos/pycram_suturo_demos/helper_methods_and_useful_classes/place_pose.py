from typing import List

from pycram.datastructures.pose import PoseStamped
from pycram.designators.location_designator import SemanticCostmapLocation
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body


def get_poses_for_object_on_body(
    for_object: Body,
    body,
    world: World,
    amount_of_locations: int,
    link_is_center_link: bool = False,
) -> List[PoseStamped]:
    location_description = SemanticCostmapLocation(
        body=body,
        for_object=for_object,
    )
    z_offset = (
        (
            body.collision.as_bounding_box_collection_in_frame(world.root)
            .bounding_box()
            .height
            / 2
        )
        - 0.03
        if link_is_center_link
        else 0
    )
    poses: List[PoseStamped] = []
    for i, pose in enumerate(location_description):
        pose.position.z += z_offset
        poses.append(pose)
        if i >= amount_of_locations - 1:
            break
    return poses


def get_pose_for_object_on_body(
    for_object: Body,
    body,
    world: World,
    link_is_center_link: bool = False,
) -> PoseStamped:
    return get_poses_for_object_on_body(
        for_object=for_object,
        body=body,
        world=world,
        amount_of_locations=1,
        link_is_center_link=link_is_center_link,
    )[0]
