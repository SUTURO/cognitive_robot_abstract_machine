from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world import World
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Milk,
    Fridge,
    Container,
    Door,
    Handle,
)
from semantic_digital_twin.reasoning import query_object_destination


def test_milk_goes_to_fridge():
    world = World(name="test_world")

    fridge_container = Container(
        name=PrefixedName("fridge_container"),
        body=Body(name=PrefixedName("fridge_container_body")),
    )
    fridge_handle = Handle(
        name=PrefixedName("fridge_handle"),
        body=Body(name=PrefixedName("fridge_handle_body")),
    )
    fridge_door = Door(
        name=PrefixedName("fridge_door"),
        body=Body(name=PrefixedName("fridge_door_body")),
        handle=fridge_handle,
    )
    fridge = Fridge(container=fridge_container, door=fridge_door)

    milk = Milk(
        name=PrefixedName("milk"),
        body=Body(name=PrefixedName("milk_body")),
        destination_class_names=["Fridge"],   # key point
    )

    with world.modify_world():
        world.add_semantic_annotation(fridge)
        world.add_semantic_annotation(milk)

    targets = query_object_destination(world, milk)
    assert fridge in targets