# test/semantic_digital_twin_test/test_reasoning/test_object_destination.py

import pytest
from dataclasses import field
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world import World
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Milk,
    Fridge,
    Cup,
    Cupboard,
    Sink,
    GarbageBin,
    Bottle,
    Table,
    HasSupportingSurface,
)
from semantic_digital_twin.reasoning.queries.object_destination import query_object_destination
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.geometry import Scale
from semantic_digital_twin.world_description.world_entity import Body


def test_milk_goes_to_fridge():
    world = World(name="test_world")

    # Erstelle root Body mit name
    root = Body(name=PrefixedName("root"))
    with world.modify_world():
        world.add_body(root)

    # Erstelle Fridge
    with world.modify_world():
        fridge = Fridge.create_with_new_body_in_world(
            name=PrefixedName("fridge"),
            world=world,
            scale=Scale(1.0, 1.0, 2.0),
        )

    # Erstelle Milk
    with world.modify_world():
        milk = Milk.create_with_new_body_in_world(
            name=PrefixedName("milk"),
            world=world,
            scale=Scale(0.1, 0.1, 0.2),
        )

    # Aktiviere modify_world für add_semantic_annotation
    with world.modify_world():
        world.add_semantic_annotation(fridge)
        world.add_semantic_annotation(milk)

    targets = query_object_destination(world, milk)
    assert fridge in targets
    assert len(targets) == 1


def test_cup_goes_to_cupboard_table_sink():
    world = World(name="test_world")

    # Erstelle root Body mit name
    root = Body(name=PrefixedName("root"))
    with world.modify_world():
        world.add_body(root)

    # Erstelle Cupboard
    with world.modify_world():
        cupboard = Cupboard.create_with_new_body_in_world(
            name=PrefixedName("cupboard"),
            world=world,
            scale=Scale(0.8, 0.8, 1.8),
        )

    # Erstelle Table
    with world.modify_world():
        table = Table.create_with_new_body_in_world(
            name=PrefixedName("table"),
            world=world,
            scale=Scale(1.0, 1.0, 0.1),
        )

    # Erstelle Sink
    with world.modify_world():
        sink = Sink.create_with_new_body_in_world(
            name=PrefixedName("sink"),
            world=world,
            scale=Scale(0.8, 0.6, 0.2),
        )

    # Erstelle Cup
    with world.modify_world():
        cup = Cup.create_with_new_body_in_world(
            name=PrefixedName("cup"),
            world=world,
            scale=Scale(0.05, 0.05, 0.08),
        )

    assert getattr(type(cup), "destination_class_names", None), "Klassenliste Cup leer?"


    with world.modify_world():
        world.add_semantic_annotation(cupboard)
        world.add_semantic_annotation(table)
        world.add_semantic_annotation(sink)
        world.add_semantic_annotation(cup)

    targets = query_object_destination(world, cup)
    assert cupboard in targets
    assert table in targets
    assert sink in targets
    assert len(targets) == 3


def test_bottle_goes_to_sink_cupboard():
    world = World(name="test_world")

    # Erstelle root Body mit name
    root = Body(name=PrefixedName("root"))
    with world.modify_world():
        world.add_body(root)

    # Erstelle Sink
    with world.modify_world():
        sink = Sink.create_with_new_body_in_world(
            name=PrefixedName("sink"),
            world=world,
            scale=Scale(0.8, 0.6, 0.2),
        )

    # Erstelle Cupboard
    with world.modify_world():
        cupboard = Cupboard.create_with_new_body_in_world(
            name=PrefixedName("cupboard"),
            world=world,
            scale=Scale(0.8, 0.8, 1.8),
        )

    # Erstelle Bottle
    with world.modify_world():
        bottle = Bottle.create_with_new_body_in_world(
            name=PrefixedName("bottle"),
            world=world,
            scale=Scale(0.05, 0.05, 0.2),
        )


    with world.modify_world():
        world.add_semantic_annotation(sink)
        world.add_semantic_annotation(cupboard)
        world.add_semantic_annotation(bottle)

    targets = query_object_destination(world, bottle)
    assert sink in targets
    assert cupboard in targets
    assert len(targets) == 2


def test_garbage_bin_goes_to_itself():
    world = World(name="test_world")

    # Erstelle root Body mit name
    root = Body(name=PrefixedName("root"))
    with world.modify_world():
        world.add_body(root)

    # Erstelle GarbageBin
    with world.modify_world():
        bin = GarbageBin.create_with_new_body_in_world(
            name=PrefixedName("garbage_bin"),
            world=world,
            scale=Scale(0.5, 0.5, 0.8),
        )

    # Erstelle GarbageBin (zweite Instanz)
    with world.modify_world():
        bin2 = GarbageBin.create_with_new_body_in_world(
            name=PrefixedName("garbage_bin_2"),
            world=world,
            scale=Scale(0.5, 0.5, 0.8),
        )


    with world.modify_world():
        world.add_semantic_annotation(bin)
        world.add_semantic_annotation(bin2)

    targets = query_object_destination(world, bin)
    assert bin in targets
    assert bin2 in targets
    assert len(targets) == 2