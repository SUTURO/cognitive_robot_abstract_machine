"""
Tests for simulating cloth in MuJoCo.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta
from pathlib import Path
from xml.etree import ElementTree

import mujoco
import numpy
import pytest
import trimesh
from typing_extensions import Iterator

from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.adapters.multi_sim import (
    ContactCategories,
    MujocoBuilder,
    MujocoSim,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import TShirt
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.cloth import (
    ClothSheet,
    Cotton,
    GarmentFile,
)
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Color, Mesh, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% scene


@dataclass
class TShirtAboveTable:
    """
    A T-shirt lying flat a little above a table top, ready to drop onto it.
    """

    world: World
    """
    The world holding the table and the T-shirt.
    """

    t_shirt: ClothSheet
    """
    The T-shirt.
    """

    table_top_height: float
    """
    Height of the table top, in metres.
    """


@pytest.fixture
def t_shirt_above_table() -> TShirtAboveTable:
    """
    :return: A world with a table and a T-shirt lying flat 10 cm above its top.
    """
    table_top_height = 0.7
    table_thickness = 0.04
    world = World()
    with world.modify_world():
        root = Body(name=PrefixedName("root"))
        world.add_body(root)
        table = Body(name=PrefixedName("table"))
        table.collision = ShapeCollection(
            [
                Box(
                    origin=HomogeneousTransformationMatrix(reference_frame=table),
                    scale=Scale(1.2, 1.2, table_thickness),
                )
            ],
            reference_frame=table,
        )
        table.visual = table.collision
        world.add_connection(
            FixedConnection(
                parent=root,
                child=table,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    z=table_top_height - table_thickness / 2, reference_frame=root
                ),
            )
        )
    # the T-shirt's file lays it out from the neckline along x
    t_shirt = ClothSheet(
        name="t_shirt",
        surface=GarmentFile.T_SHIRT.create_surface(
            HomogeneousTransformationMatrix.from_xyz_rpy(
                x=-0.4, z=table_top_height + 0.1
            )
        ),
        mass=0.255,
    )
    world.add_simulator_property(t_shirt)
    return TShirtAboveTable(
        world=world, t_shirt=t_shirt, table_top_height=table_top_height
    )


@pytest.fixture
def simulation(t_shirt_above_table: TShirtAboveTable) -> Iterator[MujocoSim]:
    """
    :return: A stepped simulation of the T-shirt above the table, solved the way cloth
        simulates at an affordable cost.
    """
    simulation = MujocoSim(
        world=t_shirt_above_table.world,
        headless=True,
        step_size=2e-3,
        integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
        solver=mujoco.mjtSolver.mjSOL_CG,
    )
    simulation.start_stepped_simulation()
    yield simulation
    simulation.stop_simulation()


# %% building


def world_holding(cloth: ClothSheet) -> World:
    """
    :param cloth: The cloth the world is to hold.
    :return: A world holding nothing but the cloth.
    """
    world = World()
    with world.modify_world():
        world.add_body(Body(name=PrefixedName("root")))
    world.add_simulator_property(cloth)
    return world


def build_model_of(cloth: ClothSheet, directory: Path) -> mujoco.MjModel:
    """
    :param cloth: The cloth to build.
    :param directory: Where the built scene is written.
    :return: The model the builder makes of a world holding nothing but the cloth.
    """
    MujocoBuilder().build_world(
        world=world_holding(cloth), file_path=str(directory / "scene.xml")
    )
    return mujoco.MjModel.from_xml_path(str(directory / "scene.xml"))


@pytest.fixture
def triangle() -> Mesh:
    """
    :return: A single triangle of cloth, small enough to build in an instant.
    """
    return Mesh.from_trimesh(
        trimesh.Trimesh(
            vertices=[[0.0, 0.0, 1.0], [0.1, 0.0, 1.0], [0.0, 0.1, 1.0]],
            faces=[[0, 1, 2]],
        )
    )


def test_a_cloth_sheet_starts_where_its_surface_lies(triangle: Mesh) -> None:
    """
    Every vertex of the simulated cloth starts at its vertex of the surface.
    """
    cloth = ClothSheet(name="cloth", surface=triangle, mass=0.15)
    simulation = MujocoSim(world=world_holding(cloth), headless=True, step_size=2e-3)
    simulation.start_stepped_simulation()
    try:
        simulation.step_simulation(timedelta(milliseconds=2))
        positions = simulation.simulator.get_flex_vertex_positions(
            flex_name=cloth.name
        ).result
    finally:
        simulation.stop_simulation()

    assert numpy.allclose(positions, cloth.starting_surface.vertices, atol=1e-4)


@pytest.mark.parametrize("mass", [0.15, 0.0015, 1.5e-6])
def test_a_cloth_sheet_weighs_its_mass_but_every_vertex_at_least_the_minimum_mass(
    tmp_path: Path, triangle: Mesh, mass: float
) -> None:
    """
    The cloth's mass is spread evenly over its vertices, none lighter than the builder's
    minimum mass.
    """
    cloth = ClothSheet(name="cloth", surface=triangle, mass=mass)

    model = build_model_of(cloth, tmp_path)

    vertex_count = len(cloth.starting_surface.vertices)
    vertex_bodies = model.flex_vertbodyid[:vertex_count]
    vertex_mass = max(
        cloth.mass / vertex_count, MujocoBuilder.cloth_vertex_minimum_mass
    )
    assert model.body_mass[vertex_bodies].sum() == pytest.approx(
        vertex_count * vertex_mass
    )


def test_a_cloth_sheet_is_drawn_in_its_color(tmp_path: Path, triangle: Mesh) -> None:
    """
    The simulation draws the cloth in the color the cloth sheet is given.
    """
    cloth = ClothSheet(
        name="cloth",
        surface=triangle,
        mass=0.15,
        color=Color(hue=0.6, saturation=0.5, value=0.9, opacity=0.8),
    )

    model = build_model_of(cloth, tmp_path)

    flex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, cloth.name)
    assert model.flex_rgba[flex_id].tolist() == pytest.approx(
        list(cloth.color.to_rgba())
    )


def test_the_vertices_of_a_cloth_sheet_touch_with_the_builders_vertex_contact(
    tmp_path: Path, triangle: Mesh
) -> None:
    """
    The sphere of a vertex takes its margin and solver parameters from the builder.
    """
    cloth = ClothSheet(name="cloth", surface=triangle, mass=0.15)
    contact = MujocoBuilder().cloth_vertex_contact

    model = build_model_of(cloth, tmp_path)

    spheres = model.body_geomadr[
        model.flex_vertbodyid[: len(cloth.starting_surface.vertices)]
    ]
    assert model.geom_margin[spheres] == pytest.approx(
        contact.margin_to_radius_ratio * cloth.collision_radius
    )
    assert model.geom_solref[spheres[0]].tolist() == pytest.approx(
        contact.stiffness.to_list()
    )
    assert model.geom_solimp[spheres[0]].tolist() == pytest.approx(
        contact.impedance.to_list()
    )


def test_a_cloth_sheets_edges_are_damped_as_much_as_it_says(
    tmp_path: Path, triangle: Mesh
) -> None:
    """
    The simulation damps the cloth's edges by the cloth sheet's edge damping.
    """
    cloth = ClothSheet(
        name="cloth", surface=triangle, mass=0.15, fabric=Cotton(edge_damping=0.25)
    )

    model = build_model_of(cloth, tmp_path)

    flex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, cloth.name)
    assert model.flex_edgedamping[flex_id] == pytest.approx(cloth.fabric.edge_damping)


def test_only_a_world_holding_cloth_lowers_the_mass_every_body_weighs_at_least(
    tmp_path: Path, triangle: Mesh
) -> None:
    """
    A world without cloth keeps :attr:`MujocoBuilder.minimum_mass`, which robots rely
    on.
    """
    empty_world = World()
    with empty_world.modify_world():
        empty_world.add_body(Body(name=PrefixedName("root")))
    world_with_cloth = world_holding(
        ClothSheet(name="cloth", surface=triangle, mass=0.15)
    )

    bound_masses = []
    for index, world in enumerate([empty_world, world_with_cloth]):
        scene = tmp_path / f"scene_{index}.xml"
        MujocoBuilder().build_world(world=world, file_path=str(scene))
        bound_masses.append(
            float(ElementTree.parse(scene).find("compiler").get("boundmass"))
        )

    assert bound_masses == [
        MujocoBuilder.minimum_mass,
        MujocoBuilder.cloth_vertex_minimum_mass,
    ]


def test_a_t_shirt_weighs_exactly_the_mass_it_is_given(tmp_path: Path) -> None:
    """
    The minimum mass does not add to a T-shirt: each of its vertices weighs its share.
    """
    t_shirt = ClothSheet(
        name="t_shirt",
        surface=GarmentFile.T_SHIRT.create_surface(HomogeneousTransformationMatrix()),
        mass=0.255,
    )

    model = build_model_of(t_shirt, tmp_path)

    vertex_bodies = model.flex_vertbodyid[: len(t_shirt.starting_surface.vertices)]
    assert model.body_mass[vertex_bodies].sum() == pytest.approx(t_shirt.mass)


def test_every_vertex_of_a_cloth_sheet_collides_with_a_sphere_of_its_own(
    tmp_path: Path, triangle: Mesh
) -> None:
    """
    Every vertex carries one massless sphere of the cloth's collision radius, which
    collides with ordinary geoms and not with the spheres of other vertices.
    """
    cloth = ClothSheet(name="cloth", surface=triangle, mass=0.15)

    model = build_model_of(cloth, tmp_path)

    vertex_count = len(cloth.starting_surface.vertices)
    vertex_bodies = model.flex_vertbodyid[:vertex_count]
    assert model.body_geomnum[vertex_bodies].tolist() == [1] * vertex_count
    spheres = model.body_geomadr[vertex_bodies]
    assert (
        model.geom_type[spheres].tolist()
        == [mujoco.mjtGeom.mjGEOM_SPHERE] * vertex_count
    )
    assert model.geom_size[spheres, 0] == pytest.approx(cloth.collision_radius)
    assert (model.geom_contype[spheres] == ContactCategories.CLOTH_VERTEX).all()
    assert (model.geom_conaffinity[spheres] == ContactCategories.DEFAULT).all()
    assert not ContactCategories.CLOTH_VERTEX & ContactCategories.DEFAULT


def test_the_cloth_sheet_of_an_annotation_is_built(
    tmp_path: Path, triangle: Mesh
) -> None:
    """
    A cloth sheet an annotation carries is built exactly as one the world holds.
    """
    world = World()
    with world.modify_world():
        world.add_body(Body(name=PrefixedName("root")))
    t_shirt = TShirt.get_annotation_specification(
        "t_shirt", TShirt.get_default_root_kinematic_structure_entity_specification()
    ).spawn(world)
    with world.modify_world():
        t_shirt.add_cloth_sheet(surface=triangle, mass=0.15)
    scene = tmp_path / "scene.xml"

    MujocoBuilder().build_world(world=world, file_path=str(scene))

    model = mujoco.MjModel.from_xml_path(str(scene))
    assert model.nflex == 1
    assert (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, t_shirt.cloth_sheet.name)
        == 0
    )


def test_a_cloth_sheet_touches_what_it_collides_with_at_its_collision_radius(
    tmp_path: Path, triangle: Mesh
) -> None:
    """
    The simulation lets the cloth touch other objects at the cloth sheet's collision
    radius from its surface.
    """
    cloth = ClothSheet(name="cloth", surface=triangle, mass=0.15, collision_radius=2e-3)

    model = build_model_of(cloth, tmp_path)

    flex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, cloth.name)
    assert model.flex_radius[flex_id] == pytest.approx(cloth.collision_radius)


def test_a_cloth_sheet_does_not_collide_with_itself(
    tmp_path: Path, triangle: Mesh
) -> None:
    """
    The simulation lets the parts of a cloth pass through each other.
    """
    cloth = ClothSheet(name="cloth", surface=triangle, mass=0.15)

    model = build_model_of(cloth, tmp_path)

    flex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, cloth.name)
    assert model.flex_selfcollide[flex_id] == mujoco.mjtFlexSelf.mjFLEXSELF_NONE


def test_a_cloth_sheet_colliding_with_itself_keeps_its_parts_apart(
    tmp_path: Path, triangle: Mesh
) -> None:
    """
    The simulation keeps the parts of a cloth that collides with itself from passing
    through each other.
    """
    cloth = ClothSheet(
        name="cloth", surface=triangle, mass=0.15, collides_with_itself=True
    )

    model = build_model_of(cloth, tmp_path)

    flex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, cloth.name)
    assert model.flex_selfcollide[flex_id] == mujoco.mjtFlexSelf.mjFLEXSELF_AUTO


# %% simulating


def test_a_t_shirt_dropped_onto_a_table_lands_on_it_and_keeps_its_extent(
    t_shirt_above_table: TShirtAboveTable, simulation: MujocoSim
) -> None:
    """
    A T-shirt of two layers dropped onto a table lands on its top, sinking into it by
    less than its thickness, and keeps its spread-out extent.

    .. note:: The T-shirt keeps the relief it was laid out with, since that is the shape
        it bends back to, so only its lowest vertices touch the table.
    """
    t_shirt = t_shirt_above_table.t_shirt

    simulation.step_simulation(timedelta(seconds=1))
    positions = simulation.simulator.get_flex_vertex_positions(
        flex_name=t_shirt.name
    ).result

    height_above_table = positions[:, 2] - t_shirt_above_table.table_top_height
    assert (
        -t_shirt.fabric.thickness < height_above_table.min() <= t_shirt.collision_radius
    )
    assert numpy.allclose(
        numpy.ptp(positions[:, :2], axis=0),
        t_shirt.starting_surface.extents[:2],
        atol=2e-2,
    )


def test_a_t_shirt_dropped_onto_a_table_stays_out_of_its_surface(
    t_shirt_above_table: TShirtAboveTable, simulation: MujocoSim
) -> None:
    """
    No vertex sinks below the table top by more than half the fabric's thickness while
    the T-shirt lands, and once it has settled its shell reaches into the table by no
    more than a tenth of the collision radius.
    """
    t_shirt = t_shirt_above_table.t_shirt
    radius = t_shirt.collision_radius
    tolerated_depth = radius / 10
    tolerated_landing_depth = t_shirt.fabric.thickness / 2

    for _ in range(30):
        simulation.step_simulation(timedelta(milliseconds=100))
        positions = simulation.simulator.get_flex_vertex_positions(
            flex_name=t_shirt.name
        ).result
        height_above_table = positions[:, 2] - t_shirt_above_table.table_top_height
        assert height_above_table.min() > -tolerated_landing_depth

    assert height_above_table.min() - radius > -tolerated_depth
