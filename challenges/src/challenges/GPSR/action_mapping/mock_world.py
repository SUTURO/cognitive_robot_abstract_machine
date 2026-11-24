"""The kitchen the generated command of :func:`main` is mapped to actions
in."""

from __future__ import annotations

# %% Imports
import json
import logging
import threading
import weakref
from dataclasses import dataclass, field
from datetime import timedelta
from enum import StrEnum
from pathlib import Path

import coraplex
import mujoco
import numpy
from challenges.GPSR.action_mapping.action_mapping import (
    DetectedObjectType,
    PersonDetection,
    main as action_main,
)
from cramspeech.inference.plan_model import PlanModel
from cramspeech.microphone import Microphone
from cramspeech.speech_recognizer import SpeechRecognizer
from cramspeech.transcription import WhisperTranscriber
from coraplex.demonstrations import RobotDemonstration
from coraplex.perception import RoboKudoQuery
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import PhysicallySimulatedPlanExecutor, PlanExecutor
from coraplex.robot_plans.transformation_by_room import RoomVisitPlanner
from coraplex.simulated_perception import SimulatedHsrHeadCameraPerception
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.executor import ExecutorExtension
from giskardpy.motion_control import MotionControl
from cramph.statechart import Statechart
from krrood.patterns.path_enumeration import PathEnumeration
from robokudo.analysis_engine_process import AnalysisEngineProcess
from semantic_digital_twin.adapters.multi_sim import (
    ContactCategories,
    MujocoCamera,
    MujocoGeom,
    MujocoSim,
    MujocoSynchronizer,
)
from semantic_digital_twin.adapters.package_resolver import SearchPathFileResolver
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.predefined_maps.kitchen_environment import KitchenEnvironment
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Bedroom,
    Floor,
    Human,
    Kitchen,
    LivingRoom,
    Milk,
    Room,
)
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.specifications.connections import (
    Connection6DoFSpecification,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.semantic_annotations import (
    SemanticAnnotationWithRootSpecification,
)
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.contact import ContactParameters
from semantic_digital_twin.world_description.world_entity import Body

# %% Scene vocabulary


class SceneName(StrEnum):
    """The names of the scene's parts, as the kitchen and the generated command
    call them."""

    COUNTER_TOP = "counter_top"
    OVEN_TOWER = "oven_tower"
    SIDEBOARD = "sideboard"
    """The top of the kitchen island."""
    KITCHEN = "kitchen"
    LIVING_ROOM = "living_room"
    BED_ROOM = "bed_room"
    MILK = "milk"
    PERSON = "person"
    HUMAN = "human"
    """The body of the human model, as its URDF names it."""


class SceneFile(PathEnumeration):
    """The files the added objects are built from, under the coraplex
    resources."""

    MILK = (
        Path(coraplex.__file__).resolve().parents[2]
        / "resources"
        / "objects"
        / "milk.stl"
    )
    """The mesh of the milk carton."""

    HUMAN = (
        Path(coraplex.__file__).resolve().parents[2]
        / "resources"
        / "objects"
        / "human"
        / "human.urdf"
    )
    """The rigid human model, whose meshes lie beside it."""


# %% Classes of the named objects


@dataclass(eq=False)
class Person(Human):
    """A person, named so that the generated command's ``"person"`` finds this
    class."""


# %% Simulation


# MuJoCo Performance improvement
class OncePerCallSyncedMujocoSim(MujocoSim):
    """
    A MuJoCo simulation that copies its state into the world once per call of
    :meth:`step_simulation`.
    """

    def step_simulation(self, duration: timedelta) -> None:
        """Differs from :meth:`MujocoSim.step_simulation` in copying the
        physics back into the world only after the last physics step of the
        call."""
        physics_step = timedelta(seconds=self.simulator.step_size)
        number_of_steps = round(duration / physics_step)
        if number_of_steps == 0:
            super().step_simulation(duration)
            return
        previous_sync_rate = self.synchronizer.sync_rate_hz
        self.synchronizer.sync_rate_hz = 0
        super().step_simulation(physics_step * (number_of_steps - 1))
        self.synchronizer.sync_rate_hz = MujocoSynchronizer.UNTHROTTLED_SYNC_RATE_HZ
        super().step_simulation(physics_step)
        self.synchronizer.sync_rate_hz = previous_sync_rate


# MuJoco Performance improvement
@dataclass
class PhysicallySimulatedGraspingPlanExecutor(PhysicallySimulatedPlanExecutor):
    """
    A physically simulated executor that still counts the robot as at rest while its
    fingers press on an object they hold by friction.
    """

    speed_at_rest: float = field(default=0.05, kw_only=True)
    """
    The fastest a degree of freedom may move for the robot to count as at rest; fingers
    pressing on a held carton keep moving at about 0.04.
    """

    def default_executor_extensions(self) -> list[ExecutorExtension]:
        """Differs from :meth:`PhysicallySimulatedPlanExecutor.default_executor_ext
        ensions` in counting the robot as at rest up to :attr:`speed_at_rest`."""
        extensions = super().default_executor_extensions()
        for extension in extensions:
            if isinstance(extension, MotionControl):
                extension.speed_at_rest = self.speed_at_rest
        return extensions


# %% Demonstration


@dataclass
class GeneratedCommandDemonstration(RobotDemonstration):
    """
    The robot carries out the actions mapped from the generated command of
    :func:`~challenges.GPSR.action_mapping.action_mapping.main`, in the kitchen of
    :class:`~semantic_digital_twin.predefined_maps.kitchen_environment.KitchenEnvironment`
    with rooms of their kind, a milk carton on its counter top and a person at its
    kitchen window.
    """

    used_robot: type[AbstractRobot] = HSRB

    ros_node_name: str = "gpsr_generated_command_demo"

    executor_type: type[PlanExecutor] = PhysicallySimulatedGraspingPlanExecutor

    collision_avoidance: bool = True

    # better MuJoCo performance
    scene_joints_movable: bool = False

    """The kind of each of the kitchen's rooms, by the room's name."""

    kitchen_environment: KitchenEnvironment = field(default_factory=KitchenEnvironment)
    """The kitchen the scene is built from, whose measurements place the
    person."""

    person_distance_to_window: float = 0.25
    """The gap between the person's back and the front edge of the window sill,
    in m."""

    friction_cone: mujoco.mjtCone = mujoco.mjtCone.mjCONE_ELLIPTIC
    """The friction cone the contacts use; the elliptic cone keeps a held
    carton from creeping between the fingers."""

    impedance_ratio: float = 10.0
    """How much harder sliding is than pressing in, as MuJoCo recommends for
    grasping with an elliptic cone."""

    no_slip_iterations: int = 10
    """Passes spent removing the slip soft contacts leave, so a held carton
    stays where the fingers hold it."""

    # Perception added
    perception: SimulatedHsrHeadCameraPerception | None = None
    """The perception the plan asks to detect people, or ``None`` to find people
    without perception."""

    # Perception added
    query: RoboKudoQuery = field(
        default_factory=lambda: RoboKudoQuery(object_type=DetectedObjectType.HUMAN)
    )
    """What the plan asks :attr:`perception` once the robot faces the person."""

    # makes it possible to ignore collision
    robot_body_contact_category: ContactCategories = ContactCategories(2)
    """The contact category of every part of the robot but its grippers, which
    collides with nothing outside that category."""

    # NLP added
    json_plan: str = field(kw_only=True)
    """The plan JSON the plan model wrote for the command the robot carries out."""

    instruction: str = field(kw_only=True)
    """The instruction the human spoke, recorded via the NLP pipeline."""

    # Not needed because of Kitchen Environment
    def scene_specification(self) -> WorldSpecification:
        """Never called: :meth:`build_scene` builds the kitchen directly, since
        :class:`~semantic_digital_twin.predefined_maps.kitchen_environment.Kitc
        henEnvironment` builds a :class:`~semantic_digital_twin.world.World`,
        not a :class:`WorldSpecification`."""
        raise NotImplementedError

    # build scene via kitchen environment instead of scene_specification
    def build_scene(self) -> World:
        """Differs from :meth:`RobotDemonstration.build_scene` in starting from
        the kitchen instead of :meth:`scene_specification`."""
        scene = self.kitchen_environment.create_world()
        self.populate_scene(scene)
        if not self.scene_joints_movable:
            self.replace_joints_with_fixed_connections(scene)
        return scene

    # specify which robot to spawn
    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(self.used_robot)

    # ignore furniture collision
    def build_simulated_world(self) -> World:
        """Differs from :meth:`RobotDemonstration.build_simulated_world` in
        letting every part of the robot but its grippers pass through the
        scene in the physics, and in letting the robot touch the floors, which
        collision avoidance would otherwise keep it from standing on, and the
        milk, which only friction holds in the gripper, so collision avoidance
        never counts it as held."""
        world = super().build_simulated_world()
        robot = world.get_semantic_annotations_by_type(self.used_robot)[0]
        floor_bodies = [
            body
            for floor in world.get_semantic_annotations_by_type(Floor)
            for body in floor.bodies
        ]
        with world.modify_world():
            for touched_bodies in (
                floor_bodies,
                [world.get_body_by_name(SceneName.MILK)],
            ):
                world.collision_manager.add_ignore_collision_rule(
                    AllowCollisionBetweenGroups(
                        body_group_a=robot.bodies_with_collision,
                        body_group_b=touched_bodies,
                    )
                )

        # TODO: What happens here
        gripper_bodies = {
            body
            for arm in robot.all_arms
            for body in arm.end_effector.bodies_with_collision
        }
        for body in robot.bodies_with_collision:
            if body in gripper_bodies:
                continue
            for geometry in body.collision:
                geometry.add_simulator_property(
                    MujocoGeom(
                        contact_type=self.robot_body_contact_category,
                        contact_affinity=self.robot_body_contact_category,
                    )
                )
        return world

    # Perception added: add mujoco camera to toyas head, builds the once-per-call synced simulation and sets
    # the grasp friction parameters (elliptic cone, impedance ratio, no-slip iterations)
    def build_simulation(self, world: World) -> MujocoSim:
        """Differs from :meth:`RobotDemonstration.build_simulation` in contacts
        that let the gripper hold a carton, and in copying the physics into the
        world once per tick, see :class:`OncePerCallSyncedMujocoSim`, with a
        camera mounted at the head's RGB-D camera."""
        MujocoCamera.mount_hsr_head(world)
        return OncePerCallSyncedMujocoSim(
            world=world,
            headless=self.headless,
            cone=self.friction_cone,
            impratio=self.impedance_ratio,
            noslip_iterations=self.no_slip_iterations,
        )

    # returns milk as kinematic structure
    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(SceneName.MILK)

    # adds objects into the world and assigns room types
    def populate_scene(self, world: World) -> None:
        """Puts the milk on the counter top and the person in front of the kitchen window."""
        self.place_milk(world)
        self.place_person(world)

    # starts the simulation and publishes head camera if Robokudo is used
    def start_simulation(self, world: World) -> None:
        """Differs from :meth:`RobotDemonstration.start_simulation` in then
        starting :attr:`perception`, if any."""
        super().start_simulation(world)
        if self.perception is None:
            return
        self.perception.start(world, self.simulation, self.ros_node)

    # adds frame capture and person detection if Robokudo is used
    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        extensions: list[ContextExtension] = [
            RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])
        ]
        if self.perception is not None:
            extensions.extend(self.perception.build_context_extensions(world))
            extensions.append(PersonDetection(self.query))
        return extensions

    # does not only close the sim but also stops the components for the perception part of the demo
    def tear_down(self) -> None:
        """Differs from :meth:`RobotDemonstration.tear_down` in first waiting for
        the viewer to be closed, if there is one, and stopping
        :attr:`perception`, if any."""
        viewer_window = (
            None
            if self.headless or self.simulation is None
            else self.simulation.simulator.renderer._sim()
        )
        if viewer_window is not None:
            viewer_window_destroyed = threading.Event()
            weakref.finalize(viewer_window, viewer_window_destroyed.set)
            del viewer_window
            viewer_window_destroyed.wait()
        if self.perception is not None:
            self.perception.stop()
        super().tear_down()

    # builds the plan and transforms it
    def build_statechart(self, context: StatechartContext) -> Statechart:
        """Perform the actions of every plan of the generated command, room
        visit by room visit, in the order :meth:`~coraplex.robot_plans.transfor
        mation_by_room.RoomVisitPlanner.transform_plan_by_rooms` orders them
        into."""
        generated_command = action_main(context, self.instruction, self.json_plan)
        any_room = {
            type(room) for room in context.world.get_semantic_annotations_by_type(Room)
        }
        commands = [
            plan.rewrap_into_command(generated_command.instruction, any_room)
            for plan in generated_command.plans
        ]
        room_visits = RoomVisitPlanner().transform_plan_by_rooms(commands)
        statechart = Statechart(context=context)
        statechart.add_node(
            Sequence(
                nodes=[
                    action
                    for room_visit in room_visits
                    for action in room_visit.actions
                ]
            )
        )
        return statechart

    # spawns the milk at the edge of the counter top so the robot can easier reach it
    def place_milk(self, world: World) -> Milk:
        """
        :param world: The kitchen.
        :return: A new milk carton standing on the counter top as close to its front edge as it
            can without overhanging, free to be picked up.
        """
        milk = SemanticAnnotationWithRootSpecification(
            name=SceneName.MILK,
            semantic_annotation_type=Milk,
            root_specification=BodySpecification.mesh(
                SceneName.MILK,
                str(SceneFile.MILK),
                connection_specification=Connection6DoFSpecification(),
            ),
        ).spawn(world)
        ContactParameters.create_for_grasped_object().apply_to([milk.root])
        counter_top = world.get_body_by_name(SceneName.COUNTER_TOP)
        self.stand_on(milk.root, counter_top, world)
        self.slide_to_front_edge(milk.root, counter_top, world)
        return milk

    # places the person in front if the kitchen window without colliding with the world
    def place_person(self, world: World) -> Person:
        """
        :param world: The kitchen.
        :return: A new human standing on the kitchen floor with their back to the
            window wall, :attr:`person_distance_to_window` in front of its sill and
            midway between the oven tower and the sideboard, facing into the kitchen.
            Nothing in the simulation collides with the human.
        """
        human_world = URDFParser.from_file(
            str(SceneFile.HUMAN),
            path_resolver=SearchPathFileResolver(
                root_directories=[str(SceneFile.HUMAN.parent)]
            ),
        ).create_world()
        kitchen = world.get_semantic_annotation_by_name(SceneName.KITCHEN)
        *_, floor_height = (
            float(coordinate)
            for coordinate in self.centred_standing_pose(
                human_world.root, kitchen.floor.root, world
            ).position.to_np()[:3]
        )
        window_wall = self.kitchen_environment.measurements.window_wall
        oven_tower_box = (
            world.get_body_by_name(SceneName.OVEN_TOWER)
            .collision.as_bounding_box_collection_in_frame(world.root)
            .bounding_box()
        )
        sideboard_box = (
            world.get_body_by_name(SceneName.SIDEBOARD)
            .collision.as_bounding_box_collection_in_frame(world.root)
            .bounding_box()
        )
        human_box = human_world.root.collision.as_bounding_box_collection_in_frame(
            human_world.root
        ).bounding_box()
        # The window wall closes the kitchen towards +x, so a yaw of pi turns the
        # human's back (their -x side) to it.
        world.merge_world(
            human_world,
            root_connection=FixedConnection(
                parent=world.root,
                child=human_world.root,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=window_wall.sill_front_x
                    - self.person_distance_to_window
                    + human_box.min_x,
                    y=(oven_tower_box.max_y + sideboard_box.min_y) / 2,
                    z=floor_height,
                    yaw=numpy.pi,
                    reference_frame=world.root,
                ),
            ),
        )
        human_body = world.get_body_by_name(SceneName.HUMAN)
        for geometry in human_body.collision:
            geometry.add_simulator_property(
                MujocoGeom(
                    contact_type=ContactCategories.NONE,
                    contact_affinity=ContactCategories.NONE,
                )
            )
        person = Person(root=human_body, name=PrefixedName(SceneName.PERSON))
        with world.modify_world():
            world.add_semantic_annotation(person)
        return person

    # puts an object onto a surface by attaching it
    def stand_on(self, body: Body, surface: Body, world: World) -> None:
        """Move ``body`` to :meth:`centred_standing_pose` on ``surface``.

        :param body: The body to move, attached to the world by a 6DoF
            connection.
        :param surface: The body to stand it on.
        :param world: The world both are in.
        """
        body.parent_connection.origin = self.centred_standing_pose(body, surface, world)

    # calculates a centered pose for an object to stand on a surface
    @staticmethod
    def centred_standing_pose(
        body: Body, surface: Body, world: World
    ) -> HomogeneousTransformationMatrix:
        """
        :param body: The body to stand, in this world or still in a world of its own.
        :param surface: The body to stand it on, in ``world``.
        :param world: The world ``surface`` is in.
        :return: Where ``body`` stands upright in the middle of ``surface``, in the frame
            of the root of ``world``.
        """
        surface_box = surface.collision.as_bounding_box_collection_in_frame(
            world.root
        ).bounding_box()
        body_box = body.collision.as_bounding_box_collection_in_frame(
            body
        ).bounding_box()
        return HomogeneousTransformationMatrix.from_xyz_rpy(
            x=(surface_box.min_x + surface_box.max_x) / 2,
            y=(surface_box.min_y + surface_box.max_y) / 2,
            z=surface_box.max_z - body_box.min_z,
            reference_frame=world.root,
        )

    # puts an object to the edge of a surface for better reachability
    @staticmethod
    def slide_to_front_edge(body: Body, surface: Body, world: World) -> None:
        """Slide ``body``, standing on ``surface``, along the y-axis until its
        side is flush with the edge of ``surface`` facing the origin of the
        world, where the robot starts.

        :param body: The body to slide, attached to the world by a 6DoF
            connection.
        :param surface: The body it stands on.
        :param world: The world both are in.
        """
        surface_box = surface.collision.as_bounding_box_collection_in_frame(
            world.root
        ).bounding_box()
        body_box = body.collision.as_bounding_box_collection_in_frame(
            body
        ).bounding_box()
        world_root_T_body = body.parent_connection.origin
        if surface_box.min_y > 0:
            world_root_T_body.y = surface_box.min_y - body_box.min_y
        else:
            world_root_T_body.y = surface_box.max_y - body_box.max_y
        body.parent_connection.origin = world_root_T_body


# runs everything
def main(
    headless: bool = False,
    perceives: bool = True,
    instruction: str | None = None,
    plan_model_path: Path = "/home/hanna/SUTURO_Master/nlp_llm/plan-q4.gguf",
) -> World:
    """Run the demonstration.

    :param headless: Whether the physics runs without a viewer.
    :param perceives: Whether RoboKudo is started and asked to detect the person.
    :param instruction: The command the robot carries out, or ``None`` to hear it
        through the microphone.
    :param plan_model_path: The GGUF file of the model that writes the plans for the
        command.
    :return: The world the command was carried out in.
    """
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)
    plan_model = PlanModel(model_path=plan_model_path)
    if instruction is None:
        recognizer = SpeechRecognizer(
            microphone=Microphone(), transcriber=WhisperTranscriber()
        )
        logger.info("Speak now")
        instruction = recognizer.hear_next_utterance(timeout=30.0).text
    instruction_plans = plan_model.write_plans(instruction)

    json_plan = json.dumps(instruction_plans.to_json(), separators=(",", ":"))
    logger.info("Generated plan for %r: %s", instruction, json_plan)

    return GeneratedCommandDemonstration(
        scene_joints_movable=False,
        headless=headless,
        json_plan=json_plan,
        instruction=instruction,
        perception=(
            SimulatedHsrHeadCameraPerception(
                AnalysisEngineProcess(
                    analysis_engine="human_detection", shows_views=False
                )
            )
            if perceives
            else None
        ),
    ).run()


if __name__ == "__main__":
    main()
