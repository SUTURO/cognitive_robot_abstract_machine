"""
The GPSR demonstration: the HSR carries out a spoken command in a simulated kitchen,
following the plans a language model writes for it.
"""

from __future__ import annotations

# %% Imports
import json
import logging
import threading
import weakref
from dataclasses import dataclass, field
from datetime import timedelta
from enum import IntFlag, StrEnum
from pathlib import Path

import coraplex
import mujoco
import numpy
from coraplex.robot_plans.generated_commands import (
    DetectedObjectType,
    GeneratedCommand,
    PersonDetection,
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
    Floor,
    Human,
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
    """The counter top the milk stands on."""

    OVEN_TOWER = "oven_tower"
    """The oven tower on one side of the kitchen window."""

    SIDEBOARD = "sideboard"
    """The top of the kitchen island."""

    KITCHEN = "kitchen"
    """The kitchen, whose floor the person stands on."""

    MILK = "milk"
    """The milk carton the robot can pick up."""

    PERSON = "person"
    """The person the robot can find."""

    HUMAN = "human"
    """The body of the human model, as its URDF names it."""


class SceneContactCategory(IntFlag):
    """
    The MuJoCo contact categories the demonstration adds to the shared default one.
    """

    ROBOT_BODY = 2
    """
    Every part of the robot but its grippers, which collides with nothing outside it.
    """


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


# %% Simulation


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
        """
        Differs from
        :meth:`~coraplex.plans.executors.PhysicallySimulatedPlanExecutor.default_executor_extensions`
        in counting the robot as at rest up to :attr:`speed_at_rest`.
        """
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
    :attr:`json_plan`, in the kitchen of
    :class:`~semantic_digital_twin.predefined_maps.kitchen_environment.KitchenEnvironment`
    with rooms of their kind, a milk carton on its counter top and a person at its
    kitchen window.
    """

    used_robot: type[AbstractRobot] = HSRB
    """The HSR, whose head camera the perception looks through."""

    ros_node_name: str = "gpsr_generated_command_demo"
    """The name of the ROS node of this demonstration."""

    executor_type: type[PlanExecutor] = PhysicallySimulatedGraspingPlanExecutor
    """Runs the plan in MuJoCo, counting fingers that press on the milk as at rest."""

    collision_avoidance: bool = True
    """Whether the robot's motions keep clear of the furniture."""

    scene_joints_movable: bool = False
    """Whether the kitchen's doors and drawers move; fixed, they simulate faster."""

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

    perception: SimulatedHsrHeadCameraPerception | None = None
    """The perception the plan asks to detect people, or ``None`` to find people
    without perception."""

    query: RoboKudoQuery = field(
        default_factory=lambda: RoboKudoQuery(object_type=DetectedObjectType.HUMAN)
    )
    """What the plan asks :attr:`perception` once the robot faces the person."""

    robot_body_contact_category: ContactCategories = ContactCategories(
        SceneContactCategory.ROBOT_BODY
    )
    """The contact category of every part of the robot but its grippers, which
    collides with nothing outside that category."""

    json_plan: str = field(kw_only=True)
    """The plan JSON the plan model wrote for the command the robot carries out."""

    instruction: str = field(kw_only=True)
    """The instruction the human spoke, recorded via the NLP pipeline."""

    def scene_specification(self) -> WorldSpecification:
        """
        Never called: :meth:`build_scene` builds the scene from
        :attr:`kitchen_environment` instead.
        """
        raise NotImplementedError

    def build_scene(self) -> World:
        """Differs from :meth:`RobotDemonstration.build_scene` in starting from
        the kitchen instead of :meth:`scene_specification`."""
        scene = self.kitchen_environment.create_world()
        self.populate_scene(scene)
        if not self.scene_joints_movable:
            self.replace_joints_with_fixed_connections(scene)
        return scene

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(self.used_robot)

    def build_simulated_world(self) -> World:
        """
        Differs from :meth:`RobotDemonstration.build_simulated_world` in
        :meth:`allow_touching_floors_and_milk` and
        :meth:`let_robot_body_pass_through_scene`.
        """
        world = super().build_simulated_world()
        [robot] = world.get_semantic_annotations_by_type(self.used_robot)
        self.allow_touching_floors_and_milk(robot, world)
        self.let_robot_body_pass_through_scene(robot)
        return world

    @staticmethod
    def allow_touching_floors_and_milk(robot: AbstractRobot, world: World) -> None:
        """
        Let collision avoidance allow the robot to touch the floors it stands on and
        the milk, which only friction holds in the gripper.

        :param robot: The robot.
        :param world: The world holding the robot, the floors and the milk.
        """
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

    def let_robot_body_pass_through_scene(self, robot: AbstractRobot) -> None:
        """
        Put every collision shape of the robot but those of its grippers into
        :attr:`robot_body_contact_category`, so in MuJoCo only the grippers touch the
        scene.

        :param robot: The robot.
        """
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

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(SceneName.MILK)

    def populate_scene(self, world: World) -> None:
        """Puts the milk on the counter top and the person in front of the kitchen window."""
        self.place_milk(world)
        self.place_person(world)

    def start_simulation(self, world: World) -> None:
        """Differs from :meth:`RobotDemonstration.start_simulation` in then
        starting :attr:`perception`, if any."""
        super().start_simulation(world)
        if self.perception is None:
            return
        self.perception.start(world, self.simulation, self.ros_node)

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        [robot] = world.get_semantic_annotations_by_type(self.used_robot)
        extensions: list[ContextExtension] = [RobotAccess(robot)]
        if self.perception is not None:
            extensions.extend(self.perception.build_context_extensions())
            extensions.append(PersonDetection(self.query))
        return extensions

    def tear_down(self) -> None:
        """
        Differs from :meth:`RobotDemonstration.tear_down` in first
        :meth:`wait_until_viewer_closed` and stopping :attr:`perception`, if any.
        """
        self.wait_until_viewer_closed()
        if self.perception is not None:
            self.perception.stop()
        super().tear_down()

    def wait_until_viewer_closed(self) -> None:
        """
        Block until the MuJoCo viewer window is closed; return at once if there is
        none.
        """
        viewer_window = (
            None
            if self.headless or self.simulation is None
            else self.simulation.simulator.renderer._sim()
        )
        if viewer_window is None:
            return
        viewer_window_destroyed = threading.Event()
        weakref.finalize(viewer_window, viewer_window_destroyed.set)
        del viewer_window
        viewer_window_destroyed.wait()

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Perform the actions of every plan of the generated command, room visit by
        room visit, in the order
        :meth:`~coraplex.robot_plans.transformation_by_room.RoomVisitPlanner.transform_plan_by_rooms`
        orders them into.
        """
        generated_command = self.map_generated_command(context)
        fallback_room_types = {
            type(room) for room in context.world.get_semantic_annotations_by_type(Room)
        }
        commands = [
            plan.to_command(generated_command.instruction, fallback_room_types)
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

    def map_generated_command(self, context: StatechartContext) -> GeneratedCommand:
        """
        Map :attr:`json_plan` to the actions the robot performs, and log each of them.

        :param context: The context of the plan the command runs in.
        :return: The command with its plans of actions.
        """
        command = GeneratedCommand.from_json(
            json.loads(self.json_plan), self.instruction, context
        )
        logger = logging.getLogger(__name__)
        for plan in command.plans:
            for action in plan.actions:
                logger.info(
                    "%s | location: %s | object: %s | person: %s",
                    type(action.action).__name__,
                    None if action.location is None else type(action.location).__name__,
                    None
                    if action.object_annotation is None
                    else type(action.object_annotation).__name__,
                    None if action.person is None else action.person.name,
                )
                logger.info("%s", action.action)
        return command

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
        milk.root.parent_connection.origin = self.centred_standing_pose(
            milk.root, counter_top, world
        )
        self.slide_to_front_edge(milk.root, counter_top, world)
        return milk

    def place_person(self, world: World) -> Human:
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
        floor_height = float(
            self.centred_standing_pose(human_world.root, kitchen.floor.root, world).z
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
        person = Human(root=human_body, name=PrefixedName(SceneName.PERSON))
        with world.modify_world():
            world.add_semantic_annotation(person)
        return person

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
        front_edge_y = min(surface_box.min_y, surface_box.max_y, key=abs)
        if front_edge_y == surface_box.min_y:
            world_root_T_body.y = surface_box.min_y - body_box.min_y
        else:
            world_root_T_body.y = surface_box.max_y - body_box.max_y
        body.parent_connection.origin = world_root_T_body


def main(
    headless: bool = False,
    perceives: bool = True,
    instruction: str | None = None,
    plan_model_path: Path = "/home/hanna/SUTURO_Master/nlp_llm/plan-q4.gguf",
    listening_timeout: float = 30.0,
) -> World:
    """Run the demonstration.

    :param headless: Whether the physics runs without a viewer.
    :param perceives: Whether RoboKudo is started and asked to detect the person.
    :param instruction: The command the robot carries out, or ``None`` to hear it
        through the microphone.
    :param plan_model_path: The GGUF file of the model that writes the plans for the
        command.
    :param listening_timeout: How long to wait for the command to be spoken, in
        seconds.
    :return: The world the command was carried out in.
    """
    logger = logging.getLogger(__name__)
    plan_model = PlanModel(model_path=plan_model_path)
    if instruction is None:
        recognizer = SpeechRecognizer(
            microphone=Microphone(), transcriber=WhisperTranscriber()
        )
        logger.info("Speak now")
        instruction = recognizer.hear_next_utterance(timeout=listening_timeout).text
    instruction_plans = plan_model.write_plans(instruction)

    json_plan = json.dumps(instruction_plans.to_json())
    logger.info("Generated plan for %r: %s", instruction, json_plan)

    return GeneratedCommandDemonstration(
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
    logging.getLogger(__name__).setLevel(logging.INFO)
    main()
