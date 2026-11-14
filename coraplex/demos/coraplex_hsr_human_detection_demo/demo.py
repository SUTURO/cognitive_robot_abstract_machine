"""
The HSR parks its arm and turns towards a human standing to its left, then asks RoboKudo to
detect the human through its simulated head camera. The viewer stays open until closed.

Run it from this folder::

    python demo.py
"""

# %% import
from __future__ import annotations

import threading
import weakref
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

import numpy

from coraplex.demonstrations import RobotDemonstration
from coraplex.perception import CameraFrameCapture, RoboKudoQuery, RoboKudoQueryTask
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import PhysicallySimulatedPlanExecutor, PlanExecutor
from coraplex.robot_plans.actions.core.navigation import FaceAtAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.node import StatechartNode
from cramph.statechart import Statechart
from krrood.patterns.path_enumeration import PathEnumeration
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Pose
from robokudo.analysis_engine_process import AnalysisEngineProcess
from robokudo.descriptors.camera_configs.config_hsr_ros2 import HsrRos2CameraConfig
from robokudo.io.simulated_camera import SimulatedCameraPublisher
from semantic_digital_twin.adapters.multi_sim import MujocoCamera, MujocoSim
from semantic_digital_twin.adapters.package_resolver import SearchPathFileResolver
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.hsrb import HSRB, HSRBHeadRGBDCamera
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import Human
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
)
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Scale

# %% the scene


class SceneFile(PathEnumeration):
    """
    The files the scene is built from, under the coraplex resources.
    """

    HUMAN = (
        Path(__file__).resolve().parents[2]
        / "resources"
        / "objects"
        / "human"
        / "human.urdf"
    )
    """
    The rigid human, whose meshes lie beside it.
    """


class SceneBody(StrEnum):
    """
    The names of the scene's bodies.
    """

    FLOOR = "floor"
    """The floor supporting the robot and human."""

    HUMAN = "human"
    """The human model in the scene."""


# %% the demonstration


@dataclass
class HumanDetectionDemonstration(RobotDemonstration):
    """
    The HSR parks its arm and turns towards a person to its left before asking its analysis
    engine a query.
    """

    used_robot: type[AbstractRobot] = HSRB
    """
    The HSR used for the demonstration.
    """

    ros_node_name: str = "hsr_human_detection_demo"
    """
    The ROS node publishing the simulated camera.
    """

    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor
    """
    The executor that performs the plan in simulation.
    """

    floor: BodySpecification = field(
        default_factory=lambda: BodySpecification.box(
            SceneBody.FLOOR,
            Scale(8.0, 8.0, 0.02),
            parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(z=-0.01),
        )
    )
    """
    The floor the robot and the human stand on.
    """

    human_distance: float = 3.5
    """
    How far to the robot's left the human stands, in meters.
    """

    analysis_engine: AnalysisEngineProcess | None = None
    """
    The RoboKudo pipeline, or None to run the motion without perception.
    """

    query: RoboKudoQuery = field(
        default_factory=lambda: RoboKudoQuery(object_type="human")
    )
    """
    What the plan asks :attr:`analysis_engine` after facing the human.
    """

    camera_configuration: HsrRos2CameraConfig = field(
        default_factory=HsrRos2CameraConfig
    )
    """
    The topics and frames the head camera is published on, those of the real HSR.
    """

    camera_publisher: SimulatedCameraPublisher | None = field(init=False, default=None)
    """
    Publishes the head camera while :attr:`analysis_engine` runs.
    """

    def scene_specification(self) -> WorldSpecification:
        return WorldSpecification(objects=[self.floor])

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(self.used_robot)

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(SceneBody.HUMAN)

    def populate_scene(self, world: World) -> None:
        """
        Stand the human :attr:`human_distance` to the left of where the robot starts,
        turned to face it.

        :param world: The world to add the human to.
        """
        human = URDFParser.from_file(
            str(SceneFile.HUMAN),
            path_resolver=SearchPathFileResolver(
                root_directories=[str(SceneFile.HUMAN.parent)]
            ),
        ).create_world()
        world.merge_world(
            human,
            root_connection=FixedConnection(
                parent=world.root,
                child=human.root,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    y=self.human_distance, yaw=-numpy.pi / 2
                ),
            ),
        )
        with world.modify_world():
            world.add_semantic_annotation(
                Human(root=world.get_body_by_name(SceneBody.HUMAN))
            )

    def build_simulation(self, world: World) -> MujocoSim:
        """
        Differs from
        :meth:`~coraplex.demonstrations.RobotDemonstration.build_simulation` in mounting
        a MuJoCo camera at the head's RGB-D camera first.

        :param world: The world containing the robot and its head camera.
        :return: The simulation of the world with its mounted camera.
        """
        MujocoCamera.mount_hsr_head(world)
        return super().build_simulation(world)

    def start_simulation(self, world: World) -> None:
        """
        Differs from
        :meth:`~coraplex.demonstrations.RobotDemonstration.start_simulation` in then
        publishing the head camera and starting :attr:`analysis_engine`, if any.

        :param world: The world containing the robot and its head camera.
        """
        super().start_simulation(world)
        if self.analysis_engine is None:
            return
        [camera] = world.get_semantic_annotations_by_type(HSRBHeadRGBDCamera)
        self.camera_publisher = SimulatedCameraPublisher(
            camera=camera,
            simulation=self.simulation,
            node=self.ros_node,
            topics=self.camera_configuration,
            frames=self.camera_configuration,
        )
        self.analysis_engine.start()

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        extensions: list[ContextExtension] = [
            RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])
        ]
        if self.camera_publisher is not None:
            extensions.append(CameraFrameCapture(self.camera_publisher.publish))
        return extensions

    def build_statechart(self, context: StatechartContext) -> Statechart:
        """
        Park the arm and face the human before asking the analysis engine.

        :param context: The plan's world and available resources.
        :return: The statechart for the demonstration.
        """
        robot = context.require_extension(RobotAccess).robot
        human = context.world.get_body_by_name(SceneBody.HUMAN)
        steps: list[StatechartNode] = [
            ParkArmsAction(arms=robot.all_arms),
            FaceAtAction(target=Pose.from_xyz_rpy(reference_frame=human)),
        ]
        if self.analysis_engine is not None:
            steps.append(RoboKudoQueryTask(query=self.query))
        statechart = Statechart(context=context)
        statechart.add_node(Sequence(nodes=steps))
        return statechart

    def tear_down(self) -> None:
        """
        Extend :meth:`~coraplex.demonstrations.RobotDemonstration.tear_down` by waiting
        for the viewer to close and stopping the analysis engine.
        """
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
        if self.camera_publisher is not None:
            self.analysis_engine.stop()
        super().tear_down()


def main(headless: bool = False, perceives: bool = True) -> World:
    """
    Run the demonstration.

    :param headless: Whether the physics runs without a viewer.
    :param perceives: Whether RoboKudo is started and asked for humans.
    :return: The world the human was placed in.
    """
    return HumanDetectionDemonstration(
        headless=headless,
        analysis_engine=(
            AnalysisEngineProcess(analysis_engine="human_detection")
            if perceives
            else None
        ),
    ).run()


if __name__ == "__main__":
    main()
