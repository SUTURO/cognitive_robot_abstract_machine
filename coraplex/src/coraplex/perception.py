from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import timedelta
from enum import StrEnum

import numpy as np

from krrood.ormatic.utils import classproperty
from cramph.node import EndedByOwner, FailsOnObservingFalse
from cramph.context import ContextExtension
from cramph.data_types import ObservationStateValues
from cramph.node import StatechartNode
from giskardpy.motion_statechart.ros_context import RosContextExtension
from krrood.adapters.json_serializer import SubclassJSONSerializer, from_json, to_json
from rclpy.action import ActionClient
from rclpy.node import Node
from semantic_digital_twin.adapters.world_entity_kwargs_tracker import (
    WorldEntityWithIDKwargsTracker,
)
from semantic_digital_twin.reasoning.robot_predicates import VisibleTo
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.mixins import (
    HasSeatPlaces,
    IsPerceivable,
)
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    RotationMatrix,
)
from krrood_spatial_math.spatial_types import Pose, Point3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import (
    Scale,
    VolumetricBoundingBox,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Chair,
    Human,
)
from semantic_digital_twin.specifications.kinematic_structure_entities import (
    BodySpecification,
    RegionSpecification,
)
from semantic_digital_twin.specifications.semantic_annotations import (
    SemanticAnnotationWithRootSpecification,
)
from semantic_digital_twin.world_description.world_entity import (
    SemanticAnnotation,
    Body,
    Region,
)
from typing_extensions import Any, Self, TYPE_CHECKING

from coraplex.datastructures.enums import PerceptionSource
from coraplex.exceptions import (
    AmbiguousDetection,
    PerceivedObjectNotInWorld,
    PerceptionSourceUnavailable,
    UnidentifiedDetections,
)

if TYPE_CHECKING:
    from geometry_msgs.msg import PoseStamped
    from robokudo_msgs.msg import ObjectDesignator

ROBOKUDO_QUERY_ACTION_NAME = "robokudo/query"
"""
Name of the action RoboKudo answers perception queries on.
"""

# %% queries


@dataclass
class PerceptionQuery(SubclassJSONSerializer):
    """
    What to look for and where.

    Travels to whichever process answers it, so the world and the robot are carried by
    reference and resolved against the world on the receiving side.
    """

    semantic_annotation: type[SemanticAnnotation]
    """
    The semantic annotation for which to perceive.
    """

    region: VolumetricBoundingBox
    """
    The region in which the object should be detected.
    """

    robot: AbstractRobot
    """
    ' Robot annotation of the robot that should perceive the object.
    """

    world: World
    """
    The world in which the object should be detected.
    """

    trust_detected_orientation: bool = True
    """
    Whether to also apply the perception source's detected orientation.

    When False, only the detected position is applied to the world; the object's
    existing orientation is kept. See :meth:`Detection.apply_to`.
    """

    def from_world(self) -> list[Body]:
        """
        Answer this query from the world model alone.

        :return: The bodies of the queried annotation that lie inside the region and are
            visible to the robot's camera. A body several matching annotations describe
            is one object, so it is reported once.
        """
        bodies = dict.fromkeys(
            body
            for semantic_annotation in self.world.get_semantic_annotations_by_type(
                self.semantic_annotation
            )
            for body in semantic_annotation.bodies
        )
        region_bodies = [
            body
            for body in bodies
            if self.region.contains(body.global_transform.position)
        ]

        robot_camera = self.robot.get_default_camera()
        return [
            body
            for body in region_bodies
            if VisibleTo(entity=body, camera=robot_camera)()
        ]

    def to_json(self, **kwargs) -> dict[str, Any]:
        result = super().to_json(**kwargs)
        result["semantic_annotation"] = to_json(self.semantic_annotation, **kwargs)
        result["region"] = to_json(self.region, **kwargs)
        result["robot_id"] = to_json(self.robot.id, **kwargs)
        result["trust_detected_orientation"] = self.trust_detected_orientation
        return result

    @classmethod
    def _from_json(cls, data: dict[str, Any], **kwargs) -> Self:
        tracker = WorldEntityWithIDKwargsTracker.from_kwargs(kwargs)
        return cls(
            semantic_annotation=from_json(data["semantic_annotation"], **kwargs),
            region=from_json(data["region"], **kwargs),
            robot=tracker.get(from_json(data["robot_id"])),
            world=kwargs["world"],
            trust_detected_orientation=data.get("trust_detected_orientation", True),
        )


# %% detections


@dataclass
class Detection:
    """
    One object reported by a perception source.
    """

    semantic_annotation: type[SemanticAnnotation]
    """
    The annotation the perception source identified the object as.
    """

    pose: Pose
    """
    Where the perception source saw the object.
    """

    def apply_to(
        self, world: World, trust_orientation: bool = True
    ) -> list[IsPerceivable]:
        """
        Write this detection into ``world``, moving the perceived body to where it was
        seen.

        :param world: The world holding the annotations this detection refers to.
        :param trust_orientation: Whether to also apply the detected orientation. When
            False, only the detected position is applied and the body keeps its existing
            orientation; useful while a perception source's orientation estimate is not
            yet reliable enough for grasp planning.
        :return: The annotations that were updated.
        :raises PerceivedObjectNotInWorld: If the world holds no such annotation.
        :raises AmbiguousDetection: If the annotation describes more than one body.
        """
        matching_annotations = self.resolve_annotations(world)
        matching_annotation = matching_annotations[0]
        body = matching_annotation.root
        robot = world.get_semantic_annotations_by_type(AbstractRobot)[0]

        # Foundation Pose may return a upside down pose. this detects and fixes it by rotation around x by 180 degree
        detected_global_pose = world.transform(self.pose, world.root).homogeneous_matrix
        detected_global_pose = self._with_z_up_and_x_towards(
            root_T_frame=detected_global_pose,
            root_P_target=robot.root.global_transform.position,
        )

        if trust_orientation:
            body.parent_connection.origin = detected_global_pose
        else:
            parent_T_object = world.transform(
                detected_global_pose, body.parent_connection.parent
            )
            parent_origin = body.parent_connection.origin
            body.parent_connection.origin = (
                HomogeneousTransformationMatrix.from_point_rotation_matrix(
                    point=parent_T_object.position,
                    rotation_matrix=parent_origin.rotation_matrix,
                    reference_frame=body.parent_connection.parent,
                )
            )
        return matching_annotations

    @staticmethod
    def _with_z_up_and_x_towards(
        root_T_frame: HomogeneousTransformationMatrix, root_P_target: Point3
    ) -> HomogeneousTransformationMatrix:
        """
        Return ``root_T_frame`` rotated by 180° around its own axes so that its z axis
        points along the world z axis and its x axis points towards ``root_P_target`` as
        closely as those rotations allow.

        The position is left untouched, and every axis stays on the axis it started on.
        :param root_T_frame:``HomogeneousTransformationMatrix`` which will be rotated
        :param root_P_target:``Point3`` towards which root_T_frame x axis will point
        :return: rotated transformation matrix
        """
        root_V_target = root_P_target - root_T_frame.position

        flip_z = root_T_frame[2, 2] < 0
        flip_x = root_T_frame.rotation_matrix.x_vector().dot(root_V_target) > 0

        return root_T_frame @ HomogeneousTransformationMatrix.from_xyz_rpy(
            roll=np.pi if flip_z and not flip_x else 0,
            pitch=np.pi if flip_z and flip_x else 0,
            yaw=np.pi if flip_x and not flip_z else 0,
        )

    def resolve_annotations(self, world: World) -> list[IsPerceivable]:
        """
        Find the annotations in ``world`` that this detection identified.

        Several annotations may describe the same body, which is not ambiguous; only an
        annotation describing more than one body is.

        :param world: The world to search.
        :return: The matching annotations, all sharing one root body.
        :raises PerceivedObjectNotInWorld: If the world holds no instance of the
            annotation.
        :raises AmbiguousDetection: If the matching annotations sit on different bodies.
        """
        candidates = world.get_semantic_annotations_by_type(self.semantic_annotation)
        if not candidates:
            raise PerceivedObjectNotInWorld(self.semantic_annotation)
        bodies = {annotation.root for annotation in candidates}
        if len(bodies) > 1:
            raise AmbiguousDetection(self.semantic_annotation, len(bodies))
        return candidates


# %% perception sources


class PerceptionInterface(ABC):
    """
    A source of detections.
    """

    @abstractmethod
    def look_for(self, query: PerceptionQuery) -> list[Detection]:
        """
        Report every object this source sees for a perception query.

        :param query: What to look for and where.
        :return: One detection per object seen, empty when the source saw nothing.
        """

    def detect(
        self, query: PerceptionQuery, accept_first_if_multiple: bool = False
    ) -> Detection | None:
        """
        Answer a perception query with the one object it asked about.

        :param query: What to look for and where.
        :param accept_first_if_multiple: Whether several candidates may be resolved by
            taking the first one. When False, several candidates raise
            :class:`~coraplex.exceptions.UnidentifiedDetections` instead of being chosen
            between.
        :return: The object this source saw, None when it saw nothing.
        :raises UnidentifiedDetections: If several candidates were seen and none of them
            may be picked.
        """
        detections = self.look_for(query)
        if not detections:
            return None
        if len(detections) > 1 and not accept_first_if_multiple:
            raise UnidentifiedDetections(query.semantic_annotation, len(detections))
        return detections[0]

    @staticmethod
    def for_source(
        source: PerceptionSource, ros_node: Node | None = None
    ) -> PerceptionInterface:
        """
        :param source: The kind of source that answers the queries.
        :param ros_node: Node a RoboKudo pipeline is reached through.
        :return: The source to answer queries with.
        """
        match source:
            case PerceptionSource.WORLD_MODEL:
                return WorldPerception()
            case PerceptionSource.ROBOKUDO:
                return RoboKudoPerception(ros_node=ros_node)


@dataclass
class WorldPerception(PerceptionInterface):
    """
    Source that reads the objects straight out of the world model.

    Reports what a perfect sensor would see: every body of the queried annotation that
    lies inside the region and is visible to the robot's camera, at the pose the world
    already holds.
    """

    def look_for(self, query: PerceptionQuery) -> list[Detection]:
        annotation_by_body = {}
        for annotation in query.world.get_semantic_annotations_by_type(
            query.semantic_annotation
        ):
            annotation_by_body.setdefault(annotation.root, type(annotation))

        return [
            Detection(
                semantic_annotation=annotation_by_body[body], pose=body.global_pose
            )
            for body in query.from_world()
            if body in annotation_by_body
        ]


@dataclass
class RoboKudoQuerying:
    """
    Asking a running RoboKudo pipeline over its query action.
    """

    ros_node: Node
    """
    Node the action client is created on.
    """

    action_name: str = ROBOKUDO_QUERY_ACTION_NAME
    """
    Action RoboKudo answers queries on.
    """

    server_timeout: timedelta = field(default=timedelta(seconds=10))
    """
    How long to wait for the action server before giving up.
    """

    def ask(self, requested: ObjectDesignator) -> list[ObjectDesignator]:
        """
        :param requested: What the pipeline is asked for.
        :return: The object designators the pipeline answered with.
        :raises PerceptionSourceUnavailable: If no pipeline serves the query action.
        """
        from robokudo_msgs.action import Query

        client = ActionClient(self.ros_node, Query, self.action_name)
        if not client.wait_for_server(timeout_sec=self.server_timeout.total_seconds()):
            raise PerceptionSourceUnavailable(self.action_name)
        return list(client.send_goal(Query.Goal(obj=requested)).result.res)

    @staticmethod
    def pose_in(world: World, pose_stamped: PoseStamped) -> Pose:
        """
        :param world: The world holding the frame the pose is stamped with.
        :param pose_stamped: A pose the pipeline reported.
        :return: The reported pose, relative to its frame in ``world``.
        """
        return Pose.from_xyz_quaternion(
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z,
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w,
            reference_frame=world.get_kinematic_structure_entity_by_name(
                pose_stamped.header.frame_id
            ),
        )


@dataclass
class RoboKudoPerception(RoboKudoQuerying, PerceptionInterface):
    """
    Source that asks a running RoboKudo pipeline over its query action.
    """

    def look_for(self, query: PerceptionQuery) -> list[Detection]:
        from robokudo_msgs.msg import ObjectDesignator

        designators = self.ask(
            ObjectDesignator(type=query.semantic_annotation.__name__.lower())
        )
        return [
            Detection(
                semantic_annotation=query.semantic_annotation,
                pose=self.pose_in(query.world, designator.pose[0]),
            )
            for designator in designators
            if designator.pose and self._can_be_requested_object(designator, query)
        ]

    @staticmethod
    def _can_be_requested_object(
        designator: ObjectDesignator, query: PerceptionQuery
    ) -> bool:
        """
        Whether a reported designator could be the object the query asked for.

        A pipeline that only localizes reports no class label, and every object it found
        is then a candidate. One that classifies lets the objects that were not asked
        for be discarded here; the detection is then identified as the queried
        annotation.

        :param designator: The designator the pipeline reported.
        :param query: The query it was answering.
        :return: True if the designator is a candidate for the requested object.
        """
        if not designator.type:
            return True
        return (
            designator.type.strip().lower()
            == query.semantic_annotation.__name__.lower()
        )


# %% seats


class SeatQueryTerm(StrEnum):
    """
    The words a seat query and its answer are written in.
    """

    SEAT = "seat"
    """
    What a seat query asks for.
    """

    FREE = "free"
    """
    The attribute of a seat no one sits on.
    """

    OCCUPIED = "occupied"
    """
    The attribute of a seat someone sits on.
    """


@dataclass
class SeatReport:
    """
    One seat a perception source saw, and whether someone sits on it.
    """

    pose: Pose
    """
    Where the source saw the seat.
    """

    occupied: bool
    """
    Whether the source saw someone sitting on the seat.
    """

    seat_place: Region | None = None
    """
    The seat place of the world the seat is, None for a seat the world does not know.
    """


class SeatPerception(ABC):
    """
    A source of the seats in view and of whether someone sits on them.
    """

    @abstractmethod
    def report_seats(self, robot: AbstractRobot) -> list[SeatReport]:
        """
        :param robot: The robot whose camera looks.
        :return: One report per seat in view of the robot's camera.
        """


@dataclass
class WorldSeatPerception(SeatPerception):
    """
    Source that reads the seats straight out of the world model.

    Reports every seat place of the furniture the robot's camera sees, occupied when a
    :class:`~semantic_digital_twin.semantic_annotations.semantic_annotations.Human` the
    camera sees is in it.
    """

    def report_seats(self, robot: AbstractRobot) -> list[SeatReport]:
        camera = robot.get_default_camera()
        reports = []
        for furniture in robot._world.get_semantic_annotations_by_type(HasSeatPlaces):
            if not any(
                VisibleTo(entity=body, camera=camera)() for body in furniture.bodies
            ):
                continue
            reports.extend(
                SeatReport(
                    pose=seat_place.global_pose,
                    occupied=any(
                        VisibleTo(entity=person.root, camera=camera)()
                        for person in Human.get_instances_in_region(seat_place)
                    ),
                    seat_place=seat_place,
                )
                for seat_place in furniture.seat_places
            )
        return reports


@dataclass
class RoboKudoSeatPerception(RoboKudoQuerying, SeatPerception):
    """
    Source that asks a running RoboKudo pipeline for the seats in view.

    The pipeline identifies a seat place of the world by its name as the designator's
    ``uid`` and reports whether a seat is free among its attributes.
    """

    def report_seats(self, robot: AbstractRobot) -> list[SeatReport]:
        from robokudo_msgs.msg import ObjectDesignator

        world = robot._world
        return [
            SeatReport(
                pose=self.pose_in(world, designator.pose[0]),
                occupied=SeatQueryTerm.OCCUPIED in designator.attribute,
                seat_place=(
                    world.get_kinematic_structure_entity_by_name(designator.uid)
                    if designator.uid
                    else None
                ),
            )
            for designator in self.ask(ObjectDesignator(type=SeatQueryTerm.SEAT))
            if designator.pose
        ]


# %% perceiving inside the motion chart


@dataclass(eq=False, repr=False, kw_only=True)
class WorldWritingPerceptionTask(
    EndedByOwner, FailsOnObservingFalse, StatechartNode, ABC
):
    """
    Statechart node that asks a perception source and writes what it saw into the
    world.

    The node moves nothing, so it adds no motion constraints and its owner decides when
    it succeeded, like a task of a motion. The query is answered on the node's first tick,
    so the whole perception takes one tick however long the source needs to reply, and
    the node then keeps the observation that answer gave.

    .. warning:: That tick blocks until the source replies, which on the real robot holds
        up the control loop for as long as the pipeline takes to answer.
    """

    answered_by: PerceptionSource
    """
    Which kind of source answers the query.

    Carried by the node rather than read from the context, because on the real robot the
    chart is answered in the controller's process, whose context does not say how the
    plan is executed.
    """

    _observation: ObservationStateValues | None = field(
        init=False, default=None, repr=False
    )
    """
    The observation this node keeps once the query has been answered, None before that.
    """

    @classproperty
    def required_context_extensions(cls) -> tuple[type[ContextExtension], ...]:
        return super().required_context_extensions + (RosContextExtension,)

    @property
    def ros_node(self) -> Node:
        """
        :return: The node a perception source outside this process is reached through.
        """
        return self.context.require_extension(RosContextExtension).ros_node

    def on_start(self) -> None:
        self._observation = None

    def on_tick(self) -> ObservationStateValues | None:
        if self._observation is None:
            self._observation = self.answer_query()
        return self._observation

    @abstractmethod
    def answer_query(self) -> ObservationStateValues:
        """
        Ask the perception source and write what it saw into the world.

        :return: The observation the node keeps from then on.
        """


@dataclass(eq=False, repr=False, kw_only=True)
class PerceptionTask(WorldWritingPerceptionTask):
    """
    Perception that moves the one object the query asks for to where it was seen.

    A source that saw nothing makes the node observe ``FALSE``, which fails it, so a
    surrounding :class:`~cramph.composites.TryInOrder` can go and look somewhere else.
    """

    query: PerceptionQuery
    """
    What to look for and where.
    """

    perception_source: PerceptionInterface | None = field(init=False, default=None)
    """
    The source answering the query, resolved during :meth:`set_up`.
    """

    accept_first_if_multiple: bool = False
    """
    Whether several candidates may be resolved by taking the first one.

    When False, several candidates raise
    :class:`~coraplex.exceptions.UnidentifiedDetections` instead of being chosen between.
    """

    def set_up(self) -> None:
        super().set_up()
        self.perception_source = PerceptionInterface.for_source(
            self.answered_by, self.ros_node
        )

    def answer_query(self) -> ObservationStateValues:
        """
        :return: True once the object was seen, False when the source saw nothing.
        """
        detection = self.perception_source.detect(
            self.query, self.accept_first_if_multiple
        )
        if detection is None:
            return ObservationStateValues.FALSE
        detection.apply_to(
            self.query.world, trust_orientation=self.query.trust_detected_orientation
        )
        return ObservationStateValues.TRUE


@dataclass(eq=False, repr=False, kw_only=True)
class SeatPerceptionTask(WorldWritingPerceptionTask):
    """
    Perception that makes the people on the seats in view the ones the source saw there.

    A seat the world does not know is a chair: the one of the world whose seat place is
    within :attr:`chair_matching_distance`, otherwise a new one is added. The people on a
    seat seen free are removed, and a person is seated on a seat seen occupied that holds
    no one.
    """

    robot: AbstractRobot
    """
    The robot whose camera looks.
    """

    chair_seat_place_size: Scale = field(default_factory=lambda: Scale(0.5, 0.5, 0.5))
    """
    The extents of the seat place of a chair added to the world, in meters.
    """

    chair_matching_distance: float = 0.5
    """
    Distance in meters from a seat place of a chair below which a seat without a seat
    place is that chair.
    """

    seat_perception: SeatPerception | None = field(init=False, default=None)
    """
    The source reporting the seats, resolved during :meth:`set_up`.
    """

    def set_up(self) -> None:
        super().set_up()
        match self.answered_by:
            case PerceptionSource.WORLD_MODEL:
                self.seat_perception = WorldSeatPerception()
            case PerceptionSource.ROBOKUDO:
                self.seat_perception = RoboKudoSeatPerception(ros_node=self.ros_node)

    def answer_query(self) -> ObservationStateValues:
        """
        :return: True, since seeing no seat is an answer too.
        """
        world = self.robot._world
        reports = self.seat_perception.report_seats(self.robot)
        seat_places = []
        for report in reports:
            if report.seat_place is not None:
                seat_places.append(report.seat_place)
                continue
            world_T_seat = world.transform(report.pose, world.root).homogeneous_matrix
            distances = {
                seat_place: np.linalg.norm(
                    seat_place.global_transform.position.to_np()[:3]
                    - world_T_seat.position.to_np()[:3]
                )
                for chair in world.get_semantic_annotations_by_type(Chair)
                for seat_place in chair.seat_places
            }
            closest = min(distances, key=distances.get, default=None)
            if (
                closest is not None
                and distances[closest] < self.chair_matching_distance
            ):
                seat_places.append(closest)
                continue
            name = (
                f"detected_chair_{len(world.get_semantic_annotations_by_type(Chair))}"
            )
            seat_place_name = f"{name}_seat_place"
            chair = SemanticAnnotationWithRootSpecification(
                name=name,
                semantic_annotation_type=Chair,
                root_specification=BodySpecification(
                    name=name,
                    child_specifications=[
                        RegionSpecification.box(
                            seat_place_name, self.chair_seat_place_size
                        )
                    ],
                ),
            ).spawn(world, parent_T_self=world_T_seat)
            seat_place = world.get_kinematic_structure_entity_by_name(seat_place_name)
            with world.modify_world():
                chair.add_seat_place(seat_place)
            seat_places.append(seat_place)
        occupants = {
            seat_place: Human.get_instances_in_region(seat_place)
            for seat_place in seat_places
        }
        people_who_left = [
            occupant
            for report, seat_place in zip(reports, seat_places)
            if not report.occupied
            for occupant in occupants[seat_place]
        ]
        newly_occupied_seat_places = {
            seat_place: seat_place.global_transform
            for report, seat_place in zip(reports, seat_places)
            if report.occupied and not occupants[seat_place]
        }
        with world.modify_world():
            for occupant in people_who_left:
                world.remove_semantic_annotation(occupant)
                world.remove_connection(occupant.root.parent_connection)
                world.remove_kinematic_structure_entity(occupant.root)
            for seat_place, world_T_seat_place in newly_occupied_seat_places.items():
                name = f"{seat_place.name.name}_occupant"
                Human.get_annotation_specification(name, BodySpecification(name)).spawn(
                    world, parent_T_self=world_T_seat_place
                )
        return ObservationStateValues.TRUE
