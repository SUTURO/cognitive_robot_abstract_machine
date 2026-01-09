from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
import trimesh
from probabilistic_model.probabilistic_circuit.rx.helper import (
    uniform_measure_of_event,
)
from random_events.product_algebra import Event
from typing_extensions import (
    TYPE_CHECKING,
    List,
    Optional,
    Self,
    Iterable,
    Type,
)

from krrood.ormatic.utils import classproperty
from ..datastructures.prefixed_name import PrefixedName
from ..datastructures.variables import SpatialVariables
from ..spatial_types import Point3, HomogeneousTransformationMatrix, Vector3
from ..spatial_types.derivatives import DerivativeMap
from ..utils import Direction
from ..world import World
from ..world_description.connections import (
    RevoluteConnection,
    FixedConnection,
    PrismaticConnection,
    ActiveConnection1DOF,
)
from ..world_description.degree_of_freedom import DegreeOfFreedomLimits, DegreeOfFreedom
from ..world_description.geometry import Scale
from ..world_description.shape_collection import BoundingBoxCollection
from ..world_description.world_entity import (
    SemanticAnnotation,
    Body,
    Region,
    KinematicStructureEntity,
    Connection,
)

if TYPE_CHECKING:
    from .semantic_annotations import (
        Drawer,
        Door,
        Handle,
        Hinge,
        Slider,
        Aperture,
    )


@dataclass(eq=False)
class IsPerceivable:
    """
    A mixin class for semantic annotations that can be perceived.
    """

    class_label: Optional[str] = field(default=None, kw_only=True)
    """
    The exact class label of the perceived object.
    """


@dataclass(eq=False)
class HasRootBody(SemanticAnnotation, ABC):
    """
    Abstract base class for all household objects. Each semantic annotation refers to a single Body.
    Each subclass automatically derives a MatchRule from its own class name and
    the names of its HouseholdObject ancestors. This makes specialized subclasses
    naturally more specific than their bases.
    """

    body: Body = field(kw_only=True)

    @classproperty
    @abstractmethod
    def _parent_connection_type(self) -> Type[Connection]: ...

    @property
    def bodies(self) -> Iterable[Body]:
        return [self.body]

    @classmethod
    def create_with_new_body_in_world(
        cls,
        name: PrefixedName,
        world: World,
        world_root_T_self: Optional[HomogeneousTransformationMatrix] = None,
        connection_limits: Optional[DegreeOfFreedomLimits] = None,
        active_axis: Vector3 = Vector3.Z(),
        connection_multiplier: float = 1.0,
        connection_offset: float = 0.0,
        **kwargs,
    ) -> Self:
        """
        Create a new semantic annotation with a new body in the given world.
        If you need more parameters for your subclass, please override this method similar.
        If you override this method, to ensure its LSP compliant, use keyword arguments as
        described in PEP3102 https://peps.python.org/pep-3102/.

        An example of this can be seen in HasCase.create_with_new_body_in_world.
        """
        body = Body(name=name)

        return cls._create_with_connection_in_world(
            connection_type=cls._parent_connection_type,
            name=name,
            world=world,
            body=body,
            world_root_T_self=world_root_T_self,
            connection_multiplier=connection_multiplier,
            connection_offset=connection_offset,
            active_axis=active_axis,
            connection_limits=connection_limits,
        )

    @classmethod
    def _create_with_connection_in_world(
        cls,
        connection_type: Type[Connection],
        name: PrefixedName,
        world: World,
        body: Body,
        world_root_T_self: Optional[HomogeneousTransformationMatrix] = None,
        connection_limits: Optional[DegreeOfFreedomLimits] = None,
        active_axis: Vector3 = Vector3.Z(),
        connection_multiplier: float = 1.0,
        connection_offset: float = 0.0,
    ):
        if connection_type is None:
            raise ValueError(
                f"connection_type must not be None. You probably forgot to set the class variable "
                f"_parent_connection_type for class {cls.__name__}."
            )

        self_instance = cls(name=name, body=body)
        world_root_T_self = (
            world_root_T_self
            if world_root_T_self is not None
            else HomogeneousTransformationMatrix()
        )

        with world.modify_world():
            world.add_semantic_annotation(self_instance)
            world.add_kinematic_structure_entity(body)
            if issubclass(connection_type, ActiveConnection1DOF):
                limits = connection_limits or cls._generate_default_dof_limits(
                    connection_type
                )
                if limits.lower_limit.position >= limits.upper_limit.position:
                    raise ValueError(
                        f"Lower limit for {name} must be strictly less than upper limit."
                    )
                dof = DegreeOfFreedom(
                    name=PrefixedName("dof", str(name)),
                    upper_limits=limits.upper_limit,
                    lower_limits=limits.lower_limit,
                )
                world.add_degree_of_freedom(dof)
                world_root_C_self = cls._parent_connection_type(
                    parent=world.root,
                    child=body,
                    parent_T_connection_expression=world_root_T_self,
                    multiplier=connection_multiplier,
                    offset=connection_offset,
                    axis=active_axis,
                    dof_id=dof.id,
                )
            elif connection_type == FixedConnection:
                world_root_C_self = FixedConnection(
                    parent=world.root,
                    child=body,
                    parent_T_connection_expression=world_root_T_self,
                )
            else:
                raise ValueError(f"Invalid connection type: {connection_type}")
            world.add_connection(world_root_C_self)

        return self_instance

    @classmethod
    def _generate_default_dof_limits(
        cls, active_connection_type: Type[ActiveConnection1DOF]
    ) -> DegreeOfFreedomLimits:
        """ """
        lower_limits = DerivativeMap[float]()
        upper_limits = DerivativeMap[float]()
        if active_connection_type == PrismaticConnection:
            lower_limits.position = -np.inf
            upper_limits.position = np.inf
        elif active_connection_type == RevoluteConnection:
            lower_limits.position = -2 * np.pi
            upper_limits.position = 2 * np.pi
        else:
            raise ValueError(
                f"Invalid active connection type: {active_connection_type}"
            )

        return DegreeOfFreedomLimits(lower_limit=lower_limits, upper_limit=upper_limits)


@dataclass(eq=False)
class HasRootRegion(SemanticAnnotation, ABC):
    """
    A mixin class for semantic annotations that have a region.
    """

    region: Region = field(kw_only=True)

    @classmethod
    def create_with_new_region_in_world(
        cls,
        name: PrefixedName,
        world: World,
        world_root_T_self: Optional[HomogeneousTransformationMatrix] = None,
        **kwargs,
    ) -> Self:
        """
        Create a new semantic annotation with a new region in the given world.
        """
        body = Region(name=name)

        return cls._create_with_fixed_connection_in_world(
            name, world, body, world_root_T_self
        )

    @classmethod
    def _create_with_fixed_connection_in_world(
        cls, name, world, region, world_root_T_self
    ):
        self_instance = cls(name=name, region=region)
        world_root_T_self = (
            world_root_T_self
            if world_root_T_self is not None
            else HomogeneousTransformationMatrix()
        )

        with world.modify_world():
            world.add_semantic_annotation(self_instance)
            world.add_kinematic_structure_entity(region)
            world_root_C_self = FixedConnection(
                parent=world.root,
                child=region,
                parent_T_connection_expression=world_root_T_self,
            )
            world.add_connection(world_root_C_self)

        return self_instance


@dataclass(eq=False)
class SemanticAssociation(ABC):

    def get_new_parent_T_self(
        self: HasRootBody | Self,
        parent_kinematic_structure_entity: KinematicStructureEntity,
    ) -> HomogeneousTransformationMatrix:
        return (
            parent_kinematic_structure_entity.global_pose.inverse()
            @ self.body.global_pose
        )

    def get_self_T_new_child(
        self: HasRootBody | Self,
        child_kinematic_structure_entity: KinematicStructureEntity,
    ) -> HomogeneousTransformationMatrix:
        return (
            self.body.global_pose.inverse()
            @ child_kinematic_structure_entity.global_pose
        )

    def get_new_grandparent(
        self: HasRootBody | Self,
        parent_kinematic_structure_entity: KinematicStructureEntity,
    ):
        grandparent_kinematic_structure_entity = (
            parent_kinematic_structure_entity.parent_connection.parent
        )
        new_hinge_parent = (
            grandparent_kinematic_structure_entity
            if grandparent_kinematic_structure_entity != self.body
            else self.body.parent_kinematic_structure_entity
        )
        return new_hinge_parent

    def _attach_parent_entity_in_kinematic_structure(
        self: HasRootBody | Self,
        new_parent_entity: KinematicStructureEntity,
    ):
        if new_parent_entity._world != self._world:
            raise ValueError(
                "Semantic annotation must be part of the same world as the parent."
            )
        if new_parent_entity == self.body.parent_kinematic_structure_entity:
            return

        world = self._world
        new_parent_T_self = self.get_new_parent_T_self(new_parent_entity)

        with world.modify_world():
            parent_C_self = self.body.parent_connection
            world.remove_connection(parent_C_self)

            new_parent_C_self = FixedConnection(
                parent=new_parent_entity,
                child=self.body,
                parent_T_connection_expression=new_parent_T_self,
            )
            world.add_connection(new_parent_C_self)

    def _attach_child_entity_in_kinematic_structure(
        self: HasRootBody | Self,
        child_kinematic_structure_entity: KinematicStructureEntity,
    ):
        if child_kinematic_structure_entity._world != self._world:
            raise ValueError("Hinge must be part of the same world as the door.")

        if self == child_kinematic_structure_entity.parent_kinematic_structure_entity:
            return

        world = self._world
        self_T_new_child = self.get_self_T_new_child(child_kinematic_structure_entity)

        with world.modify_world():
            parent_C_new_child = child_kinematic_structure_entity.parent_connection
            world.remove_connection(parent_C_new_child)

            self_C_new_child = FixedConnection(
                parent=self.body,
                child=child_kinematic_structure_entity,
                parent_T_connection_expression=self_T_new_child,
            )
            world.add_connection(self_C_new_child)


@dataclass(eq=False)
class HasApertures(SemanticAssociation, ABC):

    apertures: List[Aperture] = field(default_factory=list, hash=False, kw_only=True)

    def add_aperture(self: HasRootBody | Self, aperture: Aperture):
        """
        Cuts a hole in the semantic annotation's body for the given body annotation.

        :param body_annotation: The body annotation to cut a hole for.
        """
        self._remove_aperture_geometry_from_parent(aperture)
        self._attach_child_entity_in_kinematic_structure(aperture.region)
        self.apertures.append(aperture)

    def _remove_aperture_geometry_from_parent(
        self: HasRootBody | Self, aperture: Aperture
    ):
        hole_event = aperture.region.area.as_bounding_box_collection_in_frame(
            self.body
        ).event
        wall_event = self.body.collision.as_bounding_box_collection_in_frame(
            self.body
        ).event
        new_wall_event = wall_event - hole_event
        new_bounding_box_collection = BoundingBoxCollection.from_event(
            self.body, new_wall_event
        ).as_shapes()
        self.body.collision = new_bounding_box_collection
        self.body.visual = new_bounding_box_collection


@dataclass(eq=False)
class HasHinge(SemanticAssociation, ABC):
    """
    A mixin class for semantic annotations that have hinge joints.
    """

    hinge: Optional[Hinge] = field(init=False, default=None)

    def add_hinge(
        self: HasRootBody | Self,
        hinge: Hinge,
    ):
        """
        Adds a door to the parent world using a new door hinge body with a revolute connection.
        """
        self._attach_parent_entity_in_kinematic_structure(
            hinge.body,
        )
        self.hinge = hinge


@dataclass(eq=False)
class HasSlider(SemanticAssociation, ABC):
    """
    A mixin class for semantic annotations that have hinge joints.
    """

    slider: Optional[Slider] = field(init=False, default=None)

    def add_slider(
        self: HasRootBody | Self,
        slider: Slider,
    ):
        """
        Adds a door to the parent world using a new door hinge body with a revolute connection.

        :param door_factory: The factory used to create the door.
        :param parent_T_hinge: The transformation matrix defining the door's position and orientation relative
        to the parent world.
        :param parent_world: The world to which the door will be added.
        """
        self._attach_parent_entity_in_kinematic_structure(
            slider.body,
        )
        self.slider = slider


@dataclass(eq=False)
class HasDrawers(SemanticAssociation, ABC):
    """
    A mixin class for semantic annotations that have drawers.
    """

    drawers: List[Drawer] = field(default_factory=list, hash=False, kw_only=True)

    def add_drawer(
        self,
        drawer: Drawer,
    ):
        """
        Adds a door to the parent world using a new door hinge body with a revolute connection.

        :param door_factory: The factory used to create the door.
        :param parent_T_door: The transformation matrix defining the door's position and orientation relative
        to the parent world.
        :param parent_world: The world to which the door will be added.
        """

        self._attach_child_entity_in_kinematic_structure(drawer.body)
        self.drawers.append(drawer)


@dataclass(eq=False)
class HasDoors(SemanticAssociation, ABC):
    """
    A mixin class for semantic annotations that have doors.
    """

    doors: List[Door] = field(default_factory=list, hash=False, kw_only=True)

    def add_door(
        self,
        door: Door,
    ):
        """
        Adds a door to the parent world using a new door hinge body with a revolute connection.

        :param door_factory: The factory used to create the door.
        :param parent_T_door: The transformation matrix defining the door's position and orientation relative
        to the parent world.
        :param parent_world: The world to which the door will be added.
        """

        self._attach_child_entity_in_kinematic_structure(door.body)
        self.doors.append(door)


@dataclass(eq=False)
class HasLeftRightDoor(HasDoors):

    left_door: Optional[Door] = None
    right_door: Optional[Door] = None

    def add_door_to_world(
        self,
        door_factory: Door,
        parent_T_door: HomogeneousTransformationMatrix,
        opening_axis: Vector3,
        parent_world: World,
    ):
        raise NotImplementedError(
            "To add a door to a annotation inheriting from HasLeftRightDoor, please use add_right_door_to_world or add_left_door_to_world respectively"
        )

    def add_right_door(
        self,
        door: Door,
    ):
        raise NotImplementedError()

    def add_left_door(
        self,
        door: Door,
    ):
        raise NotImplementedError()

    @classmethod
    def create_with_left_right_door_in_world(
        cls: SemanticAnnotation | Self, left_door: Door, right_door: Door
    ) -> Self:
        """
        Create a DoubleDoor semantic annotation with the given left and right doors.

        :param left_door: The left door of the double door.
        :param right_door: The right door of the double door.
        :returns: A DoubleDoor semantic annotation.
        """
        if left_door._world != right_door._world:
            raise ValueError("Left and right door must be part of the same world.")
        double_door = cls(left_door=left_door, right_door=right_door)
        world = left_door._world
        with world.modify_world():
            world.add_semantic_annotation(double_door)

        return double_door


@dataclass(eq=False)
class HasHandle(SemanticAssociation, ABC):

    handle: Optional[Handle] = None
    """
    The handle of the semantic annotation.
    """

    def add_handle(
        self: HasRootBody | Self,
        handle: Handle,
    ):
        """
        Adds a handle to the parent world with a fixed connection.

        :param parent_T_handle: The transformation matrix defining the handle's position and orientation relative
        to the parent world.
        :param parent_world: The world to which the handle will be added.
        """
        self._attach_child_entity_in_kinematic_structure(
            handle.body,
        )
        self.handle = handle


@dataclass(eq=False)
class HasSupportingSurface(HasRootBody, ABC):
    """
    A semantic annotation that represents a supporting surface.
    """

    supporting_surface: Region = field(init=False, default=None)

    def calculate_supporting_surface(
        self,
        upward_threshold: float = 0.95,
        clearance_threshold: float = 0.5,
        min_surface_area: float = 0.0225,  # 15cm x 15cm
    ):
        mesh = self.body.combined_mesh
        if mesh is None:
            return
        # --- Find upward-facing faces ---
        normals = mesh.face_normals
        upward_mask = normals[:, 2] > upward_threshold

        if not upward_mask.any():
            return

        # --- Find connected upward-facing regions ---
        upward_face_indices = np.nonzero(upward_mask)[0]
        submesh_up = mesh.submesh([upward_face_indices], append=True)
        face_groups = submesh_up.split(only_watertight=False)

        # Compute total area for each group
        large_groups = [g for g in face_groups if g.area >= min_surface_area]

        if not large_groups:
            return

        # --- Merge qualifying upward-facing submeshes ---
        candidates = trimesh.util.concatenate(large_groups)

        # --- Check vertical clearance using ray casting ---
        face_centers = candidates.triangles_center
        ray_origins = face_centers + np.array([0, 0, 0.01])  # small upward offset
        ray_dirs = np.tile([0, 0, 1], (len(ray_origins), 1))

        locations, index_ray, _ = mesh.ray.intersects_location(
            ray_origins=ray_origins, ray_directions=ray_dirs
        )

        # Compute distances to intersections (if any)
        distances = np.full(len(ray_origins), np.inf)
        distances[index_ray] = np.linalg.norm(
            locations - ray_origins[index_ray], axis=1
        )

        # Filter faces with enough space above
        clear_mask = (distances > clearance_threshold) | np.isinf(distances)

        if not clear_mask.any():
            raise ValueError(
                "No upward-facing surfaces with sufficient clearance found."
            )

        candidates_filtered = candidates.submesh([clear_mask], append=True)

        # --- Build the region ---
        points_3d = [
            Point3(
                x,
                y,
                z,
                reference_frame=self.body,
            )
            for x, y, z in candidates_filtered.vertices
        ]

        self.supporting_surface = Region.from_3d_points(
            name=PrefixedName(
                f"{self.body.name.name}_supporting_surface_region",
                self.body.name.prefix,
            ),
            points_3d=points_3d,
        )

    def points_on_surface(self, amount: int = 100) -> List[Point3]:
        """
        Get points that are on the table.

        :amount: The number of points to return.
        :returns: A list of points that are on the table.
        """
        area_of_table = BoundingBoxCollection.from_shapes(self.body.collision)
        event = area_of_table.event
        p = uniform_measure_of_event(event)
        p = p.marginal(SpatialVariables.xy)
        samples = p.sample(amount)
        z_coordinate = np.full(
            (amount, 1), max([b.max_z for b in area_of_table]) + 0.01
        )
        samples = np.concatenate((samples, z_coordinate), axis=1)
        return [Point3(*s, reference_frame=self.body) for s in samples]


@dataclass(eq=False)
class HasCaseAsRootBody(HasSupportingSurface, ABC):

    @classproperty
    @abstractmethod
    def opening_direction(self) -> Direction: ...

    @classmethod
    def create_with_new_body_in_world(
        cls,
        name: PrefixedName,
        world: World,
        world_root_T_self: Optional[HomogeneousTransformationMatrix] = None,
        connection_limits: Optional[DegreeOfFreedomLimits] = None,
        active_axis: Vector3 = Vector3.Z(),
        connection_multiplier: float = 1.0,
        connection_offset: float = 0.0,
        *,
        scale: Scale = Scale(),
        wall_thickness: float = 0.01,
    ) -> Self:
        container_event = cls._create_container_event(scale, wall_thickness)

        body = Body(name=name)
        collision_shapes = BoundingBoxCollection.from_event(
            body, container_event
        ).as_shapes()
        body.collision = collision_shapes
        body.visual = collision_shapes
        return cls._create_with_connection_in_world(
            connection_type=cls._parent_connection_type,
            name=name,
            world=world,
            body=body,
            world_root_T_self=world_root_T_self,
            connection_multiplier=connection_multiplier,
            connection_offset=connection_offset,
            active_axis=active_axis,
            connection_limits=connection_limits,
        )

    @classmethod
    def _create_container_event(cls, scale: Scale, wall_thickness: float) -> Event:
        """
        Return an event representing a container with walls of a specified thickness.
        """
        outer_box = scale.to_simple_event()
        inner_box = Scale(
            scale.x - wall_thickness,
            scale.y - wall_thickness,
            scale.z - wall_thickness,
        ).to_simple_event(cls.opening_direction, wall_thickness)

        container_event = outer_box.as_composite_set() - inner_box.as_composite_set()

        return container_event
