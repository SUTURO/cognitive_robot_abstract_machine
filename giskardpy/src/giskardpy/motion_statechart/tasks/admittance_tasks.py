from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from krrood.symbolic_math.float_variable_data import hidden_index_name
from krrood.symbolic_math.symbolic_math import (
    CompiledFunction,
    VariableParameters,
)
from semantic_digital_twin.spatial_types import Point3, Vector3
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)

from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import ObservationStateValues
from giskardpy.motion_statechart.graph_node import NodeArtifacts
from giskardpy.motion_statechart.ros2_nodes.force_torque_monitor import (
    ForceTorqueSymbolNode,
)
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPosition,
    CartesianTask,
)


@dataclass(eq=False, repr=False)
class AdmittanceCartesianPosition(CartesianTask):
    """
    Cartesian position task with linear admittance control.

    Per axis in the goal frame, the virtual dynamics are
        mass * acceleration + damping * velocity + stiffness * position
            = measured_force - desired_force

    where ``position`` is the admittance offset added to the nominal goal.
    ``build`` is symbolic: the QP sees goal_point + admittance_position +
    acceleration_feedforward_gain * admittance_velocity (guideline 5).
    ``on_tick`` is numerical: it integrates ``(position, velocity)`` with
    semi-implicit Euler using the constant ``mass/damping/stiffness`` and
    the per-tick FT reading rotated into the goal frame.

    Observation TRUE compares the *nominal* goal to the tip, so termination
    still fires when a non-zero admittance offset is active.
    """

    goal_point: Point3 = field(kw_only=True)
    """Nominal target point (in the goal reference frame)."""

    ft_node: ForceTorqueSymbolNode = field(kw_only=True)
    """Source of the symbolic force/torque vector."""

    desired_force: Vector3 = field(default=None, kw_only=True)
    """Contact force balanced by the admittance, in the goal frame.
    Default zero. Biasing 3-5 N into the surface removes tap-and-bounce."""

    mass: Vector3 = field(default=None, kw_only=True)
    """Virtual mass per axis in kg. Default (1, 1, 1)."""

    damping: Vector3 = field(default=None, kw_only=True)
    """Virtual damping per axis in N s/m. Default (20, 20, 20)."""

    stiffness: Vector3 = field(default=None, kw_only=True)
    """Virtual stiffness per axis in N/m. Default (0, 0, 0): pure
    accommodation (yields and stays yielded). (guideline 2)"""

    inertia_compensation: Vector3 = field(default=None, kw_only=True)
    """Post-sensor inertia subtracted from ``mass`` so the rendered
    inertia is ``mass - inertia_compensation``. Default zero. (guideline 3)"""

    acceleration_feedforward_gain: float = field(default=0.0, kw_only=True)
    """Seconds of phase lead added to the QP goal as
    ``gain * admittance_velocity``. Passive for any value >= 0. (guideline 5)"""

    threshold: float = field(default=0.01, kw_only=True)
    """Goal-reached distance (against nominal goal), in meters."""

    reference_velocity: Optional[float] = field(
        default_factory=lambda: CartesianPosition.default_reference_velocity,
        kw_only=True,
    )
    """Reference velocity for normalization, in m/s."""

    _admittance_position: Vector3 = field(init=False, default=None)
    _admittance_velocity: Vector3 = field(init=False, default=None)
    _admittance_position_start: int = field(init=False, default=0)
    _admittance_velocity_start: int = field(init=False, default=0)

    _damping_array: np.ndarray = field(init=False, default=None)
    _stiffness_array: np.ndarray = field(init=False, default=None)
    _effective_mass_array: np.ndarray = field(init=False, default=None)
    _desired_force_array: np.ndarray = field(init=False, default=None)

    _compiled_force_in_goal_frame: CompiledFunction = field(init=False, default=None)

    @property
    def goal_reference_frame(self) -> KinematicStructureEntity:
        return self.goal_point.reference_frame

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        artifacts = super().build(context)
        self._resolve_defaults()
        self._rebind_ft_node()
        self._ensure_ft_symbols_registered(context)
        self._register_state_symbols(context)
        self._validate_stability(context)
        self._compile_force_in_goal_frame(context)

        # guideline 5
        correction = (
            self._admittance_position
            + self.acceleration_feedforward_gain * self._admittance_velocity
        )
        corrected_goal_point = self.goal_point + correction
        root_P_corrected = self.root_T_goal_reference_frame @ corrected_goal_point

        root_P_current = context.world.compose_forward_kinematics_expression(
            self.root_link, self.tip_link
        ).to_position()

        artifacts.constraints.add_point_goal_constraints(
            frame_P_goal=root_P_corrected,
            frame_P_current=root_P_current,
            reference_velocity=self.reference_velocity,
            quadratic_weight=self.weight,
        )

        root_P_nominal = self.root_T_goal_reference_frame @ self.goal_point
        artifacts.observation = (
            root_P_nominal.euclidean_distance(root_P_current) < self.threshold
        )
        return artifacts

    def on_tick(
        self, context: MotionStatechartContext
    ) -> Optional[ObservationStateValues]:
        if not self.ft_node.has_msg():
            return None

        data = context.float_variable_data.data
        position_start = self._admittance_position_start
        velocity_start = self._admittance_velocity_start
        position = data[position_start : position_start + 3]
        velocity = data[velocity_start : velocity_start + 3]

        force_in_goal_frame = self._compiled_force_in_goal_frame(
            context.world.state.positions, data
        )[:3]
        force_error = force_in_goal_frame - self._desired_force_array

        # Semi-implicit Euler: update velocity first, then position.
        control_dt = context.qp_controller_config.control_dt
        acceleration = (
            force_error
            - self._damping_array * velocity
            - self._stiffness_array * position
        ) / self._effective_mass_array
        next_velocity = velocity + control_dt * acceleration
        next_position = position + control_dt * next_velocity

        context.float_variable_data.set_value(self._admittance_position, next_position)
        context.float_variable_data.set_value(self._admittance_velocity, next_velocity)
        return None

    def on_reset(self, context: MotionStatechartContext):
        context.float_variable_data.set_value(self._admittance_position, np.zeros(3))
        context.float_variable_data.set_value(self._admittance_velocity, np.zeros(3))

    def _resolve_defaults(self):
        if self.desired_force is None:
            self.desired_force = Vector3(reference_frame=self.goal_reference_frame)
        if self.mass is None:
            self.mass = Vector3(x=1.0, y=1.0, z=1.0)
        if self.damping is None:
            self.damping = Vector3(x=20.0, y=20.0, z=20.0)
        if self.stiffness is None:
            self.stiffness = Vector3()
        if self.inertia_compensation is None:
            self.inertia_compensation = Vector3()

        self._desired_force_array = self.desired_force.to_np()[:3].astype(np.float64)
        self._damping_array = self.damping.to_np()[:3].astype(np.float64)
        self._stiffness_array = self.stiffness.to_np()[:3].astype(np.float64)
        mass_array = self.mass.to_np()[:3].astype(np.float64)
        inertia_compensation_array = (
            self.inertia_compensation.to_np()[:3].astype(np.float64)
        )
        # guideline 3
        self._effective_mass_array = mass_array - inertia_compensation_array

        if np.any(mass_array <= 0):
            raise ValueError(f"mass must be > 0 on every axis, got {mass_array}.")
        if np.any(self._effective_mass_array <= 0):
            raise ValueError(
                f"mass - inertia_compensation must be > 0 on every axis, "
                f"got mass={mass_array}, "
                f"inertia_compensation={inertia_compensation_array}."
            )
        if self.acceleration_feedforward_gain < 0:
            raise ValueError(
                f"acceleration_feedforward_gain must be >= 0, "
                f"got {self.acceleration_feedforward_gain}."
            )

    def _validate_stability(self, context: MotionStatechartContext):
        # guideline 6: semi-implicit Euler stability bound.
        control_dt = context.qp_controller_config.control_dt
        natural_frequency = np.sqrt(self._stiffness_array / self._effective_mass_array)
        if np.any(control_dt * natural_frequency >= 1.0):
            raise ValueError(
                f"Stability bound control_dt * sqrt(stiffness / effective_mass) < 1 "
                f"violated: control_dt={control_dt}, "
                f"natural_frequency={natural_frequency}. "
                f"Lower stiffness, raise mass, or shrink control_dt."
            )

    def _rebind_ft_node(self):
        # JSON round-trip in giskard.execute detaches ft_node; rebind to the
        # live instance the MSC actually ticks. No-op when the task is built
        # outside a MotionStatechart (e.g. unit tests).
        if self._motion_statechart is None:
            return
        for node in self.motion_statechart.get_nodes_by_type(ForceTorqueSymbolNode):
            if node.name == self.ft_node.name:
                self.ft_node = node
                return

    def _ensure_ft_symbols_registered(self, context: MotionStatechartContext):
        # MSC build order is not guaranteed, so register defensively.
        if not hasattr(self.ft_node.force, hidden_index_name):
            context.float_variable_data.register_expression(self.ft_node.force)
        if not hasattr(self.ft_node.torque, hidden_index_name):
            context.float_variable_data.register_expression(self.ft_node.torque)

    def _register_state_symbols(self, context: MotionStatechartContext):
        self._admittance_position = Vector3.create_with_variables(
            f"{self.name}/admittance_position"
        )
        self._admittance_position.reference_frame = self.goal_reference_frame
        self._admittance_velocity = Vector3.create_with_variables(
            f"{self.name}/admittance_velocity"
        )
        self._admittance_velocity.reference_frame = self.goal_reference_frame
        context.float_variable_data.register_expression(self._admittance_position)
        context.float_variable_data.register_expression(self._admittance_velocity)
        self._admittance_position_start = getattr(
            self._admittance_position, hidden_index_name
        )
        self._admittance_velocity_start = getattr(
            self._admittance_velocity, hidden_index_name
        )

    def _compile_force_in_goal_frame(self, context: MotionStatechartContext):
        goal_T_sensor = context.world.compose_forward_kinematics_expression(
            self.goal_reference_frame, self.ft_node.reference_frame
        )
        force_in_goal_frame = goal_T_sensor.to_rotation_matrix() @ self.ft_node.force
        self._compiled_force_in_goal_frame = force_in_goal_frame.compile(
            parameters=VariableParameters.from_lists(
                context.world.state.position_float_variables,
                context.float_variable_data.variables,
            ),
            sparse=False,
        )


@dataclass(eq=False, repr=False)
class LowerUntilContact(CartesianPosition):
    """Cartesian position task that terminates on FT contact instead of on
    goal reach. Target a point below the surface; the force observation
    halts the descent, so surface depth need not be exact."""

    ft_node: ForceTorqueSymbolNode = field(kw_only=True)
    """Source of the symbolic force vector."""

    force_threshold: float = field(default=3.0, kw_only=True)
    """Force magnitude in N at which contact is declared."""

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        artifacts = super().build(context)
        artifacts.observation = self.ft_node.force.norm() > self.force_threshold
        return artifacts
