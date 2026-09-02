from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from krrood.symbolic_math.symbolic_math import (
    CompiledFunction,
    FloatVariable,
    VariableParameters,
    vstack,
)
from semantic_digital_twin.robots.abstract_robot import ForceTorqueSensor
from semantic_digital_twin.spatial_types import Point3, Vector3
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)

from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import ObservationStateValues
from giskardpy.motion_statechart.graph_node import NodeArtifacts
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPosition,
    CartesianTask,
)


class NonPositiveVirtualMassError(Exception):
    """Raised when an admittance would render a non-positive mass, which no
    integration step can keep stable."""

    def __init__(self, mass: Vector3) -> None:
        super().__init__(f"mass must be > 0 on every axis, got {mass.to_np()[:3]}.")


@dataclass(eq=False, repr=False)
class AdmittanceCartesianPosition(CartesianTask):
    """
    Cartesian position task with linear admittance control.

    The QP sees ``goal_point + admittance_position``. Rotating the live wrench into the
    goal frame and integrating the virtual dynamics are one expression, compiled in
    :meth:`build`, so a tick is a single call on the world state, the sensor inputs and
    the offset.

    Observation TRUE compares the *nominal* goal to the tip, so termination still
    fires when a non-zero admittance offset is active.

    .. todo:: Two guidelines are not implemented: post-sensor inertia compensation
        (guideline 3), which renders ``mass - inertia_compensation`` and belongs on the
        force/torque sensor annotation because it describes what is mounted past the
        sensor, and acceleration feedforward (guideline 5), which adds
        ``gain * admittance_velocity`` phase lead to the QP goal and is controller
        tuning rather than robot knowledge.
    """

    goal_point: Point3 = field(kw_only=True)
    """Nominal target point (in the goal reference frame)."""

    desired_force: Vector3 = field(default_factory=Vector3, kw_only=True)
    """Contact force balanced by the admittance, in the goal frame. Biasing 3-5 N into
    the surface removes tap-and-bounce."""

    mass: Vector3 = field(
        default_factory=lambda: Vector3(x=1.0, y=1.0, z=1.0), kw_only=True
    )
    """Virtual mass per axis, in kg."""

    damping: Vector3 = field(
        default_factory=lambda: Vector3(x=20.0, y=20.0, z=20.0), kw_only=True
    )
    """Virtual damping per axis, in N s/m."""

    stiffness: Vector3 = field(default_factory=Vector3, kw_only=True)
    """Virtual stiffness per axis, in N/m."""

    threshold: float = field(default=0.01, kw_only=True)
    """Goal-reached distance (against the nominal goal), in m."""

    reference_velocity: float | None = field(
        default_factory=lambda: CartesianPosition.default_reference_velocity,
        kw_only=True,
    )
    """Reference velocity for normalization, in m/s."""

    _admittance_position: Vector3 | None = field(init=False, default=None)
    """Symbolic admittance offset added to the nominal goal. Registered as a float
    variable, because the QP evaluates the goal expression against that array."""

    _state: np.ndarray = field(init=False, default_factory=lambda: np.zeros(6))
    """Live offset and its rate, the input and the output of :attr:`_compiled_step`.
    Task-owned rather than shared, so its size never depends on the rest of the chart."""

    _compiled_step: CompiledFunction | None = field(init=False, default=None)
    """Wrench rotation and one integration step, compiled together in :meth:`build`."""

    _force_torque_sensor: ForceTorqueSensor | None = field(init=False, default=None)
    """Live sensor annotation resolved from ``tip_link`` in :meth:`build`."""

    @property
    def goal_reference_frame(self) -> KinematicStructureEntity:
        return self.goal_point.reference_frame

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        artifacts = super().build(context)
        self._force_torque_sensor = ForceTorqueSensor.for_tip(
            context.world, self.tip_link
        )
        self._compile_step(context)

        root_P_corrected = self.root_T_goal_reference_frame @ (
            self.goal_point + self._admittance_position
        )
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
    ) -> ObservationStateValues | None:
        if not self._force_torque_sensor.has_received_wrench:
            return None
        self._state[:] = self._compiled_step(
            context.world.state.positions,
            context.world.sensor_inputs.data,
            self._state,
        )
        context.float_variable_data.set_value(
            self._admittance_position, self._state[:3]
        )
        return None

    def on_reset(self, context: MotionStatechartContext) -> None:
        self._state[:] = 0.0
        context.float_variable_data.set_value(
            self._admittance_position, self._state[:3]
        )

    def _compile_step(self, context: MotionStatechartContext) -> None:
        """Compile the live wrench, rotated into the goal frame and integrated by one
        step of the virtual dynamics, into :attr:`_compiled_step`. The dynamics are
        diagonal, so each axis of the goal frame integrates its own
        ``mass * acceleration + damping * velocity + stiffness * position =
        measured_force - desired_force``.

        The integration is semi-implicit Euler with the damping and stiffness terms
        taken implicitly, which is unconditionally stable for any positive parameters;
        an explicit damping term instead diverges once ``control_dt * damping / mass``
        exceeds 2.

        The offset symbols are registered as float variables so the QP can read the
        offset, while the step takes them as its own parameter group: it stays a
        fixed-size problem no matter how many nodes the chart holds.
        """
        if any(float(self.mass[axis]) <= 0 for axis in range(3)):
            raise NonPositiveVirtualMassError(self.mass)

        position_variables = [
            FloatVariable(name=f"{self.name}/admittance_position.{axis}")
            for axis in ("x", "y", "z")
        ]
        velocity_variables = [
            FloatVariable(name=f"{self.name}/admittance_velocity.{axis}")
            for axis in ("x", "y", "z")
        ]
        self._admittance_position = Vector3(
            *position_variables, reference_frame=self.goal_reference_frame
        )
        context.float_variable_data.register_expression(self._admittance_position)

        goal_T_sensor = context.world.compose_forward_kinematics_expression(
            self.goal_reference_frame, self._force_torque_sensor.root
        )
        measured_force = (
            goal_T_sensor.to_rotation_matrix() @ self._force_torque_sensor.force
        )
        control_dt = context.qp_controller_config.control_dt

        next_positions, next_velocities = [], []
        for axis in range(3):
            mass, damping = self.mass[axis], self.damping[axis]
            stiffness = self.stiffness[axis]
            position, velocity = position_variables[axis], velocity_variables[axis]
            force_error = measured_force[axis] - self.desired_force[axis]
            next_velocity = (
                mass * velocity + (force_error - stiffness * position) * control_dt
            ) / (mass + damping * control_dt + stiffness * control_dt**2)
            next_positions.append(position + next_velocity * control_dt)
            next_velocities.append(next_velocity)

        self._compiled_step = vstack(next_positions + next_velocities).compile(
            parameters=VariableParameters.from_lists(
                context.world.state.position_float_variables,
                context.world.sensor_inputs.variables,
                position_variables + velocity_variables,
            ),
            sparse=False,
        )


@dataclass(eq=False, repr=False)
class LowerUntilContact(CartesianPosition):
    """Cartesian position task that terminates on force/torque contact instead of on
    goal reach. Target a point below the surface; the force observation halts the
    descent, so the surface depth need not be exact."""

    force_threshold: float = field(default=3.0, kw_only=True)
    """Force magnitude in N at which contact is declared."""

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        artifacts = super().build(context)
        sensor = ForceTorqueSensor.for_tip(context.world, self.tip_link)
        artifacts.observation = sensor.force.norm() > self.force_threshold
        return artifacts
