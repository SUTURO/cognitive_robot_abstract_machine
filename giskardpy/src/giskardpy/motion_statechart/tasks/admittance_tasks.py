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
    Classic linear admittance control on a Cartesian position goal.

    Models the end effector as a virtual spring-mass-damper per axis in the
    goal reference frame:

    .. math::
        M_d\\,\\ddot{x}_a + C_d\\,\\dot{x}_a + K_d\\,x_a = F_{ext} - F_{desired}

    where :math:`x_a` is the admittance displacement (added to the nominal
    goal position), :math:`F_{ext}` is the measured wrench expressed in the
    goal frame, and :math:`F_{desired}` is the contact force the controller
    tries to balance. Setting :math:`K_d=0` on an axis disables spring-back
    (the gripper yields and stays yielded), which is the default for table
    wiping.

    The admittance state :math:`(x_a, \\dot{x}_a)` is integrated in
    ``on_tick`` (semi-implicit Euler) and exposed to the QP through
    `FloatVariableData`. The QP sees the effective goal point
    ``goal_point + x_a`` as a symbolic expression compiled once at build
    time. The transformation of the measured wrench from the FT sensor
    frame into the goal frame is also compiled once and reuses the live
    joint state.

    Observation TRUE is reported when the **nominal** goal point is within
    ``threshold``, so the task still terminates correctly while a non-zero
    correction is active.
    """

    goal_point: Point3 = field(kw_only=True)
    """Nominal target point (in the goal reference frame)."""

    ft_node: ForceTorqueSymbolNode = field(kw_only=True)
    """Source of the symbolic force/torque vector."""

    f_desired: Vector3 = field(default=None, kw_only=True)
    """
    Desired force vector in the goal reference frame, same sign convention
    as the FT sensor reading. Defaults to the zero vector.
    """

    M: Vector3 = field(default=None, kw_only=True)
    """Virtual mass per axis in kg. Default ``(1, 1, 1)``."""

    C: Vector3 = field(default=None, kw_only=True)
    """Virtual damping per axis in N s / m. Default ``(20, 20, 20)``."""

    K: Vector3 = field(default=None, kw_only=True)
    """
    Virtual stiffness per axis in N / m. Default ``(0, 0, 0)`` (no
    spring-back). Set non-zero on rigid axes if you want the correction to
    decay back to zero when forces vanish.
    """

    threshold: float = field(default=0.01, kw_only=True)
    """Distance threshold (against nominal goal) for goal achievement, in meters."""

    reference_velocity: Optional[float] = field(
        default_factory=lambda: CartesianPosition.default_reference_velocity,
        kw_only=True,
    )
    """Reference velocity for normalization, in m/s."""

    _x_admittance: Vector3 = field(init=False, default=None)
    _xd_admittance: Vector3 = field(init=False, default=None)
    _x_idx: int = field(init=False, default=0)
    _xd_idx: int = field(init=False, default=0)

    _M_np: np.ndarray = field(init=False, default=None)
    _C_np: np.ndarray = field(init=False, default=None)
    _K_np: np.ndarray = field(init=False, default=None)
    _f_desired_np: np.ndarray = field(init=False, default=None)

    _compiled_force_in_goal: CompiledFunction = field(init=False, default=None)

    @property
    def goal_reference_frame(self) -> KinematicStructureEntity:
        return self.goal_point.reference_frame

    def build(self, context: MotionStatechartContext) -> NodeArtifacts:
        artifacts = super().build(context)
        self._resolve_defaults()
        self._rebind_ft_node()
        self._ensure_ft_symbols_registered(context)
        self._register_state_symbols(context)
        self._compile_force_in_goal_expression(context)

        corrected_goal_point = self.goal_point + self._x_admittance
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

        nominal_root_P = self.root_T_goal_reference_frame @ self.goal_point
        artifacts.observation = (
            nominal_root_P.euclidean_distance(root_P_current) < self.threshold
        )
        return artifacts

    def on_tick(
        self, context: MotionStatechartContext
    ) -> Optional[ObservationStateValues]:
        if not self.ft_node.has_msg():
            return None

        data = context.float_variable_data.data
        x_a = data[self._x_idx : self._x_idx + 3].copy()
        xd_a = data[self._xd_idx : self._xd_idx + 3].copy()

        f_in_goal = self._compiled_force_in_goal(
            context.world.state.positions, data
        )[:3]
        f_err = f_in_goal - self._f_desired_np

        # Semi-implicit Euler: update velocity first, then use it to update position.
        dt = context.qp_controller_config.control_dt
        xdd = (f_err - self._C_np * xd_a - self._K_np * x_a) / self._M_np
        xd_new = xd_a + dt * xdd
        x_new = x_a + dt * xd_new

        print(f"[adm] f_in_goal={f_in_goal} x_new={x_new}")

        context.float_variable_data.set_value(self._x_admittance, x_new)
        context.float_variable_data.set_value(self._xd_admittance, xd_new)
        return None

    def on_reset(self, context: MotionStatechartContext):
        context.float_variable_data.set_value(self._x_admittance, np.zeros(3))
        context.float_variable_data.set_value(self._xd_admittance, np.zeros(3))

    def _resolve_defaults(self):
        if self.f_desired is None:
            self.f_desired = Vector3(reference_frame=self.goal_reference_frame)
        if self.M is None:
            self.M = Vector3(x=1.0, y=1.0, z=1.0)
        if self.C is None:
            self.C = Vector3(x=20.0, y=20.0, z=20.0)
        if self.K is None:
            self.K = Vector3()
        self._f_desired_np = self.f_desired.to_np()[:3].astype(np.float64)
        self._M_np = self.M.to_np()[:3].astype(np.float64)
        self._C_np = self.C.to_np()[:3].astype(np.float64)
        self._K_np = self.K.to_np()[:3].astype(np.float64)
        if np.any(self._M_np <= 0):
            raise ValueError(
                f"Admittance mass M must be > 0 on every axis, got {self._M_np}."
            )

    def _rebind_ft_node(self):
        # `giskard.execute(msc)` round-trips the MSC through JSON, which produces
        # a detached copy of self.ft_node — its has_msg() and symbol slots are
        # never written because the MSC ticks a different instance.
        live_ft_nodes = self.motion_statechart.get_nodes_by_type(ForceTorqueSymbolNode)
        for node in live_ft_nodes:
            if node.name == self.ft_node.name:
                self.ft_node = node
                return

    def _ensure_ft_symbols_registered(self, context: MotionStatechartContext):
        # The FT node normally registers its own symbols in its build(), but
        # MotionStatechart does not guarantee build order, so do it defensively.
        if not hasattr(self.ft_node.force, hidden_index_name):
            context.float_variable_data.register_expression(self.ft_node.force)
        if not hasattr(self.ft_node.torque, hidden_index_name):
            context.float_variable_data.register_expression(self.ft_node.torque)

    def _register_state_symbols(self, context: MotionStatechartContext):
        self._x_admittance = Vector3.create_with_variables(
            f"{self.name}/x_admittance"
        )
        self._x_admittance.reference_frame = self.goal_reference_frame
        self._xd_admittance = Vector3.create_with_variables(
            f"{self.name}/xd_admittance"
        )
        self._xd_admittance.reference_frame = self.goal_reference_frame
        context.float_variable_data.register_expression(self._x_admittance)
        context.float_variable_data.register_expression(self._xd_admittance)
        self._x_idx = getattr(self._x_admittance, hidden_index_name)
        self._xd_idx = getattr(self._xd_admittance, hidden_index_name)

    def _compile_force_in_goal_expression(self, context: MotionStatechartContext):
        goal_ref_T_ft = context.world.compose_forward_kinematics_expression(
            self.goal_reference_frame, self.ft_node.reference_frame
        )
        force_in_goal_frame = (
            goal_ref_T_ft.to_rotation_matrix() @ self.ft_node.force
        )
        self._compiled_force_in_goal = force_in_goal_frame.compile(
            parameters=VariableParameters.from_lists(
                context.world.state.position_float_variables,
                context.float_variable_data.variables,
            ),
            sparse=False,
        )
