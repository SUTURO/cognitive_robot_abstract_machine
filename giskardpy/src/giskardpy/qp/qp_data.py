from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from typing import Union

import numpy as np
import scipy.sparse as sp
from scipy.sparse import issparse
from typing_extensions import Self


@dataclass
class Conditioning:
    """
    Change the conditioning of a QP problem.
    Inherit from this to implement different strategies
    """

    C: sp.csc_matrix | None = field(default=None)
    R_eq: sp.csc_matrix | None = field(default=None)
    R_neq: sp.csc_matrix | None = field(default=None)

    def apply(self, qp_data: QPData) -> QPData:
        """
        Apply the conditioning to the QP problem data.
        """
        new_qp_data = deepcopy(qp_data)
        new_qp_data = self._apply_column_scaling(new_qp_data)
        new_qp_data = self._apply_row_scaling_eq(new_qp_data)
        new_qp_data = self._apply_row_scaling_neq(new_qp_data)
        return new_qp_data

    def _apply_column_scaling(self, qp_data: QPData) -> QPData:
        if self.C is not None:
            qp_data.quadratic_weights = self.C @ qp_data.quadratic_weights @ self.C
            qp_data.linear_weights = self.C @ qp_data.linear_weights

            # Since x = C @ x_hat, the box constraints L <= x <= U become L <= C @ x_hat <= U.
            # For a diagonal matrix C with positive entries, this is equivalent to
            # C_inv @ L <= x_hat <= C_inv @ U.
            diagonal_C = self.C.diagonal()
            diagonal_C_inv = np.zeros_like(diagonal_C)
            mask = diagonal_C != 0
            diagonal_C_inv[mask] = 1.0 / diagonal_C[mask]
            C_inv = sp.diags(diagonal_C_inv)
            qp_data.box_lower_constraints = C_inv @ qp_data.box_lower_constraints
            qp_data.box_upper_constraints = C_inv @ qp_data.box_upper_constraints
            qp_data.eq_matrix = qp_data.eq_matrix @ self.C
            if qp_data.neq_matrix.shape[0] * qp_data.neq_matrix.shape[1] != 0:
                qp_data.neq_matrix = qp_data.neq_matrix @ self.C
        return qp_data

    def _apply_row_scaling_eq(self, qp_data: QPData) -> QPData:
        if self.R_eq is not None:
            qp_data.eq_matrix = self.R_eq @ qp_data.eq_matrix
            qp_data.eq_bounds = self.R_eq @ qp_data.eq_bounds
        return qp_data

    def _apply_row_scaling_neq(self, qp_data: QPData) -> QPData:
        if self.R_neq is not None:
            qp_data.neq_matrix = self.R_neq @ qp_data.neq_matrix
            qp_data.neq_bounds = self.R_neq @ qp_data.neq_bounds
        return qp_data

    def unapply(self, xdot: np.ndarray) -> np.ndarray:
        """
        Retrieve the xdot of the original QP Problem
        """
        if self.C is None:
            return xdot
        return self.C @ xdot


@dataclass
class HessianOneConditioning(Conditioning):
    def _apply_column_scaling(self, qp_data: QPData) -> QPData:
        diagonal = 1 / np.sqrt(qp_data.quadratic_weights)
        diagonal[qp_data.quadratic_weights == 0] = 1.0
        self.C = sp.diags(diagonal, format="csc")
        return super()._apply_column_scaling(qp_data)

    def _apply_row_scaling_eq(self, qp_data: QPData) -> QPData:
        asdf = np.abs(qp_data.eq_matrix.toarray()).max(axis=1)
        asdf[asdf == 0] = 1.0
        self.R_eq = sp.diags(1 / asdf, format="csc")
        return super()._apply_row_scaling_eq(qp_data)


@dataclass
class MyConditioning(Conditioning):
    def _apply_column_scaling(self, qp_data: QPData) -> QPData:
        asdf = np.abs(qp_data.eq_matrix.toarray()).max(axis=0)
        asdf[qp_data.quadratic_weights != 0] = 1
        asdf[qp_data.quadratic_weights == 0] = 1 / asdf[qp_data.quadratic_weights == 0]
        self.C = sp.diags(asdf, format="csc")
        return super()._apply_column_scaling(qp_data)


@dataclass
class Relaxo:
    def partially_relaxed(self, relaxed_solution: np.ndarray) -> QPData:
        relaxed_qp_data = QPData(
            quadratic_weights=self.filtered.quadratic_weights.copy(),
            linear_weights=self.filtered.linear_weights,
            box_lower_constraints=self.filtered.box_lower_constraints.copy(),
            box_upper_constraints=self.filtered.box_upper_constraints.copy(),
            eq_matrix=self.filtered.eq_matrix,
            eq_bounds=self.filtered.eq_bounds,
            neq_matrix=self.filtered.neq_matrix,
            neq_lower_bounds=self.filtered.neq_lower_bounds,
            neq_upper_bounds=self.filtered.neq_upper_bounds,
        )
        lower_box_filter = relaxed_solution < self.filtered.box_lower_constraints
        upper_box_filter = relaxed_solution > self.filtered.box_upper_constraints
        relaxed_qp_data.box_lower_constraints[lower_box_filter] -= 100
        relaxed_qp_data.box_upper_constraints[upper_box_filter] += 100
        relaxed_qp_data.quadratic_weights[lower_box_filter | upper_box_filter] *= 1000

        return relaxed_qp_data

    def relaxed(self) -> QPData:
        relaxed_qp_data = QPData(
            quadratic_weights=self.filtered.quadratic_weights,
            linear_weights=self.filtered.linear_weights,
            box_lower_constraints=self.filtered.box_lower_constraints.copy(),
            box_upper_constraints=self.filtered.box_upper_constraints.copy(),
            eq_matrix=self.filtered.eq_matrix,
            eq_bounds=self.filtered.eq_bounds,
            neq_matrix=self.filtered.neq_matrix,
            neq_lower_bounds=self.filtered.neq_lower_bounds,
            neq_upper_bounds=self.filtered.neq_upper_bounds,
        )

        relaxed_qp_data.box_lower_constraints[self.num_non_constraints :] -= 100
        relaxed_qp_data.box_upper_constraints[self.num_non_constraints :] += 100

        return relaxed_qp_data


@dataclass
class QPDataFilter:
    zero_quadratic_weight_filter: np.ndarray
    bE_filter: np.ndarray
    bA_filter: np.ndarray

    def apply_filters(self, qp_data: QPData) -> QPData:
        return QPData(
            quadratic_weights=self._filter_quadratic_weights(qp_data.quadratic_weights),
            linear_weights=self._filter_linear_weights(qp_data.linear_weights),
            box_lower_constraints=self._filter_box_lower_constraints(
                qp_data.box_lower_constraints
            ),
            box_upper_constraints=self._filter_box_upper_constraints(
                qp_data.box_upper_constraints
            ),
            eq_matrix=self._filter_eq_matrix(qp_data.eq_matrix),
            eq_bounds=self._filter_eq_bounds(qp_data.eq_bounds),
            neq_matrix=self._filter_neq_matrix(qp_data.neq_matrix),
            neq_lower_bounds=self._filter_neq_lower_bounds(qp_data.neq_lower_bounds),
            neq_upper_bounds=self._filter_neq_upper_bounds(qp_data.neq_upper_bounds),
        )

    def _filter_quadratic_weights(self, quadratic_weights: np.ndarray) -> np.ndarray:
        return quadratic_weights[self.zero_quadratic_weight_filter]

    def _filter_linear_weights(self, linear_weights: np.ndarray) -> np.ndarray:
        return linear_weights[self.zero_quadratic_weight_filter]

    def _filter_box_lower_constraints(
        self, box_lower_constraints: np.ndarray
    ) -> np.ndarray:
        return box_lower_constraints[self.zero_quadratic_weight_filter]

    def _filter_box_upper_constraints(
        self, box_upper_constraints: np.ndarray
    ) -> np.ndarray:
        return box_upper_constraints[self.zero_quadratic_weight_filter]

    def _filter_eq_matrix(self, eq_matrix: sp.csc_matrix) -> sp.csc_matrix:
        if len(eq_matrix.shape) > 1 and eq_matrix.shape[0] * eq_matrix.shape[1] > 0:
            return eq_matrix[self.bE_filter, :][:, self.zero_quadratic_weight_filter]
        return eq_matrix

    def _filter_eq_bounds(self, eq_bounds: np.ndarray) -> np.ndarray:
        return eq_bounds[self.bE_filter]

    def _filter_neq_matrix(self, neq_matrix: sp.csc_matrix) -> sp.csc_matrix:
        if len(neq_matrix.shape) > 1 and neq_matrix.shape[0] * neq_matrix.shape[1] > 0:
            return neq_matrix[:, self.zero_quadratic_weight_filter][self.bA_filter, :]
        return neq_matrix

    def _filter_neq_lower_bounds(self, neq_lower_bounds: np.ndarray) -> np.ndarray:
        return neq_lower_bounds[self.bA_filter]

    def _filter_neq_upper_bounds(self, neq_upper_bounds: np.ndarray) -> np.ndarray:
        return neq_upper_bounds[self.bA_filter]


@dataclass
class ZeroWeightQPDataFilter(QPDataFilter):
    @classmethod
    def from_qp_data(
        cls,
        unfiltered_qp_data: QPData,
        num_slack_variables: int,
        num_eq_slack_variables: int,
        num_neq_slack_variables: int,
    ) -> Self:
        zero_quadratic_weight_filter: np.ndarray = (
            unfiltered_qp_data.quadratic_weights != 0
        )
        # don't filter dofs with 0 weight
        zero_quadratic_weight_filter[:-num_slack_variables] = True
        slack_part = zero_quadratic_weight_filter[
            -(num_eq_slack_variables + num_neq_slack_variables) :
        ]
        bE_part = slack_part[:num_eq_slack_variables]
        bA_part = slack_part[num_eq_slack_variables:]

        bE_filter = np.ones(unfiltered_qp_data.eq_matrix.shape[0], dtype=bool)
        bE_filter.fill(True)
        if len(bE_part) > 0:
            bE_filter[-len(bE_part) :] = bE_part

        bA_filter = np.ones(unfiltered_qp_data.neq_matrix.shape[0], dtype=bool)
        bA_filter.fill(True)
        if len(bA_part) > 0:
            bA_filter[-len(bA_part) :] = bA_part
        return cls(
            zero_quadratic_weight_filter=zero_quadratic_weight_filter,
            bE_filter=bE_filter,
            bA_filter=bA_filter,
        )


@dataclass
class QPData:
    """
    Container for a QP of the form:

    min_x 0.5 * x^T np.diag(quadratic_weights) x + linear_weights^T x
    s.t. box_lower_constraints <= x <= box_upper_constraints
         eq_matrix x = eq_bounds
         neq_lower_bounds <= neq_matrix x <= neq_upper_bounds

    .. note: matrices use sparse format
    """

    quadratic_weights: np.ndarray
    linear_weights: np.ndarray

    box_lower_constraints: np.ndarray
    box_upper_constraints: np.ndarray

    eq_matrix: sp.csc_matrix
    eq_bounds: np.ndarray

    neq_matrix: sp.csc_matrix
    neq_lower_bounds: np.ndarray
    neq_upper_bounds: np.ndarray

    @property
    def sparse_hessian(self) -> sp.csc_matrix:
        return sp.diags(self.quadratic_weights)

    @property
    def dense_hessian(self) -> np.ndarray:
        return np.diag(self.quadratic_weights)

    @property
    def dense_eq_matrix(self) -> np.ndarray:
        return self.eq_matrix.toarray()

    @property
    def dense_neq_matrix(self) -> np.ndarray:
        return self.neq_matrix.toarray()

    def pretty_print_problem(self):
        return (
            f"QPData(\n"
            f"    quadratic_weights={self._np_array_to_str(self.quadratic_weights)},\n"
            f"    linear_weights={self._np_array_to_str(self.linear_weights)},\n"
            f"    box_lower_constraints={self._np_array_to_str(self.box_lower_constraints)},\n"
            f"    box_upper_constraints={self._np_array_to_str(self.box_upper_constraints)},\n"
            f"    eq_matrix={self._sparse_matrix_to_str(self.eq_matrix)},\n"
            f"    eq_bounds={self._np_array_to_str(self.eq_bounds)},\n"
            f"    neq_matrix={self._sparse_matrix_to_str(self.neq_matrix)},\n"
            f"    neq_lower_bounds={self._np_array_to_str(self.neq_lower_bounds)},\n"
            f"    neq_upper_bounds={self._np_array_to_str(self.neq_upper_bounds)},\n"
            ")"
        )

    def _np_array_to_str(self, array: np.ndarray, dtype: str = "float") -> str:
        return f"np.array({array.tolist()}, dtype={dtype})".replace("inf", "np.inf")

    def _sparse_matrix_to_str(self, matrix: sp.csc_matrix, spaces: int = 4) -> str:
        return (
            f"sp.csc_matrix(\n"
            f"{' '*spaces}(\n"
            f"{' '*spaces}    {self._np_array_to_str(matrix.data)},\n"
            f"{' '*spaces}    {self._np_array_to_str(matrix.indices, dtype='int')},\n"
            f"{' '*spaces}    {self._np_array_to_str(matrix.indptr, dtype='int')},\n"
            f"{' '*spaces}),\n"
            f"{' '*spaces}shape={matrix.shape},\n"
            f"{' '*spaces})"
        )

    def analyze_well_posedness(self):
        """
        Analyzes the QP problem data for numerical issues and poor posing.
        Prints statistics and warnings for potentially ill-posed problems.
        """
        print("--- QP Well-Posedness Analysis ---")
        self._analyze_hessian()
        self._analyze_constraints()
        print("----------------------------------")

    def _analyze_hessian(self):
        """
        Checks the condition number of the Hessian.
        """
        if self.quadratic_weights is not None:
            max_weight = np.max(np.abs(self.quadratic_weights))
            min_weight = np.min(
                np.abs(self.quadratic_weights)[np.abs(self.quadratic_weights) > 0]
            )
            condition_number = max_weight / min_weight
            print(f"  Weight Matrix max singular value: {max_weight}")
            print(f"  Weight Matrix min singular value: {min_weight}")
            print(f"  Weight Matrix Condition Number: {condition_number}")
            if condition_number > 1_000:
                print("  Warning: Weight Matrix is poorly conditioned.")

    def _analyze_constraints(self):
        """
        Checks for scale imbalances and potential rank issues in constraints.
        """
        self._check_matrix_condition(self.eq_matrix, "Equality Constraint Matrix (E)")
        self._check_matrix_condition(
            self.neq_matrix, "Inequality Constraint Matrix (A)"
        )

        # Simple infeasibility check for box constraints
        if (
            self.box_lower_constraints is not None
            and self.box_upper_constraints is not None
        ):
            violations = self.box_lower_constraints > self.box_upper_constraints
            if np.any(violations):
                print(
                    f"  WARNING: Box constraints are infeasible for indices {np.where(violations)[0]}."
                )

    def _check_matrix_condition(
        self, matrix: Union[sp.csc_matrix, np.ndarray], name: str
    ):
        if issparse(matrix):
            matrix = matrix.toarray()
        if matrix.shape[0] * matrix.shape[1] == 0:
            print(f"  {name} is empty.")
            return
        singular_value_decomposition = np.linalg.svd(matrix, compute_uv=False)
        condition_number = (
            singular_value_decomposition[0] / singular_value_decomposition[-1]
        )
        print(f"  {name} max singular value: {singular_value_decomposition[0]}")
        print(f"  {name} min singular value: {singular_value_decomposition[-1]}")
        print(f"  {name} Condition Number: {condition_number}")
        if condition_number > 1_000:
            print(f"        WARNING: this is very large.")
