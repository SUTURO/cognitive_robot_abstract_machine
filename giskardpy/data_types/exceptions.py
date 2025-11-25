from dataclasses import dataclass


class DontPrintStackTrace:
    pass


class GiskardException(Exception):
    pass


class SetupException(GiskardException):
    pass


class DuplicateNameException(GiskardException):
    pass


# %% solver exceptions
class QPSolverException(GiskardException):
    pass


class InfeasibleException(QPSolverException):
    pass


class VelocityLimitUnreachableException(QPSolverException):
    pass


class OutOfJointLimitsException(InfeasibleException):
    pass


class HardConstraintsViolatedException(InfeasibleException):
    pass


class EmptyProblemException(InfeasibleException, DontPrintStackTrace):
    def __init__(self):
        super().__init__("Empty QP problem.")


# %% world exceptions
class WorldException(GiskardException):
    pass


class UnknownGroupException(WorldException, KeyError):
    pass


class UnknownLinkException(WorldException, KeyError):
    pass


class UnknownJointException(WorldException, KeyError):
    pass


class InvalidWorldOperationException(WorldException, KeyError):
    pass


class CorruptShapeException(WorldException):
    pass


class CorruptMeshException(CorruptShapeException):
    pass


class CorruptURDFException(CorruptShapeException):
    pass


class TransformException(WorldException):
    pass


# %% error during motion problem building phase
class MotionBuildingException(GiskardException):
    pass


class InvalidGoalException(MotionBuildingException):
    pass


class UnknownGoalException(MotionBuildingException, KeyError):
    pass


class GoalInitalizationException(MotionBuildingException):
    pass


class UnknownMonitorException(MotionBuildingException, KeyError):
    pass


class MonitorInitalizationException(MotionBuildingException):
    pass


class UnknownTaskException(MotionBuildingException, KeyError):
    pass


class TaskInitalizationException(MotionBuildingException):
    pass


class NoQPControllerConfigException(MotionBuildingException):
    def __init__(
        self,
        message: str = "Motion Statechart has constraints, no QP controller config is provided.",
    ):
        super().__init__(message)


# %% errors during planning
class PlanningException(GiskardException):
    pass


class ShakingException(PlanningException):
    pass


class LocalMinimumException(PlanningException):
    pass


class MaxTrajectoryLengthException(PlanningException):
    pass


class SelfCollisionViolatedException(PlanningException):
    pass


# errors during execution
class ExecutionException(GiskardException):
    pass


class FollowJointTrajectory_INVALID_GOAL(ExecutionException):
    pass


class FollowJointTrajectory_INVALID_JOINTS(ExecutionException):
    pass


class FollowJointTrajectory_OLD_HEADER_TIMESTAMP(ExecutionException):
    pass


class FollowJointTrajectory_PATH_TOLERANCE_VIOLATED(ExecutionException):
    pass


class FollowJointTrajectory_GOAL_TOLERANCE_VIOLATED(ExecutionException):
    pass


@dataclass
class ExecutionCanceledException(ExecutionException):
    action_server_name: str
    goal_id: int

    def __post_init__(self):
        super().__init__(f"'{self.action_server_name}' goal #{self.goal_id} canceled")


class ExecutionPreemptedException(ExecutionException):
    pass


class ExecutionTimeoutException(ExecutionException):
    pass


class ExecutionAbortedException(ExecutionException):
    def __init__(self):
        super().__init__("Execution aborted by Giskard.")


class ExecutionSucceededPrematurely(ExecutionException):
    pass


# %% behavior tree exceptions
class BehaviorTreeException(GiskardException):
    pass
