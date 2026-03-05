from __future__ import division

from dataclasses import dataclass, field

from giskardpy.motion_statechart.context import BuildContext
from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.graph_node import Task, NodeArtifacts
from semantic_digital_twin.spatial_types import Point3, Vector3
from semantic_digital_twin.world_description.world_entity import KinematicStructureEntity


@dataclass(eq=False, repr=False)
class GraspBar(Task):
    """
    Like a CartesianPose but with more freedom.
    tip_link is allowed to be at any point along bar_axis within bar_center +/- bar_length/2.
    Aligns tip_grasp_axis with bar_axis but allows rotation around it.
    """

    root_link: KinematicStructureEntity = field(kw_only=True)
    tip_link: KinematicStructureEntity = field(kw_only=True)
    tip_grasp_axis: Vector3 = field(kw_only=True)
    bar_center: Point3 = field(kw_only=True)
    bar_axis: Vector3 = field(kw_only=True)
    bar_length: float = field(kw_only=True)
    threshold: float = field(default=0.01, kw_only=True)
    reference_linear_velocity: float = field(default=0.1, kw_only=True)
    reference_angular_velocity: float = field(default=0.5, kw_only=True)
    weight: float = field(default=DefaultWeights.WEIGHT_ABOVE_CA, kw_only=True)

    def build(self, context: BuildContext) -> NodeArtifacts:
        artifacts = NodeArtifacts()

        root_P_bar_center = context.world.transform(
            target_frame=self.root_link, spatial_object=self.bar_center
        )
        root_V_bar_axis = context.world.transform(
            target_frame=self.root_link, spatial_object=self.bar_axis
        )
        root_V_bar_axis.scale(1)

        tip_V_tip_grasp_axis = context.world.transform(
            target_frame=self.tip_link, spatial_object=self.tip_grasp_axis
        )
        tip_V_tip_grasp_axis.scale(1)

        root_T_tip = context.world.compose_forward_kinematics_expression(
            self.root_link, self.tip_link
        )
        root_V_tip_grasp_axis = root_T_tip @ tip_V_tip_grasp_axis

        artifacts.constraints.add_vector_goal_constraints(
            frame_V_current=root_V_tip_grasp_axis,
            frame_V_goal=root_V_bar_axis,
            reference_velocity=self.reference_angular_velocity,
            weight=self.weight,
        )

        root_P_tip = root_T_tip.to_position()
        root_P_line_start = root_P_bar_center + root_V_bar_axis * (self.bar_length / 2)
        root_P_line_end = root_P_bar_center - root_V_bar_axis * (self.bar_length / 2)

        dist, nearest = root_P_tip.distance_to_line_segment(root_P_line_start, root_P_line_end)

        artifacts.constraints.add_point_goal_constraints(
            frame_P_current=root_P_tip,
            frame_P_goal=nearest,
            reference_velocity=self.reference_linear_velocity,
            weight=self.weight,
        )

        artifacts.observation = dist <= self.threshold
        return artifacts
