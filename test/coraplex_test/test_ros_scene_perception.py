"""
RoboKudo's complete action result is preserved as an unrestricted observation batch.
"""

from __future__ import annotations

from unittest.mock import Mock
from threading import Event, Thread
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

import pytest
from action_msgs.msg import GoalStatus
from rclpy.task import Future
from robokudo_msgs.action import Query
from robokudo_msgs.msg import ObjectDesignator, ShapeSize
from geometry_msgs.msg import PoseStamped
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table
from semantic_digital_twin.world_description.geometry import Scale

from coraplex.ros_scene_perception import RoboKudoScenePerception
from coraplex.scene_perception import SceneQuery, SceneReport
from coraplex.exceptions import ScenePerceptionFailed
from semantic_digital_twin.world import World

# %% final action results


@pytest.mark.parametrize(
    "status",
    [
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_ABORTED,
        GoalStatus.STATUS_CANCELED,
    ],
)
def test_scene_query_uses_each_label_only_after_success(
    monkeypatch: pytest.MonkeyPatch, status: int
) -> None:
    """
    An unrestricted query retains distinct categories; unsuccessful results are errors.
    """
    result = Mock(
        status=status,
        result=Query.Result(
            res=[
                ObjectDesignator(uid="one", type="cup"),
                ObjectDesignator(uid="two", type="bottle"),
            ]
        ),
    )
    result_future = Future()
    result_future.set_result(result)
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = result_future
    goal_future = Future()
    goal_future.set_result(handle)
    client = Mock()
    client.send_goal_async.return_value = goal_future
    monkeypatch.setattr(
        "coraplex.ros_scene_perception.ActionClient", Mock(return_value=client)
    )
    backend = RoboKudoScenePerception(ros_node=Mock())
    query = SceneQuery(world=World.create_with_root_body("root"))
    if status != GoalStatus.STATUS_SUCCEEDED:
        with pytest.raises(ScenePerceptionFailed):
            backend.observe(query)
    else:
        batch = backend.observe(query)
        assert [obj.category for obj in batch.objects] == ["cup", "bottle"]
        assert [obj.identity for obj in batch.objects] == ["one", "two"]
    assert client.send_goal_async.call_args.args[0].obj.type == ""
    client.destroy.assert_called_once()


def test_result_timeout_requests_cancellation(monkeypatch: pytest.MonkeyPatch) -> None:
    """
    An accepted request that never completes is canceled without a partial batch.
    """
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = Future()
    accepted = Future()
    accepted.set_result(handle)
    client = Mock()
    client.send_goal_async.return_value = accepted
    monkeypatch.setattr(
        "coraplex.ros_scene_perception.ActionClient", Mock(return_value=client)
    )
    backend = RoboKudoScenePerception(ros_node=Mock(), response_timeout=0)
    with pytest.raises(ScenePerceptionFailed):
        backend.observe(SceneQuery(world=World.create_with_root_body("root")))
    handle.cancel_goal_async.assert_called_once()
    client.destroy.assert_called_once()


def test_rejected_goal_does_not_request_a_result(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    A rejected action is not mistaken for a completed empty scene.
    """
    handle = Mock(accepted=False)
    accepted = Future()
    accepted.set_result(handle)
    client = Mock()
    client.send_goal_async.return_value = accepted
    monkeypatch.setattr(
        "coraplex.ros_scene_perception.ActionClient", Mock(return_value=client)
    )
    with pytest.raises(ScenePerceptionFailed):
        RoboKudoScenePerception(ros_node=Mock()).observe(
            SceneQuery(world=World.create_with_root_body("root"))
        )
    handle.get_result_async.assert_not_called()
    client.destroy.assert_called_once()


# %% measured geometry


def test_measured_scene_result_reaches_the_grouped_report(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    ROS labels, dimensions and reference frames reach the furniture description.
    """
    world = World.create_with_root_body("root")
    Table.create_with_new_body_in_world(
        world=world, name="arbitrary", scale=Scale(2, 2, 1)
    )
    pose = PoseStamped()
    pose.header.frame_id = world.root.name.name
    pose.pose.position.z = 0.55
    pose.pose.orientation.w = 1.0
    size = ShapeSize()
    size.dimensions.x = size.dimensions.y = size.dimensions.z = 0.1
    item = ObjectDesignator(type="cup", uid="detected", pose=[pose], shape_size=[size])
    completed = Future()
    completed.set_result(
        Mock(status=GoalStatus.STATUS_SUCCEEDED, result=Query.Result(res=[item]))
    )
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = completed
    accepted = Future()
    accepted.set_result(handle)
    client = Mock()
    client.send_goal_async.return_value = accepted
    monkeypatch.setattr(
        "coraplex.ros_scene_perception.ActionClient", Mock(return_value=client)
    )
    publisher = Mock()
    SceneReport(
        source=RoboKudoScenePerception(ros_node=Mock()), publisher=publisher
    ).announce(SceneQuery(world=world))
    assert publisher.publish.call_args.args[0].data == "I detected a cup on the table."


# %% local ROS action integration


def test_local_action_completes_before_any_report_is_published(
    rclpy_node: Node,
) -> None:
    """
    A real local action result gates speech even while its server is processing.
    """
    entered, release = Event(), Event()

    def execute(goal: ServerGoalHandle) -> Query.Result:
        """
        Hold completion until the test permits the scan to finish.
        """
        entered.set()
        release.wait(timeout=5)
        goal.succeed()
        return Query.Result(res=[ObjectDesignator(uid="local", type="cup")])

    server = ActionServer(
        rclpy_node, Query, "scene_report_test", execute_callback=execute
    )
    publisher = Mock()
    report = SceneReport(
        source=RoboKudoScenePerception(
            ros_node=rclpy_node, action_name="scene_report_test"
        ),
        publisher=publisher,
    )
    worker = Thread(
        target=report.announce,
        args=(SceneQuery(world=World.create_with_root_body("root")),),
    )
    worker.start()
    try:
        assert entered.wait(timeout=5)
        publisher.publish.assert_not_called()
    finally:
        release.set()
        worker.join(timeout=5)
        server.destroy()
    assert not worker.is_alive()
    assert [call.args[0].data for call in publisher.publish.call_args_list] == [
        "I detected a cup."
    ]
