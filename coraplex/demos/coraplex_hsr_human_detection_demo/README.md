# Perceiving through a simulated camera

This tutorial shows how to give a demonstration a camera in MuJoCo and let RoboKudo answer
questions about what the camera sees. The camera is published on the topics of the real
camera driver, so RoboKudo runs exactly as on the real robot.

## 1. Add the perception to your demonstration

A demonstration needs three additions: it runs in physics, because only MuJoCo renders the
camera, it holds a simulated perception, and its plan asks where the robot should look.

```python
from dataclasses import dataclass, field

from coraplex.demonstrations import RobotDemonstration
from coraplex.perception import RoboKudoQuery, RoboKudoQueryTask
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import PhysicallySimulatedPlanExecutor, PlanExecutor
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.statechart import Statechart
from robokudo.io.simulated_camera import RoboKudoSimulatedPerception
from semantic_digital_twin.adapters.multi_sim import SimulatedPerception
from semantic_digital_twin.semantic_annotations.semantic_annotations import Human


@dataclass
class MyDemonstration(RobotDemonstration):
    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor
    simulated_perception: SimulatedPerception | None = field(
        default_factory=RoboKudoSimulatedPerception
    )

    def build_statechart(self, context: StatechartContext) -> Statechart:
        robot = context.require_extension(extension_type=RobotAccess).robot
        statechart = Statechart(context=context)
        statechart.add_node(
            node=Sequence(
                nodes=[
                    ParkArmsAction(arms=robot.all_arms),
                    RoboKudoQueryTask(
                        query=RoboKudoQuery.from_semantic_annotation(Human)
                    ),
                ]
            )
        )
        return statechart
```

The scene, the robot and the context extensions stay as in any `RobotDemonstration`.
Without arguments, `RoboKudoSimulatedPerception` renders the RGB-D head camera of the HSR,
publishes it on the topics of the real HSR and runs RoboKudo's human detection.

`RobotDemonstration` does the rest: it mounts the camera before the simulation is built,
starts RoboKudo once the simulation runs, sends one frame right before each query and stops
RoboKudo when the run ends, even when the plan fails. `RoboKudoQueryTask` keeps what RoboKudo
reported in `answer`, every position in `map`, and fails when RoboKudo sees nothing. The human detection also answers
for a posture, such as
`RoboKudoQuery(object_type=QueryType.HUMAN, attributes=[HumanPosture.STANDING])`.

To see the answer the way the query client of RoboKudo prints it, add a step after the task:
`FunctionCall(function=lambda: print(PrettyResultPrinter().pretty_print_result(task.answer)))`,
with `FunctionCall` from `cramph.threaded_nodes` and `PrettyResultPrinter` from
`robokudo.scripts.query_test_client`.

## 2. Choose the camera and the analysis engine

Each default can be replaced on its own: `other_robot` sees through the camera of another
robot on the same topics, `without_windows` runs RoboKudo without its windows, and `gpsr`
runs GPSR, which needs its models and longer to start.

```python
from datetime import timedelta

from robokudo.analysis_engine_process import AnalysisEngineModule, AnalysisEngineProcess
from robokudo.io.simulated_camera import RoboKudoSimulatedPerception
from semantic_digital_twin.robots.pr2 import PR2KinectV1

other_robot = RoboKudoSimulatedPerception(camera_type=PR2KinectV1)

without_windows = RoboKudoSimulatedPerception(
    analysis_engine=AnalysisEngineProcess(
        AnalysisEngineModule.HUMAN_DETECTION, shows_views=False
    )
)

gpsr = RoboKudoSimulatedPerception(
    analysis_engine=AnalysisEngineProcess(
        AnalysisEngineModule.GPSR, startup_timeout=timedelta(minutes=15)
    )
)
```

Keep the default topics: both analysis engines read their camera there, whatever robot it
belongs to. The camera renders on the HSR, the PR2, `TiagoMujoco` (with
`TiagoMujocoCamera`), the MMP Dresden, Garmi, Tracy, iCub 3, Justin and the Unitree G1;
RoboKudo's answers were checked with the HSR.

## In this demo

`demo.py` stands a human 3.5 m to the left of the HSR. The HSR parks its arm, turns towards
the human and asks for a human, then prints everything RoboKudo found, as the query client
of RoboKudo does:

```
Formatted Result:
{
  "res": [
    {
      "type": "person",
      "pose": [{"header": {"frame_id": "map"},
                "pose": {"position": {"x": -0.01, "y": 3.39, "z": 0.91}, "orientation": ...}}],
      "pose_source": ["ClusterPositionAnnotator"],
      ...
```

`map` is the origin of the simulated world, so the person is found 3.39 m to the left of
where the robot started, at the front of the body.

Instead of a default in the class, `main` hands the perception in, as
`simulated_perception=RoboKudoSimulatedPerception() if perceives else None`, so the demo can
also run without RoboKudo. Run it from this folder:

| Run | What you see |
|---|---|
| `python demo.py` | The MuJoCo viewer and the RoboKudo windows. |
| `python test_demo.py` | Only the motion, without a viewer and without RoboKudo, as in CI. |

The human detection only needs the YOLO pose model, which it downloads on its first run. It
needs no GPSR model and nothing from `robokudo_suturo`.
