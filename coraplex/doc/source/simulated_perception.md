# Simulated Perception

{class}`coraplex.simulated_perception.SimulatedHsrHeadCameraPerception` lets a demonstration ask a
[RoboKudo](https://robokudo.ai.uni-bremen.de/) analysis engine what the HSR sees, while the HSR only exists in a
MuJoCo simulation. It publishes the simulated head camera on the same ROS topics and TF frames as the real HSR,
starts the analysis engine in its own process, and hands the plan what it needs to capture a frame before each
query.

The GPSR demonstration in `challenges/src/challenges/GPSR/demonstration.py` uses it. The steps below
add it to a new {class}`~coraplex.demonstrations.RobotDemonstration`.

## Requirements

- The robot is the HSR (`used_robot = HSRB`), put into the scene by its `RobotSpecification`, so the world holds its
  `HSRBHeadRGBDCamera`.
- The plan runs physically simulated (`executor_type` is a `PhysicallySimulatedPlanExecutor` or a subclass). Only then
  is there a MuJoCo simulation to take camera images from.
- RoboKudo, `robokudo_msgs` and the analysis engine you want to run are installed. coraplex lists RoboKudo as the
  optional dependency `simulated-perception`:

  ```bash
  pip install -e "coraplex[simulated-perception]"
  ```

## Adding it to a demonstration

### 1. Add the perception and the query as fields

```python
from coraplex.perception import RoboKudoQuery
from coraplex.simulated_perception import SimulatedHsrHeadCameraPerception


@dataclass
class MyDemonstration(RobotDemonstration):
    used_robot: type[AbstractRobot] = HSRB
    executor_type: type[PlanExecutor] = PhysicallySimulatedPlanExecutor

    perception: SimulatedHsrHeadCameraPerception | None = None
    """The perception the plan asks, or ``None`` to run without perception."""

    query: RoboKudoQuery = field(
        default_factory=lambda: RoboKudoQuery(object_type="human")
    )
    """What the plan asks :attr:`perception`."""
```

Leaving `perception` at `None` runs the same demonstration without RoboKudo, for example in tests.

### 2. Mount the head camera before the simulation is built

The camera has to be mounted on the head before the MuJoCo simulation is built, so mount it in `build_simulation`:

```python
from semantic_digital_twin.adapters.multi_sim import MujocoCamera, MujocoSim


    def build_simulation(self, world: World) -> MujocoSim:
        MujocoCamera.mount_hsr_head(world)
        return super().build_simulation(world)
```

If your demonstration builds its own simulation instead of calling `super()`, as the GPSR demonstration does, call
`MujocoCamera.mount_hsr_head(world)` there before constructing it.

### 3. Start the perception once the simulation runs

```python
    def start_simulation(self, world: World) -> None:
        super().start_simulation(world)
        if self.perception is None or self.simulation is None:
            return
        self.perception.start(world, self.simulation, self.ros_node)
```

`start` blocks until the analysis engine runs, for at most two minutes.

### 4. Give the plan the frame capture

```python
from cramph.context import ContextExtension
from coraplex.plans.context_extensions import RobotAccess


    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        extensions: list[ContextExtension] = [
            RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])
        ]
        if self.perception is not None:
            extensions.extend(self.perception.build_context_extensions(world))
        return extensions
```

This adds two extensions: `CameraFrameCapture`, without which a query is sent but no new camera image is published
for it, and `PerceptionWorldFrame`, which tells the plan that the world frame RoboKudo reports poses in (`map` for the
HSR configuration) is the root of the simulated world.

### 5. Ask the query in the plan

`RoboKudoQueryTask` publishes a camera frame, sends the query and observes `TRUE` if RoboKudo found something. Turn
the robot towards what it should see first:

```python
from coraplex.perception import RoboKudoQueryTask


    def build_statechart(self, context: StatechartContext) -> Statechart:
        robot = context.require_extension(RobotAccess).robot
        target = context.world.get_body_by_name("human")
        steps: list[StatechartNode] = [
            ParkArmsAction(arms=robot.all_arms),
            FaceAtAction(target=Pose.from_xyz_rpy(reference_frame=target)),
        ]
        if self.perception is not None:
            steps.append(RoboKudoQueryTask(query=self.query))
        statechart = Statechart(context=context)
        statechart.add_node(Sequence(nodes=steps))
        return statechart
```

After the run, the task's `answer` holds everything RoboKudo reported.

### 6. Stop the perception when tearing down

```python
    def tear_down(self) -> None:
        if self.perception is not None:
            self.perception.stop()
        super().tear_down()
```

`stop` also removes the camera's publishers from the node. It does nothing if the perception was never started, so it is safe to call after a failed run.

### 7. Choose the analysis engine when running the demonstration

```python
from robokudo.analysis_engine_process import AnalysisEngineProcess


def main(headless: bool = False, perceives: bool = True) -> World:
    return MyDemonstration(
        headless=headless,
        perception=(
            SimulatedHsrHeadCameraPerception(
                AnalysisEngineProcess(
                    analysis_engine="human_detection", shows_views=False
                )
            )
            if perceives
            else None
        ),
    ).run()
```

- `analysis_engine` is the module below `descriptors/analysis_engines` of the RoboKudo package, for example
  `human_detection`. Which one to pick depends on what the plan asks, see
  [Querying humans and objects](#querying-humans-and-objects).
- `ros_package` is the ROS package that holds it, `robokudo` by default.
- `shows_views=False` keeps RoboKudo's windows closed.
- `camera_configuration` of `SimulatedHsrHeadCameraPerception` changes the topics and frames. By default they are
  those of the real HSR.

## Querying humans and objects

`SimulatedHsrHeadCameraPerception` does not care what is asked: it only publishes the camera and runs the engine. What
can be asked is decided by the analysis engine, and what is asked by the `RoboKudoQuery` each `RoboKudoQueryTask`
carries.

| Analysis engine | Answers |
|---|---|
| `human_detection` | Only queries with `object_type="human"`, optionally with one posture in `attributes`. Every other query gets no answer. |
| `gpsr.gpsr_implementation` | Humans (posture, waving, pointing, in a room, near furniture, by clothing) and objects (all objects, by attribute, in a room), faces and doors. |

The `object_type` values the GPSR engine understands are the members of
`robokudo.annotators.gpsr.gpsr_results.QueryType`, so build queries from that enum instead of typing the strings:

```python
from robokudo.annotators.gpsr.gpsr_results import QueryType

waving_person = RoboKudoQuery(object_type=QueryType.HUMAN, attributes=["waving"])
every_object = RoboKudoQuery(object_type=QueryType.OBJECT)
objects_in_kitchen = RoboKudoQuery(object_type=QueryType.OBJECT, location="kitchen")
```

To ask for humans and objects in one demonstration, run the GPSR engine and let each action build the query for what
it is looking for, instead of giving the demonstration one fixed `query` field.

### Looking for one kind of object or for every object

An object query without anything else is answered with every object the pipeline detects. Naming a semantic type in
`descriptions` makes the GPSR engine keep only the objects of that type:

```python
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk

every_object = RoboKudoQueryTask(query=RoboKudoQuery(object_type=QueryType.OBJECT))
only_milk = RoboKudoQueryTask(
    query=RoboKudoQuery(object_type=QueryType.OBJECT, descriptions=[Milk.__name__])
)
```

Both tasks observe `TRUE` if the pipeline reported anything and `FALSE` otherwise, so a plan can react to not finding
the milk, for example inside a `TryInOrder`.

### Reading what was perceived

Once the task answered, `perceived_objects` holds a `PerceivedObject` for every reported object that has a pose: its
`object_type` as the pipeline named it, and its `pose` in the frame the pipeline reported it in. Objects reported
without a pose are left out there, but stay in the raw `answer`.

```python
for perceived_object in every_object.perceived_objects:
    world_pose = world.transform(perceived_object.pose, world.root)
    print(perceived_object.object_type, world_pose.position)
```

The task forgets what it perceived when it is started again, so copy `perceived_objects` if a later step needs them
after the task ran a second time.

```{note}
The object pipelines of the GPSR engine detect with a trained model (`ultimate_model_v11_1.26.4`) and map its labels
with a file from the `robokudo_suturo` package. Both have to be installed, otherwise object queries are not answered.
```

## Troubleshooting

| Symptom | Cause |
|---|---|
| `NoSingleHsrHeadCamera` in `start` | The world does not hold exactly one `HSRBHeadRGBDCamera`. Put one HSR into the scene through `RobotSpecification(HSRB)`, not by loading its URDF directly. |
| `ValueError: not enough values to unpack (expected 1, got 0)` in `mount_hsr_head` | The same cause, found while mounting the camera, before the simulation is built. |
| `SimulatedPerceptionAlreadyStarted` | `start` ran a second time without `stop` in between. Stop the perception first; a second start would run a second analysis engine. |
| `SimulatedPerceptionNotStarted` | `build_context_extensions` ran before `start`, usually because the executor is not physically simulated, so `start_simulation` never started the perception. |
| `RoboKudoQueryTask` fails while it is ticked, looking up a frame | The pipeline reported a pose in a frame the world has no body or frame of that name for. Compare the reported `header.frame_id` in `answer` with the world's frame names. |
| `AnalysisEngineDidNotStart` | The analysis engine did not announce that it runs within the timeout. Check its output, which is forwarded to the demonstration's output, for an import error or a wrong `analysis_engine` name. |
