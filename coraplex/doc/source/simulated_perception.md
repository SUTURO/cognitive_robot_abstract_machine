# Simulated Perception

A {class}`~coraplex.demonstrations.RobotDemonstration` asks a [RoboKudo](https://robokudo.ai.uni-bremen.de/)
analysis engine what the HSR sees, also while the HSR only exists in a MuJoCo simulation, through its
`simulated_perception` field. {class}`robokudo.io.simulated_camera.RoboKudoSimulatedPerception` publishes the
simulated head camera on the same ROS topics and TF frames as the real HSR and runs the analysis engine in its own
process. How to add it to a demonstration is described in
`coraplex/demos/coraplex_hsr_human_detection_demo/README.md`.

## The GPSR demonstration

The GPSR demonstration in `coraplex/demos/coraplex_gpsr_demo/demo.py` takes the analysis engine as its
`analysis_engine` field and looks for people with it once it faces them:

- In a physically simulated run, the engine runs inside a `RoboKudoSimulatedPerception`, which the demonstration
  sets as its `simulated_perception`. `RobotDemonstration` then mounts the head camera, starts and stops the engine,
  and captures a frame before each query.
- On the real robot (`executor_type=RealRobotPlanExecutor`), the engine is started on its own and looks through the
  real head camera.
- Without an `analysis_engine`, the robot drives in front of people without asking RoboKudo.

```python
from robokudo.analysis_engine_process import AnalysisEngineModule, AnalysisEngineProcess

GeneratedCommandDemonstration(
    json_plan=json_plan,
    instruction=instruction,
    analysis_engine=AnalysisEngineProcess(
        analysis_engine=AnalysisEngineModule.HUMAN_DETECTION, shows_views=False
    ),
).run()
```

## Querying humans and objects

`RoboKudoSimulatedPerception` does not care what is asked: it only publishes the camera and runs the engine. What
can be asked is decided by the analysis engine, and what is asked by the `RoboKudoQuery` each `RoboKudoQueryTask`
carries.

| Analysis engine | Answers |
|---|---|
| `AnalysisEngineModule.HUMAN_DETECTION` | Only queries with `object_type="human"`, optionally with one posture in `attributes`. Every other query gets no answer. |
| `AnalysisEngineModule.GPSR` | Humans (posture, waving, pointing, in a room, near furniture, by clothing) and objects (all objects, by attribute, in a room), faces and doors. |

The `object_type` values the GPSR engine understands are the members of
`robokudo.annotators.gpsr.query_answers.QueryType`, so build queries from that enum instead of typing the strings:

```python
from coraplex.perception import RoboKudoQuery
from robokudo.annotators.gpsr.query_answers import QueryType

waving_person = RoboKudoQuery(object_type=QueryType.HUMAN, attributes=["waving"])
every_object = RoboKudoQuery(object_type=QueryType.OBJECT)
objects_in_kitchen = RoboKudoQuery(object_type=QueryType.OBJECT, location="kitchen")
```

An object query without anything else is answered with every object the pipeline detects. Naming a semantic type in
`descriptions` makes the GPSR engine keep only the objects of that type:

```python
from coraplex.perception import RoboKudoQueryTask
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk

only_milk = RoboKudoQueryTask(
    query=RoboKudoQuery(object_type=QueryType.OBJECT, descriptions=[Milk.__name__])
)
```

The task observes `TRUE` if the pipeline reported anything and `FALSE` otherwise, so a plan can react to not finding
the milk, for example inside a `TryInOrder`. Once it answered, its `answer` holds the object designators the pipeline
reported.

```{note}
The object pipelines of the GPSR engine detect with a trained model (`ultimate_model_v11_1.26.4`) and map its labels
with a file from the `robokudo_suturo` package. Both have to be installed, otherwise object queries are not answered.
```

## Troubleshooting

| Symptom | Cause |
|---|---|
| `CameraTypeNotInWorld` or `CameraAnnotationAmbiguous` | The world does not hold exactly one camera of the perception's `camera_type`, `HSRBHeadRGBDCamera` by default. Put one HSR into the scene through `RobotSpecification(HSRB)`, not by loading its URDF directly. |
| `PerceptionWithoutPhysicalSimulation` | A `simulated_perception` is set for a run that is not physically simulated, so there is no camera to render. |
| `AnalysisEngineDidNotStart` | The analysis engine did not announce that it runs within its `startup_timeout`. Check its output, which is forwarded to the demonstration's output, for an import error or a wrong `analysis_engine` name. |
