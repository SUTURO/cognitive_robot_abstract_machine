# HSR door to sofa demo

The HSR starts at the door of the kitchen lab, drives to the sofa, looks at each of its
two seat places and reports how many of them are free:

```
1 of 2 seats are free
```

The scene and the plan are the same however it runs; only how the plan is carried out
differs, by the coraplex plan executor it runs in: kinematically simulated in RViz,
physically simulated in MuJoCo, or on the real robot.

This demo builds on the free seats demo in `../coraplex_hsr_free_seats_demo`:
`DoorToSofaDemonstration` inherits `FreeSeatsDemonstration` and only changes where the
robot starts and what is printed at the end.

## Before running

Every command below runs **from inside this folder** with the workspace's virtual
environment, and with ROS sourced:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/workspace/src/cognitive_robot_abstract_machine/coraplex/demos/coraplex_hsr_door_to_sofa_demo
PY=~/.virtualenvs/cram-env/bin/python
```

Do not start the scripts as `./test_demo.py`: the shebang picks the system Python, which
lacks the workspace packages.

## Running it

| Run | Execution type | What you need | What you see |
|---|---|---|---|
| `$PY rviz_demo.py [LEFT] [RIGHT]` | kinematically simulated | ROS 2, RViz2 | The robot moving in RViz, see below. |
| `$PY mujoco_demo.py [LEFT] [RIGHT]` | physically simulated | `mujoco` | A MuJoCo viewer. |
| `$PY real_demo.py` | real | The HSR, its ROS stack and RoboKudo | The real robot driving to the sofa. |
| `$PY test_demo.py` | both simulations, without a viewer | `mujoco` | Exit code 0 if exactly one seat is reported free with a person on the right seat. |

### People on the sofa in simulation

The seat places named on the command line of a simulation have a person sitting on them,
drawn as a red box. Name none, one or both; case does not matter:

```bash
$PY rviz_demo.py              # empty sofa:       2 of 2 seats are free
$PY rviz_demo.py right        # right seat taken: 1 of 2 seats are free
$PY mujoco_demo.py LEFT RIGHT # both taken:       0 of 2 seats are free
```

`LEFT` and `RIGHT` are seen from someone sitting on the sofa.

### Watching it in RViz

Start RViz in a second terminal:

```bash
rviz2
```

- set **Fixed Frame** to `root`;
- add a **MarkerArray** display on the topic `/semworld/viz_marker`;
- set that display's **Durability** to `Transient Local`, otherwise the markers do not
  arrive.

### On the real robot

1. Bring up the HSR's ROS stack, including the world server the demo fetches its world
   from.
2. Start the perception pipeline that finds the people on the seats:

   ```bash
   python3 -m robokudo.scripts.main _ae hsr_free_seats
   ```

3. Put the robot at the door of the kitchen lab. The real robot starts wherever it
   stands; the start pose of the simulations is not used.
4. Run `$PY real_demo.py`.

## File layout

| File | Role |
|---|---|
| `demo.py` | What every run shares: `DoorToSofaDemonstration` with the start pose at the door, and `main`, which runs it and prints the number of free seats. |
| `rviz_demo.py` | Kinematic run, published for RViz. |
| `mujoco_demo.py` | Physics run in MuJoCo. |
| `real_demo.py` | Real robot. |
| `test_demo.py` | The check CI runs, see `.github/workflows/examples_and_demos.yml`. |

## What the plan does

```
MoveTorsoAction(HIGH)
ParkArmsAction(all arms)
NavigateAction(1.5 m in front of the sofa, facing it)
DetectOccupantsAction(left seat, right seat)
    for each seat: LookAtAction(seat), then ask who sits there
```

Each seat place is a region above its cushion of the kitchen's sofa
(`SofaSeat` in `semantic_digital_twin.predefined_maps.sofa_seats`). Asking who sits on a
seat replaces the people the world held there with the ones that were seen: in
simulation they are read from the world model, on the real robot RoboKudo reports them;
RoboKudo reports only people it sees sitting, so someone standing in front of the sofa
does not take a seat. A seat is free when no `Human` is in its region afterwards.

## Where the numbers come from

| Value | Where | Meaning |
|---|---|---|
| `robot_start = (0.4, 0, 0)` | `demo.py` | Just inside the opening in the south wall between `south_wall4` and `south_wall5`, facing into the room. |
| `scanning_distance = 1.5` | free seats demo | How far in front of the sofa's front edge the robot stands to scan it. |
| `person_size = 0.4 × 0.4 × 0.8` | free seats demo | The box a simulated person is shaped as. |

The door is an assumption: the south wall has a second opening at about `y = 3.6`. If the
lab's door is that one, change `robot_start` in `demo.py`.

## What has actually been run

| | Result |
|---|---|
| Kinematically simulated | A person on the right seat: `1 of 2 seats are free`. |
| Physically simulated (headless) | A person on the right seat: `1 of 2 seats are free`; the robot ends at `(3.70, 3.23)`, facing the sofa. |
| Real | **Never run.** |
