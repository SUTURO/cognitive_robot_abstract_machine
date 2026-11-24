# HSR door to sofa demo

The HSR starts at the door of the kitchen lab, drives to the sofa, looks at each of its
two seat places and reports which of them are free:

```
The left seat of the sofa is free.
```

The scene and the plan are the same however it runs; only how the plan is carried out
differs, by the coraplex plan executor it runs in: kinematically simulated in RViz,
physically simulated in MuJoCo, or on the real robot.

## Before running

Every command below runs **from inside this folder** with the workspace's virtual
environment, and with ROS sourced:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/workspace/src/cognitive_robot_abstract_machine/coraplex/demos/suturo_human_robot_interaction_challenge/milestone_0/hsr_door_to_sofa
PY=~/.virtualenvs/cram-env/bin/python
```

Do not start the scripts as `./test_find_free_seats.py`: the shebang picks the system Python, which
lacks the workspace packages.

## Running it

| Run | Execution type | What you need | What you see |
|---|---|---|---|
| `$PY find_free_seats_in_rviz.py [LEFT] [RIGHT]` | kinematically simulated | ROS 2, RViz2 | The robot moving in RViz, see below. |
| `$PY find_free_seats_in_mujoco.py [LEFT] [RIGHT]` | physically simulated | `mujoco` | A MuJoCo viewer. |
| `$PY find_free_seats_in_recording.py <recording>` | kinematically simulated, one look | ROS 2, RoboKudo, a recording | The free seats RoboKudo sees in the recording. |
| `$PY find_free_seats_on_hsr.py` | real, one look | The HSR, its ROS stack, Pumas and RoboKudo | The real robot driven by Pumas. |
| `$PY test_find_free_seats.py` | both simulations, without a viewer | `mujoco` | Exit code 0 if only the left seat is reported free with a person on the right seat. |

### People on the sofa in simulation

The seat places named on the command line of a simulation have a person sitting on them,
drawn as a red box. Name none, one or both; case does not matter:

```bash
$PY find_free_seats_in_rviz.py              # empty sofa:       The left and right seats of the sofa are free.
$PY find_free_seats_in_rviz.py right        # right seat taken: The left seat of the sofa is free.
$PY find_free_seats_in_mujoco.py LEFT RIGHT # both taken:       No seat on the sofa is free.
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

The run itself takes only a moment: the kinematic simulation puts the base straight at
each pose it drives to, so the robot jumps rather than drives. Afterwards the world stays
shown, with the robot in front of the sofa, until you press Ctrl+C; RViz may be started
before or after. To watch the robot drive, use `find_free_seats_in_mujoco.py`.

### Simulation from a recording

The recordings are `ros2 bag` folders in `~/workspace/src/kinect`, holding the HSR head
camera's color, depth and camera info topics.

1. Start RoboKudo's seat pipeline, which checks the sofa's seat places for people sitting
   on them:

   ```bash
   $PY -m robokudo.scripts.main _ae seat_place_occupancy
   ```

2. Run the demo with a recording:

   ```bash
   $PY find_free_seats_in_recording.py ~/workspace/src/kinect/07
   ```

The demo builds the kitchen under a `map` root, as the real robot's world is, serves it
to RoboKudo, and replays the recording's camera topics in a loop. The recording's own tf
is not replayed, since it puts the camera at a made-up pose (1 m above the `map` origin).
The recorded camera frame `head_rgbd_sensor_rgb_optical_frame` is fixed to the simulated
HSR's head camera instead, so RoboKudo sees the recorded images from wherever the
simulated robot looks. The plan starts only once the replay delivers images, otherwise
RoboKudo would answer from the last image of an earlier replay.

#### What the recordings give

`LEFT` and `RIGHT` are seen from someone sitting on the sofa, so the sofa's left seat is
on the right of the image.

| Recording | Scene | Said |
|---|---|---|
| 05 | empty sofa | left and right free |
| 06 | person on the right seat | left free |
| 07 | person on the left seat | right free |
| 08 | a person on each seat | none free |
| 09 | person in the middle | left and right free |
| 13A | person on the sofa, seen from the side | left and right free (wrong) |
| 13B | person on the sofa, seen from the side | left free |
| 13C | empty sofa, seen from the side | left and right free |
| 14 | person on the right seat, camera moved to the side | left and right free (wrong) |
| 15 | person standing in front of the sofa | left and right free |
| 17 | empty sofa, chair beside it | left and right free |

01 to 04, 12 and 16 show chairs only, and 00_test the lab. The simulated robot always
looks at the sofa from straight in front, about as far away as in 05 to 09, so
recordings taken from elsewhere (13A, 14) do not line up with the seat places of the
map.

### On the real robot

Never run so far. The base is driven by the lab's Pumas navigation, which plans its path
on the laser map, so the robot goes around the kitchen island by itself. Follow the
*Pumas HSR Übergabe* handover, with the map `apartment_master1` (the kitchen lab) instead
of `apartment3`, in the start scripts' `map_name:=` and as the robot's `MAP_PATH`.

1. Sync the robot clock, stop the TMC localization, start AMCL, set the start pose in
   RViz and start Pumas (`~/start_pumas_live.sh`), as the handover describes.
2. In every further terminal, `source ~/ros_clean_env.sh` first, so all processes use the
   robot's RMW, then start Giskard:
   `$PY -m giskardpy.middleware.ros2.scripts.iai_robots.hsr.hsr_velocity`
3. Start RoboKudo: `$PY -m robokudo.scripts.main _ae seat_place_occupancy`
   To add detected chairs, start `_ae free_seat_detection` instead, after putting the YOLO
   weights `chairs.pt` into `robokudo/src/robokudo/models/` (not part of the repository).
4. Put the robot at the door and run `$PY find_free_seats_on_hsr.py`.

The run has three parts:

1. Giskard parks the arm.
2. The demo sends the scanning pose to Pumas on `/move_base_simple/goal` and waits until
   tf puts `base_footprint` within 0.1 m and 0.1 rad of it, for at most 3 minutes.
3. Giskard raises the torso, the head looks at the sofa and RoboKudo checks the seats.

The sequence is `drives_with_pumas` of `SofaScanningDemonstration` in
`coraplex/src/coraplex/sofa_demonstrations.py`, which every scanning demo can switch on.
Pumas drives only between the two Giskard runs, because Giskard sends a base velocity
(zero if the base is not meant to move) on every control cycle of a run. At the end, the
sentence is printed.

The scanning pose comes from the sofa in `KitchenEnvironment`, whose frame does not line
up with the `map` frame of `apartment_master1`: the kitchen is placed in it at
the `map_T_scene` in `find_free_seats_on_hsr.py`.

## File layout

| File | Role |
|---|---|
| `free_seats_demonstration.py` | What every run shares: `DoorToSofaDemonstration`, which seats the people and scans the sofa, and `main`, which runs it and prints the free seats. |
| `coraplex/src/coraplex/sofa_demonstrations.py` | What all sofa demos share: the scene, the drive around the furniture and the physics settings. |
| `find_free_seats_in_rviz.py` | Kinematic run, published for RViz. |
| `find_free_seats_in_mujoco.py` | Physics run in MuJoCo. |
| `find_free_seats_in_recording.py`, `camera_recording.py`, `exceptions.py` | Run on a recording of the head camera. |
| `find_free_seats_on_hsr.py` | Real robot, driven by Pumas. |
| `test_find_free_seats.py` | The check CI runs, see `.github/workflows/examples_and_demos.yml`. |

## What the plan does

```
ParkArmsAction(all arms)
PathPlanningNavigateAcrossFloorsAction(1.5 m in front of the sofa, facing it)
MoveTorsoAction(HIGH)
DetectSeatsAction(left seat, right seat)
    for each seat: LookAtAction(seat), then ask for the seats in view
MoveTorsoAction(LOW)
```

The plan is the same in every run; only the executor carrying it out differs. The kitchen's
doors and drawers are fixed, and its floors do not collide: the base is always closer to
them than collision avoidance allows, and in MuJoCo the wheels would catch on the edge
between the kitchen floor and the living room floor.

The sofa is built beforehand: each of its seat places is a region above its cushion,
listed among the sofa's `seat_places` (`SofaSeat` in
`semantic_digital_twin.predefined_maps.sofa_seats`). Only with RoboKudo's
`free_seat_detection` are chairs detected too: a chair it sees that the world does not
hold yet is added as a `Chair` with one seat place.
Asking for the seats in view updates every seat that was seen: the people on a seat seen
free are removed, and someone is seated on a seat seen occupied that holds no one. In
simulation the seats and people are read from the world model, on the real robot
RoboKudo reports them; RoboKudo counts only people it sees sitting, so someone standing in
front of the sofa does not take a seat. A seat place is free when no `Human` is in it
afterwards, and the demo names the free seat places of the sofa.

## Where the numbers come from

| Value | Where | Meaning |
|---|---|---|
| `robot_start = (0.4, 0, 0)` | `sofa_demonstrations.py` | Just inside the opening in the south wall between `south_wall4` and `south_wall5`, facing into the room. |
| `clearance = 0.05` | `sofa_demonstrations.py` | How far, beyond its radius, the planned path keeps the base from obstacles. |
| `scanning_distance = 1.5` | `sofa_demonstrations.py` | How far in front of the sofa's front edge the robot stands to scan it. |
| `person_size = 0.4 × 0.4 × 0.8` | `free_seats_demonstration.py` | The box a simulated person is shaped as. |
| `scanning_distance = 1.8` | `free_seats_demonstration.py` | Puts the camera about 2.15 m from the seat cushions, the distance measured in the depth images of recordings 05 and 07. |
| `scanning_torso_state = LOW` | `free_seats_demonstration.py` | Not measured; the camera height of the recordings is unknown. |
| `position_threshold = 0.3`, `orientation_threshold = 0.1`, `timeout = 3 min` | `coraplex/src/coraplex/pumas_navigation.py` | Not measured; Pumas reports no outcome, so arriving is judged by these and by the base having stood still for `stillness_duration = 0.5 s` (within 1 cm and 0.01 rad, also not measured). The position threshold allows for Pumas stopping about 0.24 m short of the goal when an obstacle is near it. |
| `map_T_scene = (0.44, 0.14, yaw -0.058)` | `find_free_seats_on_hsr.py` | Fitted to three room corners clicked on the laser map: the far wall at the bedroom end, the kitchen wall at the fridge end and the window wall at the oven end, taking the window wall at its cable channel. The corners still differ by 7 to 15 cm after the fit, since the room is 4.72 m by 9.05 m in the laser map and 4.86 m by 8.79 m in the model. |
| `images_before_start = 30` | `camera_recording.py` | About one second of images at 30 Hz before the plan starts. |
| `chair_seat_place_size = 0.5 × 0.5 × 0.5` | `SeatPerceptionTask` | The seat place of a chair added to the world. Estimated, not measured. |
| `chair_matching_distance = 0.5` | `SeatPerceptionTask` | How close a detected chair must be to a chair of the world to be that chair. Estimated. |

The door is an assumption: the south wall has a second opening at about `y = 3.6`. If the
lab's door is that one, change `robot_start` in `sofa_demonstrations.py`.

## What has actually been run

| | Result |
|---|---|
| Kinematically simulated | A person on the right seat: `The left seat of the sofa is free.`. |
| Physically simulated (headless) | A person on the right seat: `The left seat of the sofa is free.`; the robot ends at `(3.70, 3.23)`, facing the sofa. |
| Real | **Never run**. |
