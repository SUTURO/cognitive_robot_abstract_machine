# HSR kitchen to sofa demo

The HSR starts in the kitchen of the kitchen lab, drives to the sofa, looks at it and
says which of its two seat places are free:

```
The right seat of the sofa is free.
```

| Run | Robot | What sees the sofa | Output |
|---|---|---|---|
| `rosbag_demo.py <recording>` | kinematically simulated | RoboKudo, on a recording of the real HSR's head camera | printed |
| `real_demo.py` | real HSR, driven by Pumas | RoboKudo, on the live head camera | printed and spoken by the HSR |
| `test_demo.py` | kinematically simulated | the world model (CI has no RoboKudo or recordings) | exit code |

## Before running

Every command runs from inside this folder, with ROS sourced and the workspace's
virtual environment:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/workspace/src/cognitive_robot_abstract_machine/coraplex/demos/coraplex_hsr_kitchen_to_sofa_demo
PY=~/.virtualenvs/cram-env/bin/python
```

## Simulation from a recording

The recordings are `ros2 bag` folders in `~/workspace/src/kinect`, holding the HSR head
camera's color, depth and camera info topics.

1. Start RoboKudo's seat pipeline, which checks the sofa's seat places for people sitting
   on them:

   ```bash
   $PY -m robokudo.scripts.main _ae seat_place_occupancy
   ```

2. Run the demo with a recording:

   ```bash
   $PY rosbag_demo.py ~/workspace/src/kinect/07
   ```

The demo builds the kitchen under a `map` root, as the real robot's world is, serves it
to RoboKudo, and replays the recording's camera topics in a loop. The recording's own tf
is not replayed, since it puts the camera at a made-up pose (1 m above the `map` origin).
The recorded camera frame `head_rgbd_sensor_rgb_optical_frame` is fixed to the simulated
HSR's head camera instead, so RoboKudo sees the recorded images from wherever the
simulated robot looks. The plan starts only once the replay delivers images, otherwise
RoboKudo would answer from the last image of an earlier replay.

### What the recordings give

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

## On the real robot

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
4. Put the robot in the kitchen and run `$PY real_demo.py`.

The run has three parts:

1. Giskard parks the arm.
2. The demo sends the scanning pose to Pumas on `/move_base_simple/goal` and waits until
   tf puts `base_footprint` within 0.1 m and 0.1 rad of it, for at most 3 minutes.
3. Giskard raises the torso, the head looks at the sofa and RoboKudo checks the seats.

Pumas drives only between the two Giskard runs, because Giskard sends a base velocity
(zero if the base is not meant to move) on every control cycle of a run. At the end, the
sentence is printed and sent to the HSR's voice through `RobotSpeech`
(`coraplex.scene_report_command`).

The scanning pose comes from the sofa in `KitchenEnvironment`, whose frame does not line
up with the `map` frame of `apartment_master1`: the kitchen is placed in it at
`map_T_scene` of `PumasKitchenToSofaDemonstration`.

## What the plan does

In simulation:

```
ParkArmsAction(all arms)
NavigateAction(1.8 m in front of the sofa, facing it)
MoveTorsoAction(LOW)
DetectSeatsAction(the middle of the two seat places)
MoveTorsoAction(LOW)
```

It looks once at the middle of the sofa, because the recordings show both seats in one
view.

## Where the numbers come from

| Value | Where | Meaning |
|---|---|---|
| `robot_start = (2.3, -1.0)`, facing +y | `demo.py` | In the kitchen, between the counter and the kitchen island. |
| `scanning_distance = 1.8` | `demo.py` | Puts the camera about 2.15 m from the seat cushions, the distance measured in the depth images of recordings 05 and 07. |
| `scanning_torso_state = LOW` | `demo.py` | Not measured; the camera height of the recordings is unknown. |
| `position_threshold = 0.3`, `orientation_threshold = 0.1`, `timeout = 3 min` | `pumas_navigation.py` | Not measured; Pumas reports no outcome, so arriving is judged by these alone. The position threshold allows for Pumas stopping about 0.24 m short of the goal when an obstacle is near it. |
| `map_T_scene = (0.44, 0.14, yaw -0.058)` | `pumas_navigation.py` | Fitted to three room corners clicked on the laser map: the far wall at the bedroom end, the kitchen wall at the fridge end and the window wall at the oven end, taking the window wall at its cable channel. The corners still differ by 7 to 15 cm after the fit, since the room is 4.72 m by 9.05 m in the laser map and 4.86 m by 8.79 m in the model. |
| `images_before_start = 30` | `camera_recording.py` | About one second of images at 30 Hz before the plan starts. |
