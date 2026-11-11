# HSR kitchen to sofa demo

The HSR starts in the kitchen of the kitchen lab, drives to the sofa, looks at it and
says which of its two seat places are free:

```
The right seat of the sofa is free.
```

| Run | Robot | What sees the sofa | Output |
|---|---|---|---|
| `rosbag_demo.py <recording>` | kinematically simulated | RoboKudo, on a recording of the real HSR's head camera | printed |
| `real_demo.py` | real HSR | RoboKudo, on the live head camera | printed and spoken by the HSR |
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

Never run so far. Each step runs in its own terminal with the robot's `ROS_DOMAIN_ID`.

1. Bring up the HSR's ROS stack: drivers, localization in the kitchen lab's map, the
   head camera and the voice node that listens on `/talk_request`.
2. Start Giskard for the HSR:
   `$PY -m giskardpy.middleware.ros2.scripts.iai_robots.hsr.hsr_velocity`
3. Start RoboKudo: `$PY -m robokudo.scripts.main _ae seat_place_occupancy`
4. Put the robot in the kitchen and run `$PY real_demo.py`. The sentence is printed and
   sent to the HSR's voice through `RobotSpeech` (`coraplex.scene_report_command`).

## What the plan does

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
| `images_before_start = 30` | `camera_recording.py` | About one second of images at 30 Hz before the plan starts. |
