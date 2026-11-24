# HSR look at sofa while driving demo

The HSR starts at the door of the kitchen lab and drives to the sofa in MuJoCo. Its head
camera looks at the sofa during the whole drive, and the base drives forwards around the
furniture, so the lidar at its front faces the way it drives.

Every command runs from inside this folder, with ROS sourced and the workspace's virtual
environment:

| Run | What you see |
|---|---|
| `python demo.py` | A MuJoCo viewer. |
| `python test_demo.py` | No viewer; exit code 0 if the camera looks at the sofa after the drive. |

The plan:

```
ParkArmsAction(all arms)
LookAtAction(sofa)
NavigateAndLookAtAction(
    PathPlanningNavigateAcrossFloorsAction(1.5 m in front of the sofa, facing it),
    LookAtWhileDrivingAction(sofa))
```

The scene, the drive and the physics settings are shared with the milestone 0 demos, in
`coraplex/src/coraplex/sofa_demonstrations.py`.

| Measured in MuJoCo, without a viewer | |
|---|---|
| Camera off the sofa's origin | at most 0.005 rad |
| Driving direction off the front of the base | at most 0.01 rad |

Never run on the real robot.
