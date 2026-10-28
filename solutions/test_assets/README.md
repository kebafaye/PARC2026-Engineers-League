# test_assets — local AprilTag fixtures

Not part of the submitted solution. These exist so the AprilTag pipeline in
`parc_solution` could be validated before any official PARC world ships with tags
(the Phase 1 navigation task world has none).

## What's here

| file | purpose |
|---|---|
| `generate_tag.py` | renders the tag36h11 texture used by the Gazebo model |
| `models/apriltag36h11_00000/` | a flat panel textured with that marker (visual only, no collision) |
| `worlds/task_world_apriltag_test.sdf` | copy of the official world **plus** one tag; the organizers' file is never modified |
| `launch/apriltag_test_world.launch.py` | the official `task.launch.py` pointed at that world, with an `rviz:=false` option |
| `synthetic_tag_publisher.py` | publishes a *computed* camera image of a tag at an exactly known pose |
| `calibrate_tag_size.py` | prints the `size` that `apriltag_tags.yaml` should declare |
| `publish_known_good_frame.py` | replays `frame_check4.png`, a frame known to decode |
| `grab_frame.py`, `check_camera_rates.py` | small diagnostics |

## Two things that cost real time here

**1. `size` must be measured, never assumed.** Reported tag depth scales linearly
with the `size` in `apriltag_tags.yaml`, and a wrong value looks completely healthy
from the detection side. Declaring `0.6` for a fixture whose black square was
actually `0.296 m` put the tag 2.03x too far away (4.78 m reported for a true
2.36 m) and sent Nav2 to empty floor beyond it — with `hamming: 0` and
`decision_margin: 112` the whole time. Park at a tape-measured distance and run:

```bash
python3 calibrate_tag_size.py --true-distance <m> --declared-size <m>
```

Verified correct against `synthetic_tag_publisher.py`, where the geometry is exact
rather than measured: a tag drawn at precisely `size*fx/depth` pixels reproduced a
known 3.000 m as 3.002 m.

**2. Gazebo's renderer stopped applying the texture partway through.** The panel
renders solid black regardless of texture path (relative *or* absolute), model name,
or a clean restart — an Ogre2 material-cache problem in the fixture, not in any
`parc_solution` node, and it did render correctly earlier in the same session. When
it bites, `synthetic_tag_publisher.py` is the way round it: it bypasses rendering
entirely and gives exact ground truth, which is better for calibration anyway.

## Running the end-to-end check

```bash
# terminal 1 - world (RViz off: it costs ~268% CPU and collapses the RTF)
ros2 launch src/solutions/test_assets/launch/apriltag_test_world.launch.py rviz:=false

# terminal 2 - Nav2 stack; WAIT for "Managed nodes are active"
ros2 launch parc_solution navigation.launch.py

# terminal 3 - detection, then the bridge
ros2 launch parc_solution apriltag_detection.launch.py
ros2 run parc_solution tag_goal_bridge.py

# terminal 4 - give it a tag to see, then send the robot to it
python3 src/solutions/test_assets/synthetic_tag_publisher.py --tag-size 0.6 --depth 2.5
#   ...wait for "First sighting", then stop this publisher so the cache freezes
ros2 topic pub --once /parc/go_to_tag_id std_msgs/msg/Int32 "data: 0"
```

Two sequencing traps worth knowing, both hit for real:

- **Wait for `Managed nodes are active`, not just for `world_anchor`.** If the robot
  is still moving when `navigation.launch.py` starts, `local_costmap` times out
  waiting for `odom -> base_footprint`, the lifecycle manager aborts the whole
  bringup, and every goal afterwards is answered `Action server is inactive.
  Rejecting the goal.`
- **Stop the robot before re-parking it.** Gazebo keeps applying the last `cmd_vel`
  indefinitely, so killing Nav2 mid-manoeuvre leaves the robot spinning and a
  `set_pose` won't stick. Publish a zero `Twist` on
  `/robot_base_controller/cmd_vel_unstamped` first.
