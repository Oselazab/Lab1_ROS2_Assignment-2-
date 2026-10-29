# MEC356 - Lab 1: ROS 2, Turtlesim, Actions and Services

## Team

| Name | Student ID |
|---|---|
| Osama Medhat | 2304156 |
| Ziad Mohamed | 2304154 |
| Anas Mohamed | 2304149 |

## Packages

- `lab1_interfaces`: `TargetPose` message (x, y, yaw), `GoToPose` and `DrawSquare` actions, `SetSpeedLimits` service.
- `lab1_control`: pose controller, reset and parameter services, square orchestration, launch file and parameter YAML.

Targets use turtlesim coordinates and radians. `/go_to_pose` implements the assignment's goToPose action; `/draw_square` draws counterclockwise from the centre. The node `/lab1_server` contains both action servers and both services. The square resets first, then sends exactly four sequential pose goals. No extra repositioning goal is used. Final corner headings prepare each following side.

## Build on Ubuntu 24.04 / ROS 2 Jazzy

ROS 2 Jazzy Desktop and colcon must already be installed. Open Ubuntu (on this laptop: `wsl -d Ubuntu-24.04`). Keep the build workspace on Ubuntu's filesystem rather than the Google Drive mount.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install ros-jazzy-turtlesim python3-colcon-common-extensions
mkdir -p ~/mec356_lab1_ws/src
```

Copy **lab1_interfaces** and **lab1_control** from this folder into `~/mec356_lab1_ws/src/`. If cloning a repository containing this folder's contents, clone it into `~/mec356_lab1_ws/src/lab1` instead. Then:

```bash
cd ~/mec356_lab1_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch lab1_control lab1.launch.py
```

The launch starts turtlesim and all servers, with speeds loaded from `lab1_control/config/limits.yaml`. Keep it running. In **each new Ubuntu terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source ~/mec356_lab1_ws/install/setup.bash
```

## Inspect the system (R1, R2, R6, R8)

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list -t
ros2 interface show lab1_interfaces/msg/TargetPose
ros2 interface show lab1_interfaces/action/GoToPose
ros2 interface show lab1_interfaces/action/DrawSquare
ros2 interface show lab1_interfaces/srv/SetSpeedLimits
ros2 topic echo /turtle1/pose
```

The controller subscribes to `/turtle1/pose` (`turtlesim/msg/Pose`) and publishes `/turtle1/cmd_vel` (`geometry_msgs/msg/Twist`). Reset uses `/clear` (`std_srvs/srv/Empty`), `/turtle1/teleport_absolute` (`turtlesim/srv/TeleportAbsolute`) and `/turtle1/set_pen` (`turtlesim/srv/SetPen`).

For a manual teleop check, with no action running:

```bash
ros2 run turtlesim turtle_teleop_key
```

## Pose action (R3)

```bash
ros2 action send_goal /go_to_pose lab1_interfaces/action/GoToPose "{target: {x: 8.0, y: 7.0, yaw: 1.5708}}" --feedback
```

Try left/down/behind targets and different final headings. Position tolerance is 0.04 units; yaw tolerance is 0.03 rad. Targets must be finite, with x and y in [0.2, 10.8]. Yaw is wrapped when calculating errors. A goal aborts after 180 seconds or if pose updates stop for two seconds. Very low valid speed limits may therefore cause a timeout. Concurrent motion goals are rejected. Do not drive with teleop while an action is running.

## Reset service (R4)

```bash
ros2 service call /reset_turtle std_srvs/srv/Trigger '{}'
```

The reset disables the pen, teleports to (5.544445, 5.544445, 0), clears the screen and restores the white pen. It returns a success/message response and rejects a reset during motion. Call it twice to verify repeatability.

## Speed parameters and service (R7)

```bash
ros2 param list /lab1_server
ros2 param get /lab1_server max_linear_speed
ros2 param get /lab1_server max_angular_speed
ros2 param set /lab1_server max_linear_speed 1.0
ros2 param set /lab1_server max_angular_speed 1.5
ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear_speed: 0.8, max_angular_speed: 1.2}"
ros2 param get /lab1_server max_linear_speed
ros2 param get /lab1_server max_angular_speed
ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear_speed: -1.0, max_angular_speed: 1.0}"
```

The last call must fail. Both limits must be positive finite doubles. The service updates both atomically, so an invalid request changes neither. Speeds are read every control cycle (50 Hz), including changes made while moving. Defaults are 2.0 units/s and 2.0 rad/s. Use decimal values in parameter commands.

## Square action (R5)

```bash
ros2 action send_goal /draw_square lab1_interfaces/action/DrawSquare '{side_length: 2.0}' --feedback
```

Repeat the command to verify a clean reset on the second run. Feedback reports the current side and completed sides. Side lengths must be finite and between 0.2 and approximately 5.2555 so the square stays inside the window. Each successful square logs exactly four accepted goToPose goals. An active square rejects other square requests and unrelated pose requests. Cancellation propagates to the current pose action and stops the turtle.

## Design pattern bonus

`controller.py` separates the pure pose-control strategy from the ROS action/service orchestration in `server.py`. This Strategy-style separation makes the steering law replaceable without changing action transport and allows numerical testing without ROS. The controller has two explicit phases: turn/translate toward the point, then align final yaw. The square action composes the existing reset service and pose action rather than duplicating steering logic.

A reentrant callback group with an eight-thread executor allows subscriptions, service replies and action results to progress while orchestration waits. Locks guard action admission and reset ownership. All goals stop velocity output when they end.

## Automated integration checks

With the launch running, execute this folder's `tests/verify_lab.py` in a sourced Ubuntu terminal:

```bash
python3 /path/to/Lab1/tests/verify_lab.py
```

The script tests repeated reset, speed validation, four motion directions and yaw, rejected goals, live speed caps, motion cancellation, two squares, square bounds, square cancellation and recovery. It moves the turtle and changes parameters; run it when no other client is controlling the turtle. See `TEST_RESULTS.md` for the actual local run.

## Optional rosbag bonus procedure

This procedure is provided for recording your own demonstration; no bag is claimed until it has been recorded and replayed.

In a sourced terminal, reset first and start recording:

```bash
ros2 service call /reset_turtle std_srvs/srv/Trigger '{}'
ros2 bag record -o lab1_square /turtle1/cmd_vel /turtle1/pose
```

In another terminal, run the square action. After it succeeds, stop recording with Ctrl+C and inspect:

```bash
ros2 bag info lab1_square
```

Confirm both topics have nonzero message counts. For replay, stop the full lab launch and run **only turtlesim** in one terminal:

```bash
ros2 run turtlesim turtlesim_node
```

In another sourced terminal, restore the start pose and clear, then replay only velocity commands. Publishing recorded poses would interfere with pose observations.

```bash
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen '{off: 1}'
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute '{x: 5.544445, y: 5.544445, theta: 0.0}'
ros2 service call /clear std_srvs/srv/Empty '{}'
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen '{r: 255, g: 255, b: 255, width: 3, off: 0}'
ros2 bag play lab1_square --topics /turtle1/cmd_vel
```

Verify the path visually and capture evidence for the README. Velocity replay is open loop: timing can cause small differences.

## Submission

Upload the two package folders and README (optionally tests, test notes and verified bonus evidence). Exclude `build/`, `install/` and `log/`. Repository: https://github.com/Oselazab/Lab1_ROS2_Assignment-2-. One team member should submit the repository link through the assignment's Google Form.

