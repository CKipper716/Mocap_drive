# TurtleBot3 Waffle: drive to a motion-capture goal

`mocap_drive.py` is a single ROS 2 Humble Python script. It uses the robot's
motion-capture pose and the goal object's position, turns toward the goal, drives
with heading corrections, and slows down near the goal. It commands at 20 Hz,
caps forward speed at 0.10 m/s and turning at 0.5 rad/s, and stops within 5 cm
of the target by default. That is a configured tolerance, not measured hardware
accuracy. Once reached, it keeps publishing zero velocity until you stop it.
Both positions are refreshed throughout the drive; each control update uses the
latest robot pose and target position. Goal orientation is ignored. Keep the goal stationary during a run.

## Files and settings

Extract the complete ZIP and keep `mocap_drive.py` beside its bundled `new_NAT`
folder. The client is at `new_NAT/samples/PythonClient` (one `new_NAT` level).
The bundle contains an updated copy of your instructor's client; the Desktop
originals were not edited. No manual client patch is needed.
Run on the ROS 2 Humble Linux computer with network access to both Motive and
the robot; ROS 2 and the TurtleBot3 packages must already be installed.

- Robot ID: **67**.
- Goal ID: **69**.
- Client IP: **192.168.0.232**; must belong to the computer running this script.
- Motive server IP: **192.168.0.245**, multicast enabled.
- Velocity topic: **/cmd_vel**, type **geometry_msgs/msg/Twist**.
- Coordinate assumptions: Motive Y-up, positions in meters, navigation x=X/y=-Z.

## NatNet integration already included

The supplied client decodes tracking validity but its original callback only
passes ID, position and rotation. The bundled copy adds a separate
`rigid_body_valid_listener`, called after a rigid body is fully decoded.
The driving script uses this callback, so invalid tracking clears the stored
pose immediately. The original three-argument callback is preserved for
compatibility with your instructor's sample and the original test script.
NatNet versions without a decoded validity flag stay stopped.

The supporting files are:
- `NatNetClient.py`: network connection and packet decoding.
- `MoCapData.py`: classes storing positions, rotations and tracking validity.
- `DataDescriptions.py`: classes describing tracked objects and other assets.
- `PythonSample.py`: the instructor's example client, included for reference.

## Align the robot pose

In Motive, set the robot rigid-body origin at the midpoint between its drive
wheels and align its local +X axis with the robot's physical forward direction.
The script projects that axis into the X/-Z ground plane to calculate heading.
If the rigid body's forward alignment differs by a constant yaw, use
`--yaw-offset-deg VALUE`: this is the physical forward heading minus the heading
of the rigid body's +X axis, positive counterclockwise viewed from above.
An offset does not correct a misplaced rigid-body origin.

Before driving, verify that moving the robot forward changes its mocap position
in the direction of the computed heading, and that turning left increases that
heading (apart from angle wrapping). Incorrect alignment can make the robot
steer the wrong way. The yaw formula here explicitly projects local +X; it does
not assume the old test script's formula establishes the physical forward axis.

## Run

Start the Waffle's normal Humble bringup on the robot:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_bringup robot.launch.py
```

On the computer running this script, use the same ROS_DOMAIN_ID as the robot,
source your usual TurtleBot workspace if required, and check the command topic:

```bash
source /opt/ros/humble/setup.bash
ros2 topic info /cmd_vel
```

It should have a robot subscriber and type `geometry_msgs/msg/Twist`. Stop
teleoperation/Nav2 or other velocity-command publishers before this controller.
In Motive, enable live rigid-body streaming for both objects. Then run from the
folder containing the script. Robot 67 and target object 69 are the defaults:

```bash
python3 mocap_drive.py
```

For a physical goal object occupying floor space, choose enough separation for
both objects' dimensions. For example, to stop about 40 cm between the two
rigid-body origins (plus the 5 cm tolerance):

```bash
python3 mocap_drive.py --stop-distance 0.40
```

Without `--stop-distance`, the robot aims to put its own origin at the goal's
horizontal position. Use that only when the goal marker does not obstruct the
robot, such as a target above the floor. This minimal controller has no obstacle
avoidance; use a clear floor area and keep the stop control accessible.

It publishes zero if either pose is invalid or more than 0.25 seconds old and
resumes when both become valid again, unless the goal was already reached.
Ctrl+C/SIGTERM sends repeated zero commands before cleanup. Abrupt process,
network or computer failure requires the robot base's command timeout/watchdog;
the script cannot send a stop after such a failure. Freshness is measured from
packet reception, not camera exposure time.

## Validation and sources

The steering math was checked with a simple ideal differential-drive simulation,
including goals behind the robot, angle wrapping, speed limits and stop distance.
Synthetic NatNet packets were also used to check valid/invalid tracking and
compatibility with the original callback. This is not a live test on ROS, the
NatNet connection or the physical TurtleBot.

- ROS / TurtleBot3 command interface and bringup:
  https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/
- Motive streaming:
  https://docs.optitrack.com/v3.0/motive/data-streaming
