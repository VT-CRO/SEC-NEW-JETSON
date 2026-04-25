# Crobot Controller
## Purpose
This package implements the ros2 control plugin, `crobot_controller/CrobotDriveController`.

It receives data for command velocity, sweeper position, winch velocity, and the flag dropper, converting them into hardware command interfaces for the declared joints. It also publishes odometry and broadcasts the `odom -> base_link` TF transform.

## Role Within Wider System
```text
Behavior Tree / Nav2
        |
        v
/cmd_vel and mechanism command topics
        |
        v
crobot_controller/CrobotDriveController
        |
        v
ros2_control command interfaces
        |
        v
crobot_hardware hardware interface
        |
        v
motors / servos / embedded hardware
```

## Topics Subscribed to
- `/cmd_vel`
- `/sweeper_position_controller/commands`
- `/winch_velocity_controller/commands`
- `/flagdropper_controller/commands`
##

### `/cmd_vel`
Message Type: `geometry_msgs/msg/Twist`

Used for:

- robot translation in `linear.x`
- robot strafing in `linear.y`
- robot rotation in `angular.z`

Published by:

- `crobot_behavior` nodes, (E.g. `PressButton` or `TurnCrank`)
- Nav2
- teleop

##
### `/sweeper_position_controller/commands`
Message Type: `std_msgs/msg/Float64`

Used for:

- moving the sweeper up/down

Published by:
- `crobot_behvior` `SweeperControl` node

##
### `/winch_velocity_controller/commands`
Message Type: `std_msgs/msg/Float64`

Used for:

- controlling winch velocity

Published by:
- `crobot_behvior` `TurnCrank` node

##
### `/flagdropper_controller/commands`
Message Type: `std_msgs/msg/Float64`

Used for:

- Dropping the flag

Published by:
- `crobot_behavior` `FlagDropper` node

## Topics Published To
### `~/odom`
Message Type: `nav_msgs/msg/Odometry`, which becomes `/crobot_drive_controller/odom` due to the namespace

Used for:
- localization
- communication with Nav2

## Other Packages Crobot Controller Communicates With

### `crobot_behavior`

The behavior tree sends commands for controlling mechanisms and velocity. As mentioned earlier, it publishes to these topics:

- `/cmd_vel`
- `/sweeper_position_controller/commands`
- `/winch_velocity_controller/commands`
- `/flagdropper_controller/commands`

##
### Nav2 / `crobot_navigation`

Nav2 publishes `/cmd_vel` during autonomous navigation. The controller turns this into wheel/ankle commands

##
### `crobot_description`

Provides the URDF/Xacro joint names and ros2_control hardware declarations

##
### `crobot_hardware`

The Crobot Hardware package has the hardware interfaces that are controlled by the Crobot Controller

##
### `crobot_gazebo`

Gaebo provides simulated ros2 control hardware interfaces, so the controller can be used for simulated joints

# How to Edit This Package
## Editing Robot Geometru

Edit these parameters in the yaml file(s):

```yaml
wheel_separation_width
wheel_separation_length
wheel_radius
```

These affect the swerve kinematics.

## To Change Max Speed
Again in the yaml files, edit:

```yaml
max_linear_velocity
max_angular_velocity
```

Note: `assumed_servo_speed_` should match how quickly the physical steering servos can actually move.

## Useful Commands for Debugging
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 param list /crobot_drive_controller
ros2 topic echo /cmd_vel
ros2 topic echo /crobot_drive_controller/odom
```