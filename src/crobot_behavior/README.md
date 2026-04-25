# Crobot Behavior
## Purpose
This package implements a behavior tree that allows us to implement high-level task sequences made up of custom actions nodes.

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

## Main Executable
### `bt_launch`

Note that this file (`launch/bt_launch.ccp`) is a c++ executable, not a Python launch file. It does the following:

- Creates a ros node names `bt_launch`
- Creates a `NavigationServer` node named `crobot_navigation`
- Creates a BehaviorTree.CPP "factory"
- Registers the custom behavior nodes (`GoToPose`, `SweeperControl`,`EmbeddedModeControl`, etc.)
- Loads the XML tree from `trees/default.xml`
- Ticks the tree at 20 Hz
- Calls `rclcpp::spin_some` so ROS callbacks can run while the tree is ticking
- Publishes several zero `/cmd_vel` messages when the tree finishes or fails, so the robot stops moving when done

Also, note that the default tree path is hardcoded:
```cpp
std::string xml_path = pkg_path + "/trees/default.xml";
```
So, if you want to use a different xml file, just change that line.

## How to Add New Behaviors
1. Create header file in `src/crobot_behavior/include/crobot_behavior/action_nodes` directory
2. Create source file in `src/crobot_behavior/src/action_nodes` directory
3. Determine whether your behavior needs to be a `BT::SyncActionNode`, `BT::StatefulActionNode`, or `BT::ConditionNode`
4. Add your `.cpp` file to the CMake
5. Register this node in `bt_launch.cpp` following the same conventions as the others

Use `StatefulActionNode` for actions that take multiple ticks, such as waiting, driving for a certain amount time, or waiting for the result of another action. Use `SyncActionNode` for actions that complete immediately, and `ConditionNode` for true/false checks. Read the [C++ behavior tree documentation](https://www.behaviortree.dev/docs/intro) for more information. 
