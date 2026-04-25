# Crobot_behaviorTree
Behavior Tree for Crobot. Defines and execute commands using the behaviortree package and Nav2

## Launching the Behavior Tree
Launch:
```
ros2 launch PATH-TO-CROBOT-BRINGUP/launch/behavior.launch.py
```
The following line will launch bt_launch.cpp, alongside other supplements.

In addition, you can run:
```
ros2 launch rviz2
```
to see the robot and plot paths manually.

## Behavior Tree Overview
The behavior tree defines the decision the robot will make autonomously. This is done through a command-server structure.

### Structure
- The behavior tree package provides a skeleton for the command.
- A command is defined in src/action_nodes, using the provided inheritance structure from behaviortree package.
- Each command is then registered in launch/bt_launch.cpp through the BehaviorTreeFactory, provided by behaviortree package.
- Each command is then passed to the robot through the xml files located in the "trees" directory using XML tags.



## Testing without a robot
To test the behavior tree without a robot, we can use the ros2 provided turtlebot for simulation.

1. Export the turtlebot model
```
export TURTLEBOT3_MODEL=burger
```

2. Launch the Gazebo
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

3. Launch the Navigation Server
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py   use_sim_time:=True   params_file:=/home/ros/ws/SEC-NEW-JETSON/src/crobot_behavior/config/nav2_params.yaml
```

4. Launch the behavior tree
```
ros2 launch src/crobot_bringup/launch/bringup.launch.py
```
