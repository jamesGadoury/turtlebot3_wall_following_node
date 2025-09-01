# turtlebot3_wall_following_node
ROS node using a naive turtlebot3 wall following algorithm. Targets ros2 jazzy.

## deps
```
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-sim -y
```

expects following repos to also be in ./src:
- https://github.com/jamesGadoury/turtlebot3_msgs
- https://github.com/jamesGadoury/turtlebot3
- https://github.com/jamesGadoury/turtlebot3_simulations
- https://github.com/jamesGadoury/DynamixelSDK

## build
```
colcon build --packages-select turtlebot3_wall_following
```

## run

start sim:
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

start node:
```
ros2 run  turtlebot3_wall_following turtlebot3_wall_following_node
```

start node w/ debug logs:
```
ros2 run  turtlebot3_wall_following turtlebot3_wall_following_node --ros-args --log-level DEBUG
```
