# turtlebot3_wall_following_node
ROS node using a naive turtlebot3 wall following algorithm.

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
