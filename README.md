# turtlebot3_wall_following_node
ROS node using a naive turtlebot3 wall following algorithm. Targets ros2 jazzy.

## Note

This probably should have been implemented as a ros action server instead, but don't feel like refactoring.

## Dependencies
```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-sim -y
```

Expects following repos to also be in `./src`:
- https://github.com/jamesGadoury/turtlebot3_msgs
- https://github.com/jamesGadoury/turtlebot3
- https://github.com/jamesGadoury/turtlebot3_simulations
- https://github.com/jamesGadoury/DynamixelSDK

## Build
```bash
colcon build --packages-select turtlebot3_wall_following_node
```

## Run

### Full Simulation

Start Gazebo simulation:
```bash
./scripts/run_sim.sh
```

Start rviz2:
```bash
./scripts/run_rviz.sh
```

Start wall following node:
```bash
# NOTE: you need to teleop to face the wall you want to follow first
./scripts/run_wall_follower.sh
```