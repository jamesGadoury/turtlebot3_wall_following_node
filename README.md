# turtlebot3_wall_following_node
ROS node using a naive turtlebot3 wall following algorithm. Targets ros2 jazzy.

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
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Start wall following node (requires `use_sim_time:=true` for Gazebo):
```bash
ros2 run turtlebot3_wall_following_node turtlebot3_wall_following_node --ros-args -p use_sim_time:=true
```

Start node with debug logs:
```bash
ros2 run turtlebot3_wall_following_node turtlebot3_wall_following_node --ros-args -p use_sim_time:=true --log-level wall_follower:=DEBUG
```

Start RViz with custom configuration:
```bash
rviz2 -d src/turtlebot3_wall_following_node/rviz/view_workflow.rviz
```

**Visualization**: The wall following node publishes visualization markers showing the controller's internal state:
- **Cyan sphere** (`/wall_following/wall_point`): Wall point (averaged K-nearest right-side laser points)
- **Magenta sphere** (`/wall_following/lookahead_point`): Lookahead point (detected forward-right corner prediction, when present)
- **Yellow sphere** (`/wall_following/target_point`): Target point (lookahead if detected, otherwise wall point)

**Note**: The `use_sim_time:=true` parameter is required when running with Gazebo to ensure proper temporal synchronization between laser scans and robot poses. When running on a real robot, omit this parameter.


## Parameter Tuning

The controller parameters are defined in [right_wall_following_controller.hpp](include/turtlebot3_wall_following_node/right_wall_following_controller.hpp) and must be manually tuned in Gazebo simulation. This guide explains each parameter and how to adjust them based on observed behavior.

### Tuning Setup

Start Gazebo and the wall following node with RViz visualization:

```bash
# Terminal 1: Start Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Start wall following node
ros2 run turtlebot3_wall_following_node turtlebot3_wall_following_node --ros-args -p use_sim_time:=true

# Terminal 3: Start RViz with visualization
rviz2 -d src/turtlebot3_wall_following_node/rviz/view_workflow.rviz
```

Watch the robot's behavior and use the visualization markers to understand what the controller is doing:
- **Cyan sphere**: Wall point being tracked (average of K-nearest right-side laser points)
- **Magenta sphere**: Lookahead point (forward-right corner detection)
- **Yellow sphere**: Target point (lookahead if detected, otherwise wall point)

### Parameter Categories and Tuning Guide

#### 1. Detection Parameters

**`k_nearest_points`** (default: 7)
- **What it does**: Number of nearest laser points to average for wall position
- **Symptoms of incorrect value**:
  - Too low (1-3): Jittery wall tracking, sensitive to sensor noise
  - Too high (>10): Overly smoothed, slow response to geometry changes
- **How to tune**: Start with 7. Increase if tracking is jittery, decrease if robot cuts corners
- **Typical range**: 3-10

#### 2. Lookahead Detection Parameters

**`lookahead_max_distance_m`**
- **What it does**: Maximum range to search for upcoming corners
- **Symptoms of incorrect value**:
  - Too short: Late corner detection, cutting inside corners
  - Too long: False detections from distant walls, erratic behavior
- **How to tune**: Should detect corners ~0.5-1.0 robot lengths ahead
- **Typical range**: 0.4-0.8

**`lookahead_min_x_m`**
- **What it does**: Minimum forward distance for valid lookahead (prevents lateral wall detection)
- **Symptoms of incorrect value**:
  - Too low: Treats current wall as lookahead, causes oscillation
  - Too high: Misses tight corners, late corner response
- **How to tune**: Should be ~60-80% of `lookahead_max_distance_m`
- **Typical range**: 0.3-0.5

**`lookahead_forward_ratio`**
- **What it does**: Ensures lookahead point is ahead, not beside robot (requires `x > ratio*|y|`)
- **Symptoms of incorrect value**:
  - Too low (<0.4): False detections from walls beside robot
  - Too high (>0.8): Misses corners that aren't perfectly ahead
- **How to tune**: Test on 90° inside corners - should detect reliably
- **Typical range**: 0.5-0.7

#### 3. Emergency Braking Parameters

**`emergency_distance_m`**
- **What it does**: Distance threshold that triggers emergency slowdown
- **Symptoms of incorrect value**:
  - Too short: Collisions or near-misses with obstacles
  - Too long: Excessive braking, robot moves too slowly
- **How to tune**: Should trigger ~2 robot diameters from obstacle
- **Typical range**: 0.3-0.5

**`emergency_angle_rad`**
- **What it does**: Forward cone angle for emergency detection (only triggers if obstacle ahead)
- **Symptoms of incorrect value**:
  - Too small: Doesn't brake when approaching walls at angles
  - Too large: Triggers on walls beside robot, unnecessary braking
- **How to tune**: Test approaching wall head-on - should brake smoothly
- **Typical range**: 0.5-1.0 rad (28-57°)

**`emergency_speed_ratio`**
- **What it does**: Speed reduction during emergency (as fraction of max velocity)
- **Symptoms of incorrect value**:
  - Too low: Stops completely, appears stuck
  - Too high: Insufficient slowdown, still collides
- **How to tune**: During emergency, robot should slow enough to avoid collision
- **Typical range**: 0.15-0.3

#### 4. PD Control Gains

**`angular_kp`**
- **What it does**: Proportional gain for angular velocity (immediate correction)
- **Symptoms of incorrect value**:
  - Too low: Sluggish turns, drifts away from wall
  - Too high: Oscillation, wobbling motion
- **How to tune**: Start low, increase until response is crisp without oscillation
- **Typical range**: 0.8-1.5
- **Tuning tip**: If robot oscillates, reduce Kp OR increase Kd

**`angular_kd`**
- **What it does**: Derivative gain for angular velocity (damping, opposes rate of change)
- **Symptoms of incorrect value**:
  - Too low: Oscillation, overshoot on turns
  - Too high: Over-damped, sluggish response
- **How to tune**: Increase if robot oscillates, decrease if response is too slow
- **Typical range**: 0.2-0.6
- **Tuning tip**: Kd should be ~30-50% of Kp

#### 5. Speed Control Parameters

**`min_linear_velocity_ratio`**
- **What it does**: Minimum speed when badly misaligned (as fraction of max velocity)
- **Symptoms of incorrect value**:
  - Too low: Stops when misaligned, appears stuck
  - Too high: Doesn't slow enough for corrections, collisions
- **How to tune**: When robot is perpendicular to wall, should move slowly but not stop
- **Typical range**: 0.15-0.3

### Tuning Workflow

1. **Start with default parameters** - Observe baseline behavior
2. **Identify primary issue** - Use symptoms above to diagnose problem area
3. **Adjust one parameter at a time** - Rebuild and test after each change
4. **Test multiple scenarios**:
   - Long straight walls (tests basic tracking)
   - 90° inside corners (tests lookahead detection)
   - Varying wall distances (tests emergency braking)
   - Tight corridors (tests robustness)
5. **Iterate** - Small adjustments (10-20% changes) until behavior is acceptable

### Common Issues and Fixes

| Issue | Likely Cause | Parameter to Adjust |
|-------|--------------|---------------------|
| Oscillating/wobbling | Kp too high or Kd too low | Decrease `angular_kp` or increase `angular_kd` |
| Cuts inside corners | Lookahead detection late/weak | Increase `lookahead_max_distance_m` |
| Jittery tracking | Not enough spatial averaging | Increase `k_nearest_points` |
| Collisions | Emergency braking insufficient | Increase `emergency_distance_m` or decrease `emergency_speed_ratio` |
| Too slow overall | Emergency triggering too often | Decrease `emergency_distance_m` or increase `emergency_angle_rad` |

### Editing Parameters

Parameters are hardcoded in [right_wall_following_controller.hpp:26-54](include/turtlebot3_wall_following_node/right_wall_following_controller.hpp#L26-L54). To modify:

1. Edit the `ControllerParams` struct default values
2. Rebuild the package:
   ```bash
   colcon build --packages-select turtlebot3_wall_following_node
   ```
3. Source and retest:
   ```bash
   source install/setup.bash
   ros2 run turtlebot3_wall_following_node turtlebot3_wall_following_node --ros-args -p use_sim_time:=true
   ```
