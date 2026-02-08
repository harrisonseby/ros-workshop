---
title: "ROS2 Navigation 2 Workshop"
subtitle: "Building Autonomous Navigation Systems"
author: "ROS2 Workshop"
date: "2026"
---

# ROS2 Navigation 2 Workshop
## Building Autonomous Navigation Systems

---

# Agenda

1. Introduction to Navigation 2
2. Nav2 Architecture & Components
3. Prerequisites & Setup
4. Mapping with SLAM Toolbox
5. Localization with AMCL
6. Navigation Configuration
7. Path Planning & Controllers
8. Hands-On Demo: diff_bot Navigation
9. Troubleshooting & Best Practices
10. Q&A

---

# What is Navigation 2 (Nav2)?

- **Navigation 2** is the next-generation navigation framework for ROS2
- Complete rewrite of ROS1 navigation stack
- Provides autonomous navigation capabilities for mobile robots
- Enables robots to:
  - Navigate from point A to point B
  - Avoid obstacles dynamically
  - Replan paths when needed
  - Localize themselves in the environment

---

# Why Use Nav2?

- **Modular Architecture**: Plug-and-play components
- **Behavior Trees**: Flexible mission planning
- **Advanced Algorithms**: DWB, Regulated Pure Pursuit, etc.
- **Active Development**: Maintained by Open Robotics
- **Multi-Robot Support**: Scalable for robot fleets
- **Lifecycle Management**: Better node control
- **Plugin-Based**: Easy to customize

---

# Nav2 Architecture Overview

```
┌─────────────────────────────────────────────┐
│           Nav2 System Architecture          │
├─────────────────────────────────────────────┤
│                                             │
│  ┌──────────────┐      ┌──────────────┐   │
│  │  BT Navigator│◄────►│  Controller  │   │
│  │              │      │   Server     │   │
│  └──────┬───────┘      └──────────────┘   │
│         │                                   │
│         ▼                                   │
│  ┌──────────────┐      ┌──────────────┐   │
│  │   Planner    │      │  Smoother    │   │
│  │   Server     │      │   Server     │   │
│  └──────────────┘      └──────────────┘   │
│                                             │
│  ┌──────────────┐      ┌──────────────┐   │
│  │  Behavior    │      │  Waypoint    │   │
│  │   Server     │      │  Follower    │   │
│  └──────────────┘      └──────────────┘   │
│                                             │
│  ┌──────────────┐      ┌──────────────┐   │
│  │     AMCL     │      │  Map Server  │   │
│  │ (Localization)│     │              │   │
│  └──────────────┘      └──────────────┘   │
└─────────────────────────────────────────────┘
```

---

# Core Nav2 Components

1. **BT Navigator**: Behavior tree-based navigation logic
2. **Planner Server**: Global path planning
3. **Controller Server**: Local trajectory tracking
4. **Smoother Server**: Path smoothing
5. **Behavior Server**: Recovery behaviors
6. **Map Server**: Static map management
7. **AMCL**: Adaptive Monte Carlo Localization
8. **Lifecycle Manager**: Node lifecycle control

---

# Navigation Prerequisites

## Essential Requirements:

1. **Robot Model (URDF)**
   - Proper link definitions
   - Correct inertial values
   - Sensor placements

2. **Sensor Data**
   - LaserScan or PointCloud
   - Odometry
   - Optional: IMU, Depth Camera

3. **TF Tree**
   - map → odom → base_link
   - Sensor frames

---

# TF Tree Structure

```
        map
         │
         ├─ odom (Published by odometry source)
         │   │
         │   ├─ base_link (Robot base)
         │   │   │
         │   │   ├─ lidar_link
         │   │   │
         │   │   ├─ camera_link
         │   │   │
         │   │   └─ wheel_left
         │   │
         │   └─ wheel_right
```

**Key Transforms:**
- `map → odom`: Published by AMCL or SLAM
- `odom → base_link`: Published by odometry source
- Static transforms: Published by robot_state_publisher

---

# Step 1: Mapping with SLAM

## What is SLAM?
**Simultaneous Localization and Mapping**

- Robot builds a map while localizing itself
- We use **slam_toolbox** for ROS2

## Key Components:
```yaml
# mapper_params_online_async.yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /diff_bot/scan
    mode: mapping
```

---

# Running SLAM Mapping

## Launch Command:
```bash
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=./config/mapper_params_online_async.yaml \
  use_sim_time:=true
```

## Mapping Process:
1. Start your robot simulation
2. Launch SLAM toolbox
3. Drive robot around (teleop or manual)
4. Save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

## Output:
- `my_map.yaml` - Map metadata
- `my_map.pgm` - Occupancy grid image

---

# Understanding Maps

## Map Types:
- **Occupancy Grid**: 2D representation
  - White = Free space (0)
  - Black = Occupied (100)
  - Gray = Unknown (-1)

## Map Parameters (YAML):
```yaml
image: my_map.pgm
resolution: 0.05  # meters/pixel
origin: [-10.0, -10.0, 0.0]  # [x, y, yaw]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

---

# Step 2: Localization with AMCL

## Adaptive Monte Carlo Localization

- **Particle Filter** based localization
- Estimates robot pose on a known map
- Uses laser scans to match against map

## Key AMCL Parameters:
```yaml
amcl:
  ros__parameters:
    base_frame_id: "base_link"
    global_frame_id: "map"
    odom_frame_id: "odom"
    scan_topic: /diff_bot/scan
    max_particles: 2000
    min_particles: 500
```

---

# AMCL: How It Works

1. **Initialize**: Particles spread across possible poses
2. **Predict**: Particles moved based on odometry
3. **Update**: Particles weighted by laser scan matching
4. **Resample**: Keep particles with high weights
5. **Estimate**: Compute pose from particle cloud

## Important Notes:
- Set initial pose in RViz (2D Pose Estimate)
- More particles = better accuracy, more CPU
- Works best with distinctive features in map

---

# Step 3: Path Planning

## Global Planner (Planner Server)

- Plans path from start to goal on static map
- Default: **NavFn** (Dijkstra's algorithm)
- Alternative: **Smac Planner** (hybrid A*, lattice)

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
```

---

# Path Planning Algorithms

## Available Planners:

1. **NavFn Planner**
   - Fast, reliable
   - Dijkstra's or A*
   - Good for most cases

2. **Smac Planner 2D**
   - Grid-based A*
   - Considers robot footprint

3. **Smac Hybrid Planner**
   - For car-like robots
   - Considers kinematic constraints

4. **Theta Star**
   - Any-angle planning
   - Smoother paths

---

# Step 4: Path Following (Controller)

## Controller Server

- Follows the global plan
- Generates velocity commands
- Avoids dynamic obstacles

## Default Controller: **DWB (Dynamic Window Approach)**

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5
      min_vel_x: -0.5
      max_vel_theta: 1.0
```

---

# Controller Options

## 1. DWB Controller
- Dynamic window approach
- Good for differential drive
- Highly configurable

## 2. Regulated Pure Pursuit (RPP)
- Path tracking controller
- Smoother motion
- Better for structured environments

## 3. TEB Controller
- Time Elastic Band
- Considers dynamics
- Good for car-like robots

---

# Step 5: Recovery Behaviors

## Behavior Server

When navigation fails, execute recovery behaviors:

1. **Spin**: Rotate in place to clear costmap
2. **Back Up**: Move backwards
3. **Wait**: Pause and retry
4. **Clear Costmap**: Reset obstacle information

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
```

---

# Costmaps Explained

## Two Costmaps in Nav2:

### 1. Global Costmap
- Entire known map
- For global planning
- Larger, updated less frequently

### 2. Local Costmap
- Robot's immediate surroundings
- For local planning/control
- Smaller, updated frequently

Both can have multiple **layers**!

---

# Costmap Layers

## Common Layers:

1. **Static Layer**: From map file
2. **Obstacle Layer**: From sensor data (laser)
3. **Inflation Layer**: Adds safety buffer around obstacles
4. **Voxel Layer**: 3D obstacles (from depth camera)

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      robot_radius: 0.22
      resolution: 0.05
```

---

# Behavior Trees in Nav2

## Why Behavior Trees?

- **Flexible**: Easy to modify navigation logic
- **Modular**: Compose complex behaviors
- **Reactive**: Respond to environment changes
- **Visual**: Can be visualized and edited

## Default BT:
- Navigate to pose
- Compute path to goal
- Follow path
- Recovery if stuck

---

# Nav2 Configuration: nav2_params.yaml

## Key Sections:

1. **AMCL**: Localization parameters
2. **BT Navigator**: Behavior tree settings
3. **Controller Server**: Local planning
4. **Planner Server**: Global planning
5. **Behavior Server**: Recovery behaviors
6. **Global/Local Costmap**: Obstacle handling
7. **Map Server**: Map loading

---

# Hands-On: diff_bot Navigation

## Our Setup:

- **Robot**: Differential drive robot (diff_bot)
- **Sensors**: Lidar, Odometry
- **Simulator**: Gazebo Ignition
- **Environment**: Residential world

---

# diff_bot Architecture

```
┌──────────────────────────────────────────┐
│         diff_bot_sim Package             │
├──────────────────────────────────────────┤
│                                          │
│  URDF Model                              │
│    ├─ base_link                          │
│    ├─ left_wheel, right_wheel            │
│    ├─ caster_wheel                       │
│    └─ lidar_link                         │
│                                          │
│  Gazebo Plugins                          │
│    ├─ Differential Drive                 │
│    ├─ Lidar Sensor                       │
│    └─ Odometry                           │
│                                          │
│  Launch Files                            │
│    ├─ diff_bot_ign.launch.py            │
│    ├─ mapping.launch.py                  │
│    └─ navigation.launch.py               │
└──────────────────────────────────────────┘
```

---

# Step-by-Step: Running diff_bot Navigation

## 1. Start the Simulation
```bash
ros2 launch diff_bot_sim diff_bot_ign.launch.py
```

This launches:
- Gazebo Ignition world
- Robot model
- ROS-Gazebo bridge
- Robot state publisher

---

# Step 2: Create a Map (If Needed)

```bash
# Launch SLAM
ros2 launch diff_bot_sim mapping.launch.py

# Launch teleoperation (new terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/diff_bot/cmd_vel

# Drive around to build the map

# Save the map (new terminal)
cd ~/ros_workshop/src/diff_bot_sim/config
ros2 run nav2_map_server map_saver_cli -f diff_bot_map
```

---

# Step 3: Launch Navigation

```bash
ros2 launch diff_bot_sim navigation.launch.py
```

This starts:
- **Map Server**: Loads saved map
- **AMCL**: Localization
- **Planner Server**: Path planning
- **Controller Server**: Path following
- **Behavior Server**: Recovery behaviors
- **BT Navigator**: Navigation logic
- **Lifecycle Manager**: Manages all nodes

---

# Step 4: Visualize in RViz

```bash
ros2 launch diff_bot_sim diff_bot_display.launch.py
```

## RViz Display Elements:
- Robot model
- LaserScan
- Map
- Global/Local costmaps
- Global/Local plans
- Particle cloud (AMCL)
- TF tree

---

# Step 5: Set Initial Pose & Goal

## In RViz:

1. **Click "2D Pose Estimate"**
   - Click and drag on map
   - Sets robot's initial position
   - Particles converge

2. **Click "Nav2 Goal"**
   - Click destination on map
   - Robot plans and navigates!

---

# nav2_params.yaml Breakdown

## AMCL Configuration
```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    max_particles: 2000
    min_particles: 500
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    scan_topic: /diff_bot/scan
    odom_frame_id: "odom"
    base_frame_id: "base_link"
```

**Key Points:**
- Use differential motion model
- Correct topic names
- 500-2000 particles for balance

---

# Controller Configuration

```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5
      min_vel_x: -0.2
      max_vel_theta: 1.0
      min_vel_theta: -1.0
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
```

**Tuning Tips:**
- Match robot's physical limits
- Higher frequency = smoother control
- Adjust for your robot's dynamics

---

# Planner Configuration

```yaml
planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

**Notes:**
- `tolerance`: How close to goal (meters)
- `use_astar`: A* vs Dijkstra
- `allow_unknown`: Plan through unknown areas

---

# Costmap Configuration

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["obstacle_layer", "inflation_layer"]
```

**Important:**
- `rolling_window: true` for local
- Match `robot_radius` to your robot
- Higher update frequency = more CPU

---

# Common Issues & Solutions

## Issue 1: Robot Won't Move
**Symptoms:** Nav2 active, but robot stays still

**Solutions:**
- Check `/cmd_vel` topic remapping
- Verify controller is receiving goal
- Check costmap for obstacles blocking path
- Ensure proper TF tree

```bash
# Check topics
ros2 topic list | grep cmd_vel
ros2 topic echo /diff_bot/cmd_vel

# Check TF
ros2 run tf2_tools view_frames
```

---

# Common Issues & Solutions

## Issue 2: Poor Localization
**Symptoms:** Particle cloud doesn't converge, robot drifts

**Solutions:**
- Increase max_particles (2000-5000)
- Check laser scan topic
- Verify map quality (distinctive features)
- Set better initial pose
- Check odometry quality

```bash
# Check AMCL status
ros2 topic echo /amcl_pose

# Visualize particles in RViz
# Add PoseArray display for /particlecloud
```

---

# Common Issues & Solutions

## Issue 3: Robot Gets Stuck
**Symptoms:** Robot oscillates, doesn't reach goal

**Solutions:**
- Adjust inflation radius
- Tune DWB parameters (velocity limits)
- Check footprint size
- Enable recovery behaviors
- Adjust goal tolerance

```yaml
# Try these adjustments:
inflation_layer:
  inflation_radius: 0.55  # Reduce if too conservative
  cost_scaling_factor: 3.0
```

---

# Common Issues & Solutions

## Issue 4: Path Goes Through Obstacles
**Symptoms:** Global path passes through walls

**Solutions:**
- Check costmap configuration
- Verify static layer is loaded
- Ensure map file is correct
- Check robot_radius parameter
- Increase inflation radius

```bash
# Visualize costmaps in RViz
# Add Map displays for:
# - /global_costmap/costmap
# - /local_costmap/costmap
```

---

# Common Issues & Solutions

## Issue 5: Transforms Not Available
**Symptoms:** "Transform timeout", "No transform available"

**Solutions:**
- Check TF tree completeness
- Ensure all required publishers running
- Verify frame IDs match in params
- Check `use_sim_time` consistency

```bash
# Check TF tree
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link

# Check if frames exist
ros2 run tf2_tools view_frames
evince frames.pdf
```

---

# Performance Tuning

## CPU Optimization:

1. **Reduce costmap update frequency**
   ```yaml
   update_frequency: 3.0  # Instead of 5.0
   ```

2. **Limit particles**
   ```yaml
   max_particles: 1000  # Instead of 2000
   ```

3. **Reduce costmap size**
   ```yaml
   width: 3  # Smaller local costmap
   height: 3
   ```

---

# Performance Tuning

## Smoothness Optimization:

1. **Increase controller frequency**
   ```yaml
   controller_frequency: 20.0
   ```

2. **Use path smoother**
   ```yaml
   smoother_server:
     ros__parameters:
       smoother_plugins: ["simple_smoother"]
   ```

3. **Tune DWB critics**
   ```yaml
   PathAlign.scale: 32.0
   GoalAlign.scale: 24.0
   PathDist.scale: 32.0
   ```

---

# Advanced Features

## 1. Waypoint Following
Navigate through multiple points:
```bash
ros2 action send_goal /follow_waypoints \
  nav2_msgs/action/FollowWaypoints \
  "{poses: [{...}, {...}, {...}]}"
```

## 2. Keepout Zones
Define areas robot should avoid

## 3. Speed Restricted Zones
Slow down in certain areas

## 4. Multi-robot Support
Namespace different robots

---

# Debugging Tools

## Essential Commands:

```bash
# Check all Nav2 nodes
ros2 node list | grep nav2

# Check lifecycle states
ros2 lifecycle get /controller_server

# Monitor action servers
ros2 action list

# Check parameter values
ros2 param list /controller_server
ros2 param get /controller_server use_sim_time

# View logs
ros2 topic echo /rosout
```

---

# Debugging with RViz

## Useful Displays:

1. **Map**: Static map visualization
2. **LaserScan**: Sensor data
3. **PoseArray**: AMCL particles
4. **Path** (x2): Global & local plans
5. **Map** (x2): Global & local costmaps
6. **PoseWithCovariance**: AMCL pose estimate
7. **Marker**: Footprint & safety radius
8. **TF**: Transform tree

---

# Best Practices

## 1. Start Simple
- Get basic navigation working first
- Add complexity gradually
- Test in empty environments before complex ones

## 2. Proper Calibration
- Accurate URDF dimensions
- Correct wheel radius & separation
- Good odometry calibration

## 3. Map Quality
- Drive slowly when mapping
- Cover all areas thoroughly
- Close loops when possible

---

# Best Practices

## 4. Parameter Tuning
- Start with default params
- Change one parameter at a time
- Document your changes
- Use simulator for quick iteration

## 5. Testing
- Test in various environments
- Test recovery behaviors
- Test edge cases (narrow passages)
- Monitor CPU usage

---

# Best Practices

## 6. Version Control
- Keep configuration files in git
- Document parameter changes
- Track map files
- Version your launch files

## 7. Documentation
- Document custom parameters
- Note hardware specifications
- Record calibration procedures
- Maintain troubleshooting notes

---

# Resources & Documentation

## Official Documentation:
- **Nav2 Docs**: https://navigation.ros.org
- **ROS2 Docs**: https://docs.ros.org
- **Nav2 GitHub**: https://github.com/ros-planning/navigation2

## Community:
- ROS Discourse
- Nav2 Slack Channel
- ROS Answers

## Tutorials:
- Nav2 Getting Started Guide
- Sam Lowe's Nav2 Tutorials
- Articulated Robotics YouTube

---

# Workshop Repository Structure

```
ros_workshop/
├── src/diff_bot_sim/
│   ├── config/
│   │   ├── nav2_params.yaml          # Nav2 configuration
│   │   ├── mapper_params_online_async.yaml
│   │   └── diff_bot_map.yaml         # Your saved map
│   ├── launch/
│   │   ├── diff_bot_ign.launch.py    # Gazebo simulation
│   │   ├── mapping.launch.py          # SLAM launch
│   │   └── navigation.launch.py       # Nav2 launch
│   ├── urdf/                          # Robot description
│   ├── worlds/                        # Gazebo worlds
│   └── models/                        # 3D models
└── Development notes                  # Your notes
```

---

# Quick Start Commands Summary

```bash
# Terminal 1: Launch simulation
ros2 launch diff_bot_sim diff_bot_ign.launch.py

# Terminal 2: Launch navigation
ros2 launch diff_bot_sim navigation.launch.py

# Terminal 3: Launch RViz
ros2 launch diff_bot_sim diff_bot_display.launch.py

# In RViz:
# 1. Set initial pose (2D Pose Estimate)
# 2. Set navigation goal (Nav2 Goal)
# 3. Watch your robot navigate!
```

---

# Workshop Exercise Ideas

## Exercise 1: Basic Navigation
- Launch diff_bot in simulation
- Create a map using SLAM
- Navigate to different goals

## Exercise 2: Parameter Tuning
- Adjust robot speed limits
- Modify inflation radius
- Compare navigation performance

## Exercise 3: Complex Environment
- Test in cluttered space
- Add dynamic obstacles
- Observe recovery behaviors

---

# Workshop Exercise Ideas

## Exercise 4: Custom Behavior Tree
- Create custom BT XML
- Add custom recovery behavior
- Modify navigation logic

## Exercise 5: Waypoint Mission
- Create waypoint list
- Navigate through multiple points
- Loop back to start

## Exercise 6: Multi-Goal Navigation
- Set up patrol route
- Implement docking behavior
- Add custom actions

---

# Key Takeaways

1. **Nav2 is powerful but complex** - start simple
2. **TF tree must be correct** - map→odom→base_link
3. **Good map = good navigation** - invest time in mapping
4. **Parameter tuning is essential** - match your robot
5. **Use RViz for debugging** - visualize everything
6. **Recovery behaviors save missions** - configure them well
7. **Test thoroughly** - edge cases matter
8. **Documentation helps** - keep good notes

---

# Next Steps

## After the Workshop:

1. Experiment with your own robot
2. Try different environments
3. Implement custom plugins
4. Contribute to Nav2 community
5. Build complete autonomous systems
6. Explore advanced features (keepout zones, etc.)
7. Integrate with other systems (manipulation, vision)

---

# Additional Topics to Explore

- **Nav2 Smac Planner**: Advanced planning
- **Graceful Controller**: Alternative to DWB
- **Collision Monitor**: Safety layer
- **Costmap Filters**: Advanced costmap features
- **Docking**: Precision navigation
- **Assisted Teleop**: Human-in-the-loop
- **Fleet Management**: Multi-robot coordination

---

# Questions?

## Contact & Support:

- Workshop Repository: Check README.md
- Development Notes: See "Development notes" file
- Community Help: ROS Discourse, Slack

## Thank you for attending!

### Happy Navigating! 🤖

---

# Backup Slides

---

# Complete nav2_params.yaml Structure

```yaml
# 1. AMCL - Localization
amcl: {...}

# 2. BT Navigator - Behavior tree coordination
bt_navigator: {...}

# 3. Controller Server - Local planning
controller_server: {...}

# 4. Planner Server - Global planning
planner_server: {...}

# 5. Smoother Server - Path smoothing
smoother_server: {...}

# 6. Behavior Server - Recovery behaviors
behavior_server: {...}

# 7. Global Costmap - For global planning
global_costmap: {...}

# 8. Local Costmap - For local planning
local_costmap: {...}

# 9. Map Server - Map loading
map_server: {...}
```

---

# TF Debugging Commands

```bash
# View entire TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Echo specific transform
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link

# Monitor TF
ros2 run tf2_ros tf2_monitor

# Check TF in RViz
# Add TF display and enable frame names
```

---

# Lifecycle Node Management

```bash
# Get node state
ros2 lifecycle get /controller_server

# Available states:
# - unconfigured
# - inactive  
# - active
# - finalized

# Transition commands
ros2 lifecycle set /controller_server configure
ros2 lifecycle set /controller_server activate
ros2 lifecycle set /controller_server deactivate
ros2 lifecycle set /controller_server cleanup
```

---

# Action Server Interaction

```bash
# List action servers
ros2 action list

# Get action info
ros2 action info /navigate_to_pose

# Send navigation goal
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, 
   pose: {position: {x: 1.0, y: 0.5, z: 0.0}, 
   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"

# Cancel goal
ros2 action send_goal /navigate_to_pose --cancel
```
