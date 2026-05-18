# Autonomous Navigation with ROS2: WATonomous ASD Assignment

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.


1. This assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

Link to Onboarding Assignment: https://wiki.watonomous.ca/

---

## Navigation Implementation

### Problem

We have a robot with tank drive in a static environment that must autonomously navigate to any given point chosen by the user. In this environment, we directly have access to the exact position and orientation of the robot. We also have the distance to surrounding obstacles through a lidar sensor. 

---

### Implemented Solution

As raw lidar scans arrive on the `/lidar` topic, 4 ROS nodes make a pipeline to eventually generate velocity commands on the `/cmd_vel` topic, with the robot stopping upon reaching the goal.

### Nodes

- **Costmap** — converts each lidar scan into a local occupancy grid centered on the robot, marking obstacles and inflating cost around them
- **Map Memory** — fuses incoming costmaps into a persistent global map using odometry to transform local coordinates into world coordinates
- **Planner** — runs A* on the global map to find a path from the robot's current position to a user-specified goal, replanning as the map updates
- **Control** — follows the planned path using Pure Pursuit control, publishing velocity commands to `/cmd_vel` and stopping when the goal is reached

### Stack

- **ROS 2 Humble**: opensource middleware suite that handles all interprocess communication 
- **Gazebo**: robot simulator providing lidar, odometry, and physics
- **Docker**: containerization
- **Foxglove**: visualization and debugging of ROS topics in real time

### ROS Topics Used:

-  `/lidar`  Gazebo → Costmap 

-  `/costmap`  Costmap → Map Memory 

-  `/odom/filtered`  Gazebo → Map Memory, Planner, Control 

-  `/map`  Map Memory → Planner 

-  `/goal_point`  User → Planner 

-  `/path`  Planner → Control 

- `/cmd_vel`  Control → Gazebo 



