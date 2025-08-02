# ROS 2 Dijkstra Path Planner

![ROS 2](https://img.shields.io/badge/ROS-2-brightgreen)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![License](https://img.shields.io/badge/License-MIT-green)

A ROS 2 package implementing Dijkstra's algorithm for 2D grid-based path planning, with support for dynamic map updates and path visualization.

## Features

- 🗺️ Configurable grid maps via parameters or topics
- 🚦 Obstacle avoidance in 2D environments
- 📡 ROS 2 interface for integration with other nodes
- 📊 Real-time path visualization in RViz
- ⚙️ Supports both static and dynamic maps

## Installation

### Prerequisites
- ROS 2 Humble (or newer)
- Python 3.8+
- NumPy (`sudo apt install python3-numpy`)

### Building the Package
```bash
cd ~/ros2_ws/src
git clone https://github.com/Ayushb10212/dijkstra_planner.git
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select dijkstra_planner
source install/setup.bash
Usage
Basic Demo
bash

# Run with default parameters
ros2 launch dijkstra_planner dijkstra_launch.py

# Or with custom config
ros2 run dijkstra_planner dijkstra_node --ros-args --params-file src/dijkstra_planner/config/grid_config.yaml


