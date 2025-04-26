# px4_offboard

**ROS 2 Multi-UAV PX4 Interface**

This package provides a set of ROS 2 Python nodes and launch files for offboard velocity control, visualization, and swarm management of PX4-based drones in Gazebo (Ignition). It dynamically bridges sensor topics (camera, depth, point cloud) via `ros_gz_bridge` and publishes static transforms for RViz displays.

## Features

- Offboard velocity control node for individual drones (`velocity_control`)
- Automatic arm and takeoff sequence (`swarm_auto`)
- Teleoperation control via keyboard (`teleop_swarm`)
- Multi-drone visualizer with RViz tf and trajectories (`visualizer`)
- Swarm process manager (`swarm_processes`)
- Dynamic ROS-Gazebo topic bridging for IMX214 and StereoOV7251 sensors
- Static TF publishers for camera frames

## Directory Structure

```
px4_offboard/
├── CMakeLists.txt         # Build settings (ament_python)
├── package.xml            # ROS 2 package manifest
├── setup.py               # Python package setup
├── setup.cfg              # Linter config
├── resource/visualize.rviz# RViz config
├── launch/
│   ├── offboard_velocity_control.launch.py  # Velocity control demo
│   └── swarm.launch.py                     # Swarm + sensor bridge
└── px4_offboard/          # Python modules
    ├── control.py
    ├── processes.py
    ├── swarm_auto.py
    ├── swarm_processes.py
    ├── teleop_swarm.py
    ├── velocity_control.py
    └── visualizer.py
``` 

## Installation

```bash
cd /path/to/your/ws/src
git clone <this-repo>
cd /path/to/your/ws
dependencies=(
  ros-humble-ros-gz-bridge
  ros-humble-tf2-ros
  ros-humble-nav-msgs
  ros-humble-visualization-msgs
)
sudo apt update && sudo apt install "${dependencies[@]}"
colcon build --packages-select px4_offboard
echo "source install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Launch Examples

### Single Drone Velocity Control

```bash
ros2 launch px4_offboard offboard_velocity_control.launch.py
```

### Multi-Drone Swarm with Sensor Bridge

Edit `launch/swarm.launch.py` to set `model_names` and `world_name` as needed.

```bash
ros2 launch px4_offboard swarm.launch.py
```

## Nodes and Scripts

Registered console scripts (via `setup.py`):

- `visualizer`         &mdash; RViz multi-drone visualizer
- `velocity_control`   &mdash; Offboard velocity control node
- `control`            &mdash; (keyboard) teleop twist node
- `processes`          &mdash; Spawns swarm processes in terminals
- `swarm_processes`    &mdash; Internal swarm manager
- `teleop_swarm`       &mdash; Teleoperation for swarm
- `swarm_auto`         &mdash; Automated arm + hover sequence

## Configuration

- **`model_names`**: List of model IDs in Gazebo (e.g. `x500_depth_1, x500_depth_2`)
- **`world_name`**: Gazebo world to use (default: `simple_tunnel_03`)
- Sensor topic bridges and static TFs are generated dynamically based on these.

## Troubleshooting

- **RViz Message Filter**: Increase the `Queue Size` in Image/PointCloud displays if you see warnings about dropped messages.
- Ensure Gazebo is publishing the camera topics under `/world/<world_name>/model/<model>/link/...`.

## Contributing

Feel free to open issues and pull requests for new features, bug fixes, or improvements.

---

*Maintainer*: Malay Phadke (<malayp003@gmail.com>)*
