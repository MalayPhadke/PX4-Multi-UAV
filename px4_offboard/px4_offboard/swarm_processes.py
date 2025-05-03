#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time
import os

# Set environment variables for Gazebo
os.environ["GZ_SIM_RESOURCE_PATH"] = f"{os.environ['HOME']}/PX4-Autopilot/Tools/simulation/gz/models:{os.environ['HOME']}/PX4-Autopilot/Tools/simulation/gz/worlds"

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",
    
    # Start Gazebo with the maze world first
    f"cd ~/PX4-Autopilot && gz sim -r {os.environ['HOME']}/PX4-Autopilot/Tools/simulation/gz/worlds/maze.sdf",
    
    # Wait for Gazebo to fully start before launching PX4
    "sleep 5",
    
    # Run the PX4 SITL simulation with maze world
    "cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_depth ./build/px4_sitl_default/bin/px4 -i 1",
    "cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='0,1,0.5' PX4_SIM_MODEL=gz_x500_depth ./build/px4_sitl_default/bin/px4 -i 2",

    # Run QGroundControl
    "cd ~/Downloads && ./QGroundControl.AppImage"
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(2)
