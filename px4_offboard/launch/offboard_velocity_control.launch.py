#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Malay Phadke"
__contact__ = "malayp003@gmail.com"

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    # Bridge nodes for camera-related topics
    camera_bridge_nodes = [
        # RGB Camera Image
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_camera',
            arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
            parameters=[{"lazy": True}],  # Lazy mode waits for topic
            output='screen'
        ),

        # Camera Info (Calibration, Intrinsic Parameters)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_camera_info',
            arguments=['/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            parameters=[{"lazy": True}],
            output='screen'
        ),

        # Depth Camera Image
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_depth_camera',
            arguments=['/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image'],
            parameters=[{"lazy": True}],
            output='screen'
        ),

        # Depth Camera Point Cloud
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_depth_camera_points',
            arguments=['/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
            parameters=[{"lazy": True}],
            output='screen'
        ),
    ]
    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='visualizer',
            name='visualizer'
        ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),
        # Node(
        #     package='px4_offboard',
        #     namespace='px4_offboard',
        #     executable='control',
        #     name='control',
        #     prefix='gnome-terminal --',
        # ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='velocity_control',
            name='velocity'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        ),
        TimerAction(
            period=5.0,
            actions=camera_bridge_nodes
        )

    ])
