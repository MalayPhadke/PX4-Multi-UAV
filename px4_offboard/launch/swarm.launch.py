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
__contact__ = "https://malayphadke.github.io/portfolio/"

###funny thing
##ros gz bridge doesnt work   what():  Invalid topic name: topic name must not contain characters other than alphanumerics, '_', '~', '{', or '}':
#   '/world/default/model/x500_depth_0/link/OakD-Lite/base_link/sensor/IMX214/image'
##just changed OakD-Lite to OakDLite and it works now
###hehe spent days on this 



from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rcl_interfaces.msg import ParameterType
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    # Bridge nodes for camera-related topics
    model_names = ["x500_depth_1", "x500_depth_2"]  # Add more drones if needed
 
    world_name = 'maze'
    camera_bridge_nodes = []
    for model in model_names:
        # IMX214 topics
        camera_bridge_nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f"gz_bridge_{model}_imx214_camera_info",
            arguments=[f"/world/{world_name}/model/{model}/link/OakDLite/base_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"],
            parameters=[{"lazy": True}],
            output='screen',
        ))
        camera_bridge_nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f"gz_bridge_{model}_imx214_image",
            arguments=[f"/world/{world_name}/model/{model}/link/OakDLite/base_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image"],
            parameters=[{"lazy": True}],
            output='screen',
        ))
        # StereoOV7251 topics
        camera_bridge_nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f"gz_bridge_{model}_stereoov7251_camera_info",
            arguments=[f"/world/{world_name}/model/{model}/link/OakDLite/base_link/sensor/StereoOV7251/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"],
            parameters=[{"lazy": True}],
            output='screen',
        ))
        camera_bridge_nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f"gz_bridge_{model}_stereoov7251_depth_image",
            arguments=[f"/world/{world_name}/model/{model}/link/OakDLite/base_link/sensor/StereoOV7251/depth_image@sensor_msgs/msg/Image@gz.msgs.Image"],
            parameters=[{"lazy": True}],
            output='screen',
        ))
        camera_bridge_nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f"gz_bridge_{model}_stereoov7251_depth_points",
            arguments=[f"/world/{world_name}/model/{model}/link/OakDLite/base_link/sensor/StereoOV7251/depth_image/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"],
            parameters=[{"lazy": True}],
            output='screen',
        ))

    static_tf_nodes = []
    for model in model_names:
        # IMX214 sensor frame (match header.frame_id from Gazebo)
        static_tf_nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f"static_tf_{model}_imx214",
            arguments=['--frame-id', f"{model}/OakDLite/base_link",
              '--child-frame-id', f"{model}/OakDLite/base_link/IMX214",
              '--x', '0.12', '--y', '0.03', '--z', '0.242',
              '--roll', '0', '--pitch', '0', '--yaw', '0'],
            output='screen'
        ))
        # StereoOV7251 sensor frame
        static_tf_nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f"static_tf_{model}_stereoov7251",
            arguments=['--frame-id', f"{model}/OakDLite/base_link",
              '--child-frame-id', f"{model}/OakDLite/base_link/StereoOV7251",
              '--x', '0.12', '--y', '0.03', '--z', '0.242',
              '--roll', '0', '--pitch', '0', '--yaw', '0'],
            output='screen'
        ))

    return LaunchDescription([
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='bridge_clock',
            arguments=[f"/world/{world_name}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"],
            remappings=[(f'/world/{world_name}/clock', '/clock')],
            output='screen'
        ),
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='visualizer',
            name='visualizer',
            parameters=[{'drone_namespaces': 'px4_1,px4_2'}, {'use_sim_time': True}],
        ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='swarm_processes',
            name='swarm_processes',
            prefix='gnome-terminal --'
        ),
        # Drone 1 controller
        Node(
            package='px4_offboard',
            executable='velocity_control',
            name='velocity_control_px41',
            output='screen',
            parameters=[{'drone_name': 'x500_depth_1', 'vehicle_namespace': 'px4_1'}],
        ),
        # Drone 2 controller
        Node(
            package='px4_offboard',
            executable='velocity_control',
            name='velocity_control_px42',
            output='screen',
            parameters=[{'drone_name': 'x500_depth_2', 'vehicle_namespace': 'px4_2'}],
        ),
        # Teleoperation node for swarm control
        Node(
            package='px4_offboard',
            executable='teleop_swarm',
            name='teleop_swarm',
            prefix='gnome-terminal --',
        ),
        # Node(
        #     package='px4_offboard',
        #     executable='odom_to_tf',
        #     name='odom_to_tf',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}],
        # ),
        # All static transforms for sensor frames
        *static_tf_nodes,
        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     parameters=[{'use_sim_time': True}],
        #     arguments=['-d', os.path.join(package_dir, 'visualize.rviz')]
        # ),
        TimerAction(
            period=5.0,
            actions=camera_bridge_nodes
        )
    ])
