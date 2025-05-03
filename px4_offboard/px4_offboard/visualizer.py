# #!/usr/bin/env python
# ############################################################################
# #
# #   Copyright (C) 2022 PX4 Development Team. All rights reserved.
# #
# # Redistribution and use in source and binary forms, with or without
# # modification, are permitted provided that the following conditions
# # are met:
# #
# # 1. Redistributions of source code must retain the above copyright
# #    notice, this list of conditions and the following disclaimer.
# # 2. Redistributions in binary form must reproduce the above copyright
# #    notice, this list of conditions and the following disclaimer in
# #    the documentation and/or other materials provided with the
# #    distribution.
# # 3. Neither the name PX4 nor the names of its contributors may be
# #    used to endorse or promote products derived from this software
# #    without specific prior written permission.
# #
# # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# # OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# # POSSIBILITY OF SUCH DAMAGE.
# #
# ############################################################################

# __author__ = "Malay Phadke"
# __contact__ = "malayp003@gmail.com"

# from re import M
# import numpy as np

# import rclpy
# from rclpy.node import Node
# from rclpy.clock import Clock
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# from px4_msgs.msg import VehicleAttitude
# from px4_msgs.msg import VehicleLocalPosition
# from px4_msgs.msg import TrajectorySetpoint
# from geometry_msgs.msg import PoseStamped, Point, TransformStamped
# from nav_msgs.msg import Path
# from visualization_msgs.msg import Marker

# import tf2_ros
# import tf_transformations
# import math

# def vector2PoseMsg(frame_id, position, attitude):
#     pose_msg = PoseStamped()
#     pose_msg.header.frame_id=frame_id
#     pose_msg.pose.orientation.w = attitude[0]
#     pose_msg.pose.orientation.x = attitude[1]
#     pose_msg.pose.orientation.y = attitude[2]
#     pose_msg.pose.orientation.z = attitude[3]
#     pose_msg.pose.position.x = position[0]
#     pose_msg.pose.position.y = position[1]
#     pose_msg.pose.position.z = position[2]
#     return pose_msg

# class PX4Visualizer(Node):

#     def __init__(self):
#         super().__init__('px4_visualizer')

#         # Configure dynamic subscriptions/publishers for multiple drones
#         qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
#             history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
#             depth=1,
#         )
#         # namespaces list via comma-separated string parameter
#         desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
#         param = self.declare_parameter('drone_namespaces', '', descriptor=desc)
#         # split CSV into list, ignore empties
#         self.namespaces = [ns for ns in param.value.split(',') if ns]

#         # storage
#         self.vehicle_attitudes = {}
#         self.vehicle_local_positions = {}
#         self.vehicle_local_velocities = {}
#         self.setpoint_positions = {}
#         self.vehicle_path_msgs = {}
#         self.setpoint_path_msgs = {}
#         self.pose_pubs = {}
#         self.velocity_pubs = {}
#         self.vehicle_path_pubs = {}
#         self.setpoint_path_pubs = {}
#         print(self.namespaces)
#         for idx, ns in enumerate(self.namespaces):
#             # init state
#             self.vehicle_attitudes[ns] = np.array([1.0, 0.0, 0.0, 0.0])
#             self.vehicle_local_positions[ns] = np.array([0.0, 0.0, 0.0])
#             self.vehicle_local_velocities[ns] = np.array([0.0, 0.0, 0.0])
#             self.setpoint_positions[ns] = np.array([0.0, 0.0, 0.0])
#             self.vehicle_path_msgs[ns] = Path()
#             self.setpoint_path_msgs[ns] = Path()
#             # subscriptions
#             self.create_subscription(
#                 VehicleAttitude,
#                 f'/{ns}/fmu/out/vehicle_attitude',
#                 self._make_attitude_callback(ns),
#                 qos_profile)
#             self.create_subscription(
#                 VehicleLocalPosition,
#                 f'/{ns}/fmu/out/vehicle_local_position',
#                 self._make_local_position_callback(ns),
#                 qos_profile)
#             self.create_subscription(
#                 TrajectorySetpoint,
#                 f'/{ns}/fmu/in/trajectory_setpoint',
#                 self._make_trajectory_callback(ns),
#                 qos_profile)
#             # publishers
#             self.pose_pubs[ns] = self.create_publisher(PoseStamped, f'/px4_visualizer/{ns}/vehicle_pose', 10)
#             self.velocity_pubs[ns] = self.create_publisher(Marker, f'/px4_visualizer/{ns}/vehicle_velocity', 10)
#             self.vehicle_path_pubs[ns] = self.create_publisher(Path, f'/px4_visualizer/{ns}/vehicle_path', 10)
#             self.setpoint_path_pubs[ns] = self.create_publisher(Path, f'/px4_visualizer/{ns}/setpoint_path', 10)
#         # common timer
#         timer_period = 0.05  # seconds
#         self.create_timer(timer_period, self.cmdloop_callback)

#         # TF broadcaster for map->drone frames
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

#     # callback factories for per-drone state updates
#     def _make_attitude_callback(self, ns):
#         def callback(msg):
#             # Convert PX4 quaternion [w,x,y,z] to tf quaternion [x,y,z,w]
#             ned = msg.q
#             ned_tf = [ned[1], ned[2], ned[3], ned[0]]
#             # static NED->ENU rotation: roll=π, pitch=0, yaw=π/2
#             q_ned2enu = tf_transformations.quaternion_from_euler(math.pi, 0.0, math.pi/2)
#             # apply rotation
#             enu_tf = tf_transformations.quaternion_multiply(q_ned2enu, ned_tf)
#             # convert back to PX4 format [w,x,y,z]
#             self.vehicle_attitudes[ns][0] = enu_tf[3]
#             self.vehicle_attitudes[ns][1] = enu_tf[0]
#             self.vehicle_attitudes[ns][2] = enu_tf[1]
#             self.vehicle_attitudes[ns][3] = enu_tf[2]

#         return callback

#     def _make_local_position_callback(self, ns):
#         def callback(msg):
#             pos = self.vehicle_local_positions[ns]
#             vel = self.vehicle_local_velocities[ns]
#             # proper NED->ENU: x_ENU = msg.y, y_ENU = msg.x, z_ENU = -msg.z
#             pos[0] = msg.y
#             pos[1] = msg.x
#             pos[2] = -msg.z
#             vel[0] = msg.vy; vel[1] = -msg.vx; vel[2] = -msg.vz
#         return callback
    

#     def _make_trajectory_callback(self, ns):
#         def callback(msg):
#             sp = self.setpoint_positions[ns]
#             sp[0] = msg.position[0]; sp[1] = -msg.position[1]; sp[2] = -msg.position[2]
#         return callback

#     def create_arrow_marker(self, id, tail, vector):
#         msg = Marker()
#         msg.action = Marker.ADD
#         msg.header.frame_id = 'map'
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.ns = 'arrow'
#         msg.id = id
#         msg.type = Marker.ARROW
#         msg.scale.x = 0.1
#         msg.scale.y = 0.2
#         msg.scale.z = 0.0
#         msg.color.r = 0.5
#         msg.color.g = 0.5
#         msg.color.b = 0.0
#         msg.color.a = 1.0
#         dt = 0.3
#         tail_point = Point()
#         tail_point.x = tail[0]
#         tail_point.y = tail[1]
#         tail_point.z = tail[2]
#         head_point = Point()
#         head_point.x = tail[0] + dt * vector[0]
#         head_point.y = tail[1] + dt * vector[1]
#         head_point.z = tail[2] + dt * vector[2]
#         msg.points = [tail_point, head_point]
#         return msg

#     def cmdloop_callback(self):
#         # current time for stamping and TF
#         now = self.get_clock().now().to_msg()
#         for idx, ns in enumerate(self.namespaces):
#             pos = self.vehicle_local_positions[ns]
#             att = self.vehicle_attitudes[ns]
#             # vehicle pose
#             pose_msg = vector2PoseMsg('map', pos, att)
#             pose_msg.header.stamp = now
#             self.pose_pubs[ns].publish(pose_msg)
#             # vehicle path
#             path = self.vehicle_path_msgs[ns]
#             path.header = pose_msg.header
#             path.poses.append(pose_msg)
#             self.vehicle_path_pubs[ns].publish(path)
#             # setpoint path
#             sp_msg = vector2PoseMsg('map', self.setpoint_positions[ns], att)
#             sp_path = self.setpoint_path_msgs[ns]
#             sp_path.header = sp_msg.header
#             sp_path.poses.append(sp_msg)
#             self.setpoint_path_pubs[ns].publish(sp_path)
#             # velocity arrow
#             vel_msg = self.create_arrow_marker(idx, pos, self.vehicle_local_velocities[ns])
#             vel_msg.header.stamp = now
#             vel_msg.id = idx
#             self.velocity_pubs[ns].publish(vel_msg)
#             # broadcast TF for this drone
#             t = TransformStamped()
#             t.header.stamp = now
#             t.header.frame_id = 'map'
#             t.child_frame_id = ns
#             t.transform.translation.x = float(pos[0])
#             t.transform.translation.y = float(pos[1])
#             t.transform.translation.z = float(pos[2])
#             t.transform.rotation.w = float(att[0])
#             t.transform.rotation.x = float(att[1])
#             t.transform.rotation.y = float(att[2])
#             t.transform.rotation.z = float(att[3])
#             self.tf_broadcaster.sendTransform(t)

# def main(args=None):
#     rclpy.init(args=args)

#     px4_visualizer = PX4Visualizer()

#     rclpy.spin(px4_visualizer)

#     px4_visualizer.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



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

from re import M
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

import tf2_ros, math, tf_transformations

def vector2PoseMsg(frame_id, position, attitude):
    pose_msg = PoseStamped()
    pose_msg.header.frame_id=frame_id
    pose_msg.pose.orientation.w = attitude[0]
    pose_msg.pose.orientation.x = attitude[1]
    pose_msg.pose.orientation.y = attitude[2]
    pose_msg.pose.orientation.z = attitude[3]
    pose_msg.pose.position.x = position[0]
    pose_msg.pose.position.y = position[1]
    pose_msg.pose.position.z = position[2]
    return pose_msg

class PX4Visualizer(Node):

    def __init__(self):
        super().__init__('px4_visualizer')

        # Configure dynamic subscriptions/publishers for multiple drones
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )
        # namespaces list via comma-separated string parameter
        desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        param = self.declare_parameter('drone_namespaces', '', descriptor=desc)
        # split CSV into list, ignore empties
        self.namespaces = [ns for ns in param.value.split(',') if ns]

        # storage
        self.vehicle_attitudes = {}
        self.vehicle_local_positions = {}
        self.vehicle_local_velocities = {}
        self.setpoint_positions = {}
        self.vehicle_path_msgs = {}
        self.setpoint_path_msgs = {}
        self.pose_pubs = {}
        self.velocity_pubs = {}
        self.vehicle_path_pubs = {}
        self.setpoint_path_pubs = {}
        print(self.namespaces)
        for idx, ns in enumerate(self.namespaces):
            # init state
            self.vehicle_attitudes[ns] = np.array([1.0, 0.0, 0.0, 0.0])
            self.vehicle_local_positions[ns] = np.array([0.0, 0.0, 0.0])
            self.vehicle_local_velocities[ns] = np.array([0.0, 0.0, 0.0])
            self.setpoint_positions[ns] = np.array([0.0, 0.0, 0.0])
            self.vehicle_path_msgs[ns] = Path()
            self.setpoint_path_msgs[ns] = Path()
            # subscriptions
            self.create_subscription(
                VehicleAttitude,
                f'/{ns}/fmu/out/vehicle_attitude',
                self._make_attitude_callback(ns),
                qos_profile)
            self.create_subscription(
                VehicleLocalPosition,
                f'/{ns}/fmu/out/vehicle_local_position',
                self._make_local_position_callback(ns),
                qos_profile)
            self.create_subscription(
                TrajectorySetpoint,
                f'/{ns}/fmu/in/trajectory_setpoint',
                self._make_trajectory_callback(ns),
                qos_profile)
            # publishers
            self.pose_pubs[ns] = self.create_publisher(PoseStamped, f'/px4_visualizer/{ns}/vehicle_pose', 10)
            self.velocity_pubs[ns] = self.create_publisher(Marker, f'/px4_visualizer/{ns}/vehicle_velocity', 10)
            self.vehicle_path_pubs[ns] = self.create_publisher(Path, f'/px4_visualizer/{ns}/vehicle_path', 10)
            self.setpoint_path_pubs[ns] = self.create_publisher(Path, f'/px4_visualizer/{ns}/setpoint_path', 10)
        # common timer
        timer_period = 0.05  # seconds
        self.create_timer(timer_period, self.cmdloop_callback)

        # TF broadcaster for map->drone frames
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def _make_attitude_callback(self, ns):
        def callback(msg):
            # Original NED quaternion from PX4 [w, x, y, z]
            q_ned = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
            # Create NED to ENU rotation quaternion (yaw=π/2)
            q_ned2enu = tf_transformations.quaternion_from_euler(0.0, 0.0, math.pi/2)
            # Combine rotations: q_enu = q_ned2enu * q_ned
            q_enu = tf_transformations.quaternion_multiply(q_ned2enu, q_ned)
            # Store ENU quaternion [w, x, y, z]
            self.vehicle_attitudes[ns][0] = q_enu[3]  # w
            self.vehicle_attitudes[ns][1] = q_enu[0]  # x
            self.vehicle_attitudes[ns][2] = q_enu[1]  # y
            self.vehicle_attitudes[ns][3] = q_enu[2]  # z
        return callback

    # 2. Corrected Local Position Callback
    def _make_local_position_callback(self, ns):
        def callback(msg):
            pos = self.vehicle_local_positions[ns]
            vel = self.vehicle_local_velocities[ns]
            # NED to ENU position
            pos[0] = msg.y   # ENU x = NED y
            pos[1] = msg.x   # ENU y = NED x
            pos[2] = -msg.z  # ENU z = -NED z
            # NED to ENU velocity
            vel[0] = msg.vy  # ENU vx = NED vy (East)
            vel[1] = msg.vx  # ENU vy = NED vx (North)
            vel[2] = -msg.vz # ENU vz = -NED vz (Down)
        return callback

    # 3. Corrected Trajectory Callback
    def _make_trajectory_callback(self, ns):
        def callback(msg):
            sp = self.setpoint_positions[ns]
            # NED to ENU setpoint
            sp[0] = msg.position[1]  # ENU x = NED y
            sp[1] = msg.position[0]  # ENU y = NED x
            sp[2] = -msg.position[2] # ENU z = -NED z
        return callback

    def create_arrow_marker(self, id, tail, vector):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = 'arrow'
        msg.id = id
        msg.type = Marker.ARROW
        msg.scale.x = 0.1
        msg.scale.y = 0.2
        msg.scale.z = 0.0
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.0
        msg.color.a = 1.0
        dt = 0.3
        tail_point = Point()
        tail_point.x = tail[0]
        tail_point.y = tail[1]
        tail_point.z = tail[2]
        head_point = Point()
        head_point.x = tail[0] + dt * vector[0]
        head_point.y = tail[1] + dt * vector[1]
        head_point.z = tail[2] + dt * vector[2]
        msg.points = [tail_point, head_point]
        return msg

    def cmdloop_callback(self):
        # current time for stamping and TF
        now = self.get_clock().now().to_msg()
        for idx, ns in enumerate(self.namespaces):
            pos = self.vehicle_local_positions[ns]
            att = self.vehicle_attitudes[ns]
            # vehicle pose
            pose_msg = vector2PoseMsg('map', pos, att)
            pose_msg.header.stamp = now
            self.pose_pubs[ns].publish(pose_msg)
            # vehicle path
            path = self.vehicle_path_msgs[ns]
            path.header = pose_msg.header
            path.poses.append(pose_msg)
            self.vehicle_path_pubs[ns].publish(path)
            # setpoint path
            sp_msg = vector2PoseMsg('map', self.setpoint_positions[ns], att)
            sp_path = self.setpoint_path_msgs[ns]
            sp_path.header = sp_msg.header
            sp_path.poses.append(sp_msg)
            self.setpoint_path_pubs[ns].publish(sp_path)
            # velocity arrow
            vel_msg = self.create_arrow_marker(idx, pos, self.vehicle_local_velocities[ns])
            vel_msg.header.stamp = now
            vel_msg.id = idx
            self.velocity_pubs[ns].publish(vel_msg)
            # broadcast TF for this drone
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map'
            t.child_frame_id = ns
            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])
            t.transform.rotation.w = float(att[0])
            t.transform.rotation.x = float(att[1])
            t.transform.rotation.y = float(att[2])
            t.transform.rotation.z = float(att[3])
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    px4_visualizer = PX4Visualizer()

    rclpy.spin(px4_visualizer)

    px4_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()