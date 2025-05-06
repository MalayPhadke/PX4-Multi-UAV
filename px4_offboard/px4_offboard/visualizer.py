# #!/usr/bin/env python3

__author__ = "Malay Phadke"
__contact__ = "malayp003@gmail.com"

import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

import tf2_ros
import tf_transformations
from tf2_ros import StaticTransformBroadcaster

# Improved frame_transforms functions for consistent NED to ENU conversion
class FrameTransforms:
    """Python implementation of frame transforms between NED and ENU"""
    
    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        """Convert euler angles to quaternion"""
        return tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    
    @staticmethod
    def ned_to_enu_quaternion(q_ned):
        """Transform quaternion from NED to ENU frame.
        
        Args:
            q_ned: Quaternion in NED frame [w, x, y, z]
            
        Returns:
            Quaternion in ENU frame [w, x, y, z]
        """
        # Method 1: Using rotation matrices (more reliable)
        # Convert PX4 quaternion [w, x, y, z] to TF quaternion [x, y, z, w]
        q_ned_tf = [q_ned[1], q_ned[2], q_ned[3], q_ned[0]]
        
        # Define the NED to ENU rotation quaternion
        # This is a 90-degree CCW rotation around Z followed by a 180-degree rotation around X
        q_rot = tf_transformations.quaternion_from_euler(math.pi, 0.0, math.pi/2)
        
        # Apply transformation using quaternion multiplication
        q_enu_tf = tf_transformations.quaternion_multiply(q_rot, q_ned_tf)
        
        # Convert back to PX4 order [w, x, y, z]
        return [q_enu_tf[3], q_enu_tf[0], q_enu_tf[1], q_enu_tf[2]]
    
    @staticmethod
    def ned_to_enu_position(pos_ned):
        """Transform position vector from NED to ENU frame.
        
        Args:
            pos_ned: Position in NED frame [north, east, down]
            
        Returns:
            Position in ENU frame [east, north, up]
        """
        # NED to ENU transformation:
        # x_enu = y_ned (east = east)
        # y_enu = x_ned (north = north)
        # z_enu = -z_ned (up = -down)
        return np.array([pos_ned[1], pos_ned[0], -pos_ned[2]])
    
    @staticmethod
    def ned_to_enu_velocity(vel_ned):
        """Transform velocity vector from NED to ENU frame.
        
        Args:
            vel_ned: Velocity in NED frame [vn, ve, vd]
            
        Returns:
            Velocity in ENU frame [ve, vn, vu]
        """
        return np.array([vel_ned[1], vel_ned[0], -vel_ned[2]])

def vector2PoseMsg(frame_id, position, attitude):
    """Convert position and attitude to PoseStamped message"""
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.orientation.w = float(attitude[0])
    pose_msg.pose.orientation.x = float(attitude[1])
    pose_msg.pose.orientation.y = float(attitude[2])
    pose_msg.pose.orientation.z = float(attitude[3])
    pose_msg.pose.position.x = float(position[0])
    pose_msg.pose.position.y = float(position[1])
    pose_msg.pose.position.z = float(position[2])
    return pose_msg

class PX4Visualizer(Node):

    def __init__(self):
        super().__init__('px4_visualizer')
        
        # Configure dynamic subscriptions/publishers for multiple drones

        # Configure QoS profile for subscription
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )
        
        # Get namespaces from parameter
        desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        param = self.declare_parameter('drone_namespaces', '', descriptor=desc)
        self.namespaces = [ns for ns in param.value.split(',') if ns]
        
        # Print discovered namespaces
        self.get_logger().info(f"Visualizing drones with namespaces: {self.namespaces}")

        # Initialize storage for vehicle state
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
        
        # Set up publishers and subscribers for each drone namespace
        for idx, ns in enumerate(self.namespaces):
            # Initialize state storage
            self.vehicle_attitudes[ns] = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
            self.vehicle_local_positions[ns] = np.array([0.0, 0.0, 0.0])
            self.vehicle_local_velocities[ns] = np.array([0.0, 0.0, 0.0])
            self.setpoint_positions[ns] = np.array([0.0, 0.0, 0.0])
            self.vehicle_path_msgs[ns] = Path()
            self.setpoint_path_msgs[ns] = Path()
            
            # Create subscriptions
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
            
            # Create publishers
            self.pose_pubs[ns] = self.create_publisher(PoseStamped, f'/px4_visualizer/{ns}/vehicle_pose', 10)
            self.velocity_pubs[ns] = self.create_publisher(Marker, f'/px4_visualizer/{ns}/vehicle_velocity', 10)
            self.vehicle_path_pubs[ns] = self.create_publisher(Path, f'/px4_visualizer/{ns}/vehicle_path', 10)
            self.setpoint_path_pubs[ns] = self.create_publisher(Path, f'/px4_visualizer/{ns}/setpoint_path', 10)
        
        # Set up timer for periodic visualization updates
        timer_period = 0.05  # seconds
        self.create_timer(timer_period, self.cmdloop_callback)

        # TF broadcaster for transforms
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Static TF broadcaster for establishing the correct map frame 
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Also publish a fixed static transform for map frame alignment
        self.declare_parameter('fix_map_orientation', True) 
        self.fix_map_orientation = self.get_parameter('fix_map_orientation').value
        
        # If enabled, publish a static transform to fix map orientation
        if self.fix_map_orientation:
            self.publish_map_frame()

    def _make_attitude_callback(self, ns):
        """Create a callback function for vehicle attitude messages"""
        def callback(msg):
            # Use frame_transforms to convert quaternion from NED to ENU
            quat_enu = FrameTransforms.ned_to_enu_quaternion(msg.q)
            self.vehicle_attitudes[ns] = np.array(quat_enu)
            self.get_logger().debug(f"Attitude received for {ns}: {quat_enu}")
        return callback

    def _make_local_position_callback(self, ns):
        """Create a callback function for vehicle local position messages"""
        def callback(msg):
            # Extract NED position and velocity
            pos_ned = np.array([msg.x, msg.y, msg.z])
            vel_ned = np.array([msg.vx, msg.vy, msg.vz])
            
            # Transform to ENU using frame_transforms
            pos_enu = FrameTransforms.ned_to_enu_position(pos_ned)
            vel_enu = FrameTransforms.ned_to_enu_velocity(vel_ned)
            
            self.vehicle_local_positions[ns] = pos_enu
            self.vehicle_local_velocities[ns] = vel_enu
            self.get_logger().debug(f"Position received for {ns}: NED={pos_ned}, ENU={pos_enu}")
        return callback

    def _make_trajectory_callback(self, ns):
        """Create a callback function for trajectory setpoint messages"""
        def callback(msg):
            # Extract NED setpoint
            sp_ned = np.array([msg.position[0], msg.position[1], msg.position[2]])
            
            # Transform to ENU using frame_transforms
            sp_enu = FrameTransforms.ned_to_enu_position(sp_ned)
            self.setpoint_positions[ns] = sp_enu
            self.get_logger().debug(f"Setpoint received for {ns}: NED={sp_ned}, ENU={sp_enu}")
        return callback

    def create_arrow_marker(self, id, tail, vector):
        """Create an arrow marker to visualize a vector quantity (like velocity)"""
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = 'map'
        msg.ns = 'arrow'
        msg.id = id
        msg.type = Marker.ARROW
        msg.scale.x = 0.1  # shaft diameter
        msg.scale.y = 0.2  # head diameter
        msg.scale.z = 0.0  # not used
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.0
        msg.color.a = 1.0
        
        # Scale factor for velocity visualization
        dt = 0.3
        
        # Arrow start point (tail)
        tail_point = Point()
        tail_point.x = tail[0]
        tail_point.y = tail[1]
        tail_point.z = tail[2]
        
        # Arrow end point (head)
        head_point = Point()
        head_point.x = tail[0] + dt * vector[0]
        head_point.y = tail[1] + dt * vector[1]
        head_point.z = tail[2] + dt * vector[2]
        
        msg.points = [tail_point, head_point]
        return msg

    def cmdloop_callback(self):
        """Periodic callback to publish visualization messages"""
        # Get current time for message timestamps
        now = self.get_clock().now().to_msg()
        self.drones = {
            'px4_1': 'x500_depth_1/OakDLite/base_link',
            'px4_2': 'x500_depth_2/OakDLite/base_link'
        }
        for idx, ns in enumerate(self.namespaces):
            pos = self.vehicle_local_positions[ns]
            att = self.vehicle_attitudes[ns]
            
            # Publish vehicle pose
            pose_msg = vector2PoseMsg('map', pos, att)
            pose_msg.header.stamp = now
            self.pose_pubs[ns].publish(pose_msg)
            
            # Update and publish vehicle path
            self.vehicle_path_msgs[ns].header.frame_id = 'map'
            self.vehicle_path_msgs[ns].header.stamp = now
            self.vehicle_path_msgs[ns].poses.append(pose_msg)
            
            # Limit path length to prevent memory growth
            if len(self.vehicle_path_msgs[ns].poses) > 100:
                self.vehicle_path_msgs[ns].poses = self.vehicle_path_msgs[ns].poses[-100:]
                
            self.vehicle_path_pubs[ns].publish(self.vehicle_path_msgs[ns])

            # Update and publish setpoint path
            sp_msg = vector2PoseMsg('map', self.setpoint_positions[ns], att)
            sp_msg.header.stamp = now
            sp_msg.header.frame_id = 'map'
            
            self.setpoint_path_msgs[ns].header.frame_id = 'map'
            self.setpoint_path_msgs[ns].header.stamp = now
            self.setpoint_path_msgs[ns].poses.append(sp_msg)
            
            # Limit setpoint path length
            if len(self.setpoint_path_msgs[ns].poses) > 100:
                self.setpoint_path_msgs[ns].poses = self.setpoint_path_msgs[ns].poses[-100:]
                
            self.setpoint_path_pubs[ns].publish(self.setpoint_path_msgs[ns])
            
            # Publish velocity arrow
            vel_msg = self.create_arrow_marker(idx, pos, self.vehicle_local_velocities[ns])
            vel_msg.header.stamp = now
            self.velocity_pubs[ns].publish(vel_msg)
            
            # Broadcast TF from map to drone frame
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map'  # Now it has correct ENU orientation
            t.child_frame_id = ns
            
            # Translation component (position)
            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])
            
            # Rotation component (orientation)
            t.transform.rotation.w = float(att[0])
            t.transform.rotation.x = float(att[1])
            t.transform.rotation.y = float(att[2])
            t.transform.rotation.z = float(att[3])
            
            self.tf_broadcaster.sendTransform(t)
            
            # Additionally broadcast a TF from drone to baselink frame for better RViz compatibility
            # This ensures consistent ENU orientation in the baselink frame
            bl = TransformStamped()
            bl.header.stamp = now
            bl.header.frame_id = ns
            bl.child_frame_id = self.drones[ns]
            
            # Identity transform (no additional offset)
            bl.transform.translation.x = 0.0
            bl.transform.translation.y = 0.0
            bl.transform.translation.z = 0.0
            
            # Identity rotation
            bl.transform.rotation.w = 1.0
            bl.transform.rotation.x = 0.0
            bl.transform.rotation.y = 0.0
            bl.transform.rotation.z = 0.0
            
            self.tf_broadcaster.sendTransform(bl)
    
    def publish_map_frame(self):
        """Publish a static transform for the map frame with proper ENU orientation"""
        # Create an explicit ENU-aligned map frame from scratch
        # This overwrites any existing map frame orientation
        
        map_transform = TransformStamped()
        map_transform.header.stamp = self.get_clock().now().to_msg()
        map_transform.header.frame_id = "enu_reference"  # A new reference frame with correct ENU orientation
        map_transform.child_frame_id = "map"            # The map frame we're fixing
        
        # Identity translation (same origin)
        map_transform.transform.translation.x = 0.0
        map_transform.transform.translation.y = 0.0
        map_transform.transform.translation.z = 0.0
        
        # To fix the map frame with Z pointing upwards, we need to rotate it 180 degrees around X
        # This transforms from a frame with Z pointing down to one with Z pointing up
        q_rotation = tf_transformations.quaternion_from_euler(math.pi, 0.0, math.pi/2)  # 180° around X-axis
        
        map_transform.transform.rotation.w = q_rotation[3]
        map_transform.transform.rotation.x = q_rotation[0]
        map_transform.transform.rotation.y = q_rotation[1]
        map_transform.transform.rotation.z = q_rotation[2]
        
        # Publish static transform
        self.static_tf_broadcaster.sendTransform(map_transform)
        self.get_logger().info("Published corrected map frame with Z axis pointing upwards")

def main(args=None):
    rclpy.init(args=args)
    px4_visualizer = PX4Visualizer()
    rclpy.spin(px4_visualizer)
    px4_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()