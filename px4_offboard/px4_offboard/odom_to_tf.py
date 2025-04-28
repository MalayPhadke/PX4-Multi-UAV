#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf_transformations

class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        # TF broadcaster for map->drone frames
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # QoS for subscriptions
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

        # drones and their child frames
        self.drones = {
            'px4_1': 'x500_depth_1/OakDLite/base_link',
            'px4_2': 'x500_depth_2/OakDLite/base_link'
        }

        # store latest PoseStamped per drone
        self.latest_pose = {ns: None for ns in self.drones}

        # subscribe to the visualizer PoseStamped topics
        for ns in self.drones:
            topic = f'/px4_visualizer/{ns}/vehicle_pose'
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, ns=ns: self.pose_cb(ns, msg),
                qos)

        # sensor static offsets and quaternion (base_link -> camera_link)
        self.sensor_offset = (.12, 0.03, 0.242)
        self.sensor_quat = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        self.sensor_suffixes = ['IMX214', 'StereoOV7251']

        # timer to broadcast base_link transforms at 30 Hz
        self.create_timer(1/30, self.broadcast_base_transforms)
        # timer to broadcast sensor transforms at 1.25 Hz
        self.create_timer(1/1.25, self.broadcast_sensor_transforms)

    def pose_cb(self, ns, msg: PoseStamped):
        """Store the latest pose for namespace ns."""
        self.latest_pose[ns] = msg

    def broadcast_base_transforms(self):
        for ns, child in self.drones.items():
            msg = self.latest_pose.get(ns)
            if msg is None:
                continue
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = ns
            t.child_frame_id = child
            t.transform.translation.x = msg.pose.position.x
            t.transform.translation.y = msg.pose.position.y
            t.transform.translation.z = msg.pose.position.z
            t.transform.rotation.x = msg.pose.orientation.x
            t.transform.rotation.y = msg.pose.orientation.y
            t.transform.rotation.z = msg.pose.orientation.z
            t.transform.rotation.w = msg.pose.orientation.w
            self.tf_broadcaster.sendTransform(t)

    def broadcast_sensor_transforms(self):
        now = self.get_clock().now().to_msg()
        for ns, child in self.drones.items():
            msg = self.latest_pose.get(ns)
            if msg is None:
                continue
            for suffix in self.sensor_suffixes:
                sensor_t = TransformStamped()
                sensor_t.header.stamp = now
                sensor_t.header.frame_id = child
                sensor_t.child_frame_id = f"{child}/{suffix}"
                sensor_t.transform.translation.x = self.sensor_offset[0]
                sensor_t.transform.translation.y = self.sensor_offset[1]
                sensor_t.transform.translation.z = self.sensor_offset[2]
                sensor_t.transform.rotation.x = self.sensor_quat[0]
                sensor_t.transform.rotation.y = self.sensor_quat[1]
                sensor_t.transform.rotation.z = self.sensor_quat[2]
                sensor_t.transform.rotation.w = self.sensor_quat[3]
                self.tf_broadcaster.sendTransform(sensor_t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
