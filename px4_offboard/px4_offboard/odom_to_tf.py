# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy
# from geometry_msgs.msg import PoseStamped, TransformStamped
# import tf2_ros
# import tf_transformations
# import math

# class OdomToTf(Node):
#     def __init__(self):
#         super().__init__('odom_to_tf')
#         # TF broadcaster for map->drone frames
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

#         # QoS for subscriptions
#         qos = QoSProfile(depth=10)
#         qos.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

#         # drones and their child frames
#         self.drones = {
#             'px4_1': 'x500_depth_1/OakDLite/base_link',
#             'px4_2': 'x500_depth_2/OakDLite/base_link'
#         }

#         # store latest PoseStamped per drone
#         self.latest_pose = {ns: None for ns in self.drones}

#         # subscribe to the visualizer PoseStamped topics
#         for ns in self.drones:
#             topic = f'/px4_visualizer/{ns}/vehicle_pose'
#             self.create_subscription(
#                 PoseStamped,
#                 topic,
#                 lambda msg, ns=ns: self.pose_cb(ns, msg),
#                 qos)

#         self.sensor_suffixes = ['IMX214', 'StereoOV7251']

#         # timer to broadcast base_link transforms at 30 Hz
#         self.create_timer(1/30, self.broadcast_base_transforms)
#         # timer to broadcast sensor transforms at 1.25 Hz
#         # self.create_timer(1/1.25, self.broadcast_sensor_transforms)

#     def pose_cb(self, ns, msg: PoseStamped):
#         """Store the latest pose for namespace ns."""
#         self.latest_pose[ns] = msg

#     def broadcast_base_transforms(self):
#         for ns, child in self.drones.items():
#             msg = self.latest_pose.get(ns)
#             if msg is None:
#                 continue
#             # Log quaternion orientation for debugging
#             # self.get_logger().info(f"Quaternion for {ns}: w={msg.pose.orientation.w}, " +
#             #                     f"x={msg.pose.orientation.x}, y={msg.pose.orientation.y}, " +
#             #                     f"z={msg.pose.orientation.z}")
#             t = TransformStamped()
#             t.header.stamp = msg.header.stamp
#             t.header.frame_id = ns
#             t.child_frame_id = child
#             pos = msg.pose.position
#             att = msg.pose.orientation

#             # Directly use PoseStamped in ENU map frame
#             t.transform.translation.x = pos.x
#             t.transform.translation.y = pos.y
#             t.transform.translation.z = pos.z
#             t.transform.rotation.x = att.x
#             t.transform.rotation.y = att.y
#             t.transform.rotation.z = att.z
#             t.transform.rotation.w = att.w
#             self.tf_broadcaster.sendTransform(t)

# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomToTf()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
