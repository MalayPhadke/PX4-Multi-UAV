#!/usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

"""
SwarmAuto script: auto-arm, offboard-mode, and hover multiple drones.
Usage:
  ros2 run px4_offboard swarm_auto <ns1> <id1> <ns2> <id2> [altitude]
Example:
  ros2 run px4_offboard swarm_auto px4_1 2 px4_2 3 2.0
"""

def make_offboard_callback(pub, msg, node):
    def callback():
        msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        pub.publish(msg)
    return callback


def make_trajectory_callback(pub, msg, node):
    def callback():
        msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        pub.publish(msg)
    return callback


def make_mode_callback(pub, system_id, node):
    def callback():
        cmd = VehicleCommand()
        cmd.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0
        cmd.param2 = 6.0
        cmd.target_system = system_id
        cmd.target_component = 1
        cmd.source_system = system_id
        cmd.source_component = 1
        cmd.from_external = True
        pub.publish(cmd)
        node.get_logger().info(f"{system_id}: OFFBOARD mode set")
    return callback


def make_arm_callback(pub, system_id, node):
    def callback():
        cmd = VehicleCommand()
        cmd.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0
        cmd.target_system = system_id
        cmd.target_component = 1
        cmd.source_system = system_id
        cmd.source_component = 1
        cmd.from_external = True
        pub.publish(cmd)
        node.get_logger().info(f"{system_id}: ARM command sent")
    return callback


class SwarmAuto(Node):
    def __init__(self):
        super().__init__('swarm_auto')
        if len(sys.argv) < 5:
            print(__doc__)
            sys.exit(1)
        ns1, id1, ns2, id2 = sys.argv[1], int(sys.argv[2]), sys.argv[3], int(sys.argv[4])
        alt = float(sys.argv[5]) if len(sys.argv) > 5 else 2.0
        drones = [(ns1, id1), (ns2, id2)]

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for ns, sys_id in drones:
            offboard_pub = self.create_publisher(
                OffboardControlMode,
                f'/{ns}/fmu/in/offboard_control_mode',
                qos,
            )
            traj_pub = self.create_publisher(
                TrajectorySetpoint,
                f'/{ns}/fmu/in/trajectory_setpoint',
                qos,
            )
            cmd_pub = self.create_publisher(
                VehicleCommand,
                f'/{ns}/fmu/in/vehicle_command',
                10,
            )

            offboard_msg = OffboardControlMode(
                position=True,
                velocity=False,
                acceleration=False,
                attitude=False,
                body_rate=False,
            )
            traj_msg = TrajectorySetpoint()
            traj_msg.position = [0.0, 0.0, -abs(alt)]
            traj_msg.velocity = [0.0, 0.0, 0.0]
            traj_msg.acceleration = [0.0, 0.0, 0.0]
            traj_msg.yaw = 0.0

            # stream at 50Hz
            timer_period = 1.0 / 50.0
            self.create_timer(timer_period, make_offboard_callback(offboard_pub, offboard_msg, self))
            self.create_timer(timer_period, make_trajectory_callback(traj_pub, traj_msg, self))

            # schedule offboard mode and arm
            self.create_timer(2.0, make_mode_callback(cmd_pub, sys_id, self))
            self.create_timer(3.0, make_arm_callback(cmd_pub, sys_id, self))

        self.get_logger().info(f"Initialized SwarmAuto for: {drones} at altitude {alt}m")


def main():
    rclpy.init()
    node = SwarmAuto()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
