#!/usr/bin/env python

# ros2 topic pub --rate 50 /px4_1/fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode "{timestamp: $(date +%s%N | cut -b1-13), position: false, velocity: true, acceleration: false, attitude: false, body_rate: false}" & ros2 topic pub --rate 50 /px4_1/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint "{timestamp: $(date +%s%N | cut -b1-13), position: [0.0, 0.0, -2.0], velocity: [0.0, 0.0, 0.0], acceleration: [0.0, 0.0, 0.0], yaw: 0.0}" & sleep 2 && ros2 topic pub /px4_1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{timestamp: $(date +%s%N | cut -b1-13), param1: 1.0, param2: 6.0, command: 176, target_system: 2, target_component: 1, source_system: 2, source_component: 1, from_external: true}" --once && sleep 1 && ros2 topic pub /px4_1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{timestamp: $(date +%s%N | cut -b1-13), param1: 1.0, param2: 0.0, command: 400, target_system: 2, target_component: 1, source_system: 2, source_component: 1, from_external: true}" --once

# ros2 topic pub --rate 50 /px4_2/fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode "{timestamp: $(date +%s%N | cut -b1-13), position: false, velocity: true, acceleration: false, attitude: false, body_rate: false}" & ros2 topic pub --rate 50 /px4_2/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint "{timestamp: $(date +%s%N | cut -b1-13), position: [0.0, 0.0, -2.0], velocity: [0.0, 0.0, 0.0], acceleration: [0.0, 0.0, 0.0], yaw: 0.0}" & sleep 2 && ros2 topic pub /px4_2/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{timestamp: $(date +%s%N | cut -b1-13), param1: 1.0, param2: 6.0, command: 176, target_system: 3, target_component: 1, source_system: 3, source_component: 1, from_external: true}" --once && sleep 1 && ros2 topic pub /px4_2/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand "{timestamp: $(date +%s%N | cut -b1-13), param1: 1.0, param2: 0.0, command: 400, target_system: 3, target_component: 1, source_system: 3, source_component: 1, from_external: true}" --once

##system id is imppppppp
# system id is either 2/3


import rclpy

import traceback
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import Bool


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # parameters for multi-drone support
        self.declare_parameter('drone_name', '')
        self.declare_parameter('vehicle_namespace', '')
        drone_name = self.get_parameter('drone_name').value
        vehicle_ns = self.get_parameter('vehicle_namespace').value
        self.system_id = int(vehicle_ns[-1])+1
        print(vehicle_ns[-1], self.system_id)
        self.vehicle_namespace = vehicle_ns
        teleop_vel_topic = f'{drone_name}/offboard_velocity_cmd' if drone_name else '/offboard_velocity_cmd'
        arm_topic = f'{drone_name}/arm_message' if drone_name else '/arm_message'
        status_topic = f'{vehicle_ns}/fmu/out/vehicle_status' if vehicle_ns else '/fmu/out/vehicle_status'
        attitude_topic = f'{vehicle_ns}/fmu/out/vehicle_attitude' if vehicle_ns else '/fmu/out/vehicle_attitude'
        offboard_mode_topic = f'{vehicle_ns}/fmu/in/offboard_control_mode' if vehicle_ns else '/fmu/in/offboard_control_mode'
        velocity_topic = f'{vehicle_ns}/fmu/in/setpoint_velocity/cmd_vel_unstamped' if vehicle_ns else '/fmu/in/setpoint_velocity/cmd_vel_unstamped'
        trajectory_topic = f'{vehicle_ns}/fmu/in/trajectory_setpoint' if vehicle_ns else '/fmu/in/trajectory_setpoint'
        vehicle_cmd_topic = f'{vehicle_ns}/fmu/in/vehicle_command' if vehicle_ns else '/fmu/in/vehicle_command'

        #Create subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            status_topic,
            self.vehicle_status_callback,
            qos_profile)
        
        self.offboard_velocity_sub = self.create_subscription(
            Twist,
            teleop_vel_topic,
            self.offboard_velocity_callback,
            qos_profile)
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            attitude_topic,
            self.attitude_callback,
            qos_profile)
        
        self.my_bool_sub = self.create_subscription(
            Bool,
            arm_topic,
            self.arm_message_callback,
            qos_profile)


        #Create publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, offboard_mode_topic, qos_profile)
        self.publisher_velocity = self.create_publisher(Twist, velocity_topic, qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, trajectory_topic, qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, vehicle_cmd_topic, 10)

        
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        self.yaw = 0.0  #yaw value we send as command
        self.trueYaw = 0.0  #current yaw value of drone
        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False
        self.current_state = "IDLE"
        self.last_state = self.current_state


    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")

    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):
        # self.get_logger().info(f"{self.current_state}, flightCheck={self.flightCheck}, nav_state={self.nav_state}")
        match self.current_state:
            case "IDLE":
                if(self.flightCheck and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                # self.get_logger().info(f"{self.flightCheck}, {self.arm_state}, {self.myCnt}")
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")
                self.arm() #send arm command

            case "TAKEOFF":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")
                self.arm() #send arm command
                self.take_off() #send takeoff command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                self.state_offboard()

        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.myCnt += 1

    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True   

    # Arms the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        # self.get_logger().info("Arm command send")

    # Takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
        # self.get_logger().info("Takeoff command send")

    #publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = self.system_id  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = self.system_id  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):
        # self.get_logger().info(f"Vehicle Status: {msg.pre_flight_checks_pass}, Arming State: {msg.arming_state}")
        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass


    #receives Twist commands from Teleop and converts NED -> FLU
    def offboard_velocity_callback(self, msg):
        #implements NED -> FLU Transformation
        # X (FLU) is -Y (NED)
        self.velocity.x = -msg.linear.y
        # Y (FLU) is X (NED)
        self.velocity.y = msg.linear.x
        # Z (FLU) is -Z (NED)
        self.velocity.z = -msg.linear.z
        # A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
        self.yaw = msg.angular.z

    #receives current trajectory values from drone and grabs the yaw value of the orientation
    def attitude_callback(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        
    #publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if(self.offboardMode == True):
            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            self.publisher_offboard_mode.publish(offboard_msg)            

            # Compute velocity in the world frame
            cos_yaw = np.cos(self.trueYaw)
            sin_yaw = np.sin(self.trueYaw)
            velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
            velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

            # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.velocity[0] = velocity_world_x
            trajectory_msg.velocity[1] = velocity_world_y
            trajectory_msg.velocity[2] = self.velocity.z
            trajectory_msg.position[0] = float('nan')
            trajectory_msg.position[1] = float('nan')
            trajectory_msg.position[2] = float('nan')
            trajectory_msg.acceleration[0] = float('nan')
            trajectory_msg.acceleration[1] = float('nan')
            trajectory_msg.acceleration[2] = float('nan')
            trajectory_msg.yaw = float('nan')
            trajectory_msg.yawspeed = self.yaw

            self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    # initialize OffboardControl and catch errors
    try:
        offboard_control = OffboardControl()
    except Exception as e:
        print("Failed to initialize OffboardControl:", e)
        traceback.print_exc()
        rclpy.shutdown()
        return
    # spin and catch runtime exceptions
    try:
        rclpy.spin(offboard_control)
    except Exception as e:
        print("Error during spin:", e)
        traceback.print_exc()
    finally:
        try:
            offboard_control.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()