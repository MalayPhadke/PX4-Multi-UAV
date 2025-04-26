#!/usr/bin/env python3
import sys
import rclpy
import geometry_msgs.msg
import std_msgs.msg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
Teleoperation for Swarm:
Use keys to control the active drone.
Press '1' or '2' to switch between drones.
W/S: Up/Down
A/D: Yaw Left/Right
Up/Down arrow: Pitch Forward/Backward
Left/Right arrow: Roll Left/Right
Space: Arm/Disarm current drone
Ctrl-C to exit.
"""

moveBindings = {
    'w': (0, 0, 1, 0),   # Z+
    's': (0, 0, -1, 0),  # Z-
    'a': (0, 0, 0, -1),  # Yaw+
    'd': (0, 0, 0, 1),   # Yaw-
    '\x1b[A': (0, 1, 0, 0),   # Up Arrow
    '\x1b[B': (0, -1, 0, 0),  # Down Arrow
    '\x1b[C': (-1, 0, 0, 0),  # Right Arrow
    '\x1b[D': (1, 0, 0, 0),   # Left Arrow
}

# read a single keypress

def getKey(settings):
    if sys.platform == 'win32':
        return msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    settings = saveTerminalSettings()
    rclpy.init()
    node = rclpy.create_node('teleop_swarm')

    qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    # list of drone namespaces
    drones = ['x500_depth_1', 'x500_depth_2']

    # publishers for velocity and arm topics per drone
    vel_pubs = {d: node.create_publisher(geometry_msgs.msg.Twist, f'/{d}/offboard_velocity_cmd', qos) for d in drones}
    arm_pubs = {d: node.create_publisher(std_msgs.msg.Bool, f'/{d}/arm_message', qos) for d in drones}

    current = drones[0]
    arm_state = {d: False for d in drones}
    speed = 0.5
    turn = 0.2

    # maintain state per drone
    state = {d: {'xv': 0.0, 'yv': 0.0, 'zv': 0.0, 'yvaw': 0.0} for d in drones}

    print(msg)
    print(f"Controlling: {current}")

    try:
        while True:
            key = getKey(settings)

            if key in moveBindings:
                x, y, z, th = moveBindings[key]
                s = state[current]
                s['xv'] += x * speed
                s['yv'] += y * speed
                s['zv'] += z * speed
                s['yvaw'] += th * turn

                twist = geometry_msgs.msg.Twist()
                twist.linear.x = s['xv']
                twist.linear.y = s['yv']
                twist.linear.z = s['zv']
                twist.angular.z = s['yvaw']
                vel_pubs[current].publish(twist)
                print(f"{current} -> X:{twist.linear.x} Y:{twist.linear.y} Z:{twist.linear.z} Yaw:{twist.angular.z}")

            elif key in ['1', '2']:
                idx = int(key) - 1
                if idx < len(drones):
                    current = drones[idx]
                    print(f"Switched to {current}")

            elif key == ' ':
                arm_state[current] = not arm_state[current]
                arm_msg = std_msgs.msg.Bool()
                arm_msg.data = arm_state[current]
                arm_pubs[current].publish(arm_msg)
                print(f"{current} arm: {arm_state[current]}")

            elif key == '\x03':  # CTRL-C
                break

    except Exception as e:
        print(e)

    finally:
        # stop all drones
        for d in drones:
            twist = geometry_msgs.msg.Twist()
            vel_pubs[d].publish(twist)

        restoreTerminalSettings(settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
