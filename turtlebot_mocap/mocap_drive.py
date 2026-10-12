#!/usr/bin/env python3
"""ROS 2 Humble: drive to a stationary mocap goal. See README.md first.

Requires a NatNet callback with (id, position, quaternion, tracking_valid).
Coordinates: Motive Y-up, meters; navigation x=X, y=-Z.
"""
import argparse
import math
from pathlib import Path
import signal
import sys
import threading
import time


def heading(rot, offset_deg):
    """Heading of the rigid body's local +X axis, plus mounting offset."""
    qx, qy, qz, qw = rot
    norm = math.sqrt(sum(q * q for q in rot))
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError("Invalid quaternion")
    qx, qy, qz, qw = (q / norm for q in rot)
    forward_x = 1 - 2 * (qy*qy + qz*qz)
    forward_y = 2 * (qw*qy - qx*qz)  # navigation y = -Motive Z
    if math.hypot(forward_x, forward_y) < 0.5:
        raise ValueError("Robot's forward axis is too far from horizontal")
    return math.atan2(forward_y, forward_x) + math.radians(offset_deg)


def velocity(robot, goal, stop_distance, tolerance):
    """Return forward speed, angular speed and whether the goal is reached."""
    x, y, yaw = robot
    dx, dy = goal[0] - x, goal[1] - y
    distance = math.hypot(dx, dy)
    if distance <= stop_distance + tolerance:
        return 0.0, 0.0, True
    error = math.atan2(math.sin(math.atan2(dy, dx) - yaw),
                       math.cos(math.atan2(dy, dx) - yaw))
    angular = max(-0.5, min(0.5, 1.5 * error))
    # Turn in place when not facing the goal. Slow down as distance shrinks.
    linear = min(0.10, 0.6 * (distance - stop_distance))
    if abs(error) > math.radians(15):
        linear = 0.0
    return linear, angular, False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id', type=int, default=67)
    parser.add_argument('--goal-id', type=int, default=69)
    parser.add_argument('--client-ip', default='192.168.0.232')
    parser.add_argument('--server-ip', default='192.168.0.245')
    parser.add_argument('--topic', default='/cmd_vel')
    parser.add_argument('--yaw-offset-deg', type=float, default=0.0)
    parser.add_argument('--stop-distance', type=float, default=0.0)
    parser.add_argument('--tolerance', type=float, default=0.05)
    parser.add_argument('--natnet-dir', default=str(
        Path(__file__).resolve().parent / 'new_NAT/samples/PythonClient'))
    args = parser.parse_args()
    if args.robot_id == args.goal_id:
        parser.error('Robot and goal must have different IDs')
    if (not all(math.isfinite(v) for v in
                (args.stop_distance, args.tolerance, args.yaw_offset_deg))
            or args.stop_distance < 0 or args.tolerance <= 0):
        parser.error('Use finite values, stop-distance >= 0 and tolerance > 0')

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    sys.path.insert(0, args.natnet_dir)
    from NatNetClient import NatNetClient

    # Keep ROS alive long enough to send zero velocity on Ctrl+C or SIGTERM.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    node = rclpy.create_node('mocap_drive')
    publisher = node.create_publisher(Twist, args.topic, 1)
    node.get_logger().info(
        f'Tracking robot {args.robot_id} toward object {args.goal_id} at 20 Hz.')
    poses = {}
    lock = threading.Lock()
    client = NatNetClient()
    missing_validity = False
    reached = False
    previous_status = None

    def receive(rb_id, pos, rot, tracking_valid=None):
        nonlocal missing_validity
        if rb_id not in (args.robot_id, args.goal_id):
            return
        pose = None
        if tracking_valid is None:
            missing_validity = True
        elif tracking_valid:
            try:
                x, _, z = pos
                yaw = heading(rot, args.yaw_offset_deg) if rb_id == args.robot_id else 0.0
                if all(math.isfinite(v) for v in (x, z, yaw)):
                    pose = (x, -z, yaw, time.monotonic())
            except (TypeError, ValueError):
                pass
        with lock:
            poses[rb_id] = pose  # Invalid tracking immediately clears the pose.

    def tick():
        nonlocal reached, previous_status
        command = Twist()
        with lock:
            robot = poses.get(args.robot_id)
            goal = poses.get(args.goal_id)
        now = time.monotonic()
        if reached:
            status = 'Goal reached; holding stopped. Restart for another run.'
        elif missing_validity:
            status = 'STOPPED: NatNet callback is missing tracking validity.'
        elif robot is None or goal is None or any(
                now - p[3] > 0.25 for p in (robot, goal)):
            status = 'STOPPED: waiting for fresh, valid robot AND goal tracking.'
        else:
            v, w, reached = velocity(robot[:3], goal[:2],
                                     args.stop_distance, args.tolerance)
            command.linear.x = v
            command.angular.z = w
            status = 'Goal reached; holding stopped. Restart for another run.' if reached else 'Driving to goal.'
        publisher.publish(command)
        if status != previous_status:
            node.get_logger().info(status)
            previous_status = status

    client.set_client_address(args.client_ip)
    client.set_server_address(args.server_ip)
    client.set_use_multicast(True)
    if not hasattr(client, 'rigid_body_valid_listener'):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('Use the bundled NatNetClient.py with tracking validity support')
    client.rigid_body_valid_listener = receive
    if hasattr(client, 'set_print_level'):
        client.set_print_level(0)
    node.create_timer(0.05, tick)  # 20 Hz control; mocap reception is independent.
    try:
        if client.run('d') is False:
            raise RuntimeError('NatNet client failed to start')
        while rclpy.ok() and not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        # Repeat stop commands before closing the publisher.
        if rclpy.ok():
            for _ in range(5):
                publisher.publish(Twist())
                time.sleep(0.05)
        try:
            # The supplied SDK shutdown assumes both sockets and threads exist.
            if (client.command_socket is not None and client.data_socket is not None
                    and client.command_thread is not None and client.data_thread is not None):
                client.shutdown()
            else:
                client.stop_threads = True
                for sock in (client.command_socket, client.data_socket):
                    if sock is not None:
                        sock.close()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
