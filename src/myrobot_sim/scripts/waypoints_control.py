#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import tf_transformations
from std_msgs.msg import Bool

# Global variables
yaw = 0.0
roll = 0.0
pitch = 0.0

x = 0.0
y = 0.0

linear_vel = 0.0
angular_vel = 0.0

last_time = None

ANGLE_TOL = 0.05
DIST_TOL = 0.05
MAX_LIN = 0.5
MAX_ANG = 1.0
K_ANG = 2.0

waypoints = [
    (3.0, 0.0, 0.0),
    (-3.0, 0.0, 0.0),
    (0.0, -9.0, 0.0),
    (0.0, 3.0, 0.0),
    (1.0, 1.0, math.pi / 2),
    (0.0, 1.0, math.pi)
]

current_waypoint = 0
state = "align_x"

node = None
cmd_pub = None

obstacle_detected = False
startup_time = None

detour_start_x = None
detour_start_y = None
detour_reference_yaw = None



def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def imu_callback(msg):
    global yaw, roll, pitch
    q = msg.orientation
    quaternion = (q.x, q.y, q.z, q.w)
    roll, pitch, yaw_val = tf_transformations.euler_from_quaternion(quaternion)
    yaw = yaw_val


def odom_callback(msg):
    global linear_vel, angular_vel
    # Only store velocities, don't touch x/y
    linear_vel = msg.twist.twist.linear.x
    angular_vel = msg.twist.twist.angular.z


def obstacle_callback(msg):
    global obstacle_detected
    obstacle_detected = msg.data


def control_loop():
    global x, y, yaw
    global current_waypoint, state
    global obstacle_detected
    global startup_time, last_time
    global detour_start_x, detour_start_y,detour_reference_yaw

    current_time = node.get_clock().now()

    if startup_time is None:
        startup_time = current_time

    if (current_time - startup_time).nanoseconds / 1e9 < 2.0:
        return

    if last_time is None:
        last_time = current_time
        return

    # integrate velocities to update pose
    dt = (current_time - last_time).nanoseconds / 1e9
    last_time = current_time
    delta_x = linear_vel * math.sin(yaw) * dt
    delta_y = -linear_vel * math.cos(yaw) * dt
    delta_yaw = angular_vel * dt

    x += delta_x
    y += delta_y
    yaw = normalize_angle(yaw + delta_yaw)

    target_x, target_y, target_yaw = waypoints[current_waypoint]
    node.get_logger().info(
        "\n---------------- DEBUG ----------------\n"
        f"CURRENT STATE: {state}\n"
        f"POSITION -> x: {x:.3f}, y: {y:.3f}, yaw: {math.degrees(yaw):.2f}°\n"
        f"TARGET   -> x: {target_x:.3f}, y: {target_y:.3f}, yaw: {math.degrees(target_yaw):.2f}°\n"
        f"VELOCITY -> linear: {linear_vel:.3f}, angular: {angular_vel:.3f}\n"
        f"OBSTACLE DETECTED: {obstacle_detected}\n"
        "--------------------------------------",
        throttle_duration_sec=1.0
    )
    node.get_logger().info(f"linear_vel raw: {linear_vel:.3f} m/s", throttle_duration_sec=1.0)


    dx = target_x - x
    dy = target_y - y
    angle_to_target = math.atan2(dy, dx)
    angle_error = normalize_angle(angle_to_target - yaw)

    node.get_logger().info(
        f"ERRORS -> dx: {dx:.3f}, dy: {dy:.3f}, "
        f"angle_to_target: {math.degrees(angle_to_target):.2f}°, "
        f"angle_error: {math.degrees(angle_error):.2f}°",
        throttle_duration_sec=1.0
    )

    cmd = TwistStamped()
    cmd.header.stamp = node.get_clock().now().to_msg()
    cmd.header.frame_id = "base_link"

    # Obstacle handling
    if obstacle_detected:
      #  if state not in ["avoid_rotate", "avoid_move"]:
            state = "avoid_rotate"

    # Avoid rotate
    if state == "avoid_rotate":
        # Always point to +90° (π/2 radians)
        detour_yaw = math.pi/2
        error = normalize_angle(detour_yaw - yaw)

        if abs(error) > ANGLE_TOL:
            cmd.twist.angular.z = max(min(K_ANG * error, MAX_ANG), -MAX_ANG)
        else:
            cmd.twist.angular.z = 0.0
            detour_start_x, detour_start_y = x, y
            state = "avoid_move"




    # Avoid move (sideways detour)
    elif state == "avoid_move":
        dx_d = x - detour_start_x
        dy_d = y - detour_start_y
        dist = math.sqrt(dx_d**2 + dy_d**2)

        if dist < 1.2:   # move sideways for 0.7 m
            cmd.twist.linear.x = 0.2
        else:
            cmd.twist.linear.x = 0.0
            state = "align_y"


        # Align X
    elif state == "align_x":
        # +X → yaw = +π/2, -X → yaw = -π/2
        target_yaw = math.pi/2 if dx > 0 else -math.pi/2
        error = normalize_angle(target_yaw - yaw)

        if abs(dx) < DIST_TOL:
            state = "align_y"
        elif abs(error) > ANGLE_TOL:
            cmd.twist.angular.z = max(min(K_ANG * error, MAX_ANG), -MAX_ANG)
        else:
            state = "move_x"

    # Move X
    elif state == "move_x":
        target_yaw = math.pi/2 if dx > 0 else -math.pi/2
        error = normalize_angle(target_yaw - yaw)

        if abs(dx) > DIST_TOL:
            cmd.twist.linear.x = min(0.5 * abs(dx), MAX_LIN)
            cmd.twist.angular.z = K_ANG * error
        else:
            state = "align_y"

    # Align Y
    elif state == "align_y":
        # +Y → yaw = π, -Y → yaw = 0
        target_yaw = math.pi if dy > 0 else 0.0
        error = normalize_angle(target_yaw - yaw)

        if abs(dy) < DIST_TOL:
            current_waypoint = (current_waypoint + 1) % len(waypoints)
            state = "align_x"
        elif abs(error) > ANGLE_TOL:
            cmd.twist.angular.z = max(min(K_ANG * error, MAX_ANG), -MAX_ANG)
        else:
            state = "move_y"

    # Move Y
    elif state == "move_y":
        target_yaw = math.pi if dy > 0 else 0.0
        error = normalize_angle(target_yaw - yaw)

        if abs(dy) > DIST_TOL:
            # Keep moving along Y
            cmd.twist.linear.x = min(0.5 * abs(dy), MAX_LIN)
            cmd.twist.angular.z = K_ANG * error
        else:
            # Y is correct, now check X
            if abs(dx) > DIST_TOL:
                # Still need to fix X → go to align_x
                state = "align_x"
            else:
                # Both X and Y are correct → advance waypoint
                current_waypoint = (current_waypoint + 1) % len(waypoints)
                state = "align_x"




    cmd_pub.publish(cmd)


def main():
    global node, cmd_pub

    rclpy.init()

    node = Node('square_follower')

    cmd_pub = node.create_publisher(
        TwistStamped,
        '/diff_drive_controller/cmd_vel',
        10
    )

    node.create_subscription(Imu, '/imu', imu_callback, 10)
    node.create_subscription(Odometry, '/diff_drive_controller/odom', odom_callback, 10)
    node.create_subscription(Bool, '/obstacle_detected', obstacle_callback, 10)

    node.create_timer(0.05, control_loop)

    node.get_logger().info("Waypoint control Node Started")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()