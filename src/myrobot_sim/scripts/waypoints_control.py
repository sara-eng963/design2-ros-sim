#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
import math
import tf_transformations

# Add all the global variables
yaw = 0.0
roll = 0.0
pitch = 0.0

x = 0.0
y = 0.0

linear_vel = 0.0
angular_vel = 0.0

last_time = None

# Defined Waypoints
waypoints = [ # x , y , yaw in radians (orientation)
    (0.0, 0.0, 0.0),  # starting point
    (1.0, 0.0, 0.0),
    (1.0, 1.0, math.pi / 2),
    (0.0, 1.0, math.pi)
]

current_waypoint = 0 # index of the current waypoint in the waypoints list
state = "rotate_to_target"

node = None
cmd_pub = None


# normalize the angle between -pi to pi
def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# IMU topic callback function
def imu_callback(msg):
    global yaw, roll, pitch

    q = msg.orientation
    quaternion = (q.x, q.y, q.z, q.w)
    roll, pitch, yaw_val = tf_transformations.euler_from_quaternion(quaternion)

    roll = roll
    pitch = pitch
    yaw = yaw_val


# odom topic callback function
def odom_callback(msg):
    global linear_vel, angular_vel

    linear_vel = msg.twist.twist.linear.x
    angular_vel = msg.twist.twist.angular.z


# clock topic callback function
def clock_callback(msg):
    global x, y, yaw

    sim_time = msg.clock
    if sim_time.sec == 0:
        x = 0.0
        y = 0.0
        yaw = 0.0


def control_loop():
    global x, y, yaw
    global linear_vel, angular_vel
    global last_time
    global current_waypoint, state

    node.get_logger().info("Control loop")

    current_time = node.get_clock().now() # in nanoseconds

    if last_time is None:
        last_time = current_time
        return

    # get the time difference
    dt = (current_time - last_time).nanoseconds / 1e9 # to seconds
    last_time = current_time

    v = linear_vel
    w = angular_vel

    # calculate the distance moved in x and y
    delta_x = v * math.cos(yaw) * dt # orientation we got from imu
    delta_y = v * math.sin(yaw) * dt
    delta_yaw = w * dt

    # update the current position of the robot
    x += delta_x
    y += delta_y
    yaw += delta_yaw
    yaw = normalize_angle(yaw) # keep the yaw between -pi and pi

    # print the position, orientation and velocity of the robot
    node.get_logger().info(
        f"Position -> x: {x:.3f}, y: {y:.3f} | "
        f"yaw: {math.degrees(yaw):.2f}° | "
        f"Vel -> linear: {linear_vel:.3f}, "
        f"angular: {angular_vel:.3f}"
    )

    # get the target waypoint
    target_x, target_y, target_yaw = waypoints[current_waypoint]

    # get the difference in current and target pose
    dx = target_x - x
    dy = target_y - y
    distance = math.sqrt(dx**2 + dy**2)

    # the angle to be rotated to the target pose
    angle_to_target = math.atan2(dy, dx)
    angle_error = normalize_angle(angle_to_target - yaw)

    # calculate the difference in target angle and current angle
    final_yaw_error = normalize_angle(target_yaw - yaw)

    node.get_logger().info(
        f"Target Pose -> x: {target_x:.3f}, y: {target_y:.3f}, yaw: {target_yaw:.3f}"
    )
    node.get_logger().info(f'Final Yaw Error -> {final_yaw_error:.3f}')

    # set the configurations of the cmd_vel
    cmd = TwistStamped()
    cmd.header.stamp = node.get_clock().now().to_msg()
    cmd.header.frame_id = "Trial_idk" # or base_link?

    # State Machine
    # First rotate to the target pose direction
    if state == "rotate_to_target":
        node.get_logger().info("rotate_to_target state")

        if abs(angle_error) > 0.005:
            cmd.twist.angular.z = -1.5 * angle_error
        else:
            state = "move_forward"

    # Second move forward to the pose
    elif state == "move_forward":
        node.get_logger().info("move_forward state")

        if distance > 0.05:
            cmd.twist.linear.x = 0.5 * distance
        else:
            state = "rotate_to_final"

    # Third rotate to the final target angle
    elif state == "rotate_to_final":
        node.get_logger().info("rotate_to_final state")

        if abs(final_yaw_error) > 0.005:
            cmd.twist.angular.z = -1.5 * final_yaw_error
        else:
            current_waypoint = (current_waypoint + 1) % len(waypoints)
            state = "rotate_to_target"

    # send the velocity command
    cmd_pub.publish(cmd)


def main():
    global node, cmd_pub, last_time

    rclpy.init()

    node = Node('square_follower')

    cmd_pub = node.create_publisher(
        TwistStamped,
        '/diff_drive_controller/cmd_vel',
        10
    )

    node.create_subscription(Imu, '/imu', imu_callback, 10)
    node.create_subscription(Odometry, '/diff_drive_controller/odom', odom_callback, 10)
    node.create_subscription(Clock, '/clock', clock_callback, 10)

    node.create_timer(0.05, control_loop)

    last_time = node.get_clock().now()

    node.get_logger().info("Robot State Printer Started")
    node.get_logger().info("Waypoint control Node Started")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()