#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import tf_transformations

# Global state
x = 0.0
y = 0.0
yaw = 0.0

initial_x = None
initial_y = None

linear_vel = 0.0


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# IMU callback
def imu_callback(msg):
    global yaw
    q = msg.orientation
    quaternion = (q.x, q.y, q.z, q.w)
    _, _, yaw_val = tf_transformations.euler_from_quaternion(quaternion)
    yaw = normalize_angle(yaw_val)


# Odometry callback
def odom_callback(msg):
    global x, y, initial_x, initial_y, linear_vel

    linear_vel = msg.twist.twist.linear.x

    raw_x = msg.pose.pose.position.x
    raw_y = msg.pose.pose.position.y

    if initial_x is None:
        initial_x = raw_x
        initial_y = raw_y

    x = raw_x - initial_x
    y = raw_y - initial_y


def main():
    rclpy.init()
    node = Node('frame_test_node')

    pub = node.create_publisher(
        TwistStamped,
        '/diff_drive_controller/cmd_vel',
        10
    )

    node.create_subscription(Imu, '/imu', imu_callback, 10)
    node.create_subscription(Odometry, '/diff_drive_controller/odom', odom_callback, 10)

    def timer_callback():
        global x, y, yaw, linear_vel

        # --- COMMAND: move straight forward ---
        cmd = TwistStamped()
        cmd.header.stamp = node.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        cmd.twist.linear.x = 0.2
        cmd.twist.angular.z = 0.0

        pub.publish(cmd)

        # --- LOGGING ---
        node.get_logger().info(
            f"\n--- FRAME TEST ---\n"
            f"POSITION -> x: {x:.3f}, y: {y:.3f}\n"
            f"YAW      -> {math.degrees(yaw):.2f}°\n"
            f"VEL      -> {linear_vel:.3f}\n"
            "------------------"
        )

    node.create_timer(0.2, timer_callback)

    node.get_logger().info("Frame test node started")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()