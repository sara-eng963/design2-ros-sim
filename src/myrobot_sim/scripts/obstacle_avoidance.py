#!/usr/bin/env python3
 
import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan 
import numpy as np 
from std_msgs.msg import Bool
 
cmd_pub = None
safe_distance = 0.5
node = None
obstacle_pub = None

def check_scan(msg):
    ranges = np.array(msg.ranges)
    ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=10.0)
    ranges = ranges[ranges > msg.range_min]
    if len(ranges) == 0:
        return False
    return float(np.min(ranges)) < safe_distance

def scan_callback(msg):
    publish_obstacle(check_scan(msg))

def scan_left_callback(msg):
    publish_obstacle(check_scan(msg))

def scan_right_callback(msg):
    publish_obstacle(check_scan(msg))

def publish_obstacle(detected):
    if detected:
       # node.get_logger().info("Obstacle detected", throttle_duration_sec=1.0)
        obstacle_pub.publish(Bool(data=True))
    else:
        obstacle_pub.publish(Bool(data=False))

def main():
    global node, obstacle_pub
    rclpy.init()
    node = Node("Obstacle_Avoidance_Node")
    obstacle_pub = node.create_publisher(Bool, '/obstacle_detected', 10)
    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    node.create_subscription(LaserScan, '/scan_left', scan_left_callback, 10)
    node.create_subscription(LaserScan, '/scan_right', scan_right_callback, 10)
    node.get_logger().info("Node is initialized")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
     main()