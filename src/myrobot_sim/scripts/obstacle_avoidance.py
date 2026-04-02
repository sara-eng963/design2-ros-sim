#!/usr/bin/env python3
 
import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan 
import numpy as np 
from std_msgs.msg import Bool, String
 
cmd_pub = None
safe_distance = 0.5
node = None
obstacle_pub = None
direction_pub = None

scan_front_blocked = False
scan_left_blocked  = False
scan_right_blocked = False

def check_scan(msg):
    ranges = np.array(msg.ranges)
    ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=10.0)
    ranges = ranges[ranges > msg.range_min]
    if len(ranges) == 0:
        return False
    return float(np.min(ranges)) < safe_distance

def compute_direction():
    """Decide which side the obstacle is on so the robot knows which way to dodge.

    Key insight: when an obstacle is straight ahead, ALL three sensors see it
    because the side sensors are close enough to also pick it up. So
    "both sides blocked" means FRONT, not left or right.

    Priority rules (fault-proof):
      left AND right both blocked        → 'front'  (dead ahead — all beams hit it)
      only left blocked, right clear     → 'left'   (obstacle on left  → dodge right)
      only right blocked, left clear     → 'right'  (obstacle on right → dodge left)
      only front blocked, both sides clear → 'front' (narrow object, dodge left default)
    """
    if scan_left_blocked and scan_right_blocked:
        return 'front'
    if scan_left_blocked and not scan_right_blocked:
        return 'left'
    if scan_right_blocked and not scan_left_blocked:
        return 'right'
    return 'front'

def scan_callback(msg):
    global scan_front_blocked
    scan_front_blocked = check_scan(msg)
    any_blocked = scan_front_blocked or scan_left_blocked or scan_right_blocked
    publish_obstacle(any_blocked)
    if any_blocked:
        direction_pub.publish(String(data=compute_direction()))

def scan_left_callback(msg):
    global scan_left_blocked
    scan_left_blocked = check_scan(msg)
    any_blocked = scan_front_blocked or scan_left_blocked or scan_right_blocked
    publish_obstacle(any_blocked)
    if any_blocked:
        direction_pub.publish(String(data=compute_direction()))

def scan_right_callback(msg):
    global scan_right_blocked
    scan_right_blocked = check_scan(msg)
    any_blocked = scan_front_blocked or scan_left_blocked or scan_right_blocked
    publish_obstacle(any_blocked)
    if any_blocked:
        direction_pub.publish(String(data=compute_direction()))

def publish_obstacle(detected):
    if detected:
       # node.get_logger().info("Obstacle detected", throttle_duration_sec=1.0)
        obstacle_pub.publish(Bool(data=True))
    else:
        obstacle_pub.publish(Bool(data=False))

def main():
    global node, obstacle_pub, direction_pub
    rclpy.init()
    node = Node("Obstacle_Avoidance_Node")
    obstacle_pub = node.create_publisher(Bool, '/obstacle_detected', 10)
    direction_pub = node.create_publisher(String, '/obstacle_direction', 10)
    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    node.create_subscription(LaserScan, '/scan_left', scan_left_callback, 10)
    node.create_subscription(LaserScan, '/scan_right', scan_right_callback, 10)
    node.get_logger().info("Node is initialized")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
     main()
     