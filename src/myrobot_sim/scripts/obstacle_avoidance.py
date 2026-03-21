#!/usr/bin/env python3
 
import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import TwistStamped 
import numpy as np 
from std_msgs.msg import Bool  # add this line
 
cmd_pub = None 
safe_distance = 0.5
node = None
obstacle_pub = None

def scan_callback(msg):
    global safe_distance, cmd_pub, node
    ranges = np.array(msg.ranges)
    ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=10.0) 

    # Don't slice at all — the whole array is already your front view
    front_ranges = ranges
    front_ranges = front_ranges[front_ranges > msg.range_min]  # filter invalid readings

 
    if len(ranges) == 0: 
            return 
    
    
    # front_ranges = np.concatenate((ranges[:30], ranges[-30:])) 
 
    min_distance = np.min(front_ranges) 
 
    cmd = TwistStamped() 
    cmd.header.stamp = node.get_clock().now().to_msg() 
    cmd.header.frame_id = "Trial_idk" # or base_link?
 
    if min_distance < safe_distance: 
       # cmd.twist.linear.x = 0.0 
       # cmd.twist.angular.z = 0.5 # will only use this code as a sensor and not to move
        node.get_logger().info("Obstacle detected ") 
       # cmd_pub.publish(cmd)
        obstacle_pub.publish(Bool(data=True)) # to tell waypoints  
    else: 
        obstacle_pub.publish(Bool(data=False))
        return  # DO NOT publish cmd_vel
    # cmd_pub.publish(cmd) # 7atetha fo2 

def main():
    global cmd_pub, node, obstacle_pub #to use it in other functions
    rclpy.init()
    node = Node("Obstacle_Avoidance_Node")
    cmd_pub = node.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
    obstacle_pub = node.create_publisher(Bool, '/obstacle_detected', 10)
    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    node.get_logger().info("Node is initialized")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
     main()