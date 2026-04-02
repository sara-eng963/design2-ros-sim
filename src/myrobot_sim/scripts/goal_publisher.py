#!/usr/bin/env python3

import math

import rclpy
import rclpy.parameter
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
import tf_transformations


class GoalPublisher(Node):
	def __init__(self):
		super().__init__(
			'goal_publisher',
			parameter_overrides=[
				rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)
			],
		)

		self.publisher_ = self.create_publisher(PoseArray, '/navigation_goals', 10)
		self.published_once = False

		# Waypoints are owned by this node and sent once at startup.
		self.waypoints = [
			(0.0, -2.5, 0.0),
			(0.0, 0.0, 0.0),
			(-1.0, -2.5, 0.0),
			(0.0, 0.0, 0.0),
			(-2.0, -2.5, 0.0),
			(0.0, 0.0, 0.0),
		]

		# Publish once after 3 seconds.
		self.timer = self.create_timer(3.0, self.publish_goals_once)
		self.get_logger().info('Goal publisher started, publishing /navigation_goals in 3s')

	def publish_goals_once(self):
		if self.published_once:
			return

		msg = PoseArray()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = 'map'

		for x, y, yaw in self.waypoints:
			pose = Pose()
			pose.position.x = float(x)
			pose.position.y = float(y)
			pose.position.z = 0.0

			qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
			pose.orientation.x = qx
			pose.orientation.y = qy
			pose.orientation.z = qz
			pose.orientation.w = qw
			msg.poses.append(pose)

		self.publisher_.publish(msg)
		self.published_once = True
		self.get_logger().info(f'Published {len(msg.poses)} goals on /navigation_goals')
		self.timer.cancel()


def main():
	rclpy.init()
	node = GoalPublisher()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
