#!/usr/bin/env python3

import threading
import tkinter as tk

import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool


class StartStopGUI(Node):

    def __init__(self):
        super().__init__('start_stop_gui',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL,
                                 True)
                         ])

        # Latched QoS — late-joining subscribers receive the last published value
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.start_pub = self.create_publisher(Bool, '/start_signal', latched_qos)
        self.stop_pub  = self.create_publisher(Bool, '/stop_signal',  latched_qos)

        self.get_logger().info('Start/Stop GUI node ready — waiting for button press.')

    def publish_start(self):
        self.start_pub.publish(Bool(data=True))
        self.get_logger().info('*** START SIGNAL SENT — robot will begin navigating ***')

    def publish_stop(self):
        self.stop_pub.publish(Bool(data=True))
        self.get_logger().info('*** STOP SIGNAL SENT — robot will halt immediately ***')


def run_gui(ros_node):
    root = tk.Tk()
    root.title('Robot Start / Stop Control')
    root.geometry('320x160')
    root.resizable(False, False)

    tk.Label(root, text='Robot Control Panel',
             font=('Helvetica', 14, 'bold')).pack(pady=14)

    btn_frame = tk.Frame(root)
    btn_frame.pack()

    btn_start = tk.Button(
        btn_frame,
        text='START',
        font=('Helvetica', 13, 'bold'),
        bg='#27ae60', fg='white',
        width=10, height=2,
        command=ros_node.publish_start
    )
    btn_start.pack(side=tk.LEFT, padx=20)

    btn_stop = tk.Button(
        btn_frame,
        text='STOP',
        font=('Helvetica', 13, 'bold'),
        bg='#e74c3c', fg='white',
        width=10, height=2,
        command=ros_node.publish_stop
    )
    btn_stop.pack(side=tk.RIGHT, padx=20)

    root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = StartStopGUI()

    # Spin ROS2 in a background thread so the Tkinter main loop runs on the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    run_gui(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
