#!/usr/bin/env python3
# ^ tells Linux to run this file with Python 3

# rclpy is the ROS2 Python library — needed for every ROS2 Python node
import rclpy
import rclpy.parameter                          # lets us set parameters like use_sim_time
from rclpy.node import Node                     # base class every ROS2 node inherits from
import math                                     # for math.degrees() in log messages
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# ^ JointTrajectory   : the message type that commands a joint to move
# ^ JointTrajectoryPoint : one position target inside that message
from builtin_interfaces.msg import Duration     # represents a time duration (sec + nanosec)
from std_msgs.msg import Bool                   # simple True/False message type
from sensor_msgs.msg import JointState          # carries live joint positions from Gazebo
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# ── Constants ────────────────────────────────────────────────────────────────
# Arm joint limits (rad): lower=0.0, upper=3.14
ARM_ROTATED = -3.14  # 180° vertical rotation — negative direction (opposite way)
ARM_ZERO    = 0.0    # resting position         (0 radians)

MOVE_DURATION    = 1.5   # seconds given to the trajectory to execute (used in time_from_start)
PRE_RAISE_WAIT   = 2.0   # seconds to wait at 0° before rotating
HOLD_WAIT        = 2.0   # seconds to hold at 180° before returning
POSITION_TOL     = 0.05  # radians — how close the joint must be to the target to count as "arrived"


# ── Node class ───────────────────────────────────────────────────────────────
# 'class ArmController(Node)' means ArmController inherits everything from
# the ROS2 Node base class (publisher, subscriber, timer, logger, etc.)
class ArmController(Node):

    def __init__(self):
        # Call the parent Node constructor and give this node the name
        # 'arm_controller_node'. Also force use_sim_time=True so the node
        # uses Gazebo's clock instead of the real wall clock.
        super().__init__('arm_controller_node',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL,
                                 True)
                         ])

        # Publisher: sends JointTrajectory messages to the ros2_control arm
        # controller running inside Gazebo. Queue size = 10 messages.
        self.traj_pub = self.create_publisher(
            JointTrajectory,                        # message type
            '/arm_controller/joint_trajectory',     # topic name
            10                                      # queue size
        )

        # Publisher: sends a Bool(True) to tell waypoints_control.py
        # that the arm has finished its sequence.
        self.done_pub = self.create_publisher(Bool, '/arm_done', 10)

        # Subscriber: listens for a Bool(True) from waypoints_control.py
        # which means "robot reached a waypoint — start the arm sequence".
        # When a message arrives, self.trigger_callback is called.
        self.create_subscription(Bool, '/arm_trigger', self.trigger_callback, 10)

        # State machine variable — tracks which phase the arm is currently in.
        # Possible values: 'idle' | 'pre_raise' | 'raising' | 'hold_raised' | 'lowering'
        self.state = 'idle'

        # Records the time when the current phase started (used to measure elapsed time)
        self.phase_start = None

        # Stores the latest Arm_Joint position read from /joint_states (radians)
        self.arm_position = 0.0

        # Subscriber: reads live joint positions published by joint_state_broadcaster
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Timer: calls self.control_loop every 0.1 seconds (10 Hz)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Arm control node started, waiting for /arm_trigger.')

        # Stop flag — set to True when /stop_signal is received
        self.system_stopped = False
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(Bool, '/stop_signal',  self.stop_callback,  latched_qos)
        self.create_subscription(Bool, '/start_signal', self.start_callback, latched_qos)

    # ── Helper: build and send a trajectory message ───────────────────────────
    def send_trajectory(self, position: float, duration_sec: float = MOVE_DURATION):
        # Create an empty JointTrajectory message
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()  # timestamp = now
        msg.joint_names = ['Arm_Joint']                     # which joint to move

        # A single waypoint inside the trajectory
        pt = JointTrajectoryPoint()
        pt.positions  = [position]   # target angle in radians
        pt.velocities = [0.0]        # come to a stop at the target

        # How long the motion should take (split into whole seconds + leftover nanoseconds)
        pt.time_from_start = Duration(
            sec=int(duration_sec),                      # e.g. 1  (from 1.5)
            nanosec=int((duration_sec % 1) * 1e9)       # e.g. 500000000 (0.5 s → ns)
        )

        msg.points = [pt]           # attach the point to the message
        self.traj_pub.publish(msg)  # send it to the arm controller
    # ── Callback: reads live joint positions from Gazebo ─────────────────────
    def joint_state_callback(self, msg):
        # msg.name is a list of joint names; msg.position is the matching list of angles
        if 'Arm_Joint' in msg.name:
            idx = msg.name.index('Arm_Joint')   # find which index Arm_Joint is at
            self.arm_position = msg.position[idx]  # store its current angle in radians
    def stop_callback(self, msg):
        if msg.data:
            self.system_stopped = True
            self.get_logger().info('STOP signal received — arm halted.')

    def start_callback(self, msg):
        if msg.data:
            self.system_stopped = False
            self.get_logger().info('START signal received — arm resuming.')

    # ── Callback: fires when a message arrives on /arm_trigger ────────────────
    def trigger_callback(self, msg):
        if self.system_stopped:
            return
        # msg.data is True/False (Bool message)
        # Only start if the arm is currently idle — prevents re-triggering mid-sequence
        if msg.data and self.state == 'idle':
            self.state = 'pre_raise'
            self.phase_start = self.get_clock().now()   # start the phase clock
            self.get_logger().info('Arm triggered: holding at 0° for 2 s before rotating.')

    # ── Timer callback: runs every 0.1 s ─────────────────────────────────────
    def control_loop(self):
        if self.system_stopped:
            return
        # Do nothing while idle or before the phase clock is set
        if self.state == 'idle' or self.phase_start is None:
            return

        # Calculate how many seconds have passed since the current phase started
        elapsed = (self.get_clock().now() - self.phase_start).nanoseconds / 1e9
        #          └─ time difference as a Duration object ─┘
        #                                                    └─ convert ns → s

        if self.state == 'pre_raise':
            # Wait 2 s at resting position, then command the arm to rotate to 180°
            if elapsed >= PRE_RAISE_WAIT:
                self.send_trajectory(ARM_ROTATED)
                self.get_logger().info('Arm: rotating to 180°.')
                self.state = 'raising'
                self.phase_start = self.get_clock().now()   # reset phase clock

        elif self.state == 'raising':
            # Wait until the joint actually reaches ARM_ROTATED (within tolerance)
            # More reliable than a fixed timer — mirrors how waypoints_control checks distance
            if abs(self.arm_position - ARM_ROTATED) < POSITION_TOL:
                self.get_logger().info(f'Arm: at {math.degrees(ARM_ROTATED):.0f}° (joint confirmed), holding for 2 s.')
                self.state = 'hold_raised'
                self.phase_start = self.get_clock().now()

        elif self.state == 'hold_raised':
            # Arm stays at 180° for 2 s, then command it back to 0°
            if elapsed >= HOLD_WAIT:
                self.send_trajectory(ARM_ZERO)
                self.get_logger().info('Arm: returning to 0°.')
                self.state = 'lowering'
                self.phase_start = self.get_clock().now()

        elif self.state == 'lowering':
            # Wait until the joint actually returns to ARM_ZERO (within tolerance)
            # Only then signal done — robot is guaranteed the arm is back at rest
            if abs(self.arm_position - ARM_ZERO) < POSITION_TOL:
                self.get_logger().info('Arm: back at 0° (joint confirmed) — signalling done.')
                self.done_pub.publish(Bool(data=True))  # notify waypoints_control.py
                self.state = 'idle'
                self.phase_start = None                 # clear clock, ready for next trigger


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)           # initialise the ROS2 Python runtime
    node = ArmController()          # create our node
    rclpy.spin(node)                # keep it alive, processing callbacks
    node.destroy_node()             # clean up when spin() exits (e.g. Ctrl+C)
    rclpy.shutdown()                # shut down the ROS2 runtime


# Only run main() when this file is executed directly (not when it's imported)
if __name__ == '__main__':
    main()

