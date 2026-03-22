#!/usr/bin/env python3

import rclpy
import rclpy.parameter
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import tf_transformations
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

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

waypoints = []
goals_received = False

current_waypoint = 0
state = "align_x"

node = None
cmd_pub = None

obstacle_detected = False
startup_time = None

detour_start_x = None
detour_start_y = None
detour_reference_yaw = None

original_heading = None

obstacle_first_seen_time = None
dynamic_wait_start_time = None
pre_obstacle_state = None

arm_done_received = False
arm_trigger_pub = None

system_started = False
system_stopped = False




def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def snap_to_cardinal(angle):
    # The 4 clean axis angles: 0°, 90°, 180°, -90°
    cardinals = [0.0, math.pi/2, math.pi, -math.pi/2]
    # For each cardinal, compute the shortest angular distance to 'angle',
    # then return whichever cardinal has the smallest distance (i.e. the nearest one)
    return min(cardinals, key=lambda c: abs(normalize_angle(c - angle)))


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


def arm_done_callback(msg):
    global arm_done_received
    if msg.data:
        arm_done_received = True


def start_callback(msg):
    global system_started, system_stopped
    if msg.data:
        system_started = True
        system_stopped = False
        node.get_logger().info('START signal received — beginning/resuming navigation.')


def stop_callback(msg):
    global system_stopped
    if msg.data:
        system_stopped = True
        node.get_logger().info('STOP signal received — robot halted.')


def goals_callback(msg):
    global waypoints, goals_received, current_waypoint, state

    parsed_waypoints = []
    for pose in msg.poses:
        q = pose.orientation
        _, _, goal_yaw = tf_transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        parsed_waypoints.append((pose.position.x, pose.position.y, goal_yaw))

    if not parsed_waypoints:
        node.get_logger().warn('Received empty /navigation_goals PoseArray, still waiting for valid goals')
        return

    waypoints = parsed_waypoints
    current_waypoint = 0
    state = "align_x"
    goals_received = True
    node.get_logger().info(f'Received {len(waypoints)} navigation goals')


def control_loop():
    global x, y, yaw
    global current_waypoint, state
    global obstacle_detected
    global startup_time, last_time
    global detour_start_x, detour_start_y,detour_reference_yaw
    global original_heading
    global obstacle_first_seen_time, dynamic_wait_start_time, pre_obstacle_state
    global arm_done_received, arm_trigger_pub
    global system_started, system_stopped

    # ── Start / Stop gating ──────────────────────────────────────────────────
    if system_stopped:
        cmd = TwistStamped()
        cmd.header.stamp = node.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd_pub.publish(cmd)
        node.get_logger().info('*** SYSTEM STOPPED — robot halted ***', throttle_duration_sec=2.0)
        return

    if not system_started:
        cmd = TwistStamped()
        cmd.header.stamp = node.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd_pub.publish(cmd)
        node.get_logger().info('Waiting for /start_signal...', throttle_duration_sec=2.0)
        return

    current_time = node.get_clock().now()

    if startup_time is None:
        startup_time = current_time

    if (current_time - startup_time).nanoseconds / 1e9 < 2.0:
        return

    if not goals_received:
        cmd = TwistStamped()
        cmd.header.stamp = node.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd_pub.publish(cmd)
        node.get_logger().info("Waiting for /navigation_goals...", throttle_duration_sec=2.0)
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

    target_x, target_y, goal_yaw = waypoints[current_waypoint]
    node.get_logger().info(
        "\n---------------- DEBUG ----------------\n"
        f"CURRENT STATE: {state}\n"
        f"POSITION -> x: {x:.3f}, y: {y:.3f}, yaw: {math.degrees(yaw):.2f}°\n"
        f"TARGET   -> x: {target_x:.3f}, y: {target_y:.3f}, yaw: {math.degrees(goal_yaw):.2f}°\n"
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
        if state not in ["avoid_rotate", "avoid_move", "classifying_obstacle", "dynamic_wait"]:
            pre_obstacle_state = state
            obstacle_first_seen_time = current_time
            state = "classifying_obstacle"
            node.get_logger().info("Obstacle detected — classifying (3s wait)...", throttle_duration_sec=1.0)
        elif state == "dynamic_wait":
            # New obstacle appeared during dynamic wait → re-classify
            obstacle_first_seen_time = current_time
            state = "classifying_obstacle"
            node.get_logger().info("New obstacle during dynamic wait — re-classifying...", throttle_duration_sec=1.0)

    # Classifying obstacle: robot stops and waits 3s to determine static vs dynamic
    if state == "classifying_obstacle":
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        elapsed = (current_time - obstacle_first_seen_time).nanoseconds / 1e9
        if elapsed >= 2.0:
            if obstacle_detected:
                node.get_logger().info("Static obstacle confirmed — starting avoidance")
                state = "avoid_rotate"
            else:
                node.get_logger().info("Dynamic obstacle — stopping for 2 seconds")
                dynamic_wait_start_time = current_time
                state = "dynamic_wait"

    # Dynamic obstacle wait: obstacle cleared, stop 4s then resume
    elif state == "dynamic_wait":
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        elapsed = (current_time - dynamic_wait_start_time).nanoseconds / 1e9
        if elapsed >= 2.0:
            node.get_logger().info("Dynamic obstacle cleared — resuming navigation")
            obstacle_first_seen_time = None
            dynamic_wait_start_time = None
            state = pre_obstacle_state if pre_obstacle_state else "align_x"
            pre_obstacle_state = None

    # Avoid rotate
    elif state == "avoid_rotate":
        if detour_reference_yaw is None:
            detour_reference_yaw = normalize_angle(snap_to_cardinal(yaw) + math.pi / 2)
        error = normalize_angle(detour_reference_yaw - yaw)

        if abs(error) > ANGLE_TOL:
            cmd.twist.angular.z = max(min(K_ANG * error, MAX_ANG), -MAX_ANG)
        else:
            cmd.twist.angular.z = 0.0
            detour_start_x, detour_start_y = x, y
            detour_reference_yaw = None
            state = "avoid_move"




    # Avoid move (sideways detour)
    elif state == "avoid_move":
        dx_d = x - detour_start_x
        dy_d = y - detour_start_y
        dist = math.sqrt(dx_d**2 + dy_d**2)

        # lock yaw during detour
        cmd.twist.angular.z = 0.0

        if dist < 0.5:   # move sideways for 0.5 m
            cmd.twist.linear.x = 0.2
        else:
            cmd.twist.linear.x = 0.0
        # Decide next alignment based on original heading
            if original_heading in [0.0, math.pi]:       # was going along Y
                state = "align_y"                        # now fix y
            elif original_heading in [math.pi/2, -math.pi/2]:  # was going along X
                state = "align_x"                        # now fix x
            else:
                state = "align_x"  # fallback

        # Align X
    elif state == "align_x":
        # +X → yaw = +π/2, -X → yaw = -π/2
        target_yaw = math.pi/2 if dx > 0 else -math.pi/2
        original_heading = target_yaw
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
        original_heading = target_yaw
        error = normalize_angle(target_yaw - yaw)

        if abs(dy) < DIST_TOL:
            state = "rotate_goal"
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
                # Both X and Y are correct -> rotate to final waypoint yaw
                state = "rotate_goal"

    # Rotate to final waypoint orientation after position is reached
    elif state == "rotate_goal":
        goal_error = normalize_angle(goal_yaw - yaw)

        if abs(goal_error) > ANGLE_TOL:
            cmd.twist.angular.z = max(min(K_ANG * goal_error, MAX_ANG), -MAX_ANG)
        else:
            cmd.twist.angular.z = 0.0
            arm_done_received = False
            arm_trigger_pub.publish(Bool(data=True))
            state = "arm_action"
            node.get_logger().info("Waypoint reached — waiting for arm to complete action.")

    # Arm action: robot holds still while arm does its job
    elif state == "arm_action":
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        if arm_done_received:
            arm_done_received = False
            current_waypoint = (current_waypoint + 1) % len(waypoints)
            state = "align_x"
            node.get_logger().info(f"Arm done — moving to waypoint {current_waypoint}.")


    cmd_pub.publish(cmd)


def main():
    global node, cmd_pub, arm_trigger_pub

    rclpy.init()

    node = Node('square_follower', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)
    ])

    cmd_pub = node.create_publisher(
        TwistStamped,
        '/diff_drive_controller/cmd_vel',
        10
    )

    arm_trigger_pub = node.create_publisher(Bool, '/arm_trigger', 10)

    node.create_subscription(Imu, '/imu', imu_callback, 10)
    node.create_subscription(Odometry, '/diff_drive_controller/odom', odom_callback, 10)
    node.create_subscription(Bool, '/obstacle_detected', obstacle_callback, 10)
    node.create_subscription(PoseArray, '/navigation_goals', goals_callback, 10)
    node.create_subscription(Bool, '/arm_done', arm_done_callback, 10)

    latched_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE
    )
    node.create_subscription(Bool, '/start_signal', start_callback, latched_qos)
    node.create_subscription(Bool, '/stop_signal',  stop_callback,  latched_qos)

    node.create_timer(0.05, control_loop)

    node.get_logger().info("Waypoint control Node Started, waiting for /navigation_goals")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()