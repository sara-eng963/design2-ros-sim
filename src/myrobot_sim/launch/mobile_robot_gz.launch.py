from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = FindPackageShare(package='myrobot_sim').find('myrobot_sim')
    default_model_path = os.path.join(pkg_share, 'urdf', 'Trial_idk.urdf')

    # default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    robot_controllers = PathJoinSubstitution(
        [pkg_share, 'config', 'controllers.yaml']
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro','/home/roaa/My_Workspace/src/design2-ros-sim/src/myrobot_sim/urdf/Trial_idk.urdf']),
            'use_sim_time': True
        }]
    )

    # gz launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Bridge Node
    bridge_gz = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # General
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Gazebo_Control
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/Trial_idk/joint_state', 'joint_states'),
        ],
        output='screen'
    )

    # gz spawn robot entity
    node_gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'Trial_idk',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.03',
            '-R', '0.0',  # rotate around X (roll)
            '-P', '0.0',
            '-Y', '0.0'
        ],
        parameters=[{"use_sim_time": True}]
    )

    # controller spawn
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': Command(['xacro','/home/roaa/My_Workspace/src/design2-ros-sim/src/yrobot_sim/urdf/Trial_idk.urdf'])},
            robot_controllers,
            {"use_sim_time": True}
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        bridge_gz,
        node_gz_spawn_entity,
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])