from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    GroupAction,
    RegisterEventHandler,
    DeclareLaunchArgument
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction


def generate_launch_description():

    pkg_share = FindPackageShare(package='myrobot_sim').find('myrobot_sim')

    default_model_path = os.path.join(
        pkg_share,
        'urdf',
        'Trial_idk.urdf'
    )

    # default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    robot_controllers = PathJoinSubstitution([
        pkg_share,
        'config',
        'controllers.yaml'
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', default_model_path]),
                value_type=str
        ),
        'use_sim_time': True
    }]
    )

    world_file_path = os.path.join(pkg_share, 'world', 'my_world.sdf')   
    # gz launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': f'-r -v 4 {world_file_path}'}.items()
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
            #'/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ],
        remappings=[
            ('/world/empty/model/Robot_Body_URDF/joint_state', 'joint_states'),
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
            '-name', 'Robot_Body_URDF',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.12566',
            '-R', '0.0',  # rotate around X (roll)
            '-P', '0.0',
            '-Y', '0.0'
        ],
        parameters=[{"use_sim_time": True}]
    )

    # controller spawn
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            )
        ]
    )
    diff_drive_controller_spawner = TimerAction(
        period=7.0,  # start a bit later than joint_state_broadcaster
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller"],
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        bridge_gz,
        node_gz_spawn_entity,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])