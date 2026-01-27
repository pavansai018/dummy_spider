import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_path = get_package_share_directory('dummy_spider')
    
    # 1. Robot State Publisher
    robot_description = Command(['xacro ', os.path.join(pkg_path, 'urdf', 'spider.xacro')])
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 2. Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 3. Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'spider', '-z', '0.2'],
        output='screen'
    )

    # Bridge node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
                   '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   ],
        output='screen'
    )

    # 5. Controllers
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    load_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"],
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gz_sim,
        spawn_robot,
        bridge,
        # Wait for Gazebo to load before spawning controllers
        TimerAction(period=8.0, actions=[load_joint_state_broadcaster, load_position_controller]),
    ])