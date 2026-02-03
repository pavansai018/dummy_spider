import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Package paths
    pkg_path = get_package_share_directory('dummy_spider')
    # Change 'my_gazebo_worlds' to your actual package name where small_maze.sdf lives
    pkg_worlds = get_package_share_directory('my_gazebo_worlds') 
    
    # Files
    world_file = os.path.join(pkg_worlds, 'worlds', 'small_maze.sdf')
    xacro_file = os.path.join(pkg_path, 'urdf', 'mazespider.xacro')
    
    # 1. Robot State Publisher
    robot_description = Command(['xacro ', xacro_file])
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # 2. Gazebo Sim (Loading your Spiral Maze)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 3. Spawn Robot (Positioned at your Left Entrance)
    # Using -x -4.0, -y 4.0 to put it in the opening near out_left/out_top
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'spider', 
            '-x', '-4.0', 
            '-y', '4.0', 
            '-z', '0.2'
        ],
        output='screen'
    )

    # 4. Universal Bridge Node
    # Matches your empty-world syntax exactly for compatibility
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
        ],
        output='screen'
    )

    # 5. Controllers Spawners
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
        # Timer matches your 8s delay for stability
        TimerAction(period=8.0, actions=[load_joint_state_broadcaster, load_position_controller]),
    ])