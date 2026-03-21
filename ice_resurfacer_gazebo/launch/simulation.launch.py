import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # PATHS
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ice_gazebo = get_package_share_directory('ice_resurfacer_gazebo')
    pkg_ice_description = get_package_share_directory('ice_resurfacer_description')

    # GAZEBO SIMULATION
    # Launching Gazebo with the 'ice_rink.sdf' world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + os.path.join(pkg_ice_gazebo, 'worlds', 'ice_rink.sdf')}.items(),
    )

    # ROBOT DESCRIPTION (XACRO -> URDF)
    # Parsing the xacro file so Gazebo knows what to spawn
    xacro_file = os.path.join(pkg_ice_description, 'urdf', 'ice_resurfacer.urdf.xacro')
    robot_desc = Command(['xacro ', xacro_file])

    # Node for publishing the robot state to the ROS /robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # SPAWN THE ROBOT
    # Node for telling Gazebo to take the robot description and create a model
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'ice_resurfacer',
            '-z', '2.5',
            '-y', '13',
            '-x', '-20'
        ],
        output='screen'
    )

    # BRIDGE (ROS <-> Gazebo)
    # Connecting the simulation time so ROS knows how fast time is passing
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        output='screen'
    )

    # CONTROLLER SPAWNERS
    # Node for telling the controller manager to start the 'joint_state_broadcaster'
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Node for telling the controller manager to start the 'ackermann_steering_controller'
    ackermann_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
    )

    ekf_config_path = os.path.join(
        get_package_share_directory('ice_resurfacer_description'),
        'config',
        'ekf.yaml'
    )

    # Node for starting Extended Kalman Filter
    start_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    # Nav2 Bringup
    nav2_params_path = os.path.join(
        get_package_share_directory('ice_resurfacer_description'),
        'config',
        'nav2_params.yaml'
    )
    
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_path
        }.items()
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        diff_drive_spawner,  
        ackermann_spawner,    
        start_ekf_node,
        start_nav2_cmd,
    ])