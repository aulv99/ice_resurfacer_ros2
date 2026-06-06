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
    pkg_ice_nav = get_package_share_directory('ice_resurfacer_nav') 
    gazebo_resource_path = os.path.dirname(pkg_ice_description)

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.path.pathsep + gazebo_resource_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gazebo_resource_path

    # GAZEBO SIMULATION (Running headless with '-s')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ' -r ' + os.path.join(pkg_ice_gazebo, 'worlds', 'ice_rink.sdf')}.items(),
    )

    # ROBOT DESCRIPTION
    xacro_file = os.path.join(pkg_ice_description, 'urdf', 'ice_resurfacer2.urdf.xacro')
    robot_desc = Command(['xacro ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # SPAWN THE ROBOT
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'ice_resurfacer',
            '-z', '1.5',
            '-y', '-11', 
            '-x', '-34' 
        ],
        output='screen'
    )

    # SENSOR BRIDGE (Clock, Lidar, IMU)
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', 
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', 
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen'
    )

    # ODOMETRY BRIDGE
    gz_odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/ice_resurfacer/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[('/model/ice_resurfacer/odometry', '/ground_truth_pose')],
        output='screen'
    )
    
    # STATIC TF BRIDGE
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # MAP SERVER
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': os.path.join(pkg_ice_nav, 'maps', 'iihf_rink4.yaml')}]
    )
    
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{'autostart': True}, {'node_names': ['map_server']}]
    )

    # CONTROLLER SPAWNERS
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    ackermann_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
    )

    conditioner_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["conditioner_controller"],
        output="screen",
    )

    # EKF NODE
    ekf_config_path = os.path.join(
        pkg_ice_description,
        'config',
        'ekf.yaml'
    )

    start_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    # NAV2 BRINGUP
    nav2_params_path = os.path.join(
        pkg_ice_nav,
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
        sensor_bridge,
        gz_odom_bridge,      
        tf_map_odom,         
        map_server,          
        lifecycle_manager_map, 
        diff_drive_spawner,  
        ackermann_spawner,
        conditioner_spawner,    
        start_ekf_node,
        start_nav2_cmd,
    ])