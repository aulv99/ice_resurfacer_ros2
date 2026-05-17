import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    my_nav_dir = get_package_share_directory('ice_resurfacer_nav')

    # TF bridge. Merges the map and odom frames due to ground truth localisation
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Translates Gazebo odometry to ROS2 ground_truth_pose
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=['/model/ice_resurfacer/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry]'],
        remappings=[('/model/ice_resurfacer/odometry', '/ground_truth_pose')],
        output='screen'
    )

    # Map server. Loads the SLAM generated map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': os.path.join(my_nav_dir, 'maps', 'iihf_rink4.yaml')}]
    )

    # Lifecycle manager. Turns the map server
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{'autostart': True}, {'node_names': ['map_server']}]
    )

    # Nav2 Stack. Launches planners and costmaps
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(my_nav_dir, 'config', 'nav2_params.yaml')
        }.items()
    )

    return LaunchDescription([
        tf_map_odom,
        gz_bridge,
        map_server,
        lifecycle_manager_map,
        navigation
    ])