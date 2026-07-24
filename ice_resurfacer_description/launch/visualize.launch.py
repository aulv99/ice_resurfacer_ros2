import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ice_resurfacer_description')
    
    # Check if we are using the xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'ice_resurfacer3.urdf.xacro')

    # RViz config
    config_file = os.path.join(pkg_share, 'rviz', 'rviz_config1.rviz')

    # RViz2 
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', config_file]
    )

    return LaunchDescription([
        rviz
    ])