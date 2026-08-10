from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. The Bridge (Translates Twist to TwistStamped)
    bridge_node = Node(
        package='ice_resurfacer_control',
        executable='drive_bridge_node',
        name='drive_bridge',
        output='log' # Keeps the terminal clean, logs to a background file
    )

    # 2. The Conditioner (Handles hardware processes)
    conditioner_node = Node(
        package='ice_resurfacer_control',
        executable='conditioner_node',
        name='conditioner_manager',
        output='screen' # Prints INFO logs to the terminal
    )

    # 3. The Master Node (Mission Control)
    autonomy_node = Node(
        package='ice_resurfacer_control',
        executable='autonomy_node',
        name='Zamboni_AI',
        output='screen' # Prints your Phase tracking to the terminal
    )

    return LaunchDescription([
        bridge_node,
        conditioner_node,
        autonomy_node
    ])