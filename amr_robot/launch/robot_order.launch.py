import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    pkg_ros_ign_gazebo = get_package_share_directory('amr_robot')
    directory_path = DeclareLaunchArgument(
        'directory_path',
        default_value='/home/praneeth/ros2_ws/src/amr_example/amr_robot/config',
        description='path of the directory'
    )

    # Define nodes
    order_optimizer_node = Node(
        package='amr_robot',
        executable='order_optimizer',
        name='order_optimizer',
        output='screen',
        parameters=[{'directory_path': LaunchConfiguration('directory_path')}]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments and nodes to the launch description
    ld.add_action(directory_path)
    ld.add_action(order_optimizer_node)

    return ld
