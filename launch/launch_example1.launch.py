from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_random_trees',
            executable='display_tree_node',
            output='screen',
        ),
    ])