from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,  # Use simulated time
                'slam_toolbox/enable_interactive_mode': False,
                'slam_toolbox/map_file_name': '~/ros2_ws/src/re4life_robot_description/maps/my_map',
                'slam_toolbox/odom_frame': 'odom',
                'slam_toolbox/map_frame': 'map',
                'slam_toolbox/base_frame': 'base_link',
            }],
        ),
    ])
