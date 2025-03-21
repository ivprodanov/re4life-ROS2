from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    re4life_description = get_package_share_directory('re4life_robot_description')

    # Paths to the launch files
    navigation_launch_file = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
    localization_launch_file = os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')

    # Path to the map file
    map_yaml_path = os.path.join(re4life_description, 'maps', 'my_map.yaml')
    global_costmap_yaml = os.path.join(re4life_description, 'config', 'global_costmap.yaml')
    local_costmap_yaml = os.path.join(re4life_description, 'config', 'local_costmap.yaml')
    # Static transform publisher node for 'map' to 'odom'
    static_transform_publisher_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        output='screen'
    )

    # Static transform publisher node for 'odom' to 'base_link'
    static_transform_publisher_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link'],
        output='screen'
    )

        # Adjusting the transforms for the wheels to be upright and pointing west
    static_transform_publisher_base_to_left_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_left_wheel_broadcaster',
        arguments=[
            '-0.24508', '0.28586', '0.04478',    # xyz translation
            '0', '1.5708', '1.5708',             # rpy rotation: 90 degrees around the y-axis, then 90 degrees around the z-axis
            'base_link', 'LeftWheel_link'
        ],
        output='screen'
    )

    static_transform_publisher_base_to_right_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_right_wheel_broadcaster',
        arguments=[
            '-0.24533', '-0.28065', '0.04553',   # xyz translation
            '0', '1.5708', '1.5708',             # rpy rotation: 90 degrees around the y-axis, then 90 degrees around the z-axis
            'base_link', 'RightWheel_link'
        ],
        output='screen'
    )

    static_transform_publisher_base_to_front_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_front_wheel_broadcaster',
        arguments=[
            '0.17030', '0.00221', '0.04710',     # xyz translation
            '0', '1.5708', '1.5708',             # rpy rotation: 90 degrees around the y-axis, then 90 degrees around the z-axis
            'base_link', 'FrontWheel_link'
        ],
        output='screen'
    )

    static_transform_publisher_base_to_back_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_back_wheel_broadcaster',
        arguments=[
            '-0.45842', '0.00285', '0.06126',    # xyz translation
            '0', '1.5708', '1.5708',             # rpy rotation: 90 degrees around the y-axis, then 90 degrees around the z-axis
            'base_link', 'BackWheel_link'
        ],
        output='screen'
    )


    # Launch localization first
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_yaml_path
        }.items()
    )

    # Start the AMCL node after a delay
    start_amcl = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'global_costmap': global_costmap_yaml,
                    'local_costmap': local_costmap_yaml,
                }]
            )
        ]
    )

    # Start the navigation stack after another delay
    start_navigation = TimerAction(
        period=10.0,  # Additional delay in seconds
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch_file),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )

    return LaunchDescription([
        # Add the static transform publishers
        static_transform_publisher_map_to_odom,
        static_transform_publisher_odom_to_base,
        static_transform_publisher_base_to_left_wheel,
        static_transform_publisher_base_to_right_wheel,
        static_transform_publisher_base_to_front_wheel,
        static_transform_publisher_base_to_back_wheel,

        # Launch the localization node
        localization,

        # Start AMCL after the delay
        start_amcl,

        # Start navigation after the second delay
        start_navigation,
    ])
