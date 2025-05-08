from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    # Get the package share directory
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    re4life_description = get_package_share_directory('re4life_robot_description')

    # Robot description and controller configurations
    robot_description_path = os.path.join(re4life_description, 'urdf', 'relife_model.urdf.xacro')
    controller_config = os.path.join(re4life_description, 'config', 're4life_controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(re4life_description, 'urdf', 'relife_model.urdf.xacro')]),
        value_type=str
    )


    # Path to the map file and costmap configurations
    map_yaml_path = os.path.join(re4life_description, 'maps', 'my_gazebo_map.yaml')
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

    # Static transform publishers for the wheel links
    static_transform_publisher_base_to_left_wheel = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_left_wheel_broadcaster',
        arguments=[
            '-0.24508', '0.28586', '0.04478',    # xyz translation
            '0', '1.5708', '1.5708',             # rpy rotation
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
            '0', '1.5708', '1.5708',             # rpy rotation
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
            '0', '1.5708', '1.5708',             # rpy rotation
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
            '0', '1.5708', '1.5708',             # rpy rotation
            'base_link', 'BackWheel_link'
        ],
        output='screen'
    )

    # Map server for navigation
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 
                    'yaml_filename': map_yaml_path}]
    )

    # AMCL for localization PROBABLY THE CULPRIT FOR WRONG LOCATION IN RVIZ!!!!!!!
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Initial pose settings - match to your Gazebo spawn position
            'initial_pose.x': -2.0,  
            'initial_pose.y': 0.0,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': 0.0,
            # AMCL parameters for better convergence
            'update_min_d': 0.1,
            'update_min_a': 0.1,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'recovery_alpha_slow': 0.0,
            'recovery_alpha_fast': 0.0,
            'transform_tolerance': 1.0,
            'always_reset_initial_pose': False,
            'set_initial_pose': True,
            'tf_broadcast': True,
            'odom_frame_id': "odom",
            'base_frame_id': "base_link",

        }]
    )

    # Lifecycle manager for Nav2 components
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 4.0,
            'attempt_respawn_on_failure': True
        }]
    )

    # Navigation stack
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(re4life_description, 'config', 'controller_server.yaml')
        }.items()
    )

    # Controller node for the robot
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_config],
        output='screen'
    )


    bootstrap_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='bootstrap_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # Optional: Odometry publisher if not already handled by Gazebo or controller
    # Uncomment if you need a dedicated odometry publisher
    # odometry_publisher = Node(
    #     package='gazebo_ros',
    #     executable='gazebo_ros_diff_drive',
    #     name='diff_drive_controller',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        # First start the robot state publisher
        robot_state_publisher_node,
        
        # Then establish the TF tree with static transforms for wheel links
        static_transform_publisher_base_to_left_wheel,
        static_transform_publisher_base_to_right_wheel,
        static_transform_publisher_base_to_front_wheel,
        static_transform_publisher_base_to_back_wheel,
        
        # Start map server
        map_server_node,
                
        TimerAction(
            period=0.0,  # Start immediately
            actions=[bootstrap_map_to_odom]
        ),

        # Start AMCL (localization)
        TimerAction(
            period=2.0,  # Short delay to ensure map server is ready
            actions=[amcl_node]
        ),
        
        # Start lifecycle manager
        TimerAction(
            period=3.0,
            actions=[lifecycle_manager]
        ),
        
        # Start navigation stack
        TimerAction(
            period=5.0,  # Allow localization to initialize
            actions=[navigation]
        ),
        
        # TimerAction(
        #     period=7.0,
        #     actions=[ros2_control_node]
        # ),

        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen'
                ),
                # Node(
                #     package='controller_manager',
                #     executable='spawner',
                #     arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                #     output='screen'
                # )
            ]
        )


        # TimerAction(
        #     period=15.0,
        #     actions=[
        #         ExecuteProcess(
        #             cmd=['ros2', 'node', 'kill', '/bootstrap_map_to_odom'],
        #             output='screen'
        #         )
        #     ]
        # )
    ])