from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():

    re4life_description = get_package_share_directory("re4life_robot_description")
    re4life_description_prefix = get_package_prefix("re4life_robot_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(re4life_description, "urdf", "relife_model.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    model_path = os.path.join(re4life_description, "models")
    model_path += os.pathsep + os.path.join(re4life_description_prefix, "share")

    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    robot_description = Command(["xacro ", LaunchConfiguration("model")])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_world.world')}.items()
    )
    
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py'))
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "relife",
            "-topic", "robot_description",
            "-x", "1.0",  # X position
            "-y", "2.0",  # Y position
            "-z", "0.0",  # Z position
            "-R", "0.0",  # Roll
            "-P", "0.0",  # Pitch
            "-Y", "0.0",  # Yaw
            '--ros-args', '--log-level', 'DEBUG'
        ],
        output="screen"
    )

    return LaunchDescription([
        env_variable,
        model_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot
    ])
