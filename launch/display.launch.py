# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.parameter_descriptions import ParameterValue
# from launch.substitutions import Command, LaunchConfiguration

# def generate_launch_description():

#     model_arg = DeclareLaunchArgument(
#         name="model",
#         default_value=os.path.join(get_package_share_directory("re4life_robot_description"), "urdf", "relife-model.urdf.xacro"),
#         description="Absolute path to robot urdf file"
#     )

#     robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

#     robot_state_publisher = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         parameters=[{"robot_description": robot_description}]
#     )

#     joint_state_publisher_gui = Node(
#         package="joint_state_publisher_gui",
#         executable="joint_state_publisher_gui"
#     )

#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="screen",
#         arguments=["-d", os.path.join(get_package_share_directory("re4life_robot_description"), "rviz", "display.rviz")]
#     )

#     return LaunchDescription([
#         model_arg,
#         robot_state_publisher,
#         joint_state_publisher_gui,
#         rviz_node
#     ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("re4life_robot_description"), "urdf", "relife_model.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("re4life_robot_description"), "rviz", "display.rviz")]
    )

    controller_params_file = os.path.join(get_package_share_directory(package_name='re4life_controller'),'config', 're4life_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    return LaunchDescription([
        model_arg,
        delayed_controller_manager,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])

