from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_path
from launch.substitutions import Command

def generate_launch_description():
    ld = LaunchDescription()
    urdf_path=os.path.join(get_package_share_path("micro_mouse_description"),'urdf','micro_mouse.urdf.xacro')
    rviz_config_path=os.path.join(get_package_share_path("micro_mouse_description"),'rviz','urdf_config.rviz')
    robot_description=ParameterValue(Command(['xacro ',urdf_path]),value_type=str)


    robot_state_publisher_node=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description":robot_description}
            ]
    )

    joint_state_publisher_gui_node=Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node=Node(
        package="rviz2",
        executable="rviz2",
        arguments=[rviz_config_path]
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz2_node)
    return ld