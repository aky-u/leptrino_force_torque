from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="left_com_port",
                default_value="/dev/ttyACM0",
                description="Serial port for the left Leptrino FT sensor.",
            ),
            DeclareLaunchArgument(
                name="right_com_port",
                default_value="/dev/ttyACM1",
                description="Serial port for the right Leptrino FT sensor.",
            ),
            # Left sensor  →  /left/force_torque
            Node(
                package="leptrino_force_torque",
                namespace="left",
                executable="leptrino_force_torque_node",
                parameters=[{"com_port": LaunchConfiguration("left_com_port"), "rate": 1200}],
                output="screen",
            ),
            # Right sensor  →  /right/force_torque
            Node(
                package="leptrino_force_torque",
                namespace="right",
                executable="leptrino_force_torque_node",
                parameters=[{"com_port": LaunchConfiguration("right_com_port"), "rate": 1200}],
                output="screen",
            ),
        ]
    )
