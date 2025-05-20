from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    com_port = DeclareLaunchArgument(name="comport", default_value="/dev/ttyACM0")

    return LaunchDescription(
        [
            com_port,
            # Left sensor
            Node(
                package="leptrino_force_torque",
                namespace="left",
                executable="leptrino_force_torque_node",
                parameters=[{"com_port": "/dev/ttyACM0", "rate": 1200}],
                output="screen",
            ),
            #    # Right sensor
            #    Node(package="leptrino_force_torque",
            #         namespace="right",
            #         executable="leptrino_force_torque",
            #         parameters=[{"com_port":"/dev/LPTRN-01", "rate":1200}],
            #         output="screen")
        ]
    )
