from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        remappings=[
            ("turtle1/cmd_vel", "hocar/cmd_vel")
        ]
    )

    draw_square_node = Node(
        package="turtlesim",
        executable="draw_square",
        remappings=[
            ("turtle1/cmd_vel", "hocar/cmd_vel")
        ]
    )

    ld.add_action(turtlesim_node)
    ld.add_action(draw_square_node)
    return ld
