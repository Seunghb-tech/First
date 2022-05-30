from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    topic_subscriber_node = Node( 
        package="hocarmini_pkg",
        executable="raspi_subscriber"
    )

    turtle_teleop_key_node = Node( 
        package="turtlesim",
        executable="turtle_teleop_key",
        remappings=[ 
            ("turtle1/cmd_vel", "hocar/cmd_vel")
        ]
    )

    ld.add_action(topic_subscriber_node)
    ld.add_action(turtle_teleop_key_node)
    return ld