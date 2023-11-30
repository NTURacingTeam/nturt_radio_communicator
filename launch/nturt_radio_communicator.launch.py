# from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    radio_sender_node = Node(
        package="nturt_radio_communication",
        executable="nturt_radio_communication_sender_node",
        output="both"
    )
    # radio_reciever_node = Node(
    #     package="nturt_radio_communication",
    #     executable="nturt_radio_communication_reciever_node",
    #     output="both"
    # )
    return LaunchDescription([radio_sender_node])