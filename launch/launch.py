from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Add executable data for gps_info :
    data_node = Node(
        package='data',
        executable='data_node',
    )

    # Add executable for navigation :
    navigation_node = Node(
        package='navigation',
        executable='navigation_node',
    )

    # Add executable for QR code :
    qr_code_node = Node(
        package='qr_code',
        executable='qr_code_node',
    )

   # Add executable for camera controller :
    camera_controller_node = Node(
        package='camera_controller',
        executable='camera_controller_node',
    )

    ld.add_action(data_node)
    ld.add_action(navigation_node)
    ld.add_action(qr_code_node)
    ld.add_action(camera_controller_node)
    return ld
