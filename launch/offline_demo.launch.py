from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    ld = LaunchDescription()

    # ros params defined straight to python-dict. No need to bother with console commands and yaml files.
    camera_config = {
        'camera_device': "Dummy",   # if ommitted, default is default RS
        "camera_prefix": "",        # default is empty
        "color_width":  1280,       # Default: 1280
        "color_height": 720,        # Default: 720
        "enable_depth": False,      # default: True
        "fps": 15,                  # Default 30
    }
    camera_node = Node(
        package='camera_utils',
        executable='camera_node',
        name='Camera_Node',
        emulate_tty=True,
        output_format="[" + camera_config["camera_prefix"] + "Camera_Node] {line}",
        output='screen',
        parameters=[camera_config]
    )

    vdc_node = Node(
        package='virtual_depth_camera',
        executable='virtual_depth_camera_node',
        name='vdc_Dummy_node',
        emulate_tty=True,
        output_format='[VIRTUAL_DEPTH_CAMERA] {line}',
        output='screen',
        parameters=[{
                "mode": "DEPTH",
                "viz_enabled": True,
                "camera_prefix": "",
                "camera_frame": "camera_conveyor",
                "downscale_factor": 2.0,
                "scaling": 1.0,
                    }]
    )

    # Static Transform publisher, so as to not bother with actually dealing with calibrartion
    static_transform_publisher_node_container = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_container_sim',
        emulate_tty=True,
        output='screen',
        arguments=["0.6", "0.1", "1.0",
                   "1.0", "0.0", "0.0", "0.0",
                   "right_ur_base_link",
                   "camera_conveyor"]
    )

    ld.add_action(vdc_node)
    ld.add_action(camera_node)
    ld.add_action(static_transform_publisher_node_container)
    return ld
