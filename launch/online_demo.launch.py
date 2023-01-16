from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    ld = LaunchDescription()

    # ros params defined straight to python-dict. No need to bother with console commands and yaml files.
    camera_config = {
        'camera_device': "D415",   # if ommitted, default is default RS
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
            "mode": "MASK",         # default: MASK.  Choices: "DEPTH", "MASK"
            "viz_enabled": True,
            "camera_prefix": "",    # CAMERA_PREFIX of camera_node topic that VDC emulates
            "camera_frame": "camera_conveyor",  # camera_frame, as it is pubished by the calibration node
            "downscale_factor": 1.0,    # downscales the image, to increase performamce (e.g. 2.0). Any float will do.
            "scaling": 1.0,             # "inflates: the mask to have better coverage of the robot. keep it between [0.98, 1.0]
        }]
    )

    # ros2 launch camera_robot_calibration publish_static_tfs.launch.py calibration_files:=dummy.yaml
    ld.add_action(vdc_node)
    ld.add_action(camera_node)
    return ld
