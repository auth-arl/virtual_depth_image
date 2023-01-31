from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    ld = LaunchDescription()

    vdc_node = Node(
        package='virtual_depth_camera',
        executable='virtual_depth_camera_node',
        name='vdc_ope_node',
        emulate_tty=True,
        output_format='[VIRTUAL_DEPTH_CAMERA] {line}',
        output='screen',
        parameters=[{
            "mode": "MASK",         # default: MASK.  Choices: "DEPTH", "MASK"
            "viz_enabled": False,
            "camera_prefix": "ope_",    # CAMERA_PREFIX of camera_node topic that VDC emulates
            "camera_frame": "camera_color_optical_frame",  # camera_frame, as it is pubished by the calibration node
            "downscale_factor": 2.0,    # downscales the image, to increase performamce (e.g. 2.0). Any float will do.
            "scaling": 1.0,             # "inflates: the mask to have better coverage of the robot. keep it between [0.98, 1.0]
            "camera_info_topic":"/camera/color/camera_info",
            "camera_color_topic":"/camera/color/image_raw",
            "mask_topic":"/camera/mask",
            "composite_topic": "/camera/composite",
            "description_service": "robot_state_publisher/get_parameters",
            "description_package_name": "ur_description",
        }]
    )

    ld.add_action(vdc_node)
    return ld
