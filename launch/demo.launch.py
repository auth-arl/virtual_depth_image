from launch import LaunchDescription
from launch_ros.actions import Node

import launch.actions
import launch.substitutions
import launch_ros.actions

from os import path
from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

print("{}".format(launch.substitutions.LaunchConfiguration('test')))



def generate_launch_description():

    ld = LaunchDescription()

    # UR_description
    # ros2 launch ur_description view_ur.launch.py ur_type:='ur10'

    package_prefix = get_package_share_directory('ur_description')
    description_node=IncludeLaunchDescription(
            PythonLaunchDescriptionSource( path.join(package_prefix ,'launch/view_ur.launch.py') ),
            launch_arguments={'ur_type': 'ur5e'}.items(),
     )

     # Camera Node
    # ros2 launch realsense2_camera rs_launch.py 
    package_prefix = get_package_share_directory('realsense2_camera')
    camera_node =IncludeLaunchDescription(
            PythonLaunchDescriptionSource( path.join(package_prefix ,'launch/rs_launch.py') )
    )
    # Camera tf 
    # ros2 run tf2_ros static_transform_publisher -2 0 0  0 0 0 1 world camera_link
    calib_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='calib_node',
        emulate_tty=True,
        output_format='[Static tf publisher] {line}',
        output='screen',
        arguments=[ "-1", "0", "0" , "0", "0", "0", "1" ,"world" ,"camera_link"]
 
    )      
    
    # ros2 launch virtual_depth_camera depth.launch.py 
    package_prefix = get_package_share_directory('virtual_depth_camera')
    vdc_node =IncludeLaunchDescription(
            PythonLaunchDescriptionSource( path.join(package_prefix ,'launch/depth.launch.py') )          
    )

    ld.add_action(calib_node)
    ld.add_action(description_node)
    ld.add_action(camera_node)
    ld.add_action(vdc_node)

    return ld
