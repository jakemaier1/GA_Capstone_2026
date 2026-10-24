from launch import LaunchDescription
from launch_ros.actions import Node 
import os
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    CAMERA_TYPE = os.getenv('CAMERA_TYPE')

    if CAMERA_TYPE == 'usb':
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('usb_cam'), 'launch'),
                '/camera.launch.py'
            ])
        )
    else:
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('ascamera'), 'launch'),
                '/hp60c.launch.py'
            ])
        )

        camera_app = Node(
            package='yahboomcar_depth',
            executable='camera_app',
        )

    return LaunchDescription([
        camera_launch,
        camera_app,

    ])