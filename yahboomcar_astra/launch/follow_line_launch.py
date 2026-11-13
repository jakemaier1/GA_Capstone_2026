import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Our local copy of the A1 car bringup (base driver, IMU, EKF, joystick, ...).
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yahboomcar_astra'),
                'launch', 'yahboomcar_bringup_A1_launch.py')
        )
    )

    # Start line following a few seconds later so the driver is up first.
    follow_line = TimerAction(
        period=3.0,
        actions=[Node(
            package='yahboomcar_astra',
            executable='follow_line',
            output='screen',
        )],
    )

    return LaunchDescription([bringup_launch, follow_line])
