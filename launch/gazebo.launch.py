"""Launch Gazebo server and client with command line arguments."""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Set to "false" to run gazebo headless.'),

        DeclareLaunchArgument('world', default_value='',
                              description='Specify gazebo world file name in gazebo_package'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory("gazebo_ros"), '/launch/gzserver.launch.py']),
            launch_arguments = {
                'server_required': 'false',
                'verbose': 'true',
                'world': LaunchConfiguration('world'),
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory("gazebo_ros"), '/launch/gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui')),
        ),

    ])
