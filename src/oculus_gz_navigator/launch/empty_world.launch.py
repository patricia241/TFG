import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    launch_rmw_dir = get_package_share_directory('oculus_gz_navigator')

    '''
    paused = LaunchConfiguration('paused')

    declare_paused_cmd = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Paused')
    '''
    gazebo_cmd = Node(
            package='Gazebo',
            namespace='Gazebo',
            executable='gazebo',
            name='sim'
        )
    ld = LaunchDescription()

    ld.add_action(gazebo_cmd)
    return ld